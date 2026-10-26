#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <vector>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif
#include "2_Transport/messaging/websocket_server.hpp"
#include "4_Applications/mapping/room_map/room_map.hpp"
#include "4_Applications/pointcloud_websocket/pointcloud_websocket.hpp"
#include "models/topics.hpp"
#include "unittest/test.hpp"
#include "4_Applications/mapping/gaussian_map/gaussian_map.hpp"
#include "4_Applications/perception/foreground/foreground.hpp"

namespace {
using namespace std::chrono_literals;
#ifdef _WIN32
using Socket=SOCKET;constexpr Socket invalid=INVALID_SOCKET;
struct Runtime {Runtime(){WSADATA data{};if(WSAStartup(MAKEWORD(2,2),&data)) throw std::runtime_error("WSAStartup");}~Runtime(){WSACleanup();}};
void close_test_socket(Socket socket){closesocket(socket);}
#else
using Socket=int;constexpr Socket invalid=-1;
struct Runtime {};
void close_test_socket(Socket socket){close(socket);}
#endif
struct Client {
    Socket socket{invalid};
    ~Client(){if(socket!=invalid) close_test_socket(socket);}
    void connect(std::uint16_t port) {
        socket=::socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
        if(socket==invalid) throw std::runtime_error("test client socket");
#ifdef _WIN32
        DWORD timeout=3000;setsockopt(socket,SOL_SOCKET,SO_RCVTIMEO,reinterpret_cast<const char*>(&timeout),sizeof(timeout));
#else
        timeval timeout{3,0};setsockopt(socket,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
#endif
        sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(port);
        address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        if(::connect(socket,reinterpret_cast<sockaddr*>(&address),sizeof(address))) throw std::runtime_error("test connect");
        const std::string request="GET / HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n";
        send_bytes(std::vector<std::uint8_t>(request.begin(),request.end()));
    }
    void send_bytes(const std::vector<std::uint8_t>& bytes) {
        std::size_t sent{};while(sent<bytes.size()) {
            const auto n=send(socket,reinterpret_cast<const char*>(bytes.data()+sent),static_cast<int>(bytes.size()-sent),0);
            if(n<=0) throw std::runtime_error("test send");sent+=static_cast<std::size_t>(n);
        }
    }
    std::vector<std::uint8_t> read_bytes(std::size_t count) {
        std::vector<std::uint8_t> bytes(count);std::size_t read{};
        while(read<count) {
            const auto n=recv(socket,reinterpret_cast<char*>(bytes.data()+read),static_cast<int>(count-read),0);
            if(n<=0) throw std::runtime_error("test read timeout/close");read+=static_cast<std::size_t>(n);
        }return bytes;
    }
    void handshake() {
        std::string header;
        while(header.find("\r\n\r\n")==std::string::npos && header.size()<8192)
            header.push_back(static_cast<char>(read_bytes(1)[0]));
        if(header.find("101 Switching Protocols")==std::string::npos) throw std::runtime_error("test handshake");
    }
    std::pair<unsigned,std::vector<std::uint8_t>> read_frame() {
        const auto header=read_bytes(2);std::uint64_t length=header[1]&127U;
        if(length==126) {const auto n=read_bytes(2);length=n[0]*256U+n[1];}
        else if(length==127) {const auto n=read_bytes(8);length=0;for(const auto byte:n) length=(length<<8)|byte;}
        if(length>1000000 || (header[1]&128U)) throw std::runtime_error("invalid test frame");
        return {header[0]&15U,read_bytes(static_cast<std::size_t>(length))};
    }
};
std::uint16_t available_port() {
    Client probe;probe.socket=::socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    if(bind(probe.socket,reinterpret_cast<sockaddr*>(&address),sizeof(address))) throw std::runtime_error("test bind");
#ifdef _WIN32
    int size=sizeof(address);
#else
    socklen_t size=sizeof(address);
#endif
    if(getsockname(probe.socket,reinterpret_cast<sockaddr*>(&address),&size)) throw std::runtime_error("getsockname");
    return ntohs(address.sin_port);
}
std::vector<std::uint8_t> masked_text(const std::string& text) {
    std::vector<std::uint8_t> result{0x81};
    if(text.size()<126) result.push_back(static_cast<std::uint8_t>(0x80U|text.size()));
    else {result.push_back(0xfe);result.push_back(static_cast<std::uint8_t>(text.size()>>8));result.push_back(static_cast<std::uint8_t>(text.size()));}
    const std::array<std::uint8_t,4> mask{1,2,3,4};result.insert(result.end(),mask.begin(),mask.end());
    for(std::size_t i=0;i<text.size();++i) result.push_back(static_cast<std::uint8_t>(text[i])^mask[i%4]);return result;
}
}

VISTA_TEST(foreground_websocket_replaces_clears_stale_data_and_does_not_replay_old_objects) {
    Runtime runtime;
    vista::platform::MessageBus bus;vista::platform::StopToken stop;
    auto publisher=bus.publisher<vista::models::ForegroundFrame>(vista::models::topics::foreground_points);
    vista::application::PointCloudWebSocketConfig config;
    config.port=available_port();config.maximum_points=10;config.publish_interval=500ms;
    std::string failure;
    auto worker=vista::application::spawn_foreground_websocket(bus,{"test-foreground-socket",5},stop,config,
        [&](auto,auto error){failure=std::move(error);});
    struct Finish {vista::platform::StopToken& stop;vista::platform::MessageBus& bus;vista::platform::WorkerHandle& worker;
        ~Finish(){stop.request_stop();bus.close();worker.join();}} finish{stop,bus,worker};
    std::this_thread::sleep_for(100ms);
    Client client;client.connect(config.port);client.handshake();
    const auto metadata=client.read_frame();VISTA_CHECK(metadata.first==1);
    const std::string waiting(metadata.second.begin(),metadata.second.end());
    VISTA_CHECK(waiting.find("foreground-status")!=std::string::npos && waiting.find("\"active\":false")!=std::string::npos);
    VISTA_CHECK(client.read_frame().second.size()==32); // Nothing retained at startup.
    vista::models::ForegroundFrame frame;frame.source="test";frame.sequence=42;
    frame.received_monotonic_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    frame.points={{1,2,3,99}};frame.input_points=1;frame.reason="foreground; \"test\"\n";
    const auto sent_at=std::chrono::steady_clock::now();publisher.publish(frame);
    bool saw_motion=false;
    bool explained=false;
    for(int i=0;i<20 && !saw_motion;++i) {
        const auto packet=client.read_frame();
        if(packet.first==1) {const std::string text(packet.second.begin(),packet.second.end());
            explained=text.find("\"active\":true")!=std::string::npos && text.find("\\\"test\\\"\\n")!=std::string::npos;}
        else saw_motion=packet.second.size()==48;
    }
    VISTA_CHECK(saw_motion);
    VISTA_CHECK(explained);
    // The socket forwards a sampled result immediately, without another 500 ms
    // publication timer, and does not repeatedly transmit unchanged active data.
    VISTA_CHECK(std::chrono::steady_clock::now()-sent_at<400ms);
    fd_set ready;FD_ZERO(&ready);FD_SET(client.socket,&ready);timeval quiet{0,100000};
    VISTA_CHECK(select(static_cast<int>(client.socket)+1,&ready,nullptr,nullptr,&quiet)==0);
    publisher.publish(vista::models::ForegroundFrame{});
    bool cleared=false;
    for(int i=0;i<20 && !cleared;++i) {const auto packet=client.read_frame();cleared=packet.first==2 && packet.second.size()==32;}
    VISTA_CHECK(cleared);
    frame.received_monotonic_ns-=2'000'000'000ULL;publisher.publish(frame);
    std::this_thread::sleep_for(100ms);
    Client late;late.connect(config.port);late.handshake();
    const auto stale=late.read_frame();VISTA_CHECK(stale.first==1);
    const std::string reason(stale.second.begin(),stale.second.end());
    VISTA_CHECK(reason.find("older than 500 ms")!=std::string::npos && reason.find("\"active\":false")!=std::string::npos);
    VISTA_CHECK(late.read_frame().second.size()==32); // Delayed queue arrival is not fresh evidence.
    stop.request_stop();bus.close();worker.join();VISTA_CHECK(failure.empty());
}

VISTA_TEST(websocket_server_isolates_clients_buffers_partial_reads_and_rejects_oversized_requests) {
    Runtime runtime;
    auto server=vista::transport::WebSocketServer::listen({"127.0.0.1",available_port(),4,1000ms,250ms});
    Client first,second;first.connect(server.config().port);server.poll_accept();first.handshake();
    second.connect(server.config().port);server.poll_accept();second.handshake();
    const auto ids=server.client_ids();VISTA_CHECK(ids.size()==2 && ids[0]!=ids[1]);
    const auto frame=masked_text("VIEW first");
    first.send_bytes({frame.begin(),frame.begin()+3});
    std::this_thread::sleep_for(5ms);VISTA_CHECK(server.poll_text().empty());
    first.send_bytes({frame.begin()+3,frame.end()});second.send_bytes(masked_text("VIEW second"));
    std::vector<vista::transport::WebSocketTextMessage> requests;
    for(unsigned i=0;i<20 && requests.size()<2;++i) {
        auto values=server.poll_text();requests.insert(requests.end(),values.begin(),values.end());std::this_thread::sleep_for(5ms);
    }
    VISTA_CHECK(requests.size()==2);
    VISTA_CHECK(requests[0].client_id!=requests[1].client_id);
    VISTA_CHECK(server.send_text(ids[0],"first-only"));
    const auto response=first.read_frame();VISTA_CHECK(response.first==1);
    VISTA_CHECK(std::string(response.second.begin(),response.second.end())=="first-only");
    second.send_bytes(masked_text(std::string(2049,'x')));
    for(unsigned i=0;i<20 && server.client_count()>1;++i){server.poll_text();std::this_thread::sleep_for(5ms);}
    VISTA_CHECK(server.client_count()==1);
    const std::array<std::uint8_t,4> bytes{'L','P','C','1'};
    VISTA_CHECK(server.send_binary(ids[0],bytes.data(),bytes.size()));
    const auto binary=first.read_frame();VISTA_CHECK(binary.first==2 && binary.second.size()==4);
}

VISTA_TEST(room_map_websocket_replays_to_late_client_and_serves_camera_view) {
    Runtime runtime;
    auto options=vista::application::RoomMapConfig{};options.voxel_size_m=1;options.tile_size_m=1;
    options.minimum_observations=1;options.freeze_after=0ms;
    vista::application::RoomMapAccumulator map(options);
    map.integrate({1,{{1.25F,0.25F,0.25F,1,0,0,0},{-1.25F,0.25F,0.25F,2,0,0,0}}},0ms);map.freeze();map.flush();
    vista::platform::MessageBus bus;vista::platform::StopToken stop;
    auto publisher=bus.publisher<vista::models::RoomMapViewMessage>(vista::models::topics::room_map_view);
    // Selecting the map topic must never subscribe with a LiDAR message type.
    vista::application::PointCloudWebSocketConfig config;config.input_topic=vista::models::topics::room_map;
    config.port=available_port();config.maximum_points=10;config.publish_interval=20ms;
    std::string failure;
    auto worker=vista::application::spawn_pointcloud_websocket(bus,{"test-room-lod",4},stop,config,
        [&](auto,auto error){failure=std::move(error);});
    struct Finish {vista::platform::StopToken& stop;vista::platform::MessageBus& bus;vista::platform::WorkerHandle& worker;
        ~Finish(){stop.request_stop();bus.close();worker.join();}} finish{stop,bus,worker};
    publisher.publish({map.view_source(),7,1,2,vista::models::RoomMapState::frozen});
    std::this_thread::sleep_for(100ms);
    Client client;client.connect(config.port);client.handshake();
    const auto metadata=client.read_frame();
    VISTA_CHECK(metadata.first==1);
    VISTA_CHECK(std::string(metadata.second.begin(),metadata.second.end()).find("\"totalPoints\":2")!=std::string::npos);
    const auto overview=client.read_frame();VISTA_CHECK(overview.first==2 && overview.second.size()==64);
    client.send_bytes(masked_text("VIEW 10 600 0 0 10 1 0 0 0 -1 0 0 100 0 1 0 100 0 -1 0 100 0 0 1 100 0 0 -1 100"));
    VISTA_CHECK(client.read_frame().first==1);
    const auto view=client.read_frame();VISTA_CHECK(view.first==2 && view.second.size()==48);
    float x{};std::memcpy(&x,view.second.data()+32,4);VISTA_CHECK(x>0);
    stop.request_stop();bus.close();worker.join();VISTA_CHECK(failure.empty());
}

VISTA_TEST(gaussian_worker_and_websocket_use_full_map_checkpoint_and_clear_load_errors) {
    Runtime runtime;
    vista::application::RoomMapConfig options;
    options.voxel_size_m=1;options.tile_size_m=1;options.minimum_observations=1;
    options.freeze_after=0ms;options.surface_snapshots=true;
    vista::application::RoomMapAccumulator map(options);
    map.integrate({1,{{1.25F,.25F,.25F,77,0,0,0},{-1.25F,.25F,.25F,88,0,0,0}}},0ms);
    map.freeze();map.flush();
    vista::platform::MessageBus bus;vista::platform::StopToken stop;
    auto publisher=bus.publisher<vista::models::RoomMapViewMessage>(vista::models::topics::room_map_view);
    auto observer=bus.subscribe<vista::models::RoomMapViewMessage>(vista::models::topics::gaussian_map_view,"test-gaussian-output");
    vista::application::PointCloudWebSocketConfig config;config.gaussian_map=true;config.room_map_lod=true;
    config.port=available_port();config.maximum_points=10;config.publish_interval=20ms;
    std::string build_error,socket_error;
    std::optional<vista::application::GaussianMapReport> report;
    auto socket=vista::application::spawn_pointcloud_websocket(bus,{"test-gaussian-websocket",4},stop,config,
        [&](auto,auto error){socket_error=std::move(error);});
    auto worker=vista::application::spawn_gaussian_map_worker(bus,{"test-gaussian-builder",4},stop,{},
        [&](auto result,auto error){report=result;build_error=std::move(error);});
    struct Finish {
        vista::platform::StopToken& stop;vista::platform::MessageBus& bus;
        vista::platform::WorkerHandle& worker;vista::platform::WorkerHandle& socket;
        ~Finish(){stop.request_stop();bus.close();worker.join();socket.join();}
    } finish{stop,bus,worker,socket};
    auto source=map.view_source();
    publisher.publish({source,7,123,2,vista::models::RoomMapState::frozen});
    std::shared_ptr<const vista::models::RoomMapViewMessage> output;
    VISTA_CHECK(observer.receive_for(output,3s)==vista::platform::ReceiveStatus::message);
    VISTA_CHECK(output->source && output->state==vista::models::RoomMapState::frozen);
    const auto revision=output->revision;
    std::this_thread::sleep_for(100ms);
    Client client;client.connect(config.port);client.handshake();
    VISTA_CHECK(client.read_frame().first==1);
    const auto frame=client.read_frame();
    VISTA_CHECK(frame.first==2 && frame.second.size()==32+2*44 && std::memcmp(frame.second.data(),"GSP1",4)==0);
    client.send_bytes(masked_text("VIEW 10 600 0 0 10 1 0 0 0 -1 0 0 100 0 1 0 100 0 -1 0 100 0 0 1 100 0 0 -1 100"));
    VISTA_CHECK(client.read_frame().first==1);
    VISTA_CHECK(client.read_frame().second.size()==32+44);
    publisher.publish({source,8,124,2,vista::models::RoomMapState::frozen});
    VISTA_CHECK(observer.receive_for(output,2s)==vista::platform::ReceiveStatus::message);
    VISTA_CHECK(output->revision==revision); // No changed geometry: no rebuild/resend.
    publisher.publish({nullptr,9,125,0,vista::models::RoomMapState::load_error});
    VISTA_CHECK(client.read_frame().first==1);
    VISTA_CHECK(client.read_frame().second.size()==32);
    VISTA_CHECK(!stop.is_stop_requested());
    stop.request_stop();bus.close();worker.join();socket.join();
    VISTA_CHECK(build_error.empty() && socket_error.empty() && report && report->completed_revisions==1);
}

VISTA_TEST(room_map_websocket_sends_new_checkpoint_without_second_interval_delay) {
    struct Source final : vista::models::RoomMapViewSource {
        mutable std::atomic<unsigned> queries{};
        vista::models::RoomMapFrame select_view(const vista::models::RoomMapViewRequest&) const override {
            ++queries;return {1,{{1,2,3,1}}};
        }
        std::array<double,6> bounds() const override {return {0,0,0,4,4,4};}
    };
    Runtime runtime;
    vista::platform::MessageBus bus;vista::platform::StopToken stop;
    auto publisher=bus.publisher<vista::models::RoomMapViewMessage>(vista::models::topics::room_map_view);
    vista::application::PointCloudWebSocketConfig config;config.room_map_lod=true;
    config.port=available_port();config.publish_interval=10s;config.maximum_points=10;
    std::string failure;
    auto worker=vista::application::spawn_pointcloud_websocket(bus,{"test-checkpoint-send",4},stop,config,
        [&](auto,auto error){failure=std::move(error);});
    struct Finish {vista::platform::StopToken& stop;vista::platform::MessageBus& bus;vista::platform::WorkerHandle& worker;
        ~Finish(){stop.request_stop();bus.close();worker.join();}} finish{stop,bus,worker};
    auto source=std::make_shared<Source>();
    publisher.publish({source,1,1,1,vista::models::RoomMapState::building});
    std::this_thread::sleep_for(100ms);
    Client client;client.connect(config.port);client.handshake();
    VISTA_CHECK(client.read_frame().first==1);VISTA_CHECK(client.read_frame().first==2);
    VISTA_CHECK(source->queries==1);
    publisher.publish({source,1,2,1,vista::models::RoomMapState::building});
    std::this_thread::sleep_for(100ms);
    VISTA_CHECK(source->queries==1); // Heartbeat: no disk query or geometry resend.
    const auto start=std::chrono::steady_clock::now();
    publisher.publish({source,2,3,1,vista::models::RoomMapState::building});
    const auto metadata=client.read_frame();
    VISTA_CHECK(metadata.first==1);
    VISTA_CHECK(std::string(metadata.second.begin(),metadata.second.end()).find("\"revision\":2")!=std::string::npos);
    VISTA_CHECK(client.read_frame().first==2);
    VISTA_CHECK(std::chrono::steady_clock::now()-start<2s); // Not the 10 s camera cooldown.
    VISTA_CHECK(source->queries==2);
    stop.request_stop();bus.close();worker.join();VISTA_CHECK(failure.empty());
}
