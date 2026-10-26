#include "4_Applications/perception/foreground/foreground.hpp"

#include <iostream>
#include <sstream>
#include "2_Transport/messaging/websocket_server.hpp"
#include "3_Devices/lidars/lidar.hpp"
#include "models/topics.hpp"

namespace vista::application {
namespace {
std::string json_string(const std::string& value) {
    std::string result="\"";
    for(const auto c:value) {
        if(c=='\\' || c=='\"') {result+='\\';result+=c;}
        else if(c=='\n') result+="\\n";
        else if(c=='\r') result+="\\r";
        else if(c=='\t') result+="\\t";
        else if(static_cast<unsigned char>(c)<32) result+=' ';
        else result+=c;
    }
    return result+'\"';
}
}
platform::WorkerHandle spawn_foreground_websocket(platform::MessageBus& bus,
    platform::ThreadConfig thread,platform::StopToken stop,PointCloudWebSocketConfig config,
    PointCloudWebSocketCompletion complete) {
    validate_pointcloud_websocket_config(config);
    auto input=bus.subscribe<models::ForegroundFrame>(models::topics::foreground_points,thread.name);
    return platform::spawn_worker(std::move(thread),[input=std::move(input),config,stop,complete=std::move(complete)]() mutable {
        try {
            PointCloudWebSocketReport report;
            std::unique_ptr<transport::WebSocketServer> server;
            std::shared_ptr<const models::ForegroundFrame> latest;
            auto arrival=std::chrono::steady_clock::now();
            auto next_listen=arrival,next_publish=arrival;
            bool dirty=true,last_fresh=false;
            while(!stop.is_stop_requested()) {
                std::shared_ptr<const models::ForegroundFrame> frame;
                auto received=input.receive_for(frame,std::chrono::milliseconds(20));
                if(received==platform::ReceiveStatus::closed) break;
                while(received==platform::ReceiveStatus::message) {
                    latest=frame;++report.received_messages;arrival=std::chrono::steady_clock::now();dirty=true;
                    received=input.try_receive(frame);
                }
                const auto now=std::chrono::steady_clock::now();
                if(!server && now>=next_listen) {
                    try {
                        // A slow browser may delay ONLY this optional socket worker.
                        auto listener=transport::WebSocketServer::listen({config.bind_address,config.port,
                            config.maximum_clients,std::chrono::milliseconds(250),std::chrono::milliseconds(25)});
                        server=std::make_unique<transport::WebSocketServer>(std::move(listener));
                        std::cout<<"Foreground WebSocket listening at ws://"<<config.bind_address<<':'<<config.port<<'\n';
                    } catch(const std::exception& error) {
                        ++report.server_failures;next_listen=now+config.retry_interval;
                        std::cerr<<"Foreground socket retry: "<<error.what()<<'\n';
                    }
                }
                if(!server) continue;
                try {
                    if(server->poll_accept()) dirty=true;
                    (void)server->poll_text();
                    const auto host_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
                    const bool fresh=latest && latest->received_monotonic_ns && host_ns>=latest->received_monotonic_ns &&
                        host_ns-latest->received_monotonic_ns<500'000'000ULL && now-arrival<std::chrono::milliseconds(500);
                    // The query worker already owns the 250 ms cadence. Deliver
                    // its new result now, not after a second independent timer.
                    // Only inactive heartbeats and late connections repeat data.
                    if(!dirty && fresh==last_fresh && (fresh || now<next_publish)) continue;
                    dirty=false;last_fresh=fresh;next_publish=now+config.publish_interval;
                    if(!server->client_count()) continue;
                    models::PointCloudFrame cloud;
                    if(fresh) {
                        cloud.timestamp_ns=latest->timestamp_ns;
                        cloud.points.reserve(latest->points.size());
                        for(const auto& p:latest->points) cloud.points.push_back({p.x,p.y,p.z,p.intensity,0,0,0});
                    }
                    // Backward compatible: LPC1 stays unchanged. Old plugins
                    // ignore this text frame; new ones can explain empty output.
                    const auto reason=!latest?"waiting for foreground worker":
                        fresh || !latest->received_monotonic_ns?latest->reason:
                        "no fresh foreground scan (older than 500 ms)";
                    const bool active=fresh && latest->reason.rfind("foreground;",0)==0;
                    std::ostringstream metadata;
                    metadata<<"{\"type\":\"foreground-status\",\"version\":1,\"active\":"<<(active?"true":"false")
                        <<",\"reason\":"<<json_string(reason)
                        <<",\"inputPoints\":"<<(fresh?latest->input_points:0)
                        <<",\"backgroundPoints\":"<<(fresh?latest->background_points:0)
                        <<",\"unknownPoints\":"<<(fresh?latest->unknown_points:0)
                        <<",\"foregroundPoints\":"<<cloud.points.size()<<'}';
                    for(const auto client:server->client_ids()) server->send_text(client,metadata.str());
                    // Empty packets replace the scene too, including on late connect.
                    devices::LidarPointCloudMessage message(fresh?latest->source:"foreground",fresh?latest->sequence:0,
                        std::nullopt,0,std::move(cloud));
                    const auto packet=encode_lpc1_pointcloud(message,config.maximum_points);
                    const auto sent=server->broadcast_binary(packet.data(),packet.size());
                    report.client_deliveries+=sent;
                    if(sent) { ++report.broadcast_frames;report.encoded_points+=(packet.size()-32)/16; }
                } catch(const std::exception&) {
                    ++report.server_failures;server.reset();next_listen=now+config.retry_interval;
                }
            }
            report.dropped_input_messages=input.dropped_messages();complete(report,{});
        } catch(const std::exception& error) { complete(std::nullopt,error.what()); }
    });
}
} // namespace vista::application
