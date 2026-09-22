#include "2_Transport/messaging/websocket_server.hpp"
#include "unittest/test.hpp"

VISTA_TEST(websocket_server_builds_rfc6455_accept_key) {
    VISTA_CHECK(
        vista::transport::make_websocket_accept_key(
            "dGhlIHNhbXBsZSBub25jZQ==") ==
        "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");
}
