#include <chrono>

#include "2_Transport/peripheral/ethernet/ethernet.hpp"
#include "unittest/test.hpp"

VISTA_TEST(ethernet_reports_unresolvable_host) {
    VISTA_CHECK_THROWS(vista::transport::EthernetConnection::connect(
        "invalid host name !",
        4141,
        std::chrono::milliseconds(50),
        std::chrono::milliseconds(50)));
}
