#include "2_Transport/peripheral/librealsense_usb/librealsense_usb.hpp"
#include "unittest/test.hpp"

VISTA_TEST(librealsense_reports_expected_runtime_api) {
#if VISTA_HAS_REALSENSE
    VISTA_CHECK(vista::transport::LibrealsenseUsbConnection::runtime_api_version() ==
                25000);
#else
    VISTA_CHECK(vista::transport::LibrealsenseUsbConnection::runtime_api_version() == 0);
#endif
}
