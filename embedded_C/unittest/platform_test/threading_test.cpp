#include "1_Platform/threading/threading.hpp"
#include "unittest/test.hpp"

VISTA_TEST(thread_priority_accepts_levels_one_through_five) {
    for (std::uint8_t level = 1; level <= 5; ++level) {
        VISTA_CHECK(vista::platform::ThreadPriority(level).level() == level);
    }
}

VISTA_TEST(thread_priority_rejects_out_of_range_values) {
    VISTA_CHECK_THROWS(vista::platform::ThreadPriority(0));
    VISTA_CHECK_THROWS(vista::platform::ThreadPriority(6));
}

VISTA_TEST(stop_token_shares_state) {
    vista::platform::StopToken first;
    auto second = first;
    VISTA_CHECK(!second.is_stop_requested());
    first.request_stop();
    VISTA_CHECK(second.is_stop_requested());
}
