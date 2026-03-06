#include "sitl_timer.h"

namespace fc {
namespace sitl {

SitlTimer::SitlTimer() : time_us_(0) {}

uint64_t SitlTimer::micros() {
    return time_us_;
}

uint64_t SitlTimer::millis() {
    return time_us_ / 1000;
}

void SitlTimer::delay_us(uint32_t us) {
    time_us_ += us;
}

void SitlTimer::delay_ms(uint32_t ms) {
    time_us_ += static_cast<uint64_t>(ms) * 1000;
}

void SitlTimer::advance(uint64_t us) {
    time_us_ += us;
}

void SitlTimer::reset() {
    time_us_ = 0;
}

}  // namespace sitl
}  // namespace fc
