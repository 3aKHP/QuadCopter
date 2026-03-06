#pragma once

#include "hal_timer.h"
#include <cstdint>

namespace fc {
namespace sitl {

/// SITL Timer implementation - simulated clock (not real-time wall clock)
/// Advances only when explicitly ticked
class SitlTimer : public hal::HalTimer {
public:
    SitlTimer();

    uint64_t micros() override;
    uint64_t millis() override;
    void delay_us(uint32_t us) override;
    void delay_ms(uint32_t ms) override;

    /// Advance simulated time by given microseconds
    void advance(uint64_t us);

    /// Reset simulated time to zero
    void reset();

private:
    uint64_t time_us_;
};

}  // namespace sitl
}  // namespace fc
