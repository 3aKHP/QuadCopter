#pragma once

#include <cstdint>

namespace fc {
namespace hal {

/// Abstract timer hardware interface
class HalTimer {
public:
    virtual ~HalTimer() = default;

    /// Get current time in microseconds
    /// @return Microsecond timestamp
    virtual uint64_t micros() = 0;

    /// Get current time in milliseconds
    /// @return Millisecond timestamp
    virtual uint64_t millis() = 0;

    /// Blocking delay in microseconds
    /// @param us Microseconds to delay
    virtual void delay_us(uint32_t us) = 0;

    /// Blocking delay in milliseconds
    /// @param ms Milliseconds to delay
    virtual void delay_ms(uint32_t ms) = 0;
};

}  // namespace hal
}  // namespace fc
