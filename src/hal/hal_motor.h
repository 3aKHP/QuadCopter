#pragma once

namespace fc {
namespace hal {

/// Abstract motor output hardware interface
class HalMotor {
public:
    virtual ~HalMotor() = default;

    /// Initialize motor outputs
    /// @return true on success
    virtual bool init() = 0;

    /// Set motor output throttle
    /// @param motor_id Motor index (0-3 for quadcopter)
    /// @param throttle Throttle value in range [0.0, 1.0]
    virtual void set_output(int motor_id, float throttle) = 0;

    /// Arm motors (enable output)
    virtual void arm() = 0;

    /// Disarm motors (disable output, set to zero)
    virtual void disarm() = 0;

    /// Check if motors are armed
    /// @return true if armed
    virtual bool is_armed() const = 0;
};

}  // namespace hal
}  // namespace fc
