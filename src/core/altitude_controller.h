#pragma once

#include "pid_controller.h"

namespace fc {
namespace core {

/// Altitude hold controller using cascaded PID
/// Outer loop: altitude error → desired vertical velocity
/// Inner loop: velocity error → throttle adjustment
class AltitudeController {
public:
    AltitudeController();

    /// Compute throttle output for altitude hold
    /// @param target_alt Target altitude (m)
    /// @param current_alt Current measured altitude (m)
    /// @param vertical_velocity Current vertical velocity (m/s, positive = up)
    /// @param dt Time step in seconds
    /// @return Throttle output [0.0, 1.0]
    float compute(float target_alt, float current_alt,
                  float vertical_velocity, float dt);

    /// Reset all PID controllers
    void reset();

    /// Set hover throttle base value
    /// @param throttle Hover throttle [0.0, 1.0]
    void set_hover_throttle(float throttle) { hover_throttle_ = throttle; }

    // Access controllers for tuning
    PIDController& alt_pid() { return alt_pid_; }
    PIDController& vel_pid() { return vel_pid_; }

private:
    PIDController alt_pid_;     ///< Altitude → desired velocity
    PIDController vel_pid_;     ///< Velocity → throttle delta
    float hover_throttle_;      ///< Base hover throttle
};

}  // namespace core
}  // namespace fc
