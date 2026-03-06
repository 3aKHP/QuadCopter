#pragma once

#include "pid_controller.h"

namespace fc {
namespace core {

/// Attitude control output for roll, pitch, yaw axes
struct AttitudeOutput {
    float roll  = 0.0f;   ///< Roll control output
    float pitch = 0.0f;   ///< Pitch control output
    float yaw   = 0.0f;   ///< Yaw control output
};

/// Cascaded PID attitude controller
/// Outer loop: angle error → desired rate
/// Inner loop: rate error → control output (torque command)
class AttitudeController {
public:
    AttitudeController();

    /// Compute attitude control outputs
    /// @param target_roll  Desired roll angle (rad)
    /// @param target_pitch Desired pitch angle (rad)
    /// @param target_yaw_rate Desired yaw rate (rad/s)
    /// @param current_roll  Current roll angle (rad)
    /// @param current_pitch Current pitch angle (rad)
    /// @param gyro_x Current roll rate (rad/s)
    /// @param gyro_y Current pitch rate (rad/s)
    /// @param gyro_z Current yaw rate (rad/s)
    /// @param dt Time step in seconds
    /// @return AttitudeOutput with roll, pitch, yaw control signals
    AttitudeOutput compute(float target_roll, float target_pitch,
                           float target_yaw_rate,
                           float current_roll, float current_pitch,
                           float gyro_x, float gyro_y, float gyro_z,
                           float dt);

    /// Reset all PID controllers
    void reset();

    // Access individual controllers for gain tuning
    PIDController& roll_angle_pid()  { return roll_angle_pid_; }
    PIDController& roll_rate_pid()   { return roll_rate_pid_; }
    PIDController& pitch_angle_pid() { return pitch_angle_pid_; }
    PIDController& pitch_rate_pid()  { return pitch_rate_pid_; }
    PIDController& yaw_rate_pid()    { return yaw_rate_pid_; }

private:
    // Outer loop: angle PID (P-only typical)
    PIDController roll_angle_pid_;
    PIDController pitch_angle_pid_;

    // Inner loop: rate PID
    PIDController roll_rate_pid_;
    PIDController pitch_rate_pid_;
    PIDController yaw_rate_pid_;
};

}  // namespace core
}  // namespace fc
