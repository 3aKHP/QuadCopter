#pragma once

namespace fc {
namespace core {

/// Generic PID controller with anti-windup and derivative filtering
class PIDController {
public:
    /// Construct a PID controller
    /// @param kp Proportional gain
    /// @param ki Integral gain
    /// @param kd Derivative gain
    /// @param output_min Minimum output value
    /// @param output_max Maximum output value
    /// @param integral_max Maximum absolute integral accumulator value (anti-windup)
    PIDController(float kp, float ki, float kd,
                  float output_min, float output_max,
                  float integral_max);

    /// Compute PID output
    /// @param setpoint Desired value
    /// @param measurement Current measured value
    /// @param dt Time step in seconds
    /// @return Control output
    float compute(float setpoint, float measurement, float dt);

    /// Reset integrator and derivative state
    void reset();

    /// Update PID gains at runtime
    /// @param kp New proportional gain
    /// @param ki New integral gain
    /// @param kd New derivative gain
    void set_gains(float kp, float ki, float kd);

    // Accessors for testing
    float kp() const { return kp_; }
    float ki() const { return ki_; }
    float kd() const { return kd_; }
    float integral() const { return integral_; }

private:
    float kp_, ki_, kd_;
    float output_min_, output_max_;
    float integral_max_;
    float integral_;
    float prev_error_;
    bool first_run_;
};

}  // namespace core
}  // namespace fc
