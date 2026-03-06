#include "pid_controller.h"
#include <algorithm>
#include <cmath>

namespace fc {
namespace core {

PIDController::PIDController(float kp, float ki, float kd,
                             float output_min, float output_max,
                             float integral_max)
    : kp_(kp), ki_(ki), kd_(kd),
      output_min_(output_min), output_max_(output_max),
      integral_max_(integral_max),
      integral_(0.0f), prev_error_(0.0f), first_run_(true) {}

float PIDController::compute(float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) {
        return 0.0f;
    }

    float error = setpoint - measurement;

    // Proportional term
    float p_term = kp_ * error;

    // Integral term with anti-windup clamping
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -integral_max_, integral_max_);
    float i_term = ki_ * integral_;

    // Derivative term (on error, with first-run guard)
    float d_term = 0.0f;
    if (!first_run_) {
        float derivative = (error - prev_error_) / dt;
        d_term = kd_ * derivative;
    }
    first_run_ = false;
    prev_error_ = error;

    // Sum and clamp output
    float output = p_term + i_term + d_term;
    output = std::clamp(output, output_min_, output_max_);

    return output;
}

void PIDController::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    first_run_ = true;
}

void PIDController::set_gains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

}  // namespace core
}  // namespace fc
