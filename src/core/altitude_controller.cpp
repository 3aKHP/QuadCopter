#include "altitude_controller.h"
#include <algorithm>

namespace fc {
namespace core {

AltitudeController::AltitudeController()
    // Outer: altitude P → desired velocity (m/s)
    : alt_pid_(1.0f, 0.0f, 0.0f, -2.0f, 2.0f, 0.0f),
      // Inner: velocity PID → throttle delta
      vel_pid_(0.5f, 0.1f, 0.05f, -0.3f, 0.3f, 0.2f),
      hover_throttle_(0.5f) {}

float AltitudeController::compute(float target_alt, float current_alt,
                                   float vertical_velocity, float dt) {
    // Outer loop: altitude error → desired vertical velocity
    float desired_vel = alt_pid_.compute(target_alt, current_alt, dt);

    // Inner loop: velocity error → throttle delta
    float throttle_delta = vel_pid_.compute(desired_vel, vertical_velocity, dt);

    // Add to hover base throttle
    float throttle = hover_throttle_ + throttle_delta;

    // Clamp to valid range
    throttle = std::clamp(throttle, 0.0f, 1.0f);

    return throttle;
}

void AltitudeController::reset() {
    alt_pid_.reset();
    vel_pid_.reset();
}

}  // namespace core
}  // namespace fc
