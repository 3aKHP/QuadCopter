#include "sitl_motor.h"
#include <algorithm>

namespace fc {
namespace sitl {

SitlMotor::SitlMotor(PhysicsModel& model)
    : model_(model), armed_(false), outputs_({0, 0, 0, 0}) {}

bool SitlMotor::init() {
    outputs_.fill(0.0f);
    armed_ = false;
    return true;
}

void SitlMotor::set_output(int motor_id, float throttle) {
    if (motor_id >= 0 && motor_id < 4) {
        if (armed_) {
            outputs_[motor_id] = std::clamp(throttle, 0.0f, 1.0f);
        } else {
            outputs_[motor_id] = 0.0f;
        }
    }
}

void SitlMotor::arm() {
    armed_ = true;
}

void SitlMotor::disarm() {
    armed_ = false;
    outputs_.fill(0.0f);
}

bool SitlMotor::is_armed() const {
    return armed_;
}

}  // namespace sitl
}  // namespace fc
