#include "motor_mixer.h"
#include <algorithm>

namespace fc {
namespace core {

MotorMixer::MotorOutputs MotorMixer::mix(float throttle, float roll,
                                          float pitch, float yaw) const {
    MotorOutputs outputs;

    // X-configuration mixing matrix:
    // M0 (Front-Left,  CCW): throttle + roll - pitch - yaw
    // M1 (Front-Right, CW):  throttle - roll - pitch + yaw
    // M2 (Rear-Left,   CW):  throttle + roll + pitch + yaw
    // M3 (Rear-Right,  CCW): throttle - roll + pitch - yaw
    outputs[0] = throttle + roll - pitch - yaw;
    outputs[1] = throttle - roll - pitch + yaw;
    outputs[2] = throttle + roll + pitch + yaw;
    outputs[3] = throttle - roll + pitch - yaw;

    // Clamp to valid range [0.0, 1.0]
    for (auto& out : outputs) {
        out = std::clamp(out, 0.0f, 1.0f);
    }

    return outputs;
}

}  // namespace core
}  // namespace fc
