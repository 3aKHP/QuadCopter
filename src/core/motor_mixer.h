#pragma once

#include <array>

namespace fc {
namespace core {

/// Motor mixer for X-configuration quadcopter
/// Maps throttle + attitude control outputs to individual motor commands
///
/// Motor layout (X mode, top view):
///       Front
///    M0      M1       M0: Front-Left  (CCW)
///      \    /         M1: Front-Right (CW)
///       \/            M2: Rear-Left   (CW)
///       /\            M3: Rear-Right  (CCW)
///      /    \
///    M2      M3
///
class MotorMixer {
public:
    /// Motor output array type (4 motors)
    using MotorOutputs = std::array<float, 4>;

    MotorMixer() = default;

    /// Compute motor outputs from throttle and attitude control signals
    /// @param throttle Base throttle [0.0, 1.0]
    /// @param roll  Roll control output (positive = right roll)
    /// @param pitch Pitch control output (positive = nose up)
    /// @param yaw   Yaw control output (positive = clockwise from top)
    /// @return Array of 4 motor throttle values, clamped to [0.0, 1.0]
    MotorOutputs mix(float throttle, float roll, float pitch, float yaw) const;

    /// Number of motors
    static constexpr int NUM_MOTORS = 4;
};

}  // namespace core
}  // namespace fc
