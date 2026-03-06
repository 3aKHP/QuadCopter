#include "attitude_controller.h"

namespace fc {
namespace core {

AttitudeController::AttitudeController()
    // Outer loop: Angle P controllers
    //   output is desired angular rate in rad/s
    : roll_angle_pid_ (4.5f, 0.0f, 0.0f, -8.0f, 8.0f, 0.0f),
      pitch_angle_pid_(4.5f, 0.0f, 0.0f, -8.0f, 8.0f, 0.0f),
      // Inner loop: Rate PID controllers
      //   output is control torque (normalized)
      roll_rate_pid_ (0.15f, 0.05f, 0.002f, -0.5f, 0.5f, 0.3f),
      pitch_rate_pid_(0.15f, 0.05f, 0.002f, -0.5f, 0.5f, 0.3f),
      yaw_rate_pid_  (0.30f, 0.10f, 0.000f, -0.5f, 0.5f, 0.3f) {}

AttitudeOutput AttitudeController::compute(
    float target_roll, float target_pitch, float target_yaw_rate,
    float current_roll, float current_pitch,
    float gyro_x, float gyro_y, float gyro_z,
    float dt)
{
    AttitudeOutput out;

    // Outer loop: angle error → desired angular rate
    float desired_roll_rate  = roll_angle_pid_.compute(target_roll, current_roll, dt);
    float desired_pitch_rate = pitch_angle_pid_.compute(target_pitch, current_pitch, dt);

    // Inner loop: rate error → control output
    out.roll  = roll_rate_pid_.compute(desired_roll_rate, gyro_x, dt);
    out.pitch = pitch_rate_pid_.compute(desired_pitch_rate, gyro_y, dt);
    out.yaw   = yaw_rate_pid_.compute(target_yaw_rate, gyro_z, dt);

    return out;
}

void AttitudeController::reset() {
    roll_angle_pid_.reset();
    pitch_angle_pid_.reset();
    roll_rate_pid_.reset();
    pitch_rate_pid_.reset();
    yaw_rate_pid_.reset();
}

}  // namespace core
}  // namespace fc
