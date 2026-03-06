#include "ahrs.h"
#include <cmath>

namespace fc {
namespace core {

AHRS::AHRS(float alpha)
    : alpha_(alpha),
      roll_(0.0f), pitch_(0.0f), yaw_(0.0f),
      roll_rate_(0.0f), pitch_rate_(0.0f), yaw_rate_(0.0f),
      initialized_(false) {}

void AHRS::update(const hal::IMUData& imu, float dt) {
    if (dt <= 0.0f) return;

    // Store angular rates
    roll_rate_  = imu.gyro_x;
    pitch_rate_ = imu.gyro_y;
    yaw_rate_   = imu.gyro_z;

    // Compute attitude from accelerometer
    // Convention: level at rest → accel = [0, 0, +g]
    //   roll  = atan2(ay, az)                          → atan2(0, g) = 0 when level
    //   pitch = atan2(-ax, sqrt(ay² + az²))            → atan2(0, g) = 0 when level
    float accel_roll = std::atan2(imu.accel_y, imu.accel_z);
    float accel_pitch = std::atan2(-imu.accel_x,
        std::sqrt(imu.accel_y * imu.accel_y + imu.accel_z * imu.accel_z));

    if (!initialized_) {
        roll_  = accel_roll;
        pitch_ = accel_pitch;
        yaw_   = 0.0f;
        initialized_ = true;
        return;
    }

    // Complementary filter
    roll_  = alpha_ * (roll_  + imu.gyro_x * dt) + (1.0f - alpha_) * accel_roll;
    pitch_ = alpha_ * (pitch_ + imu.gyro_y * dt) + (1.0f - alpha_) * accel_pitch;

    // Yaw: gyro integration only (no magnetometer)
    yaw_ += imu.gyro_z * dt;

    // Normalize yaw to [-PI, PI]
    while (yaw_ > M_PI)  yaw_ -= 2.0f * static_cast<float>(M_PI);
    while (yaw_ < -M_PI) yaw_ += 2.0f * static_cast<float>(M_PI);
}

void AHRS::reset() {
    roll_ = pitch_ = yaw_ = 0.0f;
    roll_rate_ = pitch_rate_ = yaw_rate_ = 0.0f;
    initialized_ = false;
}

}  // namespace core
}  // namespace fc
