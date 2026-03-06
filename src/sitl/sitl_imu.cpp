#include "sitl_imu.h"

namespace fc {
namespace sitl {

SitlIMU::SitlIMU(PhysicsModel& model, float gyro_noise_std, float accel_noise_std)
    : model_(model),
      gyro_noise_std_(gyro_noise_std),
      accel_noise_std_(accel_noise_std),
      rng_(42),  // Fixed seed for reproducibility
      gyro_noise_(0.0f, gyro_noise_std),
      accel_noise_(0.0f, accel_noise_std) {}

bool SitlIMU::init() {
    return true;
}

hal::IMUData SitlIMU::read() {
    hal::IMUData data;
    const auto& state = model_.state();
    const auto& accel = model_.body_acceleration();

    // Gyroscope: true angular rates + noise
    data.gyro_x = state.p + gyro_noise_(rng_);
    data.gyro_y = state.q + gyro_noise_(rng_);
    data.gyro_z = state.r + gyro_noise_(rng_);

    // Accelerometer: body-frame specific force + noise
    data.accel_x = accel.ax + accel_noise_(rng_);
    data.accel_y = accel.ay + accel_noise_(rng_);
    data.accel_z = accel.az + accel_noise_(rng_);

    return data;
}

}  // namespace sitl
}  // namespace fc
