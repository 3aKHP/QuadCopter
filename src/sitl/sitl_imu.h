#pragma once

#include "hal_imu.h"
#include "physics_model.h"
#include <random>

namespace fc {
namespace sitl {

/// SITL IMU implementation - reads from physics model with simulated noise
class SitlIMU : public hal::HalIMU {
public:
    /// @param model Reference to physics model
    /// @param gyro_noise_std Gyroscope noise standard deviation (rad/s)
    /// @param accel_noise_std Accelerometer noise standard deviation (m/s²)
    SitlIMU(PhysicsModel& model,
            float gyro_noise_std = 0.01f,
            float accel_noise_std = 0.1f);

    bool init() override;
    hal::IMUData read() override;

private:
    PhysicsModel& model_;
    float gyro_noise_std_;
    float accel_noise_std_;
    std::mt19937 rng_;
    std::normal_distribution<float> gyro_noise_;
    std::normal_distribution<float> accel_noise_;
};

}  // namespace sitl
}  // namespace fc
