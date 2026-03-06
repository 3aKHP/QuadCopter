#pragma once

#include "hal_imu.h"

namespace fc {
namespace core {

/// Attitude and Heading Reference System using complementary filter
/// Fuses gyroscope and accelerometer data to estimate roll, pitch, yaw
class AHRS {
public:
    /// Construct AHRS with complementary filter coefficient
    /// @param alpha Gyro trust factor (0.0 = pure accel, 1.0 = pure gyro, typical: 0.98)
    explicit AHRS(float alpha = 0.98f);

    /// Update attitude estimate with new IMU data
    /// @param imu IMU sensor readings
    /// @param dt Time step in seconds
    void update(const hal::IMUData& imu, float dt);

    /// Reset all estimated angles to zero
    void reset();

    // Estimated angles in radians
    float roll()  const { return roll_; }
    float pitch() const { return pitch_; }
    float yaw()   const { return yaw_; }

    // Current angular rates in rad/s (from gyro)
    float roll_rate()  const { return roll_rate_; }
    float pitch_rate() const { return pitch_rate_; }
    float yaw_rate()   const { return yaw_rate_; }

private:
    float alpha_;   ///< Complementary filter coefficient

    float roll_;    ///< Estimated roll angle (rad)
    float pitch_;   ///< Estimated pitch angle (rad)
    float yaw_;     ///< Estimated yaw angle (rad)

    float roll_rate_;   ///< Current roll rate (rad/s)
    float pitch_rate_;  ///< Current pitch rate (rad/s)
    float yaw_rate_;    ///< Current yaw rate (rad/s)

    bool initialized_;
};

}  // namespace core
}  // namespace fc
