#pragma once

#include <cstdint>

namespace fc {
namespace hal {

/// IMU sensor data structure
struct IMUData {
    float gyro_x  = 0.0f;  ///< Angular velocity around X axis (rad/s)
    float gyro_y  = 0.0f;  ///< Angular velocity around Y axis (rad/s)
    float gyro_z  = 0.0f;  ///< Angular velocity around Z axis (rad/s)
    float accel_x = 0.0f;  ///< Linear acceleration along X axis (m/s^2)
    float accel_y = 0.0f;  ///< Linear acceleration along Y axis (m/s^2)
    float accel_z = 0.0f;  ///< Linear acceleration along Z axis (m/s^2)
};

/// Abstract IMU hardware interface
class HalIMU {
public:
    virtual ~HalIMU() = default;

    /// Initialize the IMU sensor
    /// @return true on success
    virtual bool init() = 0;

    /// Read current IMU data
    /// @return IMUData with latest gyro and accel readings
    virtual IMUData read() = 0;
};

}  // namespace hal
}  // namespace fc
