#pragma once

namespace fc {
namespace hal {

/// Barometer sensor data structure
struct BaroData {
    float pressure_pa   = 101325.0f;  ///< Atmospheric pressure (Pa)
    float temperature_c = 25.0f;      ///< Temperature (Celsius)
    float altitude_m    = 0.0f;       ///< Estimated altitude (m)
};

/// Abstract barometer hardware interface
class HalBarometer {
public:
    virtual ~HalBarometer() = default;

    /// Initialize the barometer sensor
    /// @return true on success
    virtual bool init() = 0;

    /// Read current barometer data
    /// @return BaroData with latest pressure, temperature, altitude
    virtual BaroData read() = 0;
};

}  // namespace hal
}  // namespace fc
