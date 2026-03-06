#pragma once

#include "hal.h"
#include "ahrs.h"
#include "attitude_controller.h"
#include "altitude_controller.h"
#include "motor_mixer.h"

namespace fc {
namespace core {

/// Flight mode enumeration
enum class FlightMode {
    DISARMED,       ///< Motors off
    STABILIZE,      ///< Attitude stabilization only (manual throttle)
    ALT_HOLD        ///< Attitude stabilization + altitude hold
};

/// Flight controller main class
/// Orchestrates sensor reading, estimation, control, and motor output
class FlightController {
public:
    /// Construct flight controller with HAL implementations
    /// @param imu   IMU hardware interface
    /// @param baro  Barometer hardware interface
    /// @param motor Motor output hardware interface
    /// @param timer Timer hardware interface
    /// @param logger Logger interface
    FlightController(hal::HalIMU& imu, hal::HalBarometer& baro,
                     hal::HalMotor& motor, hal::HalTimer& timer,
                     hal::HalLogger& logger);

    /// Initialize all hardware and controllers
    /// @return true on success
    bool init();

    /// Execute one iteration of the control loop
    /// Should be called at the main loop frequency (e.g., 500 Hz)
    void update();

    /// Arm the flight controller
    void arm();

    /// Disarm the flight controller
    void disarm();

    /// Set flight mode
    void set_mode(FlightMode mode) { mode_ = mode; }
    FlightMode mode() const { return mode_; }

    /// Set attitude targets
    /// @param roll  Target roll angle (rad)
    /// @param pitch Target pitch angle (rad)
    /// @param yaw_rate Target yaw rate (rad/s)
    void set_attitude_target(float roll, float pitch, float yaw_rate);

    /// Set altitude target (for ALT_HOLD mode)
    /// @param altitude Target altitude (m)
    void set_altitude_target(float altitude);

    /// Set manual throttle (for STABILIZE mode)
    /// @param throttle Throttle value [0.0, 1.0]
    void set_throttle(float throttle);

    // Accessors for state inspection
    const AHRS& ahrs() const { return ahrs_; }
    AttitudeController& attitude_ctrl() { return attitude_ctrl_; }
    AltitudeController& altitude_ctrl() { return altitude_ctrl_; }

private:
    // HAL references
    hal::HalIMU& imu_;
    hal::HalBarometer& baro_;
    hal::HalMotor& motor_;
    hal::HalTimer& timer_;
    hal::HalLogger& logger_;

    // Core algorithms
    AHRS ahrs_;
    AttitudeController attitude_ctrl_;
    AltitudeController altitude_ctrl_;
    MotorMixer mixer_;

    // State
    FlightMode mode_;
    float target_roll_;
    float target_pitch_;
    float target_yaw_rate_;
    float target_altitude_;
    float manual_throttle_;

    // Timing
    uint64_t last_update_us_;
    uint64_t last_log_us_;

    // Altitude estimation
    float prev_altitude_;
    float vertical_velocity_;

    static constexpr uint64_t LOG_INTERVAL_US = 20000;  // 50 Hz logging
};

}  // namespace core
}  // namespace fc
