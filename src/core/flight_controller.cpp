#include "flight_controller.h"
#include <sstream>
#include <iomanip>

namespace fc {
namespace core {

FlightController::FlightController(hal::HalIMU& imu, hal::HalBarometer& baro,
                                   hal::HalMotor& motor, hal::HalTimer& timer,
                                   hal::HalLogger& logger)
    : imu_(imu), baro_(baro), motor_(motor), timer_(timer), logger_(logger),
      ahrs_(0.98f),
      mode_(FlightMode::DISARMED),
      target_roll_(0.0f), target_pitch_(0.0f), target_yaw_rate_(0.0f),
      target_altitude_(0.0f), manual_throttle_(0.0f),
      last_update_us_(0), last_log_us_(0),
      prev_altitude_(0.0f), vertical_velocity_(0.0f) {}

bool FlightController::init() {
    logger_.log(hal::LogLevel::INFO, "Flight controller initializing...");

    if (!imu_.init()) {
        logger_.log(hal::LogLevel::ERROR, "IMU init failed");
        return false;
    }
    if (!baro_.init()) {
        logger_.log(hal::LogLevel::ERROR, "Barometer init failed");
        return false;
    }
    if (!motor_.init()) {
        logger_.log(hal::LogLevel::ERROR, "Motor init failed");
        return false;
    }

    last_update_us_ = timer_.micros();
    last_log_us_ = last_update_us_;

    logger_.log(hal::LogLevel::INFO, "Flight controller initialized OK");
    return true;
}

void FlightController::update() {
    // Calculate dt
    uint64_t now_us = timer_.micros();
    float dt = static_cast<float>(now_us - last_update_us_) * 1e-6f;
    last_update_us_ = now_us;

    // Guard against unreasonable dt
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.002f;  // Default to 500 Hz
    }

    // 1. Read sensors
    hal::IMUData imu_data = imu_.read();
    hal::BaroData baro_data = baro_.read();

    // 2. Attitude estimation
    ahrs_.update(imu_data, dt);

    // 3. Estimate vertical velocity from barometer
    vertical_velocity_ = (baro_data.altitude_m - prev_altitude_) / dt;
    prev_altitude_ = baro_data.altitude_m;

    if (mode_ == FlightMode::DISARMED) {
        // Motors off
        for (int i = 0; i < MotorMixer::NUM_MOTORS; i++) {
            motor_.set_output(i, 0.0f);
        }
        attitude_ctrl_.reset();
        altitude_ctrl_.reset();
        return;
    }

    // 4. Attitude control
    AttitudeOutput att_out = attitude_ctrl_.compute(
        target_roll_, target_pitch_, target_yaw_rate_,
        ahrs_.roll(), ahrs_.pitch(),
        ahrs_.roll_rate(), ahrs_.pitch_rate(), ahrs_.yaw_rate(),
        dt
    );

    // 5. Throttle calculation
    float throttle = 0.0f;
    if (mode_ == FlightMode::ALT_HOLD) {
        throttle = altitude_ctrl_.compute(
            target_altitude_, baro_data.altitude_m,
            vertical_velocity_, dt
        );
    } else {
        // STABILIZE mode: manual throttle
        throttle = manual_throttle_;
    }

    // 6. Motor mixing
    auto motor_outputs = mixer_.mix(throttle, att_out.roll, att_out.pitch, att_out.yaw);

    // 7. Output to motors
    for (int i = 0; i < MotorMixer::NUM_MOTORS; i++) {
        motor_.set_output(i, motor_outputs[i]);
    }

    // 8. Data logging (at reduced rate)
    if (now_us - last_log_us_ >= LOG_INTERVAL_US) {
        last_log_us_ = now_us;

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4)
            << ahrs_.roll() << ","
            << ahrs_.pitch() << ","
            << ahrs_.yaw() << ","
            << baro_data.altitude_m << ","
            << vertical_velocity_ << ","
            << motor_outputs[0] << ","
            << motor_outputs[1] << ","
            << motor_outputs[2] << ","
            << motor_outputs[3];

        logger_.log_data(timer_.millis(), oss.str());
    }
}

void FlightController::arm() {
    if (mode_ == FlightMode::DISARMED) {
        logger_.log(hal::LogLevel::INFO, "Arming...");
        motor_.arm();
        ahrs_.reset();
        attitude_ctrl_.reset();
        altitude_ctrl_.reset();
        prev_altitude_ = baro_.read().altitude_m;
        vertical_velocity_ = 0.0f;
        target_altitude_ = prev_altitude_;
        mode_ = FlightMode::STABILIZE;
        logger_.log(hal::LogLevel::INFO, "Armed - STABILIZE mode");
    }
}

void FlightController::disarm() {
    logger_.log(hal::LogLevel::INFO, "Disarming...");
    mode_ = FlightMode::DISARMED;
    motor_.disarm();
    for (int i = 0; i < MotorMixer::NUM_MOTORS; i++) {
        motor_.set_output(i, 0.0f);
    }
    logger_.log(hal::LogLevel::INFO, "Disarmed");
}

void FlightController::set_attitude_target(float roll, float pitch, float yaw_rate) {
    target_roll_ = roll;
    target_pitch_ = pitch;
    target_yaw_rate_ = yaw_rate;
}

void FlightController::set_altitude_target(float altitude) {
    target_altitude_ = altitude;
}

void FlightController::set_throttle(float throttle) {
    manual_throttle_ = throttle;
}

}  // namespace core
}  // namespace fc
