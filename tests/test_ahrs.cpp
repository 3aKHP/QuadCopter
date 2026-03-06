#include <gtest/gtest.h>
#include "ahrs.h"
#include <cmath>

using namespace fc::core;
using namespace fc::hal;

class AHRSTest : public ::testing::Test {
protected:
    AHRS ahrs{0.98f};

    // Level hover: accel = [0, 0, +g] (sensor Z-up convention)
    IMUData level_imu() {
        IMUData data;
        data.gyro_x = 0; data.gyro_y = 0; data.gyro_z = 0;
        data.accel_x = 0; data.accel_y = 0; data.accel_z = 9.81f;
        return data;
    }

    // Tilted roll: gravity projects onto Y and Z axes
    // roll angle phi: ay = g*sin(phi), az = g*cos(phi)
    IMUData tilted_roll_imu(float roll_deg) {
        float roll_rad = roll_deg * static_cast<float>(M_PI) / 180.0f;
        IMUData data;
        data.gyro_x = 0; data.gyro_y = 0; data.gyro_z = 0;
        data.accel_x = 0;
        data.accel_y = 9.81f * std::sin(roll_rad);
        data.accel_z = 9.81f * std::cos(roll_rad);
        return data;
    }
};

TEST_F(AHRSTest, InitialState) {
    EXPECT_FLOAT_EQ(ahrs.roll(), 0.0f);
    EXPECT_FLOAT_EQ(ahrs.pitch(), 0.0f);
    EXPECT_FLOAT_EQ(ahrs.yaw(), 0.0f);
}

TEST_F(AHRSTest, LevelInitialization) {
    ahrs.update(level_imu(), 0.002f);
    EXPECT_NEAR(ahrs.roll(), 0.0f, 0.01f);
    EXPECT_NEAR(ahrs.pitch(), 0.0f, 0.01f);
}

TEST_F(AHRSTest, ConvergesToTilt) {
    auto imu = tilted_roll_imu(10.0f);
    for (int i = 0; i < 5000; i++) {
        ahrs.update(imu, 0.002f);
    }
    float expected_roll = 10.0f * static_cast<float>(M_PI) / 180.0f;
    EXPECT_NEAR(ahrs.roll(), expected_roll, 0.05f);
    EXPECT_NEAR(ahrs.pitch(), 0.0f, 0.01f);
}

TEST_F(AHRSTest, GyroIntegration) {
    ahrs.update(level_imu(), 0.002f);

    IMUData imu = level_imu();
    imu.gyro_z = 0.1f;  // 0.1 rad/s yaw rate

    for (int i = 0; i < 500; i++) {
        ahrs.update(imu, 0.002f);
    }
    EXPECT_NEAR(ahrs.yaw(), 0.1f, 0.02f);
}

TEST_F(AHRSTest, Reset) {
    auto imu = tilted_roll_imu(15.0f);
    for (int i = 0; i < 100; i++) {
        ahrs.update(imu, 0.002f);
    }
    EXPECT_NE(ahrs.roll(), 0.0f);

    ahrs.reset();
    EXPECT_FLOAT_EQ(ahrs.roll(), 0.0f);
    EXPECT_FLOAT_EQ(ahrs.pitch(), 0.0f);
    EXPECT_FLOAT_EQ(ahrs.yaw(), 0.0f);
}

TEST_F(AHRSTest, AngularRatesStored) {
    IMUData imu = level_imu();
    imu.gyro_x = 1.0f;
    imu.gyro_y = 2.0f;
    imu.gyro_z = 3.0f;

    ahrs.update(imu, 0.002f);

    EXPECT_FLOAT_EQ(ahrs.roll_rate(), 1.0f);
    EXPECT_FLOAT_EQ(ahrs.pitch_rate(), 2.0f);
    EXPECT_FLOAT_EQ(ahrs.yaw_rate(), 3.0f);
}
