#include <gtest/gtest.h>
#include "motor_mixer.h"
#include <cmath>

using namespace fc::core;

class MotorMixerTest : public ::testing::Test {
protected:
    MotorMixer mixer;
};

TEST_F(MotorMixerTest, HoverSymmetric) {
    // Pure throttle, no attitude control
    auto out = mixer.mix(0.5f, 0.0f, 0.0f, 0.0f);

    // All motors should be equal
    EXPECT_FLOAT_EQ(out[0], 0.5f);
    EXPECT_FLOAT_EQ(out[1], 0.5f);
    EXPECT_FLOAT_EQ(out[2], 0.5f);
    EXPECT_FLOAT_EQ(out[3], 0.5f);
}

TEST_F(MotorMixerTest, RollRight) {
    // Positive roll → left motors up, right motors down
    auto out = mixer.mix(0.5f, 0.1f, 0.0f, 0.0f);

    // M0 (front-left) and M2 (rear-left) should increase
    EXPECT_GT(out[0], 0.5f);
    EXPECT_GT(out[2], 0.5f);

    // M1 (front-right) and M3 (rear-right) should decrease
    EXPECT_LT(out[1], 0.5f);
    EXPECT_LT(out[3], 0.5f);
}

TEST_F(MotorMixerTest, PitchForward) {
    // Negative pitch (nose down) → rear motors up, front motors down
    auto out = mixer.mix(0.5f, 0.0f, -0.1f, 0.0f);

    // M0 and M1 (front) should increase (pitch term is subtracted)
    EXPECT_GT(out[0], 0.5f);
    EXPECT_GT(out[1], 0.5f);

    // M2 and M3 (rear) should decrease
    EXPECT_LT(out[2], 0.5f);
    EXPECT_LT(out[3], 0.5f);
}

TEST_F(MotorMixerTest, YawClockwise) {
    // Positive yaw → increase CW motors (M1, M2), decrease CCW (M0, M3)
    auto out = mixer.mix(0.5f, 0.0f, 0.0f, 0.1f);

    EXPECT_GT(out[1], 0.5f);  // CW motor
    EXPECT_GT(out[2], 0.5f);  // CW motor
    EXPECT_LT(out[0], 0.5f);  // CCW motor
    EXPECT_LT(out[3], 0.5f);  // CCW motor
}

TEST_F(MotorMixerTest, OutputClampMin) {
    // Very low throttle with large control should clamp to 0
    auto out = mixer.mix(0.0f, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 4; i++) {
        EXPECT_GE(out[i], 0.0f);
    }
}

TEST_F(MotorMixerTest, OutputClampMax) {
    // Full throttle + max control should clamp to 1.0
    auto out = mixer.mix(1.0f, 0.5f, 0.5f, 0.5f);
    for (int i = 0; i < 4; i++) {
        EXPECT_LE(out[i], 1.0f);
    }
}

TEST_F(MotorMixerTest, MotorDifferencesCorrect) {
    float roll = 0.1f;
    auto out = mixer.mix(0.5f, roll, 0.0f, 0.0f);

    // Left-right difference should be 2*roll
    EXPECT_NEAR(out[0] - out[1], 2.0f * roll, 0.001f);
    EXPECT_NEAR(out[2] - out[3], 2.0f * roll, 0.001f);
}
