#include <gtest/gtest.h>
#include "altitude_controller.h"
#include <cmath>

using namespace fc::core;

class AltitudeCtrlTest : public ::testing::Test {
protected:
    AltitudeController ctrl;
};

TEST_F(AltitudeCtrlTest, HoverThrottleAtTarget) {
    // When at target altitude with zero velocity, output should be near hover throttle
    float throttle = ctrl.compute(2.0f, 2.0f, 0.0f, 0.01f);
    EXPECT_NEAR(throttle, 0.5f, 0.05f);
}

TEST_F(AltitudeCtrlTest, IncreaseThrottleBelowTarget) {
    // Below target altitude → should increase throttle above hover
    float throttle = ctrl.compute(5.0f, 2.0f, 0.0f, 0.01f);
    EXPECT_GT(throttle, 0.5f);
}

TEST_F(AltitudeCtrlTest, DecreaseThrottleAboveTarget) {
    // Above target altitude → should decrease throttle below hover
    float throttle = ctrl.compute(2.0f, 5.0f, 0.0f, 0.01f);
    EXPECT_LT(throttle, 0.5f);
}

TEST_F(AltitudeCtrlTest, OutputClamped) {
    // Extreme error should not exceed [0, 1]
    float throttle_high = ctrl.compute(100.0f, 0.0f, 0.0f, 0.01f);
    EXPECT_LE(throttle_high, 1.0f);

    AltitudeController ctrl2;
    float throttle_low = ctrl2.compute(0.0f, 100.0f, 0.0f, 0.01f);
    EXPECT_GE(throttle_low, 0.0f);
}

TEST_F(AltitudeCtrlTest, ResetClearsState) {
    ctrl.compute(5.0f, 0.0f, 0.0f, 0.01f);
    ctrl.compute(5.0f, 0.0f, 0.0f, 0.01f);

    ctrl.reset();
    EXPECT_FLOAT_EQ(ctrl.alt_pid().integral(), 0.0f);
    EXPECT_FLOAT_EQ(ctrl.vel_pid().integral(), 0.0f);
}
