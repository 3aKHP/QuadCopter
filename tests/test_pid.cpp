#include <gtest/gtest.h>
#include "pid_controller.h"
#include <cmath>

using namespace fc::core;

class PIDTest : public ::testing::Test {
protected:
    void SetUp() override {}
};

TEST_F(PIDTest, ProportionalOnly) {
    PIDController pid(1.0f, 0.0f, 0.0f, -10.0f, 10.0f, 1.0f);

    // Error = 5.0 - 0.0 = 5.0, P output = 1.0 * 5.0 = 5.0
    float output = pid.compute(5.0f, 0.0f, 0.01f);
    EXPECT_NEAR(output, 5.0f, 0.001f);
}

TEST_F(PIDTest, IntegralAccumulation) {
    PIDController pid(0.0f, 1.0f, 0.0f, -10.0f, 10.0f, 100.0f);

    // Constant error = 1.0, dt = 0.1
    // After 10 steps: integral ≈ 1.0 * 0.1 * 10 = 1.0
    for (int i = 0; i < 10; i++) {
        pid.compute(1.0f, 0.0f, 0.1f);
    }
    float output = pid.compute(1.0f, 0.0f, 0.1f);
    EXPECT_NEAR(output, 1.1f, 0.05f);  // 11 steps * 0.1 = 1.1
}

TEST_F(PIDTest, AntiWindup) {
    PIDController pid(0.0f, 1.0f, 0.0f, -10.0f, 10.0f, 0.5f);

    // Large constant error should be limited by integral_max
    for (int i = 0; i < 100; i++) {
        pid.compute(100.0f, 0.0f, 0.1f);
    }
    // Integral should be clamped to 0.5, output = 1.0 * 0.5 = 0.5
    float output = pid.compute(100.0f, 0.0f, 0.1f);
    EXPECT_NEAR(output, 0.5f, 0.01f);
}

TEST_F(PIDTest, OutputClamping) {
    PIDController pid(10.0f, 0.0f, 0.0f, -1.0f, 1.0f, 1.0f);

    // Large error should be clamped
    float output = pid.compute(100.0f, 0.0f, 0.01f);
    EXPECT_FLOAT_EQ(output, 1.0f);

    output = pid.compute(-100.0f, 0.0f, 0.01f);
    EXPECT_FLOAT_EQ(output, -1.0f);
}

TEST_F(PIDTest, DerivativeTerm) {
    PIDController pid(0.0f, 0.0f, 1.0f, -100.0f, 100.0f, 1.0f);

    // First call: no derivative (first_run guard)
    float output1 = pid.compute(0.0f, 0.0f, 0.01f);
    EXPECT_NEAR(output1, 0.0f, 0.001f);

    // Second call: error changes from 0 to 10, derivative = 10/0.01 = 1000
    float output2 = pid.compute(10.0f, 0.0f, 0.01f);
    EXPECT_NEAR(output2, 100.0f, 0.1f);  // Clamped to 100
}

TEST_F(PIDTest, Reset) {
    PIDController pid(1.0f, 1.0f, 1.0f, -10.0f, 10.0f, 10.0f);

    // Build up some state
    pid.compute(5.0f, 0.0f, 0.1f);
    pid.compute(5.0f, 0.0f, 0.1f);

    // Reset and verify clean state
    pid.reset();
    EXPECT_FLOAT_EQ(pid.integral(), 0.0f);

    // First compute after reset should have no derivative
    float output = pid.compute(1.0f, 0.0f, 0.1f);
    // P=1, I=0.1, D=0 (first run) → 1.1
    EXPECT_NEAR(output, 1.1f, 0.01f);
}

TEST_F(PIDTest, ZeroDtReturnsZero) {
    PIDController pid(1.0f, 1.0f, 1.0f, -10.0f, 10.0f, 10.0f);
    float output = pid.compute(5.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(output, 0.0f);
}

TEST_F(PIDTest, SetGains) {
    PIDController pid(1.0f, 0.0f, 0.0f, -10.0f, 10.0f, 1.0f);
    pid.set_gains(2.0f, 0.5f, 0.1f);
    EXPECT_FLOAT_EQ(pid.kp(), 2.0f);
    EXPECT_FLOAT_EQ(pid.ki(), 0.5f);
    EXPECT_FLOAT_EQ(pid.kd(), 0.1f);
}
