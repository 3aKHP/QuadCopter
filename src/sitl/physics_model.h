#pragma once

#include <array>
#include <cmath>

namespace fc {
namespace sitl {

/// Quadcopter rigid-body physics simulation model
/// Uses NED (North-East-Down) coordinate frame
/// Integrates equations of motion using 4th-order Runge-Kutta
class PhysicsModel {
public:
    /// Physical parameters of the quadcopter
    struct Params {
        float mass       = 1.2f;       ///< Mass (kg)
        float arm_length = 0.225f;     ///< Motor arm length (m)
        float Ixx        = 0.0029f;    ///< Moment of inertia about X (kg·m²)
        float Iyy        = 0.0029f;    ///< Moment of inertia about Y (kg·m²)
        float Izz        = 0.0055f;    ///< Moment of inertia about Z (kg·m²)
        float k_thrust   = 1.0e-5f;   ///< Thrust coefficient (N/(rad/s)²)
        float k_torque   = 1.5e-7f;   ///< Torque coefficient (N·m/(rad/s)²)
        float g          = 9.81f;      ///< Gravity (m/s²)
        float max_rpm    = 10000.0f;   ///< Maximum motor RPM
        float drag_coeff = 0.1f;       ///< Linear drag coefficient
    };

    /// Full state of the quadcopter
    struct State {
        // Position (NED, m)
        float x = 0.0f, y = 0.0f, z = 0.0f;
        // Velocity (NED, m/s)
        float vx = 0.0f, vy = 0.0f, vz = 0.0f;
        // Attitude (Euler angles, rad)
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        // Angular velocity (body frame, rad/s)
        float p = 0.0f, q = 0.0f, r = 0.0f;
    };

    /// State derivative for integration
    struct StateDot {
        float dx, dy, dz;
        float dvx, dvy, dvz;
        float droll, dpitch, dyaw;
        float dp, dq, dr;
    };

    PhysicsModel();
    explicit PhysicsModel(const Params& params);

    /// Step the physics simulation forward
    /// @param motor_outputs Motor throttle values [0.0, 1.0] for motors 0-3
    /// @param dt Time step in seconds
    void step(const std::array<float, 4>& motor_outputs, float dt);

    /// Reset state to initial conditions
    /// @param initial_altitude Initial altitude (positive = above ground, stored as -z in NED)
    void reset(float initial_altitude = 0.0f);

    /// Get current state
    const State& state() const { return state_; }

    /// Get physical parameters
    const Params& params() const { return params_; }
    Params& params() { return params_; }

    /// Get altitude (positive up, converted from NED)
    float altitude() const { return -state_.z; }

    /// Get vertical velocity (positive up)
    float vertical_velocity() const { return -state_.vz; }

    /// Get acceleration in body frame (for IMU simulation)
    struct BodyAccel { float ax, ay, az; };
    BodyAccel body_acceleration() const { return body_accel_; }

private:
    /// Compute state derivatives given current state and motor forces
    StateDot compute_derivatives(const State& s,
                                  const std::array<float, 4>& forces,
                                  const std::array<float, 4>& torques) const;

    /// Convert motor throttle to RPM, then to force/torque
    void compute_motor_forces(const std::array<float, 4>& motor_outputs,
                               std::array<float, 4>& forces,
                               std::array<float, 4>& torques) const;

    /// RK4 integration step
    void rk4_step(const std::array<float, 4>& motor_outputs, float dt);

    /// Add two states (for RK4)
    static State add_states(const State& s, const StateDot& sd, float scale);

    Params params_;
    State state_;
    BodyAccel body_accel_;
};

}  // namespace sitl
}  // namespace fc
