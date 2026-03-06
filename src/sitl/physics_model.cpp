#include "physics_model.h"
#include <algorithm>

namespace fc {
namespace sitl {

PhysicsModel::PhysicsModel() : params_(), state_(), body_accel_({0, 0, 0}) {}

PhysicsModel::PhysicsModel(const Params& params)
    : params_(params), state_(), body_accel_({0, 0, 0}) {}

void PhysicsModel::reset(float initial_altitude) {
    state_ = State();
    state_.z = -initial_altitude;  // NED: negative z = above ground
    body_accel_ = {0.0f, 0.0f, params_.g};  // Level rest: sensor reads [0,0,+g]
}

void PhysicsModel::compute_motor_forces(const std::array<float, 4>& motor_outputs,
                                         std::array<float, 4>& forces,
                                         std::array<float, 4>& torques) const {
    for (int i = 0; i < 4; i++) {
        float throttle = std::clamp(motor_outputs[i], 0.0f, 1.0f);
        float rpm = throttle * params_.max_rpm;
        float omega = rpm * 2.0f * static_cast<float>(M_PI) / 60.0f;
        forces[i] = params_.k_thrust * omega * omega;
        torques[i] = params_.k_torque * omega * omega;
    }
}

PhysicsModel::StateDot PhysicsModel::compute_derivatives(
    const State& s,
    const std::array<float, 4>& forces,
    const std::array<float, 4>& torques) const
{
    StateDot sd;
    sd.dx = s.vx;
    sd.dy = s.vy;
    sd.dz = s.vz;

    float total_thrust = forces[0] + forces[1] + forces[2] + forces[3];
    float cr = std::cos(s.roll),  sr = std::sin(s.roll);
    float cp = std::cos(s.pitch), sp = std::sin(s.pitch);
    float cy = std::cos(s.yaw),   sy = std::sin(s.yaw);

    // Thrust in NED: R * [0, 0, -T/m] (body -Z is upward thrust)
    sd.dvx = (-total_thrust / params_.mass) * (cr * sp * cy + sr * sy)
             - params_.drag_coeff * s.vx / params_.mass;
    sd.dvy = (-total_thrust / params_.mass) * (cr * sp * sy - sr * cy)
             - params_.drag_coeff * s.vy / params_.mass;
    sd.dvz = (-total_thrust / params_.mass) * (cr * cp) + params_.g
             - params_.drag_coeff * s.vz / params_.mass;

    // Body torques (X-configuration)
    float arm45 = params_.arm_length * 0.7071f;
    float L = arm45 * ( forces[0] - forces[1] + forces[2] - forces[3]);
    float M = arm45 * (-forces[0] - forces[1] + forces[2] + forces[3]);
    float N = -torques[0] + torques[1] + torques[2] - torques[3];

    sd.dp = (L - (params_.Izz - params_.Iyy) * s.q * s.r) / params_.Ixx;
    sd.dq = (M - (params_.Ixx - params_.Izz) * s.p * s.r) / params_.Iyy;
    sd.dr = (N - (params_.Iyy - params_.Ixx) * s.p * s.q) / params_.Izz;

    sd.droll  = s.p + (s.q * sr + s.r * cr) * std::tan(s.pitch);
    sd.dpitch = s.q * cr - s.r * sr;
    sd.dyaw   = (s.q * sr + s.r * cr) / std::max(std::cos(s.pitch), 0.001f);

    return sd;
}

PhysicsModel::State PhysicsModel::add_states(const State& s,
                                               const StateDot& sd,
                                               float scale) {
    State r;
    r.x = s.x + sd.dx * scale;  r.y = s.y + sd.dy * scale;  r.z = s.z + sd.dz * scale;
    r.vx = s.vx + sd.dvx * scale;  r.vy = s.vy + sd.dvy * scale;  r.vz = s.vz + sd.dvz * scale;
    r.roll = s.roll + sd.droll * scale;  r.pitch = s.pitch + sd.dpitch * scale;  r.yaw = s.yaw + sd.dyaw * scale;
    r.p = s.p + sd.dp * scale;  r.q = s.q + sd.dq * scale;  r.r = s.r + sd.dr * scale;
    return r;
}

void PhysicsModel::rk4_step(const std::array<float, 4>& motor_outputs, float dt) {
    std::array<float, 4> forces, torq;
    compute_motor_forces(motor_outputs, forces, torq);

    StateDot k1 = compute_derivatives(state_, forces, torq);
    StateDot k2 = compute_derivatives(add_states(state_, k1, dt / 2.0f), forces, torq);
    StateDot k3 = compute_derivatives(add_states(state_, k2, dt / 2.0f), forces, torq);
    StateDot k4 = compute_derivatives(add_states(state_, k3, dt), forces, torq);

    auto w = [](float v, float a, float b, float c, float d, float h) {
        return v + (h / 6.0f) * (a + 2.0f * b + 2.0f * c + d);
    };

    state_.x     = w(state_.x,     k1.dx,     k2.dx,     k3.dx,     k4.dx,     dt);
    state_.y     = w(state_.y,     k1.dy,     k2.dy,     k3.dy,     k4.dy,     dt);
    state_.z     = w(state_.z,     k1.dz,     k2.dz,     k3.dz,     k4.dz,     dt);
    state_.vx    = w(state_.vx,    k1.dvx,    k2.dvx,    k3.dvx,    k4.dvx,    dt);
    state_.vy    = w(state_.vy,    k1.dvy,    k2.dvy,    k3.dvy,    k4.dvy,    dt);
    state_.vz    = w(state_.vz,    k1.dvz,    k2.dvz,    k3.dvz,    k4.dvz,    dt);
    state_.roll  = w(state_.roll,  k1.droll,  k2.droll,  k3.droll,  k4.droll,  dt);
    state_.pitch = w(state_.pitch, k1.dpitch, k2.dpitch, k3.dpitch, k4.dpitch, dt);
    state_.yaw   = w(state_.yaw,   k1.dyaw,   k2.dyaw,   k3.dyaw,   k4.dyaw,   dt);
    state_.p     = w(state_.p,     k1.dp,     k2.dp,     k3.dp,     k4.dp,     dt);
    state_.q     = w(state_.q,     k1.dq,     k2.dq,     k3.dq,     k4.dq,     dt);
    state_.r     = w(state_.r,     k1.dr,     k2.dr,     k3.dr,     k4.dr,     dt);
}

void PhysicsModel::step(const std::array<float, 4>& motor_outputs, float dt) {
    rk4_step(motor_outputs, dt);

    // Ground constraint (NED: z >= 0 means at or below ground)
    if (state_.z >= 0.0f) {
        state_.z = 0.0f;
        if (state_.vz > 0.0f) state_.vz = 0.0f;
    }

    // Compute accelerometer output (specific force in sensor frame)
    // Sensor convention: Z-up body frame, level at rest reads [0, 0, +g]
    //
    // Physics: specific_force = (non-gravitational force) / mass
    // In NED: a_NED = thrust_NED/m + [0,0,g], specific_force_NED = a_NED - [0,0,g] = thrust_NED/m
    // Transform to body frame, then flip Z for sensor convention.
    float total_thrust = 0.0f;
    for (int i = 0; i < 4; i++) {
        float throttle = std::clamp(motor_outputs[i], 0.0f, 1.0f);
        float rpm = throttle * params_.max_rpm;
        float omega = rpm * 2.0f * static_cast<float>(M_PI) / 60.0f;
        total_thrust += params_.k_thrust * omega * omega;
    }

    float cr = std::cos(state_.roll),  sr = std::sin(state_.roll);
    float cp = std::cos(state_.pitch), sp = std::sin(state_.pitch);
    float cy = std::cos(state_.yaw),   sy = std::sin(state_.yaw);

    // NED acceleration and specific force
    float sf_x = (-total_thrust / params_.mass) * (cr * sp * cy + sr * sy);
    float sf_y = (-total_thrust / params_.mass) * (cr * sp * sy - sr * cy);
    float sf_z = (-total_thrust / params_.mass) * (cr * cp);  // gravity cancels out

    // Rotate specific force to body NED frame: R^T * sf_NED
    // Step 1: Rz^T
    float t1x =  cy * sf_x + sy * sf_y;
    float t1y = -sy * sf_x + cy * sf_y;
    float t1z = sf_z;
    // Step 2: Ry^T
    float t2x =  cp * t1x + sp * t1z;
    float t2y =  t1y;
    float t2z = -sp * t1x + cp * t1z;
    // Step 3: Rx^T
    float bx =  t2x;
    float by =  cr * t2y + sr * t2z;
    float bz = -sr * t2y + cr * t2z;

    // Convert NED body to sensor (Z-up): negate Z
    body_accel_.ax = bx;
    body_accel_.ay = by;
    body_accel_.az = -bz;

    // Ground override: when on ground, add normal force (gravity reaction)
    bool on_ground = (state_.z >= 0.0f);
    if (on_ground) {
        body_accel_.ax = -params_.g * sp;
        body_accel_.ay = -params_.g * sr * cp;
        body_accel_.az =  params_.g * cr * cp;
    }
}

}  // namespace sitl
}  // namespace fc
