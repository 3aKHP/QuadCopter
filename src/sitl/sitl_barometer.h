#pragma once

#include "hal_barometer.h"
#include "physics_model.h"
#include <random>

namespace fc {
namespace sitl {

/// SITL Barometer implementation - reads altitude from physics model with noise
class SitlBarometer : public hal::HalBarometer {
public:
    /// @param model Reference to physics model
    /// @param altitude_noise_std Altitude noise standard deviation (m)
    SitlBarometer(PhysicsModel& model, float altitude_noise_std = 0.5f);

    bool init() override;
    hal::BaroData read() override;

private:
    PhysicsModel& model_;
    float altitude_noise_std_;
    std::mt19937 rng_;
    std::normal_distribution<float> alt_noise_;
};

}  // namespace sitl
}  // namespace fc
