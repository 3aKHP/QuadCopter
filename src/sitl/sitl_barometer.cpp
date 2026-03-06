#include "sitl_barometer.h"

namespace fc {
namespace sitl {

SitlBarometer::SitlBarometer(PhysicsModel& model, float altitude_noise_std)
    : model_(model),
      altitude_noise_std_(altitude_noise_std),
      rng_(123),
      alt_noise_(0.0f, altitude_noise_std) {}

bool SitlBarometer::init() {
    return true;
}

hal::BaroData SitlBarometer::read() {
    hal::BaroData data;

    float true_alt = model_.altitude();
    data.altitude_m = true_alt + alt_noise_(rng_);

    // Compute pressure from altitude (simplified barometric formula)
    // P = P0 * (1 - L*h/T0)^(g*M/(R*L))
    // Simplified: P ≈ 101325 * (1 - 2.25577e-5 * h)^5.25588
    float h = true_alt;
    data.pressure_pa = 101325.0f * std::pow(1.0f - 2.25577e-5f * h, 5.25588f);
    data.temperature_c = 25.0f - 0.0065f * h;  // Standard lapse rate

    return data;
}

}  // namespace sitl
}  // namespace fc
