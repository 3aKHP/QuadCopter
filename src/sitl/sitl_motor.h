#pragma once

#include "hal_motor.h"
#include "physics_model.h"
#include <array>

namespace fc {
namespace sitl {

/// SITL Motor implementation - feeds motor outputs to physics model
class SitlMotor : public hal::HalMotor {
public:
    explicit SitlMotor(PhysicsModel& model);

    bool init() override;
    void set_output(int motor_id, float throttle) override;
    void arm() override;
    void disarm() override;
    bool is_armed() const override;

    /// Get current motor outputs (for physics model stepping)
    const std::array<float, 4>& outputs() const { return outputs_; }

private:
    PhysicsModel& model_;
    bool armed_;
    std::array<float, 4> outputs_;
};

}  // namespace sitl
}  // namespace fc
