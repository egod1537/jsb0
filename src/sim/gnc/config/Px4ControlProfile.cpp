#include "sim/gnc/config/Px4ControlProfile.hpp"

namespace gnc {
const Px4ControlProfile &GetC172xPx4ControlProfile() {
  static const Px4ControlProfile Profile = [] {
    Px4ControlProfile profile{
        .aircraftId = "c172x",
        .course = {},
        .roll = {},
        .pitch = {},
        .yaw = {},
        .tecs = {},
    };
    profile.yaw.setpointMode = Px4YawRateSetpointMode::CoordinatedTurn;
    profile.yaw.rateProportionalGain = 0.8;
    profile.yaw.rateIntegralGain = 0.0;
    profile.yaw.rateDerivativeGain = 0.0;
    profile.yaw.rateFeedForwardGain = 0.0;
    profile.yaw.sideslipToYawRateGain = 8.0;
    profile.yaw.yawRateWashoutTimeConstantSec = 0.0;
    profile.yaw.rollToYawFeedForwardGain = 0.0;
    return profile;
  }();
  return Profile;
}

Px4ControlProfile MakeC172xPx4ControlProfile() {
  return GetC172xPx4ControlProfile();
}
} // namespace gnc
