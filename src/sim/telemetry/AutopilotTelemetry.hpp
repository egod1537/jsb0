#pragma once

#include <string_view>

namespace telemetry::paths {
// Raw autopilot telemetry is SI. TECS altitude is AGL and airspeed is CAS.
inline constexpr std::string_view AutopilotTecsEnabled =
    "autopilot/tecs/enabled";
inline constexpr std::string_view AutopilotTecsAltitude =
    "autopilot/tecs/altitude_agl";
inline constexpr std::string_view AutopilotTecsTargetAltitude =
    "autopilot/tecs/target_altitude_agl";
inline constexpr std::string_view AutopilotTecsInternalAltitudeSetpoint =
    "autopilot/tecs/internal_altitude_setpoint_agl";
inline constexpr std::string_view AutopilotTecsAirspeed =
    "autopilot/tecs/airspeed_cas";
inline constexpr std::string_view AutopilotTecsTargetAirspeed =
    "autopilot/tecs/target_airspeed_cas";
inline constexpr std::string_view AutopilotTecsVerticalSpeed =
    "autopilot/tecs/vertical_speed";
inline constexpr std::string_view AutopilotTecsAirspeedRate =
    "autopilot/tecs/airspeed_rate";
inline constexpr std::string_view AutopilotTecsTargetVerticalSpeed =
    "autopilot/tecs/target_vertical_speed";
inline constexpr std::string_view AutopilotTecsTargetAirspeedRate =
    "autopilot/tecs/target_airspeed_rate";
inline constexpr std::string_view AutopilotTecsPotentialEnergy =
    "autopilot/tecs/potential_energy";
inline constexpr std::string_view AutopilotTecsPotentialEnergySetpoint =
    "autopilot/tecs/potential_energy_setpoint";
inline constexpr std::string_view AutopilotTecsPotentialEnergyError =
    "autopilot/tecs/potential_energy_error";
inline constexpr std::string_view AutopilotTecsKineticEnergy =
    "autopilot/tecs/kinetic_energy";
inline constexpr std::string_view AutopilotTecsKineticEnergySetpoint =
    "autopilot/tecs/kinetic_energy_setpoint";
inline constexpr std::string_view AutopilotTecsKineticEnergyError =
    "autopilot/tecs/kinetic_energy_error";
inline constexpr std::string_view AutopilotTecsTotalEnergy =
    "autopilot/tecs/total_energy";
inline constexpr std::string_view AutopilotTecsTotalEnergySetpoint =
    "autopilot/tecs/total_energy_setpoint";
inline constexpr std::string_view AutopilotTecsTotalEnergyError =
    "autopilot/tecs/total_energy_error";
inline constexpr std::string_view AutopilotTecsEnergyBalance =
    "autopilot/tecs/energy_balance";
inline constexpr std::string_view AutopilotTecsEnergyBalanceSetpoint =
    "autopilot/tecs/energy_balance_setpoint";
inline constexpr std::string_view AutopilotTecsEnergyBalanceError =
    "autopilot/tecs/energy_balance_error";
inline constexpr std::string_view AutopilotTecsTotalEnergyRate =
    "autopilot/tecs/total_energy_rate";
inline constexpr std::string_view AutopilotTecsTotalEnergyRateSetpoint =
    "autopilot/tecs/total_energy_rate_setpoint";
inline constexpr std::string_view AutopilotTecsTotalEnergyRateError =
    "autopilot/tecs/total_energy_rate_error";
inline constexpr std::string_view AutopilotTecsEnergyBalanceRate =
    "autopilot/tecs/energy_balance_rate";
inline constexpr std::string_view AutopilotTecsEnergyBalanceRateSetpoint =
    "autopilot/tecs/energy_balance_rate_setpoint";
inline constexpr std::string_view AutopilotTecsEnergyBalanceRateError =
    "autopilot/tecs/energy_balance_rate_error";
inline constexpr std::string_view AutopilotTecsTargetPitch =
    "autopilot/tecs/target_pitch";
inline constexpr std::string_view AutopilotTecsTargetThrottle =
    "autopilot/tecs/target_throttle";
inline constexpr std::string_view AutopilotTecsUnclampedPitch =
    "autopilot/tecs/unclamped_pitch";
inline constexpr std::string_view AutopilotTecsUnclampedThrottle =
    "autopilot/tecs/unclamped_throttle";
inline constexpr std::string_view AutopilotTecsThrottleFeedForwardTerm =
    "autopilot/tecs/throttle_ff_term";
inline constexpr std::string_view AutopilotTecsThrottleProportionalTerm =
    "autopilot/tecs/throttle_p_term";
inline constexpr std::string_view AutopilotTecsThrottleIntegralTerm =
    "autopilot/tecs/throttle_i_term";
inline constexpr std::string_view AutopilotTecsThrottleRateTerm =
    "autopilot/tecs/throttle_rate_term";
inline constexpr std::string_view AutopilotTecsPitchProportionalTerm =
    "autopilot/tecs/pitch_p_term";
inline constexpr std::string_view AutopilotTecsPitchIntegralTerm =
    "autopilot/tecs/pitch_i_term";
inline constexpr std::string_view AutopilotTecsPitchRateTerm =
    "autopilot/tecs/pitch_rate_term";
inline constexpr std::string_view AutopilotTecsPitchUpperLimited =
    "autopilot/tecs/pitch_upper_limited";
inline constexpr std::string_view AutopilotTecsPitchLowerLimited =
    "autopilot/tecs/pitch_lower_limited";
inline constexpr std::string_view AutopilotTecsPitchRateLimited =
    "autopilot/tecs/pitch_rate_limited";
inline constexpr std::string_view AutopilotTecsThrottleUpperSaturated =
    "autopilot/tecs/throttle_upper_saturated";
inline constexpr std::string_view AutopilotTecsThrottleLowerSaturated =
    "autopilot/tecs/throttle_lower_saturated";
inline constexpr std::string_view AutopilotTecsThrottleRateLimited =
    "autopilot/tecs/throttle_rate_limited";
inline constexpr std::string_view AutopilotTecsUnderspeedProtection =
    "autopilot/tecs/underspeed_protection";
inline constexpr std::string_view AutopilotTecsOverspeedProtection =
    "autopilot/tecs/overspeed_protection";
inline constexpr std::string_view AutopilotTecsThrottleIntegratorLimited =
    "autopilot/tecs/throttle_integrator_limited";
inline constexpr std::string_view AutopilotTecsPitchIntegratorLimited =
    "autopilot/tecs/pitch_integrator_limited";

inline constexpr std::string_view AutopilotPitchHoldCommandedPitch =
    "autopilot/pitch_hold/commanded_pitch";
inline constexpr std::string_view AutopilotPitchHoldPitch =
    "autopilot/pitch_hold/pitch";
inline constexpr std::string_view AutopilotPitchHoldPitchError =
    "autopilot/pitch_hold/pitch_error";
inline constexpr std::string_view AutopilotPitchHoldCommandedPitchRate =
    "autopilot/pitch_hold/commanded_pitch_rate";
inline constexpr std::string_view AutopilotPitchHoldPitchRate =
    "autopilot/pitch_hold/pitch_rate";
inline constexpr std::string_view AutopilotPitchHoldPitchRateError =
    "autopilot/pitch_hold/pitch_rate_error";
inline constexpr std::string_view AutopilotPitchHoldElevatorCommand =
    "autopilot/pitch_hold/elevator_command";
inline constexpr std::string_view AutopilotPitchHoldRateProportionalTerm =
    "autopilot/pitch_hold/rate_p_term";
inline constexpr std::string_view AutopilotPitchHoldRateIntegralTerm =
    "autopilot/pitch_hold/rate_i_term";
inline constexpr std::string_view AutopilotPitchHoldRateDerivativeTerm =
    "autopilot/pitch_hold/rate_d_term";
inline constexpr std::string_view AutopilotPitchHoldRateFeedForwardTerm =
    "autopilot/pitch_hold/rate_ff_term";
inline constexpr std::string_view AutopilotPitchHoldUnscaledTorqueCommand =
    "autopilot/pitch_hold/unscaled_torque_command";
inline constexpr std::string_view AutopilotPitchHoldRawTorqueCommand =
    "autopilot/pitch_hold/raw_torque_command";
inline constexpr std::string_view AutopilotPitchHoldPitchTorqueCommand =
    "autopilot/pitch_hold/pitch_torque_command";
inline constexpr std::string_view AutopilotPitchHoldAirspeedScaling =
    "autopilot/pitch_hold/airspeed_scaling";
inline constexpr std::string_view AutopilotPitchHoldPositiveSaturation =
    "autopilot/pitch_hold/positive_saturation";
inline constexpr std::string_view AutopilotPitchHoldNegativeSaturation =
    "autopilot/pitch_hold/negative_saturation";
inline constexpr std::string_view AutopilotPitchHoldIntegratorLimited =
    "autopilot/pitch_hold/integrator_limited";
inline constexpr std::string_view AutopilotPitchHoldTrimElevatorCommand =
    "autopilot/pitch_hold/trim_elevator_command";
inline constexpr std::string_view AutopilotPitchHoldIntegratorPositiveLimit =
    "autopilot/pitch_hold/rate_integrator_positive_limit";
inline constexpr std::string_view AutopilotPitchHoldIntegratorNegativeLimit =
    "autopilot/pitch_hold/rate_integrator_negative_limit";

inline constexpr std::string_view AutopilotCourseHoldCommandedCourse =
    "autopilot/course_hold/commanded_course";
inline constexpr std::string_view AutopilotCourseHoldCourse =
    "autopilot/course_hold/course";
inline constexpr std::string_view AutopilotCourseHoldCourseError =
    "autopilot/course_hold/course_error";
inline constexpr std::string_view AutopilotCourseHoldGroundSpeed =
    "autopilot/course_hold/ground_speed";
inline constexpr std::string_view AutopilotCourseHoldRawRollSetpoint =
    "autopilot/course_hold/raw_roll_setpoint";
inline constexpr std::string_view AutopilotCourseHoldLimitedRollSetpoint =
    "autopilot/course_hold/limited_roll_setpoint";
inline constexpr std::string_view AutopilotCourseHoldRollLimited =
    "autopilot/course_hold/roll_limited";
inline constexpr std::string_view AutopilotCourseHoldRollSetpointRateLimited =
    "autopilot/course_hold/roll_setpoint_rate_limited";

inline constexpr std::string_view AutopilotRollHoldCommandedRoll =
    "autopilot/roll_hold/commanded_roll";
inline constexpr std::string_view AutopilotRollHoldRoll =
    "autopilot/roll_hold/roll";
inline constexpr std::string_view AutopilotRollHoldRollError =
    "autopilot/roll_hold/roll_error";
inline constexpr std::string_view AutopilotRollHoldCommandedRollRate =
    "autopilot/roll_hold/commanded_roll_rate";
inline constexpr std::string_view AutopilotRollHoldRollRate =
    "autopilot/roll_hold/roll_rate";
inline constexpr std::string_view AutopilotRollHoldRollRateError =
    "autopilot/roll_hold/roll_rate_error";
inline constexpr std::string_view AutopilotRollHoldAileronCommand =
    "autopilot/roll_hold/aileron_command";

inline constexpr std::string_view AutopilotRollHoldRateProportionalTerm =
    "autopilot/roll_hold/rate_p_term";
inline constexpr std::string_view AutopilotRollHoldRateIntegralTerm =
    "autopilot/roll_hold/rate_i_term";
inline constexpr std::string_view AutopilotRollHoldRateDerivativeTerm =
    "autopilot/roll_hold/rate_d_term";
inline constexpr std::string_view AutopilotRollHoldRateFeedForwardTerm =
    "autopilot/roll_hold/rate_ff_term";

inline constexpr std::string_view AutopilotRollHoldUnscaledTorqueCommand =
    "autopilot/roll_hold/unscaled_torque_command";
inline constexpr std::string_view AutopilotRollHoldRawTorqueCommand =
    "autopilot/roll_hold/raw_torque_command";
inline constexpr std::string_view AutopilotRollHoldRollTorqueCommand =
    "autopilot/roll_hold/roll_torque_command";
inline constexpr std::string_view AutopilotRollHoldAirspeedScaling =
    "autopilot/roll_hold/airspeed_scaling";

inline constexpr std::string_view AutopilotRollHoldPositiveSaturation =
    "autopilot/roll_hold/positive_saturation";
inline constexpr std::string_view AutopilotRollHoldNegativeSaturation =
    "autopilot/roll_hold/negative_saturation";
inline constexpr std::string_view AutopilotRollHoldIntegratorLimited =
    "autopilot/roll_hold/integrator_limited";
inline constexpr std::string_view AutopilotRollHoldTrimRollCommand =
    "autopilot/roll_hold/trim_roll_command";

inline constexpr std::string_view AutopilotRollHoldRateIntegratorPositiveLimit =
    "autopilot/roll_hold/rate_integrator_positive_limit";
inline constexpr std::string_view AutopilotRollHoldRateIntegratorNegativeLimit =
    "autopilot/roll_hold/rate_integrator_negative_limit";

inline constexpr std::string_view AutopilotYawRateCommandedYawRate =
    "autopilot/yaw_rate/commanded_yaw_rate";
inline constexpr std::string_view AutopilotYawRateCoordinatedYawRate =
    "autopilot/yaw_rate/coordinated_yaw_rate";
inline constexpr std::string_view AutopilotYawRateSideslip =
    "autopilot/yaw_rate/sideslip";
inline constexpr std::string_view AutopilotYawRateSideslipCorrection =
    "autopilot/yaw_rate/sideslip_rate_correction";
inline constexpr std::string_view AutopilotYawRateYawRate =
    "autopilot/yaw_rate/yaw_rate";
inline constexpr std::string_view AutopilotYawRateFeedbackYawRate =
    "autopilot/yaw_rate/feedback_yaw_rate";
inline constexpr std::string_view AutopilotYawRateError =
    "autopilot/yaw_rate/yaw_rate_error";
inline constexpr std::string_view AutopilotYawRateProportionalTerm =
    "autopilot/yaw_rate/rate_p_term";
inline constexpr std::string_view AutopilotYawRateIntegralTerm =
    "autopilot/yaw_rate/rate_i_term";
inline constexpr std::string_view AutopilotYawRateDerivativeTerm =
    "autopilot/yaw_rate/rate_d_term";
inline constexpr std::string_view AutopilotYawRateFeedForwardTerm =
    "autopilot/yaw_rate/rate_ff_term";
inline constexpr std::string_view AutopilotYawRateRollToYawFeedForwardTerm =
    "autopilot/yaw_rate/roll_to_yaw_ff_term";
inline constexpr std::string_view AutopilotYawRateIntegrator =
    "autopilot/yaw_rate/integrator";
inline constexpr std::string_view AutopilotYawRateUnscaledTorqueCommand =
    "autopilot/yaw_rate/unscaled_torque_command";
inline constexpr std::string_view AutopilotYawRateRawTorqueCommand =
    "autopilot/yaw_rate/raw_torque_command";
inline constexpr std::string_view AutopilotYawRateYawTorqueCommand =
    "autopilot/yaw_rate/yaw_torque_command";
inline constexpr std::string_view AutopilotYawRateRawRudderCommand =
    "autopilot/yaw_rate/raw_rudder_command";
inline constexpr std::string_view AutopilotYawRateRudderCommand =
    "autopilot/yaw_rate/rudder_command";
inline constexpr std::string_view AutopilotYawRateAirspeedScaling =
    "autopilot/yaw_rate/airspeed_scaling";
inline constexpr std::string_view AutopilotYawRateTrimRudderCommand =
    "autopilot/yaw_rate/trim_rudder_command";
inline constexpr std::string_view AutopilotYawRatePositiveSaturation =
    "autopilot/yaw_rate/positive_saturation";
inline constexpr std::string_view AutopilotYawRateNegativeSaturation =
    "autopilot/yaw_rate/negative_saturation";
inline constexpr std::string_view AutopilotYawRateIntegratorLimited =
    "autopilot/yaw_rate/integrator_limited";
inline constexpr std::string_view AutopilotYawRateIntegratorPositiveLimit =
    "autopilot/yaw_rate/integrator_positive_limit";
inline constexpr std::string_view AutopilotYawRateIntegratorNegativeLimit =
    "autopilot/yaw_rate/integrator_negative_limit";
} // namespace telemetry::paths
