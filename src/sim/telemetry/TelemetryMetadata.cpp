#include "sim/telemetry/TelemetryMetadata.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <string>

namespace telemetry {
namespace {
struct TelemetryDisplayInfo {
  std::string_view rawName;
  std::string_view displayName;
  std::string_view symbol;
  std::string_view unit{};
};

constexpr std::array TelemetryDisplayNames{
    TelemetryDisplayInfo{"alpha", "Alpha", "\xCE\xB1", "rad"},
    TelemetryDisplayInfo{"beta", "Beta", "\xCE\xB2", "rad"},
    TelemetryDisplayInfo{"roll", "Roll", "\xCF\x86", "rad"},
    TelemetryDisplayInfo{"pitch", "Pitch", "\xCE\xB8", "rad"},
    TelemetryDisplayInfo{"commanded_pitch", "Commanded Pitch", "", "rad"},
    TelemetryDisplayInfo{"pitch_error", "Pitch Error", "", "rad"},
    TelemetryDisplayInfo{
        "commanded_pitch_rate", "Commanded Pitch Rate", "", "rad/s"},
    TelemetryDisplayInfo{"pitch_rate", "Pitch Rate", "", "rad/s"},
    TelemetryDisplayInfo{"pitch_rate_error", "Pitch Rate Error", "", "rad/s"},
    TelemetryDisplayInfo{"heading", "Heading", "\xCF\x88", "rad"},
    TelemetryDisplayInfo{"course", "Course", "", "rad"},
    TelemetryDisplayInfo{"commanded_course", "Commanded Course", "", "rad"},
    TelemetryDisplayInfo{"course_error", "Course Error", "", "rad"},
    TelemetryDisplayInfo{"raw_roll_setpoint", "Raw Roll Setpoint", "", "rad"},
    TelemetryDisplayInfo{
        "limited_roll_setpoint", "Limited Roll Setpoint", "", "rad"},
    TelemetryDisplayInfo{"p", "P", "p", "rad/s"},
    TelemetryDisplayInfo{"q", "Q", "q", "rad/s"},
    TelemetryDisplayInfo{"r", "R", "r", "rad/s"},
    TelemetryDisplayInfo{"p_dot", "p\xCC\x87", "p\xCC\x87", "rad/s^2"},
    TelemetryDisplayInfo{"q_dot", "q\xCC\x87", "q\xCC\x87", "rad/s^2"},
    TelemetryDisplayInfo{"r_dot", "r\xCC\x87", "r\xCC\x87", "rad/s^2"},
    TelemetryDisplayInfo{"commanded_roll", "Commanded Roll", "", "rad"},
    TelemetryDisplayInfo{"roll_error", "Roll Error", "", "rad"},
    TelemetryDisplayInfo{
        "commanded_roll_rate", "Commanded Roll Rate", "", "rad/s"},
    TelemetryDisplayInfo{"roll_rate", "Roll Rate", "", "rad/s"},
    TelemetryDisplayInfo{"roll_rate_error", "Roll Rate Error", "", "rad/s"},
    TelemetryDisplayInfo{"rate_p_term", "P Term", ""},
    TelemetryDisplayInfo{"rate_i_term", "I Term", ""},
    TelemetryDisplayInfo{"rate_d_term", "D Term", ""},
    TelemetryDisplayInfo{"rate_ff_term", "FF Term", ""},
    TelemetryDisplayInfo{"unscaled_torque_command", "Unscaled Torque", ""},
    TelemetryDisplayInfo{"raw_torque_command", "Raw Torque", ""},
    TelemetryDisplayInfo{"roll_torque_command", "Saturated Torque", ""},
    TelemetryDisplayInfo{"pitch_torque_command", "Pitch Torque", ""},
    TelemetryDisplayInfo{"airspeed_scaling", "Airspeed Scaling", ""},
    TelemetryDisplayInfo{"positive_saturation", "Positive Saturation", ""},
    TelemetryDisplayInfo{"negative_saturation", "Negative Saturation", ""},
    TelemetryDisplayInfo{"integrator_limited", "Integrator Limited", ""},
    TelemetryDisplayInfo{"trim_roll_command", "Roll Trim", ""},
    TelemetryDisplayInfo{"trim_elevator_command", "Elevator Trim", ""},
    TelemetryDisplayInfo{"rate_integrator_positive_limit", "+I Limit", ""},
    TelemetryDisplayInfo{"rate_integrator_negative_limit", "-I Limit", ""},
    TelemetryDisplayInfo{"aileron_command", "Roll Hold Aileron Command", ""},
    TelemetryDisplayInfo{"elevator_command", "Pitch Hold Elevator Command", ""},
    TelemetryDisplayInfo{
        "calibrated_airspeed", "Calibrated Airspeed CAS", "", "m/s"},
    TelemetryDisplayInfo{"true_airspeed", "True Airspeed TAS", "", "m/s"},
    TelemetryDisplayInfo{"altitude_agl", "Altitude AGL", "", "m"},
    TelemetryDisplayInfo{"aileron", "Aileron", ""},
    TelemetryDisplayInfo{"elevator", "Elevator", ""},
    TelemetryDisplayInfo{"rudder", "Rudder", ""},
    TelemetryDisplayInfo{
        "commanded_yaw_rate", "Commanded Yaw Rate", "", "rad/s"},
    TelemetryDisplayInfo{
        "coordinated_yaw_rate", "Coordinated Yaw Rate", "", "rad/s"},
    TelemetryDisplayInfo{"yaw_rate", "Yaw Rate", "", "rad/s"},
    TelemetryDisplayInfo{"feedback_yaw_rate", "Feedback Yaw Rate", "", "rad/s"},
    TelemetryDisplayInfo{"yaw_rate_error", "Yaw Rate Error", "", "rad/s"},
    TelemetryDisplayInfo{"sideslip", "Sideslip", "", "rad"},
    TelemetryDisplayInfo{
        "sideslip_rate_correction", "Beta Rate Correction", "", "rad/s"},
    TelemetryDisplayInfo{"roll_to_yaw_ff_term", "Roll-to-Yaw FF", ""},
    TelemetryDisplayInfo{"yaw_torque_command", "Yaw Torque", ""},
    TelemetryDisplayInfo{"raw_rudder_command", "Raw Rudder", ""},
    TelemetryDisplayInfo{"rudder_command", "Rudder Command", ""},
    TelemetryDisplayInfo{"enabled", "Enabled", "", "0 / 1"},
    TelemetryDisplayInfo{"target_altitude_agl", "Target Altitude AGL", "", "m"},
    TelemetryDisplayInfo{"internal_altitude_setpoint_agl",
        "Internal Altitude Setpoint AGL",
        "",
        "m"},
    TelemetryDisplayInfo{"airspeed_cas", "Airspeed CAS", "", "m/s"},
    TelemetryDisplayInfo{
        "target_airspeed_cas", "Target Airspeed CAS", "", "m/s"},
    TelemetryDisplayInfo{"vertical_speed", "Vertical Speed", "", "m/s"},
    TelemetryDisplayInfo{"airspeed_rate", "Airspeed Rate", "", "m/s^2"},
    TelemetryDisplayInfo{
        "target_vertical_speed", "Target Vertical Speed", "", "m/s"},
    TelemetryDisplayInfo{
        "target_airspeed_rate", "Target Airspeed Rate", "", "m/s^2"},
    TelemetryDisplayInfo{"potential_energy", "Potential Energy", "", "m^2/s^2"},
    TelemetryDisplayInfo{"potential_energy_setpoint",
        "Potential Energy Setpoint",
        "",
        "m^2/s^2"},
    TelemetryDisplayInfo{
        "potential_energy_error", "Potential Energy Error", "", "m^2/s^2"},
    TelemetryDisplayInfo{"kinetic_energy", "Kinetic Energy", "", "m^2/s^2"},
    TelemetryDisplayInfo{
        "kinetic_energy_setpoint", "Kinetic Energy Setpoint", "", "m^2/s^2"},
    TelemetryDisplayInfo{
        "kinetic_energy_error", "Kinetic Energy Error", "", "m^2/s^2"},
    TelemetryDisplayInfo{"total_energy", "Total Energy", "", "m^2/s^2"},
    TelemetryDisplayInfo{
        "total_energy_setpoint", "Total Energy Setpoint", "", "m^2/s^2"},
    TelemetryDisplayInfo{
        "total_energy_error", "Total Energy Error", "", "m^2/s^2"},
    TelemetryDisplayInfo{"energy_balance", "Energy Balance", "", "m^2/s^2"},
    TelemetryDisplayInfo{
        "energy_balance_setpoint", "Energy Balance Setpoint", "", "m^2/s^2"},
    TelemetryDisplayInfo{
        "energy_balance_error", "Energy Balance Error", "", "m^2/s^2"},
    TelemetryDisplayInfo{
        "total_energy_rate", "Total Energy Rate", "", "m^2/s^3"},
    TelemetryDisplayInfo{"total_energy_rate_setpoint",
        "Total Energy Rate Setpoint",
        "",
        "m^2/s^3"},
    TelemetryDisplayInfo{
        "total_energy_rate_error", "Total Energy Rate Error", "", "m^2/s^3"},
    TelemetryDisplayInfo{
        "energy_balance_rate", "Energy Balance Rate", "", "m^2/s^3"},
    TelemetryDisplayInfo{"energy_balance_rate_setpoint",
        "Energy Balance Rate Setpoint",
        "",
        "m^2/s^3"},
    TelemetryDisplayInfo{"energy_balance_rate_error",
        "Energy Balance Rate Error",
        "",
        "m^2/s^3"},
    TelemetryDisplayInfo{"target_pitch", "Target Pitch", "", "rad"},
    TelemetryDisplayInfo{
        "target_throttle", "Target Throttle", "", "normalized"},
    TelemetryDisplayInfo{"unclamped_pitch", "Unclamped Pitch", "", "rad"},
    TelemetryDisplayInfo{
        "unclamped_throttle", "Unclamped Throttle", "", "normalized"},
    TelemetryDisplayInfo{
        "throttle_ff_term", "Throttle Feed-Forward Term", "", "normalized"},
    TelemetryDisplayInfo{
        "throttle_p_term", "Throttle P Term", "", "normalized"},
    TelemetryDisplayInfo{
        "throttle_i_term", "Throttle I Term", "", "normalized"},
    TelemetryDisplayInfo{
        "throttle_rate_term", "Throttle Rate Term", "", "normalized"},
    TelemetryDisplayInfo{"pitch_p_term", "Pitch P Term", "", "rad"},
    TelemetryDisplayInfo{"pitch_i_term", "Pitch I Term", "", "rad"},
    TelemetryDisplayInfo{"pitch_rate_term", "Pitch Rate Term", "", "rad"},
    TelemetryDisplayInfo{"pitch_upper_limited", "Pitch Upper Limited", ""},
    TelemetryDisplayInfo{"pitch_lower_limited", "Pitch Lower Limited", ""},
    TelemetryDisplayInfo{"pitch_rate_limited", "Pitch Rate Limited", ""},
    TelemetryDisplayInfo{
        "throttle_upper_saturated", "Throttle Upper Saturated", ""},
    TelemetryDisplayInfo{
        "throttle_lower_saturated", "Throttle Lower Saturated", ""},
    TelemetryDisplayInfo{"throttle_rate_limited", "Throttle Rate Limited", ""},
    TelemetryDisplayInfo{"underspeed_protection", "Underspeed Protection", ""},
    TelemetryDisplayInfo{"overspeed_protection", "Overspeed Protection", ""},
    TelemetryDisplayInfo{
        "throttle_integrator_limited", "Throttle Integrator Limited", ""},
    TelemetryDisplayInfo{
        "pitch_integrator_limited", "Pitch Integrator Limited", ""},
};

std::string ToDisplayName(std::string_view value) {
  std::string result;
  result.reserve(value.size());
  bool capitalize = true;
  for (const char character : value) {
    if (character == '_' || character == '-') {
      result.push_back(' ');
      capitalize = true;
      continue;
    }
    result.push_back(capitalize ? static_cast<char>(std::toupper(
                                      static_cast<unsigned char>(character)))
                                : character);
    capitalize = false;
  }
  return result;
}

} // namespace

TelemetrySignalMetadata ResolveTelemetrySignalMetadata(std::string_view path) {
  const std::size_t separator = path.rfind('/');
  const std::string_view rawName =
      separator == std::string_view::npos ? path : path.substr(separator + 1);
  const auto display = std::find_if(TelemetryDisplayNames.begin(),
      TelemetryDisplayNames.end(),
      [rawName](const TelemetryDisplayInfo &candidate) {
        return candidate.rawName == rawName;
      });
  const std::string displayName = display == TelemetryDisplayNames.end()
                                      ? ToDisplayName(rawName)
                                      : std::string(display->displayName);
  return {.path = std::string(path),
      .displayName = displayName,
      .symbol = display == TelemetryDisplayNames.end()
                    ? std::string{}
                    : std::string(display->symbol),
      .unit = display == TelemetryDisplayNames.end()
                  ? std::string{}
                  : std::string(display->unit),
      .description = displayName};
}
} // namespace telemetry