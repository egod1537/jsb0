#pragma once

#include "common/math/Math.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string_view>

namespace gnc {
enum class UnitId {
  None,
  Second,
  Degree,
  Radian,
  DegreePerSecond,
  RadianPerSecond,
  Meter,
  MeterPerSecond,
  PerSecond,
  Normalized,
  Dimensionless,
  NormalizedPerRadian,
  NormalizedPerRadianPerSecond,
};

constexpr std::string_view GetUnitSymbol(UnitId unit) {
  switch (unit) {
  case UnitId::None:
    return "";
  case UnitId::Second:
    return "s";
  case UnitId::Degree:
    return "deg";
  case UnitId::Radian:
    return "rad";
  case UnitId::DegreePerSecond:
    return "deg/s";
  case UnitId::RadianPerSecond:
    return "rad/s";
  case UnitId::Meter:
    return "m";
  case UnitId::MeterPerSecond:
    return "m/s";
  case UnitId::PerSecond:
    return "1/s";
  case UnitId::Normalized:
    return "normalized";
  case UnitId::Dimensionless:
    return "dimensionless";
  case UnitId::NormalizedPerRadian:
    return "%/rad";
  case UnitId::NormalizedPerRadianPerSecond:
    return "%/rad/s";
  }
  return "";
}

template <typename Parameter> struct ParameterMetadata {
  Parameter parameter;
  std::string_view id;
  std::string_view displayName;
  UnitId unit;
  UnitId displayUnit;
  double minimum;
  double maximum;
  double defaultValue;
  double increment;
  std::string_view description{};
};

enum class ParameterValueTransform {
  Identity,
  DegreesToRadians,
};

template <typename Parameter, typename Settings> struct ParameterBinding {
  Parameter parameter;
  double Settings::*value;
  ParameterValueTransform transform = ParameterValueTransform::Identity;
};

constexpr double ConvertParameterUnit(double value, UnitId source,
    UnitId destination) {
  if (source == destination) {
    return value;
  }
  if (source == UnitId::Radian && destination == UnitId::Degree) {
    return math::RadToDeg(value);
  }
  if (source == UnitId::Degree && destination == UnitId::Radian) {
    return math::DegToRad(value);
  }
  if (source == UnitId::RadianPerSecond
      && destination == UnitId::DegreePerSecond) {
    return math::RadToDeg(value);
  }
  if (source == UnitId::DegreePerSecond
      && destination == UnitId::RadianPerSecond) {
    return math::DegToRad(value);
  }
  return value;
}

template <typename Parameter>
constexpr double ToParameterDisplayValue(
    const ParameterMetadata<Parameter> &metadata, double value) {
  return ConvertParameterUnit(value, metadata.unit, metadata.displayUnit);
}

template <typename Parameter>
constexpr double FromParameterDisplayValue(
    const ParameterMetadata<Parameter> &metadata, double value) {
  return ConvertParameterUnit(value, metadata.displayUnit, metadata.unit);
}

template <typename Parameter>
constexpr double GetParameterDisplayIncrement(
    const ParameterMetadata<Parameter> &metadata) {
  return ToParameterDisplayValue(metadata, metadata.increment)
         - ToParameterDisplayValue(metadata, 0.0);
}

template <typename Parameter, std::size_t Size>
constexpr const ParameterMetadata<Parameter> &GetParameterMetadata(
    Parameter parameter,
    const std::array<ParameterMetadata<Parameter>, Size> &metadata) {
  return metadata[static_cast<std::size_t>(parameter)];
}

template <typename Parameter, typename Settings, std::size_t Size>
constexpr const ParameterBinding<Parameter, Settings> &GetParameterBinding(
    Parameter parameter,
    const std::array<ParameterBinding<Parameter, Settings>, Size> &bindings) {
  return bindings[static_cast<std::size_t>(parameter)];
}

constexpr double ParameterValueToStorage(double value,
    ParameterValueTransform transform) {
  return transform == ParameterValueTransform::DegreesToRadians
             ? math::DegToRad(value)
             : value;
}

constexpr double ParameterValueFromStorage(double value,
    ParameterValueTransform transform) {
  return transform == ParameterValueTransform::DegreesToRadians
             ? math::RadToDeg(value)
             : value;
}

template <typename Parameter, typename Settings, std::size_t Size>
double GetBoundParameterValue(const Settings &settings, Parameter parameter,
    const std::array<ParameterBinding<Parameter, Settings>, Size> &bindings) {
  const auto &binding = GetParameterBinding(parameter, bindings);
  return ParameterValueFromStorage(settings.*(binding.value),
      binding.transform);
}

template <typename Parameter, typename Settings, std::size_t Size>
bool SetBoundParameterValue(Settings &settings, Parameter parameter,
    double value,
    const std::array<ParameterMetadata<Parameter>, Size> &metadata,
    const std::array<ParameterBinding<Parameter, Settings>, Size> &bindings) {
  if (!std::isfinite(value)) {
    return false;
  }
  const auto &descriptor = GetParameterMetadata(parameter, metadata);
  const auto &binding = GetParameterBinding(parameter, bindings);
  settings.*(binding.value) = ParameterValueToStorage(
      std::clamp(value, descriptor.minimum, descriptor.maximum),
      binding.transform);
  return true;
}

template <typename Parameter, typename Settings, std::size_t Size>
void NormalizeBoundParameters(Settings &settings,
    const std::array<ParameterMetadata<Parameter>, Size> &metadata,
    const std::array<ParameterBinding<Parameter, Settings>, Size> &bindings) {
  for (std::size_t index = 0; index < Size; ++index) {
    const auto &descriptor = metadata[index];
    const auto &binding = bindings[index];
    double value =
        ParameterValueFromStorage(settings.*(binding.value), binding.transform);
    if (!std::isfinite(value)) {
      value = descriptor.defaultValue;
    }
    settings.*(binding.value) = ParameterValueToStorage(
        std::clamp(value, descriptor.minimum, descriptor.maximum),
        binding.transform);
  }
}

template <typename Parameter, typename Settings, std::size_t Size>
void ResetBoundParametersToDefaults(Settings &settings,
    const std::array<ParameterMetadata<Parameter>, Size> &metadata,
    const std::array<ParameterBinding<Parameter, Settings>, Size> &bindings) {
  for (std::size_t index = 0; index < Size; ++index) {
    settings.*(bindings[index].value) =
        ParameterValueToStorage(metadata[index].defaultValue,
            bindings[index].transform);
  }
}

template <typename Parameter, typename Settings, std::size_t Size>
constexpr bool ValidateParameterSchema(
    const std::array<ParameterMetadata<Parameter>, Size> &metadata,
    const std::array<ParameterBinding<Parameter, Settings>, Size> &bindings) {
  for (std::size_t index = 0; index < Size; ++index) {
    const auto &descriptor = metadata[index];
    if (static_cast<std::size_t>(descriptor.parameter) != index
        || static_cast<std::size_t>(bindings[index].parameter) != index
        || descriptor.id.empty() || descriptor.minimum > descriptor.defaultValue
        || descriptor.defaultValue > descriptor.maximum
        || descriptor.increment <= 0.0 || bindings[index].value == nullptr) {
      return false;
    }
  }
  return true;
}
} // namespace gnc
