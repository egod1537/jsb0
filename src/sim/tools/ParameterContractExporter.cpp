#include "sim/gnc/config/Px4ControlProfile.hpp"
#include "sim/gnc/control/attitude/Px4RollParameterMetadata.hpp"

#include <array>
#include <charconv>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
std::string EscapeJson(std::string_view value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    switch (character) {
    case '\\':
      escaped += "\\\\";
      break;
    case '"':
      escaped += "\\\"";
      break;
    case '\n':
      escaped += "\\n";
      break;
    case '\r':
      escaped += "\\r";
      break;
    case '\t':
      escaped += "\\t";
      break;
    default:
      escaped += character;
      break;
    }
  }
  return escaped;
}

std::string FormatNumber(double value) {
  std::array<char, 64> buffer{};
  const auto result = std::to_chars(buffer.data(),
      buffer.data() + buffer.size(),
      value,
      std::chars_format::general);
  if (result.ec != std::errc{}) {
    throw std::runtime_error("failed to format parameter value");
  }
  return std::string(buffer.data(), result.ptr);
}

double GetProfileValue(const gnc::Px4ControlProfile &profile,
    gnc::Px4RollHoldParameter parameter) {
  return gnc::GetBoundParameterValue(profile.roll,
      parameter,
      gnc::Px4RollHoldParameterBindings);
}

std::string BuildParametersJson() {
  const auto &profile = gnc::GetC172xPx4ControlProfile();
  std::ostringstream output;
  output << "{\n"
            "  \"contract_version\": \""
         << JSB_CONTRACT_VERSION
         << "\",\n"
            "  \"schema_version\": 1,\n"
            "  \"parameters\": [\n";
  for (std::size_t index = 0; index < gnc::Px4RollHoldParameters.size();
      ++index) {
    const auto &metadata = gnc::Px4RollHoldParameters[index];
    output << "    {\n"
              "      \"id\": \""
           << EscapeJson(metadata.id)
           << "\",\n"
              "      \"display_name\": \""
           << EscapeJson(metadata.displayName)
           << "\",\n"
              "      \"description\": \""
           << EscapeJson(metadata.description)
           << "\",\n"
              "      \"module\": \"px4.roll\",\n"
              "      \"controller\": \"Px4RollController\",\n"
              "      \"type\": \"number\",\n"
              "      \"unit\": \""
           << EscapeJson(gnc::GetUnitSymbol(metadata.displayUnit))
           << "\",\n"
              "      \"minimum\": "
           << FormatNumber(metadata.minimum)
           << ",\n"
              "      \"maximum\": "
           << FormatNumber(metadata.maximum)
           << ",\n"
              "      \"algorithm_default\": "
           << FormatNumber(metadata.defaultValue)
           << ",\n"
              "      \"default_value\": "
           << FormatNumber(GetProfileValue(profile, metadata.parameter))
           << ",\n"
              "      \"increment\": "
           << FormatNumber(gnc::GetParameterDisplayIncrement(metadata))
           << ",\n"
              "      \"variants\": [\"baseline\"],\n"
              "      \"aircraft\": [\"c172x\"],\n"
              "      \"profiles\": {\n"
              "        \"c172x\": {\"value\": "
           << FormatNumber(GetProfileValue(profile, metadata.parameter))
           << "}\n"
              "      },\n"
              "      \"read_only\": false,\n"
              "      \"experimental\": false\n"
              "    }";
    output << (index + 1 == gnc::Px4RollHoldParameters.size() ? "\n" : ",\n");
  }
  output << "  ]\n}\n";
  return output.str();
}

std::string BuildParameterSetSchema() {
  std::ostringstream output;
  output << "{\n"
            "  \"$schema\": \"https://json-schema.org/draft/2020-12/schema\",\n"
            "  \"$id\": "
            "\"https://jsb0.dev/contract/v2/parameter-set.schema.json\",\n"
            "  \"title\": \"JSB0 Controller Parameter Set\",\n"
            "  \"description\": \"YAML parameter override file read from the "
            "run output directory. JSON Schema numeric values exclude NaN and "
            "infinity.\",\n"
            "  \"type\": \"object\",\n"
            "  \"additionalProperties\": false,\n"
            "  \"required\": [\"controller_parameters\"],\n"
            "  \"properties\": {\n"
            "    \"controller_parameters\": {\n"
            "      \"type\": \"object\",\n"
            "      \"additionalProperties\": false,\n"
            "      \"properties\": {\n";
  for (std::size_t index = 0; index < gnc::Px4RollHoldParameters.size();
      ++index) {
    const auto &metadata = gnc::Px4RollHoldParameters[index];
    output << "        \"" << EscapeJson(metadata.id)
           << "\": {\"type\": \"number\", \"minimum\": "
           << FormatNumber(metadata.minimum)
           << ", \"maximum\": " << FormatNumber(metadata.maximum) << "}";
    output << (index + 1 == gnc::Px4RollHoldParameters.size() ? "\n" : ",\n");
  }
  output << "      }\n"
            "    }\n"
            "  }\n"
            "}\n";
  return output.str();
}

void WriteFile(const std::filesystem::path &path, std::string_view contents) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    throw std::runtime_error("cannot open output file: " + path.string());
  }
  stream.write(contents.data(), static_cast<std::streamsize>(contents.size()));
  if (!stream) {
    throw std::runtime_error("cannot write output file: " + path.string());
  }
}
} // namespace

int main(int argc, char **argv) {
  if (argc != 3 || std::string_view(argv[1]) != "--output") {
    std::cerr << "Usage: jsb0-parameter-contract-export --output <directory>\n";
    return 2;
  }
  try {
    const std::filesystem::path outputDirectory(argv[2]);
    WriteFile(outputDirectory / "parameters.json", BuildParametersJson());
    WriteFile(outputDirectory / "parameter-set.schema.json",
        BuildParameterSetSchema());
  } catch (const std::exception &error) {
    std::cerr << "parameter contract export failed: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
