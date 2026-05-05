#pragma once

#include <array>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

namespace cerberus_bridge {

struct SensorFrame {
    std::uint64_t sequence = 0;
    std::uint64_t sim_time_us = 0;
    std::string clock_source = "fallback";
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

struct ActuatorCommand {
    std::uint64_t sequence = 0;
    std::uint64_t sim_time_us = 0;
    bool has_sim_time = false;
    std::array<float, 4> motors{};
};

inline bool starts_with(std::string_view value, std::string_view prefix) {
    return value.substr(0, prefix.size()) == prefix;
}

inline std::string to_sensor_line(const SensorFrame& frame) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(4);
    out << "SENSOR"
        << " seq=" << frame.sequence
        << " sim_time_us=" << frame.sim_time_us
        << " clock=" << frame.clock_source
        << " ax=" << frame.accel_x
        << " ay=" << frame.accel_y
        << " az=" << frame.accel_z
        << " gx=" << frame.gyro_x
        << " gy=" << frame.gyro_y
        << " gz=" << frame.gyro_z
        << " roll=" << frame.roll
        << " pitch=" << frame.pitch
        << " yaw=" << frame.yaw;
    return out.str();
}

inline std::optional<ActuatorCommand> parse_actuator_line(std::string_view line) {
    if (!starts_with(line, "ACTUATOR")) {
        return std::nullopt;
    }

    ActuatorCommand command{};
    bool has_sequence = false;
    std::array<bool, 4> has_motor{};

    std::istringstream input{std::string(line)};
    std::string token;
    while (input >> token) {
        const auto equals = token.find('=');
        if (equals == std::string::npos) {
            continue;
        }

        const auto key = token.substr(0, equals);
        const auto value = token.substr(equals + 1);

        if (key == "seq") {
            command.sequence = static_cast<std::uint64_t>(std::stoull(value));
            has_sequence = true;
            continue;
        }

        if (key == "sim_time_us") {
            command.sim_time_us = static_cast<std::uint64_t>(std::stoull(value));
            command.has_sim_time = true;
            continue;
        }

        if (key.size() == 2 && key[0] == 'm' && key[1] >= '0' && key[1] <= '3') {
            const auto index = static_cast<std::size_t>(key[1] - '0');
            command.motors[index] = std::stof(value);
            has_motor[index] = true;
        }
    }

    if (!has_sequence || !has_motor[0] || !has_motor[1] || !has_motor[2] || !has_motor[3]) {
        return std::nullopt;
    }

    return command;
}

} // namespace cerberus_bridge
