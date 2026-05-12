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
    std::uint32_t valid_flags = 0;
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;
    float quat_w = 1.0f;
    float quat_x = 0.0f;
    float quat_y = 0.0f;
    float quat_z = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float position_north_m = 0.0f;
    float position_east_m = 0.0f;
    float position_down_m = 0.0f;
    float mag_x = 0.0f;
    float mag_y = 0.0f;
    float mag_z = 0.0f;
    float pressure_pa = 101325.0f;
    float baro_altitude_m = 0.0f;
    float temperature_c = 25.0f;
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    float alt_msl_m = 0.0f;
    float vel_north_mps = 0.0f;
    float vel_east_mps = 0.0f;
    float vel_down_mps = 0.0f;
    std::uint8_t sats = 0;
    std::uint8_t fix_type = 0;
    float battery_voltage_v = 0.0f;
    std::int16_t rssi_dbm = 0;
    std::int16_t snr_db_x100 = 0;
    std::uint16_t loss_pct_x100 = 0;
};

struct ActuatorCommand {
    std::uint64_t sequence = 0;
    std::uint64_t sim_time_us = 0;
    bool has_sim_time = false;
    std::array<float, 4> motors{};
};

enum class SimControlAction {
    Reset,
    Pause,
    Play,
};

struct SimControlCommand {
    std::uint64_t sequence = 0;
    SimControlAction action = SimControlAction::Reset;
};

inline bool starts_with(std::string_view value, std::string_view prefix) {
    return value.substr(0, prefix.size()) == prefix;
}

inline std::string_view sim_control_action_name(SimControlAction action) {
    switch (action) {
        case SimControlAction::Reset:
            return "reset";
        case SimControlAction::Pause:
            return "pause";
        case SimControlAction::Play:
            return "play";
    }

    return "unknown";
}

inline std::string to_sensor_line(const SensorFrame& frame) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(7);
    out << "SENSOR"
        << " seq=" << frame.sequence
        << " sim_time_us=" << frame.sim_time_us
        << " clock=" << frame.clock_source
        << " valid=" << frame.valid_flags
        << " ax=" << frame.accel_x
        << " ay=" << frame.accel_y
        << " az=" << frame.accel_z
        << " gx=" << frame.gyro_x
        << " gy=" << frame.gyro_y
        << " gz=" << frame.gyro_z
        << " qw=" << frame.quat_w
        << " qx=" << frame.quat_x
        << " qy=" << frame.quat_y
        << " qz=" << frame.quat_z
        << " roll=" << frame.roll
        << " pitch=" << frame.pitch
        << " yaw=" << frame.yaw
        << " pn=" << frame.position_north_m
        << " pe=" << frame.position_east_m
        << " pd=" << frame.position_down_m
        << " mx=" << frame.mag_x
        << " my=" << frame.mag_y
        << " mz=" << frame.mag_z
        << " pressure_pa=" << frame.pressure_pa
        << " baro_alt_m=" << frame.baro_altitude_m
        << " temp_c=" << frame.temperature_c
        << " lat_deg=" << frame.lat_deg
        << " lon_deg=" << frame.lon_deg
        << " alt_msl_m=" << frame.alt_msl_m
        << " vn=" << frame.vel_north_mps
        << " ve=" << frame.vel_east_mps
        << " vd=" << frame.vel_down_mps
        << " sats=" << static_cast<unsigned>(frame.sats)
        << " fix=" << static_cast<unsigned>(frame.fix_type)
        << " battery_v=" << frame.battery_voltage_v
        << " rssi_dbm=" << frame.rssi_dbm
        << " snr_db_x100=" << frame.snr_db_x100
        << " loss_pct_x100=" << frame.loss_pct_x100;
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

inline std::optional<SimControlAction> parse_sim_control_action(std::string_view value) {
    if (value == "reset") {
        return SimControlAction::Reset;
    }
    if (value == "pause") {
        return SimControlAction::Pause;
    }
    if (value == "play") {
        return SimControlAction::Play;
    }

    return std::nullopt;
}

inline std::optional<SimControlCommand> parse_sim_control_line(std::string_view line) {
    if (line != "SIM_CONTROL" && !starts_with(line, "SIM_CONTROL ")) {
        return std::nullopt;
    }

    SimControlCommand command{};
    bool has_sequence = false;
    bool has_action = false;

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

        if (key == "action") {
            const auto action = parse_sim_control_action(value);
            if (!action) {
                return std::nullopt;
            }
            command.action = *action;
            has_action = true;
        }
    }

    if (!has_sequence || !has_action) {
        return std::nullopt;
    }

    return command;
}

inline std::string to_sim_control_ack_line(
    const SimControlCommand& command,
    bool ok,
    std::string_view message) {
    std::ostringstream out;
    out << "SIM_CONTROL_ACK"
        << " seq=" << command.sequence
        << " action=" << sim_control_action_name(command.action)
        << " ok=" << (ok ? 1 : 0)
        << " message=" << message;
    return out.str();
}

} // namespace cerberus_bridge
