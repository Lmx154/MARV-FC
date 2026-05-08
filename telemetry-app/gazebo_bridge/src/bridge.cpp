#include "cerberus_bridge/protocol.hpp"
#include "cerberus_bridge/tcp_server.hpp"

#include <atomic>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <fstream>
#include <csignal>
#include <iostream>
#include <optional>
#include <mutex>
#include <sstream>
#include <string_view>
#include <utility>
#include <thread>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/gps.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/transport.hh>

namespace cerberus_bridge {

namespace {
std::atomic_bool g_running{true};

constexpr std::uint32_t kValidAccel = 1u << 0;
constexpr std::uint32_t kValidGyro = 1u << 1;
constexpr std::uint32_t kValidMag = 1u << 2;
constexpr std::uint32_t kValidBaro = 1u << 3;
constexpr std::uint32_t kValidGps = 1u << 4;
constexpr std::uint32_t kValidBattery = 1u << 5;

struct BridgeConfig {
    std::string clock_topic = "/clock";
    std::string imu_topic = "/world/marv_field/model/marv_f450/link/base_link/sensor/imu_sensor/imu";
    std::string magnetometer_topic = "/world/marv_field/model/marv_f450/link/base_link/sensor/magnetometer_sensor/magnetometer";
    std::string air_pressure_topic = "/world/marv_field/model/marv_f450/link/base_link/sensor/air_pressure_sensor/air_pressure";
    std::string navsat_topic = "/world/marv_field/model/marv_f450/link/base_link/sensor/navsat_sensor/navsat";
    std::string actuator_topic = "/marv_f450/command/motor_speed";
    bool synthetic_sensors = false;
    float nominal_battery_voltage_v = 12.3f;
    double max_rotor_velocity_rad_s = 1000.0;
    std::array<double, 4> motor_directions{1.0, 1.0, -1.0, -1.0};
};

void handle_signal(int) {
    g_running = false;
}

std::string trim(std::string value) {
    auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

bool parse_bool(std::string value) {
    value = trim(std::move(value));
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value == "1" || value == "true" || value == "yes" || value == "on";
}

void load_config_file(const std::string& path, BridgeConfig& config) {
    std::ifstream input(path);
    if (!input) {
        std::cerr << "[bridge] failed to open config " << path << std::endl;
        return;
    }

    std::string line;
    while (std::getline(input, line)) {
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line.erase(comment);
        }

        const auto equals = line.find('=');
        if (equals == std::string::npos) {
            continue;
        }

        const auto key = trim(line.substr(0, equals));
        const auto value = trim(line.substr(equals + 1));
        if (key == "topics.clock" || key == "clock_topic") {
            config.clock_topic = value;
        } else if (key == "topics.imu" || key == "imu_topic") {
            config.imu_topic = value;
        } else if (key == "topics.magnetometer" || key == "magnetometer_topic") {
            config.magnetometer_topic = value;
        } else if (key == "topics.air_pressure" || key == "air_pressure_topic") {
            config.air_pressure_topic = value;
        } else if (key == "topics.navsat" || key == "navsat_topic" || key == "topics.gps") {
            config.navsat_topic = value;
        } else if (key == "actuators.topic" || key == "actuator_topic") {
            config.actuator_topic = value;
        } else if (key == "actuators.max_rotor_velocity_rad_s" || key == "max_rotor_velocity_rad_s") {
            config.max_rotor_velocity_rad_s = std::stod(value);
        } else if (key == "synthetic_sensors") {
            config.synthetic_sensors = parse_bool(value);
        } else if (key == "nominal_battery_voltage_v") {
            config.nominal_battery_voltage_v = std::stof(value);
        } else if (key.rfind("actuators.motor_", 0) == 0 && key.size() > 19) {
            const auto motor_index = static_cast<std::size_t>(key[16] - '1');
            if (motor_index < config.motor_directions.size() && key.find(".direction") != std::string::npos) {
                config.motor_directions[motor_index] = std::stod(value);
            }
        }
    }
}

std::array<float, 3> flu_to_frd(double x, double y, double z) {
    return {
        static_cast<float>(x),
        static_cast<float>(-y),
        static_cast<float>(-z),
    };
}

bool finite3(const std::array<float, 3>& values) {
    return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
}

class GazeboActuatorPublisher {
public:
    bool start(std::string topic, double max_rotor_velocity_rad_s, std::array<double, 4> directions) {
        topic_ = std::move(topic);
        max_rotor_velocity_rad_s_ = max_rotor_velocity_rad_s;
        directions_ = directions;
        publisher_ = node_.Advertise<gz::msgs::Actuators>(topic_);
        if (!publisher_) {
            std::cerr << "[bridge] failed to advertise Gazebo topic " << topic_ << std::endl;
            return false;
        }

        std::cout << "[bridge] publishing actuator commands on " << topic_ << std::endl;
        return true;
    }

    void apply(const ActuatorCommand& command) {
        gz::msgs::Actuators message;
        for (std::size_t i = 0; i < command.motors.size(); ++i) {
            const auto normalized = std::clamp(static_cast<double>(command.motors[i]), 0.0, 1.0);
            message.add_velocity(directions_[i] * normalized * max_rotor_velocity_rad_s_);
        }

        if (command.has_sim_time) {
            auto* stamp = message.mutable_header()->mutable_stamp();
            stamp->set_sec(static_cast<std::int64_t>(command.sim_time_us / 1'000'000));
            stamp->set_nsec(static_cast<std::int32_t>((command.sim_time_us % 1'000'000) * 1'000));
        }

        if (!publisher_.Publish(message)) {
            std::cerr << "[bridge] failed to publish actuator seq=" << command.sequence
                      << " on " << topic_ << std::endl;
            return;
        }

        std::cout << "[bridge] published actuator seq=" << command.sequence
                  << " sim_time_us=" << command.sim_time_us
                  << " m0=" << message.velocity(0)
                  << " m1=" << message.velocity(1)
                  << " m2=" << message.velocity(2)
                  << " m3=" << message.velocity(3)
                  << std::endl;
    }

private:
    gz::transport::Node node_;
    gz::transport::Node::Publisher publisher_;
    std::string topic_ = "/X3/gazebo/command/motor_speed";
    double max_rotor_velocity_rad_s_ = 1000.0;
    std::array<double, 4> directions_{1.0, -1.0, 1.0, -1.0};
};

class SensorCollector {
public:
    explicit SensorCollector(BridgeConfig config)
        : config_(std::move(config)) {}

    bool start() {
        bool any_subscription = false;
        if (!config_.imu_topic.empty()) {
            any_subscription |= subscribe(config_.imu_topic, &SensorCollector::on_imu);
        }
        if (!config_.magnetometer_topic.empty()) {
            any_subscription |= subscribe(config_.magnetometer_topic, &SensorCollector::on_magnetometer);
        }
        if (!config_.air_pressure_topic.empty()) {
            any_subscription |= subscribe(config_.air_pressure_topic, &SensorCollector::on_air_pressure);
        }
        if (!config_.navsat_topic.empty()) {
            any_subscription |= subscribe(config_.navsat_topic, &SensorCollector::on_navsat);
        }

        if (!any_subscription && !config_.synthetic_sensors) {
            std::cerr << "[bridge] no sensor topics configured and synthetic sensors disabled" << std::endl;
            return false;
        }

        return true;
    }

    SensorFrame sample(
        std::uint64_t sequence,
        std::uint64_t sim_time_us,
        std::string clock_source) {
        if (!config_.synthetic_sensors || has_any_real_sample()) {
            return sample_latest(sequence, sim_time_us, std::move(clock_source));
        }

        const double t = static_cast<double>(sim_time_us) / 1'000'000.0;
        SensorFrame frame{};
        frame.sequence = sequence;
        frame.sim_time_us = sim_time_us;
        frame.clock_source = std::move(clock_source);
        frame.valid_flags = kValidAccel | kValidGyro | kValidMag | kValidBaro | kValidGps | kValidBattery;
        frame.accel_x = static_cast<float>(0.10 * std::sin(t));
        frame.accel_y = static_cast<float>(0.10 * std::cos(t));
        frame.accel_z = -9.81f;
        frame.gyro_x = static_cast<float>(0.01 * std::sin(t * 0.5));
        frame.gyro_y = static_cast<float>(0.01 * std::cos(t * 0.5));
        frame.gyro_z = static_cast<float>(0.02 * std::sin(t * 0.25));
        frame.roll = static_cast<float>(0.05 * std::sin(t * 0.2));
        frame.pitch = static_cast<float>(0.05 * std::cos(t * 0.2));
        frame.yaw = static_cast<float>(0.10 * std::sin(t * 0.1));

        constexpr double base_lat = 26.3109420;
        constexpr double base_lon = -98.1747280;
        constexpr float base_alt = 28.711437225f;
        constexpr double meters_to_lat_deg = 1.0 / 111'320.0;
        constexpr double meters_to_lon_deg = 1.0 / (111'320.0 * 0.8964017989909154);

        const auto north_m = 18.0 * std::sin(t * 0.04);
        const auto east_m = 12.0 * std::cos(t * 0.04);
        frame.lat_deg = base_lat + north_m * meters_to_lat_deg;
        frame.lon_deg = base_lon + east_m * meters_to_lon_deg;
        frame.alt_msl_m = base_alt + static_cast<float>(2.5 * std::sin(t * 0.03));
        frame.vel_north_mps = static_cast<float>(18.0 * 0.04 * std::cos(t * 0.04));
        frame.vel_east_mps = static_cast<float>(-12.0 * 0.04 * std::sin(t * 0.04));
        frame.vel_down_mps = static_cast<float>(-2.5 * 0.03 * std::cos(t * 0.03));
        frame.baro_altitude_m = frame.alt_msl_m + static_cast<float>(0.25 * std::sin(t * 0.7));
        frame.pressure_pa = static_cast<float>(101325.0 * std::pow(1.0 - (frame.baro_altitude_m / 44330.0), 5.255));
        frame.temperature_c = 22.0f + static_cast<float>(1.5 * std::sin(t * 0.02));
        frame.mag_x = 21.5f + static_cast<float>(0.7 * std::sin(t * 0.09));
        frame.mag_y = -4.2f + static_cast<float>(0.5 * std::cos(t * 0.08));
        frame.mag_z = 42.8f + static_cast<float>(0.4 * std::sin(t * 0.05));
        frame.sats = 14;
        frame.fix_type = 3;
        frame.battery_voltage_v = config_.nominal_battery_voltage_v;
        return frame;
    }

private:
    template <typename Handler>
    bool subscribe(const std::string& topic, Handler handler) {
        const auto ok = node_.Subscribe(topic, handler, this);
        if (!ok) {
            std::cerr << "[bridge] failed to subscribe " << topic << std::endl;
            return false;
        }

        std::cout << "[bridge] subscribed " << topic << std::endl;
        return true;
    }

    bool has_any_real_sample() const {
        std::lock_guard lock(mutex_);
        return steady_valid_flags_ != 0;
    }

    SensorFrame sample_latest(
        std::uint64_t sequence,
        std::uint64_t sim_time_us,
        std::string clock_source) {
        std::lock_guard lock(mutex_);
        latest_.sequence = sequence;
        latest_.sim_time_us = sim_time_us;
        latest_.clock_source = std::move(clock_source);
        latest_.valid_flags = fresh_valid_flags_;
        latest_.battery_voltage_v = config_.nominal_battery_voltage_v;
        if (latest_.battery_voltage_v > 0.0f && std::isfinite(latest_.battery_voltage_v)) {
            latest_.valid_flags |= kValidBattery;
        }
        fresh_valid_flags_ = 0;
        return latest_;
    }

    void on_imu(const gz::msgs::IMU& message) {
        std::lock_guard lock(mutex_);
        std::uint32_t flags = 0;
        if (message.has_linear_acceleration()) {
            const auto& accel = message.linear_acceleration();
            const auto converted = flu_to_frd(accel.x(), accel.y(), accel.z());
            if (finite3(converted)) {
                latest_.accel_x = converted[0];
                latest_.accel_y = converted[1];
                latest_.accel_z = converted[2];
                flags |= kValidAccel;
            }
        }
        if (message.has_angular_velocity()) {
            const auto& gyro = message.angular_velocity();
            const auto converted = flu_to_frd(gyro.x(), gyro.y(), gyro.z());
            if (finite3(converted)) {
                latest_.gyro_x = converted[0];
                latest_.gyro_y = converted[1];
                latest_.gyro_z = converted[2];
                flags |= kValidGyro;
            }
        }
        if (message.has_orientation()) {
            latest_.roll = 0.0f;
            latest_.pitch = 0.0f;
            latest_.yaw = 0.0f;
        }
        mark_fresh(flags);
    }

    void on_magnetometer(const gz::msgs::Magnetometer& message) {
        if (!message.has_field_tesla()) {
            return;
        }
        const auto& mag = message.field_tesla();
        const auto converted = flu_to_frd(
            mag.x() * 1'000'000.0,
            mag.y() * 1'000'000.0,
            mag.z() * 1'000'000.0);
        if (!finite3(converted)) {
            return;
        }

        std::lock_guard lock(mutex_);
        latest_.mag_x = converted[0];
        latest_.mag_y = converted[1];
        latest_.mag_z = converted[2];
        mark_fresh(kValidMag);
    }

    void on_air_pressure(const gz::msgs::FluidPressure& message) {
        const auto pressure = static_cast<float>(message.pressure());
        if (!std::isfinite(pressure) || pressure < 1000.0f || pressure > 120000.0f) {
            return;
        }

        std::lock_guard lock(mutex_);
        latest_.pressure_pa = pressure;
        latest_.baro_altitude_m = pressure_to_msl_altitude_m(pressure);
        latest_.temperature_c = 15.0f;
        mark_fresh(kValidBaro);
    }

    void on_navsat(const gz::msgs::NavSat& message) {
        const auto lat = message.latitude_deg();
        const auto lon = message.longitude_deg();
        const auto alt = static_cast<float>(message.altitude());
        const std::array<float, 3> vel_ned{
            static_cast<float>(message.velocity_north()),
            static_cast<float>(message.velocity_east()),
            static_cast<float>(-message.velocity_up()),
        };
        if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(alt) || !finite3(vel_ned)) {
            return;
        }

        std::lock_guard lock(mutex_);
        latest_.lat_deg = lat;
        latest_.lon_deg = lon;
        latest_.alt_msl_m = alt;
        latest_.vel_north_mps = vel_ned[0];
        latest_.vel_east_mps = vel_ned[1];
        latest_.vel_down_mps = vel_ned[2];
        latest_.sats = 12;
        latest_.fix_type = 3;
        mark_fresh(kValidGps);
    }

    static float pressure_to_msl_altitude_m(float pressure_pa) {
        return 44330.0f * (1.0f - std::pow(pressure_pa / 101325.0f, 1.0f / 5.255f));
    }

    void mark_fresh(std::uint32_t flags) {
        fresh_valid_flags_ |= flags;
        steady_valid_flags_ |= flags;
    }

    BridgeConfig config_;
    gz::transport::Node node_;
    mutable std::mutex mutex_;
    SensorFrame latest_{};
    std::uint32_t fresh_valid_flags_ = 0;
    std::uint32_t steady_valid_flags_ = 0;
};

class BridgeApp {
public:
    BridgeApp(std::uint16_t port, std::chrono::milliseconds tick, BridgeConfig config)
        : port_(port),
          tick_(tick),
          config_(std::move(config)),
          clock_(config_.clock_topic, gz::transport::NetworkClock::TimeBase::SIM),
          sensor_collector_(config_) {}

    int run() {
        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);
#ifdef SIGPIPE
        std::signal(SIGPIPE, SIG_IGN);
#endif

        if (!server_.start(port_)) {
            std::cerr << "[bridge] failed to start TCP server on port " << port_ << std::endl;
            return 1;
        }

        if (!sensor_collector_.start()) {
            server_.stop();
            return 1;
        }

        if (!actuator_publisher_.start(
                config_.actuator_topic,
                config_.max_rotor_velocity_rad_s,
                config_.motor_directions)) {
            server_.stop();
            return 1;
        }

        std::cout << "[bridge] listening on 127.0.0.1:" << port_ << std::endl;
        std::cout << "[bridge] reading Gazebo sim time from " << config_.clock_topic << std::endl;
        std::cout << "[bridge] waiting for Rust backend connection" << std::endl;

        std::uint64_t fallback_sequence = 0;
        std::optional<std::uint64_t> last_sent_sim_time_us;
        std::optional<std::string> last_sent_clock_source;
        while (g_running) {
            const auto clock_sample = current_sim_time_us(fallback_sequence);

            if (!last_sent_sim_time_us || clock_sample.sim_time_us != *last_sent_sim_time_us ||
                !last_sent_clock_source || clock_sample.source != *last_sent_clock_source) {
                const auto frame = sensor_collector_.sample(
                    sim_tick_for(clock_sample.sim_time_us, fallback_sequence),
                    clock_sample.sim_time_us,
                    clock_sample.source);

                if (server_.has_client()) {
                    server_.send_line(to_sensor_line(frame));
                }

                last_sent_sim_time_us = clock_sample.sim_time_us;
                last_sent_clock_source = clock_sample.source;
            }

            while (auto line = server_.poll_line()) {
                if (const auto command = parse_actuator_line(*line)) {
                    actuator_publisher_.apply(*command);
                } else {
                    std::cout << "[bridge] ignored line: " << *line << std::endl;
                }
            }

            ++fallback_sequence;
            std::this_thread::sleep_for(tick_);
        }

        server_.stop();
        return 0;
    }

private:
    struct ClockSample {
        std::uint64_t sim_time_us = 0;
        std::string source = "fallback";
    };

    ClockSample current_sim_time_us(std::uint64_t fallback_sequence) const {
        if (clock_.IsReady()) {
            const auto sim_time_ns = clock_.Time();
            if (sim_time_ns.count() >= 0) {
                return ClockSample{
                    static_cast<std::uint64_t>(sim_time_ns.count() / 1'000),
                    "gazebo",
                };
            }
        }

        return ClockSample{
            fallback_sequence * static_cast<std::uint64_t>(tick_.count()) * 1'000,
            "fallback",
        };
    }

    std::uint64_t sim_tick_for(
        std::uint64_t sim_time_us,
        std::uint64_t fallback_sequence) const {
        const auto tick_us = static_cast<std::uint64_t>(tick_.count()) * 1'000;
        if (tick_us == 0) {
            return fallback_sequence;
        }

        return sim_time_us / tick_us;
    }

    std::uint16_t port_;
    std::chrono::milliseconds tick_;
    BridgeConfig config_;
    gz::transport::NetworkClock clock_;
    TcpServer server_;
    GazeboActuatorPublisher actuator_publisher_;
    SensorCollector sensor_collector_;
};

int run_main(int argc, char** argv) {
    std::uint16_t port = 9000;
    std::chrono::milliseconds tick{20};
    BridgeConfig config;

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = static_cast<std::uint16_t>(std::stoi(argv[++i]));
            continue;
        }

        if (arg == "--tick-ms" && i + 1 < argc) {
            tick = std::chrono::milliseconds{std::stoi(argv[++i])};
            continue;
        }

        if (arg == "--clock-topic" && i + 1 < argc) {
            config.clock_topic = argv[++i];
            continue;
        }

        if (arg == "--config" && i + 1 < argc) {
            load_config_file(argv[++i], config);
            continue;
        }

        std::cout << "Usage: " << argv[0]
                  << " [--port <port>] [--tick-ms <ms>] [--clock-topic <topic>] [--config <path>]"
                  << std::endl;
        return 1;
    }

    BridgeApp app{port, tick, config};
    return app.run();
}

} // namespace

} // namespace cerberus_bridge

int main(int argc, char** argv) {
    return cerberus_bridge::run_main(argc, argv);
}
