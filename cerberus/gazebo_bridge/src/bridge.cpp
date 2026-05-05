#include "cerberus_bridge/protocol.hpp"
#include "cerberus_bridge/tcp_server.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <optional>
#include <string_view>
#include <utility>
#include <thread>

#include <gz/msgs.hh>
#include <gz/transport.hh>

namespace cerberus_bridge {

namespace {
std::atomic_bool g_running{true};

void handle_signal(int) {
    g_running = false;
}

class GazeboActuatorPublisher {
public:
    bool start(std::string topic) {
        topic_ = std::move(topic);
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
        message.add_velocity(command.motors[0]);
        message.add_velocity(command.motors[1]);
        message.add_velocity(command.motors[2]);
        message.add_velocity(command.motors[3]);

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
                  << " m0=" << command.motors[0]
                  << " m1=" << command.motors[1]
                  << " m2=" << command.motors[2]
                  << " m3=" << command.motors[3]
                  << std::endl;
    }

private:
    gz::transport::Node node_;
    gz::transport::Node::Publisher publisher_;
    std::string topic_ = "/X3/gazebo/command/motor_speed";
};

class SensorSampler {
public:
    SensorFrame sample(
        std::uint64_t sequence,
        std::uint64_t sim_time_us,
        std::string clock_source) const {
        const double t = static_cast<double>(sim_time_us) / 1'000'000.0;
        SensorFrame frame{};
        frame.sequence = sequence;
        frame.sim_time_us = sim_time_us;
        frame.clock_source = std::move(clock_source);
        frame.accel_x = static_cast<float>(0.10 * std::sin(t));
        frame.accel_y = static_cast<float>(0.10 * std::cos(t));
        frame.accel_z = 9.81f;
        frame.gyro_x = static_cast<float>(0.01 * std::sin(t * 0.5));
        frame.gyro_y = static_cast<float>(0.01 * std::cos(t * 0.5));
        frame.gyro_z = static_cast<float>(0.02 * std::sin(t * 0.25));
        frame.roll = static_cast<float>(0.05 * std::sin(t * 0.2));
        frame.pitch = static_cast<float>(0.05 * std::cos(t * 0.2));
        frame.yaw = static_cast<float>(0.10 * std::sin(t * 0.1));
        return frame;
    }
};

class BridgeApp {
public:
    BridgeApp(std::uint16_t port, std::chrono::milliseconds tick, std::string clock_topic)
        : port_(port),
          tick_(tick),
          clock_topic_(std::move(clock_topic)),
          clock_(clock_topic_, gz::transport::NetworkClock::TimeBase::SIM) {}

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

        if (!actuator_publisher_.start("/X3/gazebo/command/motor_speed")) {
            server_.stop();
            return 1;
        }

        std::cout << "[bridge] listening on 127.0.0.1:" << port_ << std::endl;
        std::cout << "[bridge] reading Gazebo sim time from " << clock_topic_ << std::endl;
        std::cout << "[bridge] waiting for Rust backend connection" << std::endl;

        std::uint64_t fallback_sequence = 0;
        std::optional<std::uint64_t> last_sent_sim_time_us;
        std::optional<std::string> last_sent_clock_source;
        while (g_running) {
            const auto clock_sample = current_sim_time_us(fallback_sequence);

            if (!last_sent_sim_time_us || clock_sample.sim_time_us != *last_sent_sim_time_us ||
                !last_sent_clock_source || clock_sample.source != *last_sent_clock_source) {
                const auto frame = sensor_sampler_.sample(
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
    std::string clock_topic_;
    gz::transport::NetworkClock clock_;
    TcpServer server_;
    GazeboActuatorPublisher actuator_publisher_;
    SensorSampler sensor_sampler_;
};

} // namespace

} // namespace cerberus_bridge

int main(int argc, char** argv) {
    std::uint16_t port = 9000;
    std::chrono::milliseconds tick{20};
    std::string clock_topic = "/clock";

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
            clock_topic = argv[++i];
            continue;
        }

        std::cout << "Usage: " << argv[0]
                  << " [--port <port>] [--tick-ms <ms>] [--clock-topic <topic>]"
                  << std::endl;
        return 1;
    }

    cerberus_bridge::BridgeApp app{port, tick, clock_topic};
    return app.run();
}
