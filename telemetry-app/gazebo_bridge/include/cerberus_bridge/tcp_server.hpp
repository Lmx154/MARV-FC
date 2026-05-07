#pragma once

#include <cstdint>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace cerberus_bridge {

class TcpServer {
public:
    TcpServer();
    ~TcpServer();

    TcpServer(const TcpServer&) = delete;
    TcpServer& operator=(const TcpServer&) = delete;

    bool start(std::uint16_t port);
    void stop();

    bool send_line(const std::string& line);
    std::optional<std::string> poll_line();
    bool has_client() const;

private:
    void accept_loop();
    void read_loop(int client_fd);
    void close_client();
    void close_client_unlocked();

    std::uint16_t port_ = 0;
    int listen_fd_ = -1;
    int client_fd_ = -1;
    std::atomic_bool running_{false};

    mutable std::mutex mutex_;
    std::deque<std::string> incoming_lines_;
    std::thread accept_thread_;
};

} // namespace cerberus_bridge
