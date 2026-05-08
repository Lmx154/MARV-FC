#include "cerberus_bridge/tcp_server.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace cerberus_bridge {

namespace {
constexpr int kBacklog = 1;
constexpr std::size_t kBufferSize = 1024;
}

TcpServer::TcpServer() = default;

TcpServer::~TcpServer() {
    stop();
}

bool TcpServer::start(std::uint16_t port) {
    std::lock_guard lock(mutex_);
    if (running_) {
        return true;
    }

    port_ = port;
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        std::perror("socket");
        return false;
    }

    int reuse = 1;
    if (::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        std::perror("setsockopt");
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port_);

    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        std::perror("bind");
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }

    if (::listen(listen_fd_, kBacklog) < 0) {
        std::perror("listen");
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }

    running_ = true;
    accept_thread_ = std::thread(&TcpServer::accept_loop, this);
    return true;
}

void TcpServer::stop() {
    {
        std::lock_guard lock(mutex_);
        if (!running_) {
            return;
        }
        running_ = false;
    }

    if (listen_fd_ >= 0) {
        ::shutdown(listen_fd_, SHUT_RDWR);
        ::close(listen_fd_);
        listen_fd_ = -1;
    }

    close_client();

    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }
}

bool TcpServer::send_line(const std::string& line) {
    std::lock_guard lock(mutex_);
    if (client_fd_ < 0) {
        return false;
    }

    std::string payload = line;
    payload.push_back('\n');

    const auto bytes_sent = ::send(
        client_fd_,
        payload.data(),
        payload.size(),
#ifdef MSG_NOSIGNAL
        MSG_NOSIGNAL
#else
        0
#endif
    );

    if (bytes_sent < 0) {
        std::perror("send");
        close_client_unlocked();
        return false;
    }

    return true;
}

std::optional<std::string> TcpServer::poll_line() {
    std::lock_guard lock(mutex_);
    if (incoming_lines_.empty()) {
        return std::nullopt;
    }

    auto line = std::move(incoming_lines_.front());
    incoming_lines_.pop_front();
    return line;
}

bool TcpServer::has_client() const {
    std::lock_guard lock(mutex_);
    return client_fd_ >= 0;
}

void TcpServer::accept_loop() {
    while (true) {
        if (!running_) {
            break;
        }

        sockaddr_in client_address{};
        socklen_t client_length = sizeof(client_address);
        const auto client_fd = ::accept(
            listen_fd_,
            reinterpret_cast<sockaddr*>(&client_address),
            &client_length
        );

        if (client_fd < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (!running_) {
                break;
            }
            std::perror("accept");
            continue;
        }

        {
            std::lock_guard lock(mutex_);
            if (client_fd_ >= 0) {
                ::close(client_fd_);
            }
            client_fd_ = client_fd;
        }

        std::cout << "[bridge] backend connected" << std::endl;
        read_loop(client_fd);
        close_client();
        std::cout << "[bridge] backend disconnected" << std::endl;
    }
}

void TcpServer::read_loop(int client_fd) {
    std::string buffer;
    char chunk[kBufferSize];

    while (true) {
        {
            std::lock_guard lock(mutex_);
            if (!running_ || client_fd_ != client_fd) {
                break;
            }
        }

        const auto bytes_read = ::recv(client_fd, chunk, sizeof(chunk), 0);
        if (bytes_read <= 0) {
            break;
        }

        buffer.append(chunk, chunk + bytes_read);

        std::size_t newline = 0;
        while ((newline = buffer.find('\n')) != std::string::npos) {
            auto line = buffer.substr(0, newline);
            buffer.erase(0, newline + 1);

            std::lock_guard lock(mutex_);
            incoming_lines_.push_back(std::move(line));
        }
    }
}

void TcpServer::close_client() {
    std::lock_guard lock(mutex_);
    close_client_unlocked();
}

void TcpServer::close_client_unlocked() {
    if (client_fd_ >= 0) {
        ::shutdown(client_fd_, SHUT_RDWR);
        ::close(client_fd_);
        client_fd_ = -1;
    }
}

} // namespace cerberus_bridge
