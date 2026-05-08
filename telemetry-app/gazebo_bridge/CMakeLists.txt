cmake_minimum_required(VERSION 3.16)

project(cerberus_gazebo_bridge VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GZ_TRANSPORT13 REQUIRED IMPORTED_TARGET gz-transport13)
pkg_check_modules(GZ_MSGS10 REQUIRED IMPORTED_TARGET gz-msgs10)

add_executable(cerberus_gazebo_bridge
    src/bridge.cpp
    src/tcp_server.cpp
)

target_include_directories(cerberus_gazebo_bridge
    PRIVATE
        include
        src
)

target_link_libraries(cerberus_gazebo_bridge
    PRIVATE
        PkgConfig::GZ_TRANSPORT13
        PkgConfig::GZ_MSGS10
        Threads::Threads
)
