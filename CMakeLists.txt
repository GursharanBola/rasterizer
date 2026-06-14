cmake_minimum_required(VERSION 3.10)
project(rasterizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(rasterizer src/main.cpp)

target_include_directories(rasterizer PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/external/eigen"
    "${CMAKE_CURRENT_SOURCE_DIR}/external/stb_image"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
