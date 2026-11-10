cmake_minimum_required(VERSION 3.28)
project(stillflow_runner LANGUAGES CXX)
set(STILLFLOW_ANDROID_RUNNER
    ON
    CACHE INTERNAL "Build the standalone AAR consumer")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_library(runner_pixels SHARED pixels.cpp)
target_compile_features(runner_pixels PRIVATE cxx_std_20)
target_compile_options(runner_pixels PRIVATE -Wall -Wextra -Wpedantic)
