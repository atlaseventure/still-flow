cmake_minimum_required(VERSION 3.20)
project(still_flow VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc video calib3d)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.2 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
    libavformat>=61 libavcodec>=61 libavutil>=59 libswscale>=8)

add_library(stillflow_core
    src/media.cpp
    src/motion.cpp
    src/stabilizer.cpp)
target_include_directories(stillflow_core PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stillflow_core PUBLIC ${OpenCV_LIBS} Eigen3::Eigen
    nlohmann_json::nlohmann_json PkgConfig::FFMPEG)
target_compile_definitions(stillflow_core PUBLIC STILLFLOW_VERSION="${PROJECT_VERSION}")

add_executable(stillflow src/main.cpp)
target_link_libraries(stillflow PRIVATE stillflow_core)
foreach(target stillflow_core stillflow)
    target_compile_options(${target} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
endforeach()

include(GNUInstallDirs)
install(TARGETS stillflow RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
