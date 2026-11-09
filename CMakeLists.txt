cmake_minimum_required(VERSION 3.28)
project(
    still_flow
    VERSION 0.1.0
    LANGUAGES C CXX)

option(STILLFLOW_BUILD_CLI "Build the desktop command-line application" ON)
option(STILLFLOW_BUILD_ADAPTERS "Build the FFmpeg/OpenCV and desktop adapters" ON)
option(STILLFLOW_SUPERBUILD "Build pinned dependencies from source" ON)
if(STILLFLOW_BUILD_CLI AND NOT STILLFLOW_BUILD_ADAPTERS)
    message(FATAL_ERROR "The CLI requires STILLFLOW_BUILD_ADAPTERS")
endif()

option(STILLFLOW_WITH_CUDA "Build CUDA rendering when a toolkit is available" ON)
option(STILLFLOW_WITH_NVCODEC "Build FFmpeg with runtime-loaded NVDEC/NVENC support" ON)
if(STILLFLOW_BUILD_ADAPTERS AND STILLFLOW_WITH_CUDA)
    if(NOT CMAKE_CUDA_ARCHITECTURES)
        set(CMAKE_CUDA_ARCHITECTURES
            75
            CACHE STRING "CUDA architectures" FORCE)
    endif()
    include(CheckLanguage)
    check_language(CUDA)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE
        Release
        CACHE STRING "Build type" FORCE)
endif()

# The outer build owns every library dependency; the inner build compiles the app.
if(STILLFLOW_SUPERBUILD AND NOT STILLFLOW_DEPS_ROOT)
    include(cmake/Dependencies.cmake)
    return()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(STILLFLOW_DEPS_ROOT)
    find_package(
        Eigen3
        5
        REQUIRED
        CONFIG
        PATHS
        "${STILLFLOW_DEPS_ROOT}"
        NO_DEFAULT_PATH)
else()
    find_package(Eigen3 5 REQUIRED CONFIG)
endif()

include(GNUInstallDirs)
if(STILLFLOW_BUILD_ADAPTERS
   AND STILLFLOW_WITH_CUDA
   AND CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
endif()
add_subdirectory(lib)
if(STILLFLOW_BUILD_CLI)
    add_subdirectory(cli)
endif()

include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/StillFlowConfig.cmake.in "${CMAKE_CURRENT_BINARY_DIR}/StillFlowConfig.cmake"
                              INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/StillFlow")
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/StillFlowConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
install(
    EXPORT StillFlowTargets
    NAMESPACE StillFlow::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/StillFlow")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/StillFlowConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/StillFlowConfigVersion.cmake"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/StillFlow")
