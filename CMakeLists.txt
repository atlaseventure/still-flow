cmake_minimum_required(VERSION 3.28)
project(
    still_flow
    VERSION 0.1.0
    LANGUAGES C CXX)
set(STILLFLOW_TARGET_PLATFORM
    "${CMAKE_SYSTEM_NAME}"
    CACHE INTERNAL "Platform used by quality tooling")

if(CMAKE_SYSTEM_NAME STREQUAL "Linux"
   AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$"
   AND CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(stillflow_desktop_default ON)
elseif(
    CMAKE_SYSTEM_NAME STREQUAL "Android"
    AND CMAKE_ANDROID_ARCH_ABI STREQUAL "arm64-v8a"
    AND CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(stillflow_desktop_default OFF)
else()
    message(FATAL_ERROR "Still Flow supports only Linux amd64 and Android arm64-v8a")
endif()

option(BUILD_TESTING "Build regression tests" OFF)
option(STILLFLOW_BUILD_CLI "Build the desktop command-line application" ${stillflow_desktop_default})
option(STILLFLOW_BUILD_ADAPTERS "Build the FFmpeg/OpenCV and desktop adapters" ${stillflow_desktop_default})
option(STILLFLOW_WITH_OPENCV "Build the full OpenCV motion estimator" ${STILLFLOW_BUILD_ADAPTERS})
option(STILLFLOW_SUPERBUILD "Build pinned dependencies from source" ON)
if(ANDROID AND (STILLFLOW_BUILD_CLI OR STILLFLOW_BUILD_ADAPTERS))
    message(FATAL_ERROR "Android builds support the core SDK; disable STILLFLOW_BUILD_CLI and STILLFLOW_BUILD_ADAPTERS")
endif()
if(STILLFLOW_BUILD_CLI AND NOT STILLFLOW_BUILD_ADAPTERS)
    message(FATAL_ERROR "The CLI requires STILLFLOW_BUILD_ADAPTERS")
endif()

option(STILLFLOW_BUILD_EXAMPLES "Build the in-memory C example" ON)
option(STILLFLOW_BUILD_C_API "Build the stable C interface" ON)
option(STILLFLOW_C_API_SHARED "Build the C interface as a shared library" ON)
option(STILLFLOW_WITH_CUDA "Build CUDA rendering when a toolkit is available" ${stillflow_desktop_default})
option(STILLFLOW_WITH_NVCODEC "Build FFmpeg with runtime-loaded NVDEC/NVENC support" ${stillflow_desktop_default})
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
if(STILLFLOW_SUPERBUILD
   AND NOT STILLFLOW_DEPS_ROOT
   AND STILLFLOW_BUILD_ADAPTERS)
    include(cmake/Dependencies.cmake)
    if(BUILD_TESTING)
        enable_testing()
        add_test(NAME application_regression COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${CMAKE_BINARY_DIR}/app"
                                                     --output-on-failure --stop-on-failure)
    endif()
    return()
endif()

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
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
    find_package(Eigen3 5 QUIET CONFIG)
    if(NOT Eigen3_FOUND)
        include(FetchContent)
        set(BUILD_TESTING_SAVED ${BUILD_TESTING})
        set(BUILD_TESTING OFF)
        set(EIGEN_BUILD_DOC
            OFF
            CACHE BOOL "" FORCE)
        FetchContent_Declare(
            Eigen3
            URL https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar.gz
            URL_HASH SHA256=e9c326dc8c05cd1e044c71f30f1b2e34a6161a3b6ecf445d56b53ff1669e3dec)
        FetchContent_MakeAvailable(Eigen3)
        set(BUILD_TESTING ${BUILD_TESTING_SAVED})
    endif()
endif()

include(GNUInstallDirs)
if(STILLFLOW_BUILD_ADAPTERS AND STILLFLOW_DEPS_ROOT)
    # Validate the dependency package before installing any application files.
    install(CODE "set(ROOT \"${STILLFLOW_DEPS_ROOT}\")")
    install(SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/cmake/DistributionManifest.cmake")
endif()
if(STILLFLOW_BUILD_ADAPTERS
   AND STILLFLOW_WITH_CUDA
   AND CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
endif()
add_subdirectory(lib)
if(STILLFLOW_BUILD_C_API)
    add_subdirectory(api/c)
elseif(STILLFLOW_BUILD_CLI)
    message(FATAL_ERROR "The CLI requires STILLFLOW_BUILD_C_API")
endif()
if(STILLFLOW_BUILD_EXAMPLES AND STILLFLOW_BUILD_C_API)
    add_subdirectory(examples/memory)
endif()
if(STILLFLOW_BUILD_CLI)
    add_subdirectory(cli)
endif()

if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
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

install(FILES LICENSE DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/stillflow")

if(STILLFLOW_BUILD_ADAPTERS AND STILLFLOW_DEPS_ROOT)
    install(DIRECTORY "${STILLFLOW_DEPS_ROOT}/share/licenses/stillflow/"
            DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/stillflow")
endif()

if(STILLFLOW_BUILD_ADAPTERS AND STILLFLOW_DEPS_ROOT)
    install(DIRECTORY "${STILLFLOW_DEPS_ROOT}/share/stillflow/media-sources"
            DESTINATION "${CMAKE_INSTALL_DATADIR}/stillflow")
endif()

install(DIRECTORY licenses/eigen/ DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/stillflow/eigen")
get_target_property(eigen_source_roots Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
foreach(eigen_source_root IN LISTS eigen_source_roots)
    if(EXISTS "${eigen_source_root}/Eigen/Core")
        install(DIRECTORY "${eigen_source_root}/Eigen" DESTINATION "${CMAKE_INSTALL_DATADIR}/stillflow/eigen-source")
        break()
    endif()
endforeach()
