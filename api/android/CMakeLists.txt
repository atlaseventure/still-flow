cmake_minimum_required(VERSION 3.28)
project(stillflow_android LANGUAGES C CXX)
set(STILLFLOW_BUILD_ANDROID_JNI
    ON
    CACHE INTERNAL "Build the Android JNI bridge")

set(BUILD_TESTING
    OFF
    CACHE BOOL "" FORCE)
set(STILLFLOW_BUILD_ADAPTERS
    OFF
    CACHE BOOL "" FORCE)
set(STILLFLOW_BUILD_CLI
    OFF
    CACHE BOOL "" FORCE)
set(STILLFLOW_WITH_OPENCV
    ON
    CACHE BOOL "" FORCE)
set(STILLFLOW_WITH_CUDA
    OFF
    CACHE BOOL "" FORCE)
set(STILLFLOW_WITH_NVCODEC
    OFF
    CACHE BOOL "" FORCE)
set(STILLFLOW_BUILD_EXAMPLES
    OFF
    CACHE BOOL "" FORCE)
set(STILLFLOW_BUILD_C_API
    ON
    CACHE BOOL "" FORCE)
set(STILLFLOW_C_API_SHARED
    OFF
    CACHE BOOL "" FORCE)
add_subdirectory(../.. core)

add_library(stillflow_jni SHARED src/main/cpp/stillflow_jni.cpp)
if(ANDROID)
    target_sources(stillflow_jni PRIVATE src/main/cpp/gpu_images.cpp)
    target_link_libraries(stillflow_jni PRIVATE android EGL GLESv3)
endif()
target_link_libraries(stillflow_jni PRIVATE StillFlow::c)
target_compile_features(stillflow_jni PRIVATE cxx_std_20)
target_compile_options(stillflow_jni PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(stillflow_jni PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
target_link_options(stillflow_jni PRIVATE "LINKER:--no-undefined" "LINKER:--exclude-libs,ALL"
                    "LINKER:--version-script=${CMAKE_CURRENT_SOURCE_DIR}/exports.map")
set_property(
    TARGET stillflow_jni
    APPEND
    PROPERTY LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/exports.map")
if(NOT ANDROID)
    # The same bridge can be exercised with a host JVM during local validation.
    find_package(JNI REQUIRED COMPONENTS JVM)
    target_include_directories(stillflow_jni PRIVATE ${JNI_INCLUDE_DIRS})
endif()
