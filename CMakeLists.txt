cmake_minimum_required(VERSION 3.28)
project(still_flow VERSION 0.1.0 LANGUAGES C CXX)

option(STILLFLOW_WITH_CUDA "Build CUDA rendering when a toolkit is available" ON)
if(STILLFLOW_WITH_CUDA)
    if(NOT CMAKE_CUDA_ARCHITECTURES)
        set(CMAKE_CUDA_ARCHITECTURES 75 CACHE STRING "CUDA architectures" FORCE)
    endif()
    include(CheckLanguage)
    check_language(CUDA)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# The outer build owns every library dependency; the inner build compiles the app.
if(NOT STILLFLOW_DEPS_ROOT)
    include(cmake/Dependencies.cmake)
    return()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV 5.1 REQUIRED CONFIG COMPONENTS core imgproc video geometry features
    PATHS "${STILLFLOW_DEPS_ROOT}" NO_DEFAULT_PATH)
find_package(Eigen3 5 REQUIRED CONFIG PATHS "${STILLFLOW_DEPS_ROOT}" NO_DEFAULT_PATH)
find_package(nlohmann_json 3.12 REQUIRED CONFIG PATHS "${STILLFLOW_DEPS_ROOT}" NO_DEFAULT_PATH)
find_package(PkgConfig REQUIRED)
set(ENV{PKG_CONFIG_PATH} "")
set(ENV{PKG_CONFIG_LIBDIR} "${STILLFLOW_DEPS_ROOT}/lib/pkgconfig")
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH FALSE)
pkg_check_modules(FFMPEG REQUIRED libavformat>=62 libavcodec>=62 libavutil>=60 libswscale>=9)

# Resolve static archives explicitly so a system shared library cannot win lookup.
add_library(stillflow_ffmpeg INTERFACE)
target_include_directories(stillflow_ffmpeg SYSTEM INTERFACE ${FFMPEG_STATIC_INCLUDE_DIRS})
target_compile_options(stillflow_ffmpeg INTERFACE ${FFMPEG_STATIC_CFLAGS_OTHER})
foreach(library IN LISTS FFMPEG_STATIC_LIBRARIES)
    if(EXISTS "${STILLFLOW_DEPS_ROOT}/lib/lib${library}.a")
        target_link_libraries(stillflow_ffmpeg INTERFACE "${STILLFLOW_DEPS_ROOT}/lib/lib${library}.a")
    elseif(library MATCHES "^(m|pthread|dl|rt|atomic)$")
        target_link_libraries(stillflow_ffmpeg INTERFACE "${library}")
    else()
        message(FATAL_ERROR "Unmanaged FFmpeg dependency: ${library}")
    endif()
endforeach()
target_link_options(stillflow_ffmpeg INTERFACE ${FFMPEG_STATIC_LDFLAGS_OTHER})

add_library(stillflow_core
    src/media.cpp
    src/motion.cpp
    src/renderer.cpp
    src/stabilizer.cpp)
if(STILLFLOW_WITH_CUDA AND CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    target_sources(stillflow_core PRIVATE src/renderer_cuda.cu)
    target_compile_definitions(stillflow_core PRIVATE STILLFLOW_HAVE_CUDA)
    target_compile_definitions(stillflow_core PUBLIC
        STILLFLOW_CUDA_VERSION="${CMAKE_CUDA_COMPILER_VERSION}")
    set_target_properties(stillflow_core PROPERTIES CUDA_STANDARD 20
        CUDA_STANDARD_REQUIRED ON CUDA_RUNTIME_LIBRARY Static)
endif()
target_include_directories(stillflow_core PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stillflow_core PUBLIC ${OpenCV_LIBS} Eigen3::Eigen
    nlohmann_json::nlohmann_json stillflow_ffmpeg)
target_compile_definitions(stillflow_core PUBLIC STILLFLOW_VERSION="${PROJECT_VERSION}")

add_executable(stillflow src/main.cpp)
target_link_libraries(stillflow PRIVATE stillflow_core)
foreach(target stillflow_core stillflow)
    target_compile_options(${target} PRIVATE
        $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
endforeach()

include(GNUInstallDirs)
install(TARGETS stillflow RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
