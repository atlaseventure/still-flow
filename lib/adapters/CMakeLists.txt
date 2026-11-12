if(NOT STILLFLOW_WITH_OPENCV)
    message(FATAL_ERROR "Desktop image processing requires STILLFLOW_WITH_OPENCV")
endif()
if(NOT STILLFLOW_DEPS_ROOT)
    message(FATAL_ERROR "Set STILLFLOW_DEPS_ROOT to the media dependency prefix, or disable STILLFLOW_BUILD_ADAPTERS")
endif()
find_package(
    OpenCV 5.1 REQUIRED CONFIG
    COMPONENTS core
               imgproc
               video
               geometry
               features
               PATHS
               "${STILLFLOW_DEPS_ROOT}"
               NO_DEFAULT_PATH)
find_package(
    nlohmann_json
    3.12
    REQUIRED
    CONFIG
    PATHS
    "${STILLFLOW_DEPS_ROOT}"
    NO_DEFAULT_PATH)
find_package(PkgConfig REQUIRED)
set(ENV{PKG_CONFIG_PATH} "")
set(ENV{PKG_CONFIG_LIBDIR} "${STILLFLOW_DEPS_ROOT}/lib/pkgconfig")
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH FALSE)
# Re-read metadata when the source build changes codec or license options.
unset(__pkg_config_checked_FFMPEG CACHE)
pkg_check_modules(
    FFMPEG
    REQUIRED
    libavformat>=62
    libavcodec>=62
    libavutil>=60
    libswscale>=9
    libswresample>=6)

# FFmpeg stays replaceable as separate shared libraries.
add_library(stillflow_ffmpeg INTERFACE)
target_include_directories(stillflow_ffmpeg SYSTEM INTERFACE ${FFMPEG_INCLUDE_DIRS})
target_compile_options(stillflow_ffmpeg INTERFACE ${FFMPEG_CFLAGS_OTHER})
foreach(library avformat avcodec swscale swresample avutil)
    set(shared_library
        "${STILLFLOW_DEPS_ROOT}/lib/${CMAKE_SHARED_LIBRARY_PREFIX}${library}${CMAKE_SHARED_LIBRARY_SUFFIX}")
    if(NOT EXISTS "${shared_library}")
        message(FATAL_ERROR "Shared FFmpeg library missing: ${shared_library}; rebuild the dependency prefix")
    endif()
    add_library(stillflow_ffmpeg_${library} SHARED IMPORTED GLOBAL)
    set_target_properties(stillflow_ffmpeg_${library} PROPERTIES IMPORTED_LOCATION "${shared_library}")
    target_link_libraries(stillflow_ffmpeg INTERFACE stillflow_ffmpeg_${library})
    file(GLOB runtime_libraries "${shared_library}*")
    install(FILES ${runtime_libraries} DESTINATION ${CMAKE_INSTALL_LIBDIR})
endforeach()
# Verify the actual shared library, including externally supplied prefixes.
include(CheckCXXSourceRuns)
set(CMAKE_REQUIRED_LIBRARIES stillflow_ffmpeg_avcodec)
set(CMAKE_REQUIRED_INCLUDES ${FFMPEG_INCLUDE_DIRS})
unset(STILLFLOW_FFMPEG_IS_LGPL CACHE)
check_cxx_source_runs(
    "#include <cstring>
extern \"C\" {
#include <libavcodec/avcodec.h>
}
int main() { return std::strncmp(avcodec_license(), \"LGPL\", 4); }"
    STILLFLOW_FFMPEG_IS_LGPL)
unset(CMAKE_REQUIRED_LIBRARIES)
unset(CMAKE_REQUIRED_INCLUDES)
if(NOT STILLFLOW_FFMPEG_IS_LGPL)
    message(FATAL_ERROR "The desktop module requires LGPL FFmpeg shared libraries")
endif()

add_library(stillflow_media STATIC src/media.cpp src/media_adapter.cpp src/audio.cpp)
add_library(stillflow_images STATIC src/image_processing.cpp)
add_library(stillflow_host STATIC src/output_storage.cpp src/runtime.cpp src/options.cpp src/stabilize.cpp)
find_package(Threads REQUIRED)
target_link_libraries(stillflow_host PRIVATE Threads::Threads)
add_library(StillFlow::media ALIAS stillflow_media)
add_library(StillFlow::images ALIAS stillflow_images)
add_library(StillFlow::host ALIAS stillflow_host)
foreach(target stillflow_media stillflow_images stillflow_host)
    set_target_properties(${target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_include_directories(${target} PUBLIC include)
    target_include_directories(${target} SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${target} PUBLIC stillflow_application)
    target_compile_options(${target} PRIVATE $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
endforeach()
target_link_libraries(stillflow_media PRIVATE StillFlow::interop opencv_core opencv_imgproc stillflow_ffmpeg)
target_link_libraries(stillflow_images PRIVATE stillflow_vision_opencv ${OpenCV_LIBS})
target_link_libraries(stillflow_host PRIVATE opencv_core nlohmann_json::nlohmann_json stillflow_ffmpeg)
target_compile_definitions(stillflow_host PRIVATE STILLFLOW_VERSION="${PROJECT_VERSION}")

add_library(stillflow_adapters INTERFACE)
add_library(StillFlow::adapters ALIAS stillflow_adapters)
target_link_libraries(stillflow_adapters INTERFACE stillflow_media stillflow_images stillflow_host)
