include(ExternalProject)
include("${CMAKE_CURRENT_LIST_DIR}/OpenCVSource.cmake")

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR CMAKE_CROSSCOMPILING)
    message(FATAL_ERROR "The source dependency build currently supports native Linux builds")
endif()
find_program(STILLFLOW_MAKE make REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter)
set(STILLFLOW_BUILD_JOBS
    12
    CACHE STRING "Parallel jobs per dependency build")
if(NOT STILLFLOW_BUILD_JOBS MATCHES "^[1-9][0-9]*$")
    message(FATAL_ERROR "STILLFLOW_BUILD_JOBS must be a positive integer")
endif()
set(deps "${CMAKE_BINARY_DIR}/deps")
set(prefix "${deps}/install")
set(common
    DOWNLOAD_DIR
    "${deps}/downloads"
    DOWNLOAD_EXTRACT_TIMESTAMP
    TRUE
    TLS_VERIFY
    TRUE
    LOG_DOWNLOAD
    TRUE
    LOG_CONFIGURE
    TRUE
    LOG_BUILD
    TRUE
    LOG_INSTALL
    TRUE
    LOG_OUTPUT_ON_FAILURE
    TRUE)
set(dependency_cmake_args
    "-DCMAKE_INSTALL_PREFIX:PATH=${prefix}"
    -DCMAKE_INSTALL_LIBDIR:STRING=lib
    "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
    "-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}"
    "-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}"
    -DCMAKE_EXPORT_PACKAGE_REGISTRY:BOOL=OFF
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
    -DBUILD_SHARED_LIBS:BOOL=OFF)
set(build_command "${CMAKE_COMMAND}" --build <BINARY_DIR> --parallel "${STILLFLOW_BUILD_JOBS}")
set(build_env
    "${CMAKE_COMMAND}"
    -E
    env
    "PATH=${prefix}/bin:$ENV{PATH}"
    "PKG_CONFIG=${PKG_CONFIG_EXECUTABLE}"
    "PKG_CONFIG_PATH="
    "PKG_CONFIG_LIBDIR=${prefix}/lib/pkgconfig")

ExternalProject_Add(
    dep_eigen
    ${common}
    URL https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar.gz
    URL_HASH SHA256=e9c326dc8c05cd1e044c71f30f1b2e34a6161a3b6ecf445d56b53ff1669e3dec
    CMAKE_ARGS ${dependency_cmake_args}
               -DBUILD_TESTING:BOOL=OFF
               -DEIGEN_BUILD_BLAS:BOOL=OFF
               -DEIGEN_BUILD_LAPACK:BOOL=OFF
               -DEIGEN_BUILD_DEMOS:BOOL=OFF
               -DEIGEN_BUILD_DOC:BOOL=OFF
               -DEIGEN_BUILD_PKGCONFIG:BOOL=ON
    BUILD_COMMAND ${build_command})

set(stillflow_dependencies dep_eigen)
if(STILLFLOW_BUILD_ADAPTERS)
    ExternalProject_Add(
        dep_json
        ${common}
        URL https://github.com/nlohmann/json/archive/refs/tags/v3.12.0.tar.gz
        URL_HASH SHA256=4b92eb0c06d10683f7447ce9406cb97cd4b453be18d7279320f7b2f025c10187
        CMAKE_ARGS ${dependency_cmake_args} -DJSON_BuildTests:BOOL=OFF -DJSON_Install:BOOL=ON
        BUILD_COMMAND ${build_command})

    # NASM preserves x86 SIMD kernels in media dependencies.
    ExternalProject_Add(
        dep_nasm
        ${common}
        URL https://www.nasm.us/pub/nasm/releasebuilds/2.16.03/nasm-2.16.03.tar.xz
        URL_HASH SHA256=1412a1c760bbd05db026b6c0d1657affd6631cd0a63cddb6f73cc6d4aa616148
        CONFIGURE_COMMAND <SOURCE_DIR>/configure "--prefix=${prefix}" "CC=${CMAKE_C_COMPILER}"
        BUILD_COMMAND "${STILLFLOW_MAKE}" "-j${STILLFLOW_BUILD_JOBS}"
        INSTALL_COMMAND "${STILLFLOW_MAKE}" install)

    ExternalProject_Add(
        dep_meson
        ${common}
        URL https://github.com/mesonbuild/meson/archive/refs/tags/1.9.1.tar.gz
        URL_HASH SHA256=febaa8f7c1916521c53eb5fd11c0641b5eb4741c2c6e9b42c288ed62d9e4fd2c
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND "")
    ExternalProject_Get_Property(dep_meson SOURCE_DIR)
    set(meson "${Python3_EXECUTABLE}" "${SOURCE_DIR}/meson.py")

    ExternalProject_Add(
        dep_dav1d
        ${common}
        URL https://code.videolan.org/videolan/dav1d/-/archive/1.5.4/dav1d-1.5.4.tar.gz
        URL_HASH SHA256=a1d5b63d2d38ec9bd03acf643caa51fa22edd1e89c5a109c4807717216bbec07
        DEPENDS dep_nasm dep_meson
        CONFIGURE_COMMAND
            ${build_env} "CC=${CMAKE_C_COMPILER}" "CXX=${CMAKE_CXX_COMPILER}" ${meson} setup --reconfigure <BINARY_DIR>
            <SOURCE_DIR> "--prefix=${prefix}" --libdir=lib --buildtype=release --default-library=static
            -Denable_tools=false -Denable_tests=false
        BUILD_COMMAND ${build_env} ${meson} compile -C <BINARY_DIR> "-j${STILLFLOW_BUILD_JOBS}"
        INSTALL_COMMAND ${meson} install -C <BINARY_DIR>)

    ExternalProject_Add(
        dep_zlib
        ${common}
        URL https://zlib.net/fossils/zlib-1.3.1.tar.gz
        URL_HASH SHA256=9a93b2b7dfdac77ceba5a558a580e74667dd6fede4585b91eefb60f03b72df23
        CONFIGURE_COMMAND "${CMAKE_COMMAND}" -E env "CC=${CMAKE_C_COMPILER}" "CFLAGS=-O3 -fPIC" <SOURCE_DIR>/configure
                          "--prefix=${prefix}" --static
        BUILD_COMMAND "${STILLFLOW_MAKE}" "-j${STILLFLOW_BUILD_JOBS}"
        INSTALL_COMMAND "${STILLFLOW_MAKE}" install)

    ExternalProject_Add(
        dep_openh264
        ${common}
        URL https://github.com/cisco/openh264/archive/refs/tags/v2.7.0.tar.gz
        URL_HASH SHA256=5178775c8b440073e70920e97d195efff721c0e56d0a78cc82c919e95d54924d
        DEPENDS dep_nasm
        CONFIGURE_COMMAND ""
        BUILD_IN_SOURCE TRUE
        BUILD_COMMAND ${build_env} "${STILLFLOW_MAKE}" "-j${STILLFLOW_BUILD_JOBS}" libraries "CC=${CMAKE_C_COMPILER}"
                      "CXX=${CMAKE_CXX_COMPILER}" BUILDTYPE=Release
        INSTALL_COMMAND "${STILLFLOW_MAKE}" install-static "PREFIX=${prefix}")
    # NVIDIA codec support is header-only: FFmpeg loads libcuda, libnvcuvid and
    # libnvidia-encode from the driver at runtime, so builds need no CUDA Toolkit
    # and binaries still run without an NVIDIA driver. 12.2 requires driver 550+.
    set(nvidia_codec_args --disable-ffnvcodec --disable-hwaccels)
    set(ffmpeg_dependencies dep_openh264 dep_dav1d dep_zlib)
    if(STILLFLOW_WITH_NVCODEC)
        ExternalProject_Add(
            dep_nvcodec
            ${common}
            URL https://github.com/FFmpeg/nv-codec-headers/archive/refs/tags/n12.2.72.0.tar.gz
            URL_HASH SHA256=dbeaec433d93b850714760282f1d0992b1254fc3b5a6cb7d76fc1340a1e47563
            CONFIGURE_COMMAND ""
            BUILD_COMMAND ""
            BUILD_IN_SOURCE TRUE
            INSTALL_COMMAND "${STILLFLOW_MAKE}" "PREFIX=${prefix}" LIBDIR=lib install)
        list(APPEND ffmpeg_dependencies dep_nvcodec)
        # AV1 stays on dav1d: FFmpeg prefers it, and dav1d has no hwaccel.
        set(nvidia_codec_args --enable-ffnvcodec --enable-cuda --enable-nvdec --enable-nvenc --disable-hwaccels
                              --enable-hwaccel=h264_nvdec,hevc_nvdec,vp9_nvdec,mpeg2_nvdec,mpeg4_nvdec)
    endif()

    ExternalProject_Add(
        dep_ffmpeg
        ${common}
        URL https://ffmpeg.org/releases/ffmpeg-8.0.1.tar.xz
        URL_HASH SHA256=05ee0b03119b45c0bdb4df654b96802e909e0a752f72e4fe3794f487229e5a41
        DEPENDS ${ffmpeg_dependencies}
        CONFIGURE_COMMAND
            ${build_env} <SOURCE_DIR>/configure "--prefix=${prefix}" "--cc=${CMAKE_C_COMPILER}"
            "--cxx=${CMAKE_CXX_COMPILER}" --pkg-config-flags=--static --disable-autodetect --enable-shared
            --disable-static --enable-rpath --enable-pic --disable-gpl --disable-nonfree --enable-libopenh264
            --enable-libdav1d --enable-zlib --disable-programs --disable-doc --disable-debug --disable-avdevice
            --disable-avfilter --disable-swresample --disable-network ${nvidia_codec_args}
        BUILD_COMMAND ${build_env} "${STILLFLOW_MAKE}" "-j${STILLFLOW_BUILD_JOBS}"
        INSTALL_COMMAND "${STILLFLOW_MAKE}" install
        COMMAND "${CMAKE_COMMAND}" "-DPREFIX=${prefix}" -P "${CMAKE_SOURCE_DIR}/cmake/RelocateFFmpeg.cmake")

    ExternalProject_Add(
        dep_opencv
        ${common}
        URL ${STILLFLOW_OPENCV_URL}
        URL_HASH SHA256=${STILLFLOW_OPENCV_SHA256}
        DEPENDS dep_eigen dep_zlib
        CMAKE_ARGS ${dependency_cmake_args}
                   -DCMAKE_CXX_STANDARD:STRING=20
                   -DBUILD_LIST:STRING=core,imgproc,video,geometry,features
                   -DBUILD_opencv_features:BOOL=ON
                   -DBUILD_TESTS:BOOL=OFF
                   -DBUILD_PERF_TESTS:BOOL=OFF
                   -DBUILD_EXAMPLES:BOOL=OFF
                   -DBUILD_opencv_apps:BOOL=OFF
                   -DBUILD_JAVA:BOOL=OFF
                   -DBUILD_opencv_python2:BOOL=OFF
                   -DBUILD_opencv_python3:BOOL=OFF
                   -DWITH_EIGEN:BOOL=ON
                   "-DEigen3_DIR:PATH=${prefix}/share/eigen3/cmake"
                   -DWITH_HARFBUZZ:BOOL=ON
                   -DBUILD_HARFBUZZ:BOOL=ON
                   -DBUILD_ZLIB:BOOL=OFF
                   "-DZLIB_INCLUDE_DIR:PATH=${prefix}/include"
                   "-DZLIB_LIBRARY:FILEPATH=${prefix}/lib/libz.a"
                   -DWITH_CAROTENE:BOOL=OFF
                   -DWITH_KLEIDICV:BOOL=OFF
                   -DWITH_IPP:BOOL=OFF
                   -DWITH_ITT:BOOL=OFF
                   -DWITH_LAPACK:BOOL=OFF
                   -DWITH_OPENCL:BOOL=OFF
                   -DWITH_OPENCLAMDFFT:BOOL=OFF
                   -DWITH_OPENCLAMDBLAS:BOOL=OFF
                   -DWITH_TBB:BOOL=OFF
                   -DWITH_OPENMP:BOOL=OFF
                   -DPARALLEL_ENABLE_PLUGINS:BOOL=OFF
                   -DWITH_FFMPEG:BOOL=OFF
                   -DWITH_GSTREAMER:BOOL=OFF
                   -DWITH_GTK:BOOL=OFF
                   -DWITH_VTK:BOOL=OFF
                   -DWITH_V4L:BOOL=OFF
                   -DWITH_VA:BOOL=OFF
                   -DWITH_VA_INTEL:BOOL=OFF
                   -DWITH_PROTOBUF:BOOL=OFF
                   -DWITH_FLATBUFFERS:BOOL=OFF
                   -DWITH_OBSENSOR:BOOL=OFF
                   -DWITH_JPEG:BOOL=OFF
                   -DWITH_PNG:BOOL=OFF
                   -DWITH_TIFF:BOOL=OFF
                   -DWITH_WEBP:BOOL=OFF
                   -DWITH_OPENJPEG:BOOL=OFF
                   -DWITH_JASPER:BOOL=OFF
                   -DWITH_OPENEXR:BOOL=OFF
                   -DWITH_AVIF:BOOL=OFF
        BUILD_COMMAND ${build_command})

    list(APPEND stillflow_dependencies dep_opencv dep_ffmpeg dep_json)
endif()

set(distribution_files)
# Copy dependency license notices into the install prefix.
function(stillflow_dependency_notice target name)
    ExternalProject_Get_Property(${target} SOURCE_DIR)
    set(notice_dir "${prefix}/share/licenses/stillflow/${name}")
    set(commands COMMAND "${CMAKE_COMMAND}" -E make_directory "${notice_dir}")
    foreach(file IN LISTS ARGN)
        get_filename_component(filename "${file}" NAME)
        list(APPEND distribution_files "share/licenses/stillflow/${name}/${filename}")
        list(
            APPEND
            commands
            COMMAND
            "${CMAKE_COMMAND}"
            -E
            copy_if_different
            "${SOURCE_DIR}/${file}"
            "${notice_dir}/")
    endforeach()
    set(distribution_files
        "${distribution_files}"
        PARENT_SCOPE)
    ExternalProject_Add_Step(
        ${target} license_${name}
        DEPENDEES install ${commands}
        ALWAYS TRUE)
endfunction()
stillflow_dependency_notice(
    dep_eigen
    eigen
    COPYING.MPL2
    COPYING.BSD
    COPYING.APACHE
    COPYING.MINPACK
    COPYING.README)
if(STILLFLOW_BUILD_ADAPTERS)
    stillflow_dependency_notice(dep_ffmpeg ffmpeg COPYING.LGPLv2.1 LICENSE.md)
    stillflow_dependency_notice(dep_openh264 openh264 LICENSE)
    stillflow_dependency_notice(dep_dav1d dav1d COPYING)
    stillflow_dependency_notice(dep_zlib zlib LICENSE)
    stillflow_dependency_notice(dep_json nlohmann_json LICENSE.MIT)
    stillflow_dependency_notice(dep_opencv opencv LICENSE)
    stillflow_dependency_notice(dep_opencv harfbuzz 3rdparty/harfbuzz/COPYING)
    stillflow_dependency_notice(dep_opencv flann modules/flann/include/opencv2/flann/defines.h)
endif()

if(STILLFLOW_BUILD_ADAPTERS)
    set(media_sources dep_ffmpeg dep_openh264 dep_dav1d dep_zlib dep_nasm dep_meson)
    if(STILLFLOW_WITH_NVCODEC)
        list(APPEND media_sources dep_nvcodec)
    endif()
    set(source_bundle "${prefix}/share/stillflow/media-sources")
    set(source_commands COMMAND "${CMAKE_COMMAND}" -E make_directory "${source_bundle}")
    set(source_manifest "")
    foreach(dependency IN LISTS media_sources)
        ExternalProject_Get_Property(${dependency} URL URL_HASH)
        get_filename_component(archive "${URL}" NAME)
        list(APPEND distribution_files "share/stillflow/media-sources/${archive}")
        list(
            APPEND
            source_commands
            COMMAND
            "${CMAKE_COMMAND}"
            -E
            copy_if_different
            "${deps}/downloads/${archive}"
            "${source_bundle}/")
        string(APPEND source_manifest "${dependency}: ${URL} ${URL_HASH}\n")
    endforeach()
    file(WRITE "${CMAKE_BINARY_DIR}/media-sources.txt" "${source_manifest}")
    configure_file("${CMAKE_SOURCE_DIR}/cmake/MediaBuild.txt.in" "${CMAKE_BINARY_DIR}/BUILDING.txt" @ONLY)
    ExternalProject_Get_Property(dep_ffmpeg BINARY_DIR)
    ExternalProject_Add_Step(
        dep_ffmpeg source_bundle
        DEPENDEES install ${source_commands}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_BINARY_DIR}/media-sources.txt" "${BINARY_DIR}/config.h"
                "${CMAKE_BINARY_DIR}/BUILDING.txt" "${source_bundle}/"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${source_bundle}/build-source"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_SOURCE_DIR}/CMakeLists.txt"
                "${CMAKE_SOURCE_DIR}/LICENSE" "${source_bundle}/build-source/"
        COMMAND "${CMAKE_COMMAND}" -E copy_directory "${CMAKE_SOURCE_DIR}/cmake" "${source_bundle}/build-source/cmake"
        ALWAYS TRUE)
    list(
        APPEND
        distribution_files
        share/stillflow/media-sources/media-sources.txt
        share/stillflow/media-sources/config.h
        share/stillflow/media-sources/BUILDING.txt
        share/stillflow/media-sources/build-source/CMakeLists.txt
        share/stillflow/media-sources/build-source/LICENSE
        share/stillflow/media-sources/build-source/cmake/Dependencies.cmake
        share/stillflow/media-sources/build-source/cmake/RelocateFFmpeg.cmake
        share/licenses/opencv5/SoftFloat-COPYING.txt
        share/licenses/opencv5/fonts-Rubik_OFL.txt)
    string(JOIN "\n" distribution_files_text ${distribution_files})
    file(WRITE "${CMAKE_BINARY_DIR}/distribution-files.txt" "${distribution_files_text}\n")
    add_custom_target(
        dep_distribution
        COMMAND "${CMAKE_COMMAND}" "-DROOT=${prefix}" "-DREQUIRED_FILE=${CMAKE_BINARY_DIR}/distribution-files.txt"
                -DWRITE_MANIFEST=ON -P "${CMAKE_SOURCE_DIR}/cmake/DistributionManifest.cmake"
        DEPENDS ${stillflow_dependencies}
        COMMENT "Inventory dependency distribution materials"
        VERBATIM)
    list(APPEND stillflow_dependencies dep_distribution)
endif()

string(REPLACE ";" "|" cuda_architectures "${CMAKE_CUDA_ARCHITECTURES}")
set(cuda_args "-DSTILLFLOW_WITH_CUDA:BOOL=${STILLFLOW_WITH_CUDA}")
if(STILLFLOW_WITH_CUDA AND CMAKE_CUDA_COMPILER)
    list(APPEND cuda_args "-DCMAKE_CUDA_COMPILER:FILEPATH=${CMAKE_CUDA_COMPILER}"
         "-DCMAKE_CUDA_ARCHITECTURES:STRING=${cuda_architectures}")
    if(CMAKE_CUDA_HOST_COMPILER)
        list(APPEND cuda_args "-DCMAKE_CUDA_HOST_COMPILER:FILEPATH=${CMAKE_CUDA_HOST_COMPILER}")
    endif()
endif()
ExternalProject_Add(
    stillflow
    DEPENDS ${stillflow_dependencies}
    SOURCE_DIR "${CMAKE_SOURCE_DIR}"
    BINARY_DIR "${CMAKE_BINARY_DIR}/app"
    DOWNLOAD_COMMAND ""
    LIST_SEPARATOR "|"
    CMAKE_ARGS ${dependency_cmake_args}
               ${cuda_args}
               "-DBUILD_TESTING:BOOL=${BUILD_TESTING}"
               "-DSTILLFLOW_WITH_OPENCV:BOOL=${STILLFLOW_WITH_OPENCV}"
               "-DSTILLFLOW_BUILD_C_API:BOOL=${STILLFLOW_BUILD_C_API}"
               "-DSTILLFLOW_C_API_SHARED:BOOL=${STILLFLOW_C_API_SHARED}"
               "-DSTILLFLOW_BUILD_EXAMPLES:BOOL=${STILLFLOW_BUILD_EXAMPLES}"
               "-DSTILLFLOW_BUILD_CLI:BOOL=${STILLFLOW_BUILD_CLI}"
               "-DSTILLFLOW_BUILD_ADAPTERS:BOOL=${STILLFLOW_BUILD_ADAPTERS}"
               "-DSTILLFLOW_DEPS_ROOT:PATH=${prefix}"
               "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY:PATH=${CMAKE_BINARY_DIR}"
               "-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}"
    BUILD_COMMAND ${build_command}
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different <BINARY_DIR>/compile_commands.json
            "${CMAKE_BINARY_DIR}/compile_commands.json"
    BUILD_ALWAYS TRUE
    INSTALL_COMMAND "")

# cmake-lint: disable=W0106
install(
    CODE "
    execute_process(COMMAND \"${CMAKE_COMMAND}\" --install \"${CMAKE_BINARY_DIR}/app\"
        --prefix \"\${CMAKE_INSTALL_PREFIX}\" COMMAND_ERROR_IS_FATAL ANY)
")
