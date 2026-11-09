include(ExternalProject)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR CMAKE_CROSSCOMPILING)
    message(FATAL_ERROR "The source dependency build currently supports native Linux builds")
endif()
find_program(STILLFLOW_MAKE make REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter)
set(STILLFLOW_BUILD_JOBS 12 CACHE STRING "Parallel jobs per dependency build")
if(NOT STILLFLOW_BUILD_JOBS MATCHES "^[1-9][0-9]*$")
    message(FATAL_ERROR "STILLFLOW_BUILD_JOBS must be a positive integer")
endif()
set(deps "${CMAKE_BINARY_DIR}/deps")
set(prefix "${deps}/install")
set(common
    DOWNLOAD_DIR "${deps}/downloads"
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    TLS_VERIFY TRUE
    LOG_DOWNLOAD TRUE LOG_CONFIGURE TRUE LOG_BUILD TRUE LOG_INSTALL TRUE
    LOG_OUTPUT_ON_FAILURE TRUE)
set(cmake_args
    "-DCMAKE_INSTALL_PREFIX:PATH=${prefix}"
    -DCMAKE_INSTALL_LIBDIR:STRING=lib
    "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
    "-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}"
    "-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}"
    -DCMAKE_EXPORT_PACKAGE_REGISTRY:BOOL=OFF
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
    -DBUILD_SHARED_LIBS:BOOL=OFF)
set(build_command "${CMAKE_COMMAND}" --build <BINARY_DIR> --parallel "${STILLFLOW_BUILD_JOBS}")
set(build_env "${CMAKE_COMMAND}" -E env
    "PATH=${prefix}/bin:$ENV{PATH}"
    "PKG_CONFIG=${PKG_CONFIG_EXECUTABLE}"
    "PKG_CONFIG_PATH=" "PKG_CONFIG_LIBDIR=${prefix}/lib/pkgconfig")

ExternalProject_Add(dep_eigen ${common}
    URL https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar.gz
    URL_HASH SHA256=e9c326dc8c05cd1e044c71f30f1b2e34a6161a3b6ecf445d56b53ff1669e3dec
    CMAKE_ARGS ${cmake_args} -DBUILD_TESTING:BOOL=OFF
        -DEIGEN_BUILD_BLAS:BOOL=OFF -DEIGEN_BUILD_LAPACK:BOOL=OFF -DEIGEN_BUILD_DEMOS:BOOL=OFF
        -DEIGEN_BUILD_DOC:BOOL=OFF -DEIGEN_BUILD_PKGCONFIG:BOOL=ON
    BUILD_COMMAND ${build_command})

set(stillflow_dependencies dep_eigen)
if(STILLFLOW_BUILD_ADAPTERS)
ExternalProject_Add(dep_json ${common}
    URL https://github.com/nlohmann/json/archive/refs/tags/v3.12.0.tar.gz
    URL_HASH SHA256=4b92eb0c06d10683f7447ce9406cb97cd4b453be18d7279320f7b2f025c10187
    CMAKE_ARGS ${cmake_args} -DJSON_BuildTests:BOOL=OFF -DJSON_Install:BOOL=ON
    BUILD_COMMAND ${build_command})

# NASM is built locally so x264 and FFmpeg retain their x86 SIMD kernels.
ExternalProject_Add(dep_nasm ${common}
    URL https://www.nasm.us/pub/nasm/releasebuilds/2.16.03/nasm-2.16.03.tar.xz
    URL_HASH SHA256=1412a1c760bbd05db026b6c0d1657affd6631cd0a63cddb6f73cc6d4aa616148
    CONFIGURE_COMMAND <SOURCE_DIR>/configure "--prefix=${prefix}" "CC=${CMAKE_C_COMPILER}"
    BUILD_COMMAND "${STILLFLOW_MAKE}" "-j${STILLFLOW_BUILD_JOBS}"
    INSTALL_COMMAND "${STILLFLOW_MAKE}" install)

ExternalProject_Add(dep_meson ${common}
    URL https://github.com/mesonbuild/meson/archive/refs/tags/1.9.1.tar.gz
    URL_HASH SHA256=febaa8f7c1916521c53eb5fd11c0641b5eb4741c2c6e9b42c288ed62d9e4fd2c
    CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")
ExternalProject_Get_Property(dep_meson SOURCE_DIR)
set(meson "${Python3_EXECUTABLE}" "${SOURCE_DIR}/meson.py")

ExternalProject_Add(dep_dav1d ${common}
    URL https://code.videolan.org/videolan/dav1d/-/archive/1.5.4/dav1d-1.5.4.tar.gz
    URL_HASH SHA256=a1d5b63d2d38ec9bd03acf643caa51fa22edd1e89c5a109c4807717216bbec07
    DEPENDS dep_nasm dep_meson
    CONFIGURE_COMMAND ${build_env} "CC=${CMAKE_C_COMPILER}" "CXX=${CMAKE_CXX_COMPILER}"
        ${meson} setup --reconfigure <BINARY_DIR> <SOURCE_DIR> "--prefix=${prefix}" --libdir=lib
        --buildtype=release --default-library=static -Denable_tools=false -Denable_tests=false
    BUILD_COMMAND ${build_env} ${meson} compile -C <BINARY_DIR> "-j${STILLFLOW_BUILD_JOBS}"
    INSTALL_COMMAND ${meson} install -C <BINARY_DIR>)

ExternalProject_Add(dep_zlib ${common}
    URL https://zlib.net/fossils/zlib-1.3.1.tar.gz
    URL_HASH SHA256=9a93b2b7dfdac77ceba5a558a580e74667dd6fede4585b91eefb60f03b72df23
    CONFIGURE_COMMAND "${CMAKE_COMMAND}" -E env "CC=${CMAKE_C_COMPILER}"
        "CFLAGS=-O3 -fPIC" <SOURCE_DIR>/configure "--prefix=${prefix}" --static
    BUILD_COMMAND "${STILLFLOW_MAKE}" "-j${STILLFLOW_BUILD_JOBS}"
    INSTALL_COMMAND "${STILLFLOW_MAKE}" install)

ExternalProject_Add(dep_x264 ${common}
    URL https://code.videolan.org/videolan/x264/-/archive/0480cb05fa188d37ae87e8f4fd8f1aea3711f7ee/x264-0480cb05fa188d37ae87e8f4fd8f1aea3711f7ee.tar.gz
    URL_HASH SHA256=d0967a1348c85dfde363bb52610403be898171493100561efa0dd05d5fd1ae50
    DEPENDS dep_nasm
    CONFIGURE_COMMAND ${build_env} "CC=${CMAKE_C_COMPILER}"
        <SOURCE_DIR>/configure "--prefix=${prefix}" --enable-static --enable-pic
        --disable-cli --disable-opencl
    BUILD_COMMAND ${build_env} "${STILLFLOW_MAKE}" "-j${STILLFLOW_BUILD_JOBS}"
    INSTALL_COMMAND "${STILLFLOW_MAKE}" install-lib-static)

ExternalProject_Add(dep_ffmpeg ${common}
    URL https://ffmpeg.org/releases/ffmpeg-8.0.1.tar.xz
    URL_HASH SHA256=05ee0b03119b45c0bdb4df654b96802e909e0a752f72e4fe3794f487229e5a41
    DEPENDS dep_x264 dep_dav1d dep_zlib
    CONFIGURE_COMMAND ${build_env} <SOURCE_DIR>/configure
        "--prefix=${prefix}" "--cc=${CMAKE_C_COMPILER}" "--cxx=${CMAKE_CXX_COMPILER}"
        --pkg-config-flags=--static --disable-autodetect --disable-shared
        --enable-static --enable-pic --enable-gpl --enable-libx264 --enable-libdav1d --enable-zlib
        --disable-programs --disable-doc --disable-debug
        --disable-avdevice --disable-avfilter --disable-swresample
        --disable-network --disable-hwaccels
    BUILD_COMMAND ${build_env} "${STILLFLOW_MAKE}" "-j${STILLFLOW_BUILD_JOBS}"
    INSTALL_COMMAND "${STILLFLOW_MAKE}" install)

ExternalProject_Add(dep_opencv ${common}
    URL https://github.com/opencv/opencv/archive/refs/tags/5.1.0.tar.gz
    URL_HASH SHA256=638c1dae7e37e45afea21aa217dc667ad7cf57acc66cdf652c074f1e0e5ad6d2
    DEPENDS dep_eigen dep_zlib
    CMAKE_ARGS ${cmake_args}
        -DCMAKE_CXX_STANDARD:STRING=20
        -DBUILD_LIST:STRING=core,imgproc,video,geometry,features
        -DBUILD_opencv_features:BOOL=ON
        -DBUILD_TESTS:BOOL=OFF -DBUILD_PERF_TESTS:BOOL=OFF
        -DBUILD_EXAMPLES:BOOL=OFF -DBUILD_opencv_apps:BOOL=OFF
        -DBUILD_JAVA:BOOL=OFF -DBUILD_opencv_python2:BOOL=OFF -DBUILD_opencv_python3:BOOL=OFF
        -DWITH_EIGEN:BOOL=ON "-DEigen3_DIR:PATH=${prefix}/share/eigen3/cmake"
        -DWITH_HARFBUZZ:BOOL=ON -DBUILD_HARFBUZZ:BOOL=ON -DBUILD_ZLIB:BOOL=OFF
        "-DZLIB_INCLUDE_DIR:PATH=${prefix}/include" "-DZLIB_LIBRARY:FILEPATH=${prefix}/lib/libz.a"
        -DWITH_CAROTENE:BOOL=OFF -DWITH_KLEIDICV:BOOL=OFF
        -DWITH_IPP:BOOL=OFF -DWITH_ITT:BOOL=OFF -DWITH_LAPACK:BOOL=OFF
        -DWITH_OPENCL:BOOL=OFF -DWITH_OPENCLAMDFFT:BOOL=OFF -DWITH_OPENCLAMDBLAS:BOOL=OFF
        -DWITH_TBB:BOOL=OFF -DWITH_OPENMP:BOOL=OFF -DPARALLEL_ENABLE_PLUGINS:BOOL=OFF
        -DWITH_FFMPEG:BOOL=OFF -DWITH_GSTREAMER:BOOL=OFF -DWITH_GTK:BOOL=OFF
        -DWITH_VTK:BOOL=OFF -DWITH_V4L:BOOL=OFF -DWITH_VA:BOOL=OFF -DWITH_VA_INTEL:BOOL=OFF
        -DWITH_PROTOBUF:BOOL=OFF -DWITH_FLATBUFFERS:BOOL=OFF -DWITH_OBSENSOR:BOOL=OFF
        -DWITH_JPEG:BOOL=OFF -DWITH_PNG:BOOL=OFF -DWITH_TIFF:BOOL=OFF -DWITH_WEBP:BOOL=OFF
        -DWITH_OPENJPEG:BOOL=OFF -DWITH_JASPER:BOOL=OFF -DWITH_OPENEXR:BOOL=OFF -DWITH_AVIF:BOOL=OFF
    BUILD_COMMAND ${build_command})

    list(APPEND stillflow_dependencies dep_opencv dep_ffmpeg dep_json)
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
ExternalProject_Add(stillflow
    DEPENDS ${stillflow_dependencies}
    SOURCE_DIR "${CMAKE_SOURCE_DIR}"
    BINARY_DIR "${CMAKE_BINARY_DIR}/app"
    DOWNLOAD_COMMAND ""
    LIST_SEPARATOR "|"
    CMAKE_ARGS ${cmake_args} ${cuda_args}
        "-DSTILLFLOW_BUILD_CLI:BOOL=${STILLFLOW_BUILD_CLI}"
        "-DSTILLFLOW_BUILD_ADAPTERS:BOOL=${STILLFLOW_BUILD_ADAPTERS}"
        "-DSTILLFLOW_DEPS_ROOT:PATH=${prefix}"
        "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY:PATH=${CMAKE_BINARY_DIR}"
        "-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}"
    BUILD_COMMAND ${build_command}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            <BINARY_DIR>/compile_commands.json "${CMAKE_BINARY_DIR}/compile_commands.json"
    BUILD_ALWAYS TRUE
    INSTALL_COMMAND "")

install(CODE "
    execute_process(COMMAND \"${CMAKE_COMMAND}\" --install \"${CMAKE_BINARY_DIR}/app\"
        --prefix \"\${CMAKE_INSTALL_PREFIX}\" COMMAND_ERROR_IS_FATAL ANY)
")
