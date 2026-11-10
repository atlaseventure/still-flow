include(ExternalProject)
include("${CMAKE_CURRENT_LIST_DIR}/OpenCVSource.cmake")
set(STILLFLOW_BUILD_JOBS
    12
    CACHE STRING "Parallel dependency build jobs")
if(NOT STILLFLOW_BUILD_JOBS MATCHES "^[1-9][0-9]*$")
    message(FATAL_ERROR "STILLFLOW_BUILD_JOBS must be a positive integer")
endif()
set(opencv_prefix "${CMAKE_BINARY_DIR}/opencv-install")
set(OpenCV_INCLUDE_DIRS "${opencv_prefix}/include/opencv5")
file(MAKE_DIRECTORY "${OpenCV_INCLUDE_DIRS}")
set(opencv_archives)
foreach(module core flann geometry imgproc features video)
    list(APPEND opencv_archives "${opencv_prefix}/lib/libopencv_${module}.a")
endforeach()
ExternalProject_Add(
    stillflow_android_opencv
    URL ${STILLFLOW_OPENCV_URL}
    URL_HASH SHA256=${STILLFLOW_OPENCV_SHA256}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    TLS_VERIFY TRUE
    LOG_DOWNLOAD TRUE
    LOG_CONFIGURE TRUE
    LOG_BUILD TRUE
    LOG_INSTALL TRUE
    LOG_OUTPUT_ON_FAILURE TRUE
    CMAKE_ARGS "-DCMAKE_TOOLCHAIN_FILE:FILEPATH=${CMAKE_TOOLCHAIN_FILE}"
               "-DANDROID_ABI:STRING=${CMAKE_ANDROID_ARCH_ABI}"
               "-DANDROID_PLATFORM:STRING=android-${CMAKE_SYSTEM_VERSION}"
               "-DANDROID_STL:STRING=${ANDROID_STL}"
               -DCMAKE_BUILD_TYPE:STRING=Release
               -DCMAKE_CXX_STANDARD:STRING=20
               -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
               -DCMAKE_POLICY_DEFAULT_CMP0177:STRING=NEW
               "-DCMAKE_INSTALL_PREFIX:PATH=${opencv_prefix}"
               -DOPENCV_LIB_INSTALL_PATH:STRING=lib
               -DOPENCV_LIB_ARCHIVE_INSTALL_PATH:STRING=lib
               -DOPENCV_CONFIG_INSTALL_PATH:STRING=lib/cmake/opencv5
               -DOPENCV_3P_LIB_INSTALL_PATH:STRING=lib
               -DOPENCV_INCLUDE_INSTALL_PATH:STRING=include/opencv5
               -DBUILD_LIST:STRING=core,imgproc,video,geometry,features
               -DBUILD_SHARED_LIBS:BOOL=OFF
               -DBUILD_TESTS:BOOL=OFF
               -DBUILD_PERF_TESTS:BOOL=OFF
               -DBUILD_EXAMPLES:BOOL=OFF
               -DBUILD_opencv_apps:BOOL=OFF
               -DBUILD_ANDROID_PROJECTS:BOOL=OFF
               -DBUILD_ANDROID_EXAMPLES:BOOL=OFF
               -DBUILD_JAVA:BOOL=OFF
               -DBUILD_opencv_python2:BOOL=OFF
               -DBUILD_opencv_python3:BOOL=OFF
               -DBUILD_ZLIB:BOOL=OFF
               -DWITH_CPUFEATURES:BOOL=OFF
               -DWITH_EIGEN:BOOL=OFF
               -DWITH_HARFBUZZ:BOOL=OFF
               -DWITH_CAROTENE:BOOL=OFF
               -DWITH_KLEIDICV:BOOL=OFF
               -DWITH_IPP:BOOL=OFF
               -DWITH_ITT:BOOL=OFF
               -DWITH_LAPACK:BOOL=OFF
               -DWITH_OPENCL:BOOL=OFF
               -DWITH_TBB:BOOL=OFF
               -DWITH_OPENMP:BOOL=OFF
               -DWITH_FFMPEG:BOOL=OFF
               -DWITH_GSTREAMER:BOOL=OFF
               -DWITH_GTK:BOOL=OFF
               -DWITH_VTK:BOOL=OFF
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
    BUILD_COMMAND "${CMAKE_COMMAND}" --build <BINARY_DIR> --parallel "${STILLFLOW_BUILD_JOBS}"
    BUILD_BYPRODUCTS ${opencv_archives})

foreach(module core flann geometry imgproc features video)
    add_library(opencv_${module} STATIC IMPORTED GLOBAL)
    set_target_properties(opencv_${module} PROPERTIES IMPORTED_LOCATION "${opencv_prefix}/lib/libopencv_${module}.a")
    add_dependencies(opencv_${module} stillflow_android_opencv)
endforeach()
set_property(TARGET opencv_core PROPERTY INTERFACE_LINK_LIBRARIES "z;log;dl;m")
set_property(TARGET opencv_flann PROPERTY INTERFACE_LINK_LIBRARIES opencv_core)
set_property(TARGET opencv_geometry PROPERTY INTERFACE_LINK_LIBRARIES "opencv_flann;opencv_core")
set_property(TARGET opencv_imgproc PROPERTY INTERFACE_LINK_LIBRARIES "opencv_geometry;opencv_core")
set_property(TARGET opencv_features PROPERTY INTERFACE_LINK_LIBRARIES "opencv_imgproc;opencv_geometry")
set_property(TARGET opencv_video PROPERTY INTERFACE_LINK_LIBRARIES "opencv_features;opencv_imgproc")
set(OpenCV_LIBS opencv_video opencv_features opencv_imgproc opencv_geometry opencv_flann opencv_core)

# Supply the dependency package for consumers of an installed static SDK.
install(DIRECTORY "${opencv_prefix}/" DESTINATION ".")
