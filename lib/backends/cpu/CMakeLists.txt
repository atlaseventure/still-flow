if(NOT TARGET Eigen3::Eigen)
    find_package(
        Eigen3
        5
        REQUIRED
        CONFIG
        PATHS
        "${STILLFLOW_DEPS_ROOT}"
        NO_DEFAULT_PATH)
endif()
find_package(OpenCV 5.1 REQUIRED CONFIG COMPONENTS core imgproc PATHS "${STILLFLOW_DEPS_ROOT}" NO_DEFAULT_PATH)
add_library(stillflow_backend_cpu STATIC src/renderer.cpp)
add_library(StillFlow::backend_cpu ALIAS stillflow_backend_cpu)
set_target_properties(stillflow_backend_cpu PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(stillflow_backend_cpu PUBLIC include)
target_include_directories(stillflow_backend_cpu SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(
    stillflow_backend_cpu
    PUBLIC StillFlow::application
    PRIVATE opencv_core opencv_imgproc)
target_compile_options(stillflow_backend_cpu PRIVATE $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
