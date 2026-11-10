if(STILLFLOW_DEPS_ROOT)
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
else()
    find_package(OpenCV 5.1 REQUIRED CONFIG COMPONENTS core imgproc video geometry features)
endif()
add_library(stillflow_vision_opencv STATIC src/motion.cpp src/opencv.cpp)
target_include_directories(stillflow_vision_opencv SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(stillflow_vision_opencv PRIVATE EIGEN_MPL2_ONLY)
target_link_libraries(
    stillflow_vision_opencv
    PUBLIC stillflow_application
    PRIVATE ${OpenCV_LIBS} $<BUILD_INTERFACE:Eigen3::Eigen>)
set_target_properties(stillflow_vision_opencv PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(StillFlow::vision_opencv ALIAS stillflow_vision_opencv)

set_target_properties(stillflow_vision_opencv PROPERTIES EXPORT_NAME vision_opencv)
install(
    TARGETS stillflow_vision_opencv
    EXPORT StillFlowTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
