add_library(stillflow_domain STATIC domain/src/geometry.cpp domain/src/motion.cpp domain/src/trajectory.cpp
                                    domain/src/crop.cpp)
add_library(StillFlow::domain ALIAS stillflow_domain)
set_target_properties(stillflow_domain PROPERTIES EXPORT_NAME domain POSITION_INDEPENDENT_CODE ON)
target_compile_features(stillflow_domain PUBLIC cxx_std_20)
target_include_directories(stillflow_domain PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/domain/include>
                                                   $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_definitions(stillflow_domain PRIVATE EIGEN_MPL2_ONLY)
target_link_libraries(stillflow_domain PRIVATE $<BUILD_INTERFACE:Eigen3::Eigen>)

add_library(
    stillflow_application STATIC
    application/src/engine.cpp
    application/src/stream.cpp
    application/src/recovery.cpp
    application/src/proxy.cpp
    application/src/image.cpp
    application/src/rendering.cpp
    application/src/options.cpp
    application/src/stabilize.cpp
    application/src/runtime.cpp)
add_library(StillFlow::application ALIAS stillflow_application)
set_target_properties(stillflow_application PROPERTIES EXPORT_NAME application POSITION_INDEPENDENT_CODE ON)
target_compile_features(stillflow_application PUBLIC cxx_std_20)
target_include_directories(
    stillflow_application PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/application/include>
                                 $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
find_package(Threads REQUIRED)
target_link_libraries(
    stillflow_application
    PUBLIC stillflow_domain
    PRIVATE Threads::Threads)

foreach(target stillflow_domain stillflow_application)
    target_compile_options(${target} PRIVATE $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
endforeach()
install(
    TARGETS stillflow_domain stillflow_application
    EXPORT StillFlowTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY domain/include/ application/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

add_subdirectory(vision)
add_subdirectory(backends/cpu)
if(ANDROID)
    add_subdirectory(backends/gles)
endif()
if(STILLFLOW_BUILD_ADAPTERS)
    add_library(stillflow_interop INTERFACE)
    add_library(StillFlow::interop ALIAS stillflow_interop)
    target_include_directories(stillflow_interop INTERFACE interop/include)
    target_link_libraries(stillflow_interop INTERFACE StillFlow::application)
    add_subdirectory(adapters)
    add_subdirectory(backends)
endif()
