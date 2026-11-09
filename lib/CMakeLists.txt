add_library(stillflow_domain STATIC domain/src/geometry.cpp domain/src/motion.cpp domain/src/trajectory.cpp)
add_library(StillFlow::domain ALIAS stillflow_domain)
set_target_properties(stillflow_domain PROPERTIES EXPORT_NAME domain POSITION_INDEPENDENT_CODE ON)
target_compile_features(stillflow_domain PUBLIC cxx_std_20)
target_include_directories(stillflow_domain PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/domain/include>
                                                   $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(stillflow_domain PRIVATE $<BUILD_INTERFACE:Eigen3::Eigen>)

add_library(stillflow_application STATIC application/src/stabilize.cpp application/src/options.cpp
                                         application/src/image.cpp application/src/rendering.cpp)
add_library(StillFlow::application ALIAS stillflow_application)
set_target_properties(stillflow_application PROPERTIES EXPORT_NAME application POSITION_INDEPENDENT_CODE ON)
target_compile_features(stillflow_application PUBLIC cxx_std_20)
target_include_directories(
    stillflow_application PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/application/include>
                                 $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
find_package(Threads REQUIRED)
target_link_libraries(stillflow_application PUBLIC stillflow_domain Threads::Threads)

foreach(target stillflow_domain stillflow_application)
    target_compile_options(${target} PRIVATE $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
endforeach()
install(
    TARGETS stillflow_domain stillflow_application
    EXPORT StillFlowTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY domain/include/ application/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(STILLFLOW_BUILD_ADAPTERS)
    add_library(stillflow_interop INTERFACE)
    add_library(StillFlow::interop ALIAS stillflow_interop)
    target_include_directories(stillflow_interop INTERFACE interop/include)
    target_link_libraries(stillflow_interop INTERFACE StillFlow::domain)
    add_subdirectory(adapters)
    add_subdirectory(backends)
endif()
