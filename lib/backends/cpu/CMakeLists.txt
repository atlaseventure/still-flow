add_library(stillflow_backend_cpu STATIC src/renderer.cpp)
add_library(StillFlow::backend_cpu ALIAS stillflow_backend_cpu)
set_target_properties(stillflow_backend_cpu PROPERTIES EXPORT_NAME backend_cpu POSITION_INDEPENDENT_CODE ON)
target_include_directories(stillflow_backend_cpu PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(stillflow_backend_cpu PUBLIC StillFlow::application)
install(
    TARGETS stillflow_backend_cpu
    EXPORT StillFlowTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
