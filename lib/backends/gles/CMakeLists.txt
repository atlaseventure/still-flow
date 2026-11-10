add_library(stillflow_backend_gles STATIC src/renderer.cpp)
add_library(StillFlow::backend_gles ALIAS stillflow_backend_gles)
set_target_properties(stillflow_backend_gles PROPERTIES EXPORT_NAME backend_gles POSITION_INDEPENDENT_CODE ON)
target_include_directories(stillflow_backend_gles PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(
    stillflow_backend_gles
    PUBLIC StillFlow::application
    PRIVATE EGL GLESv3)
target_compile_options(stillflow_backend_gles PRIVATE -Wall -Wextra -Wpedantic)
install(
    TARGETS stillflow_backend_gles
    EXPORT StillFlowTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
