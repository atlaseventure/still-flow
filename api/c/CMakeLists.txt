set(api_kind STATIC)
if(STILLFLOW_C_API_SHARED)
    set(api_kind SHARED)
endif()
add_library(stillflow_c ${api_kind} src/stillflow.cpp)
add_library(StillFlow::c ALIAS stillflow_c)
target_link_libraries(stillflow_c PRIVATE StillFlow::application StillFlow::backend_cpu)
if(TARGET StillFlow::vision_opencv)
    target_link_libraries(stillflow_c PRIVATE StillFlow::vision_opencv)
    target_compile_definitions(stillflow_c PRIVATE STILLFLOW_C_OPENCV)
endif()
set_target_properties(
    stillflow_c
    PROPERTIES EXPORT_NAME c
               VERSION ${PROJECT_VERSION}
               SOVERSION 1)
target_compile_definitions(stillflow_c PRIVATE STILLFLOW_EXPORTS STILLFLOW_VERSION="${PROJECT_VERSION}")
target_include_directories(stillflow_c PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                              $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
if(STILLFLOW_C_API_SHARED)
    target_compile_definitions(stillflow_c PUBLIC STILLFLOW_SHARED)
    if(UNIX AND NOT APPLE)
        target_link_options(stillflow_c PRIVATE "LINKER:--version-script=${CMAKE_CURRENT_SOURCE_DIR}/exports.map")
    endif()
endif()
install(
    TARGETS stillflow_c
    EXPORT StillFlowTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES include/stillflow/stillflow.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/stillflow)
if(STILLFLOW_BUILD_ADAPTERS)
    add_library(stillflow_desktop_c SHARED src/desktop.cpp)
    add_library(StillFlow::desktop_c ALIAS stillflow_desktop_c)
    target_link_libraries(
        stillflow_desktop_c
        PUBLIC stillflow_c
        PRIVATE StillFlow::adapters StillFlow::backend_cpu)
    target_compile_definitions(stillflow_desktop_c PRIVATE STILLFLOW_DESKTOP_EXPORTS)
    if(TARGET StillFlow::backend_cuda)
        target_link_libraries(stillflow_desktop_c PRIVATE StillFlow::backend_cuda)
        target_compile_definitions(stillflow_desktop_c PRIVATE STILLFLOW_C_CUDA)
    endif()
    set_target_properties(
        stillflow_desktop_c
        PROPERTIES EXPORT_NAME desktop_c
                   VERSION ${PROJECT_VERSION}
                   SOVERSION 1
                   INSTALL_RPATH "$ORIGIN")
    if(UNIX AND NOT APPLE)
        target_link_options(stillflow_desktop_c PRIVATE "LINKER:--no-undefined")
        target_link_options(stillflow_desktop_c PRIVATE
                            "LINKER:--version-script=${CMAKE_CURRENT_SOURCE_DIR}/exports.map")
    endif()
    install(
        TARGETS stillflow_desktop_c
        EXPORT StillFlowTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
    install(FILES include/stillflow/desktop.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/stillflow)
endif()
