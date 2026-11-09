add_executable(stillflow main.cpp)
target_link_libraries(stillflow PRIVATE StillFlow::adapters)
target_compile_options(stillflow PRIVATE
    $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set_target_properties(stillflow PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
endif()
install(TARGETS stillflow RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
