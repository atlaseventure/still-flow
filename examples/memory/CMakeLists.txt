add_executable(stillflow_memory main.c)
target_link_libraries(stillflow_memory PRIVATE StillFlow::c)
set_target_properties(stillflow_memory PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON)
