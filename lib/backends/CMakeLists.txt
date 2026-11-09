add_subdirectory(cpu)
if(STILLFLOW_WITH_CUDA AND CMAKE_CUDA_COMPILER)
    add_subdirectory(cuda)
endif()
