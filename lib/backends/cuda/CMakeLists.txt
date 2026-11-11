add_library(stillflow_backend_cuda STATIC src/frames.cu src/renderer.cu src/image_processing.cu)
add_library(StillFlow::backend_cuda ALIAS stillflow_backend_cuda)
find_file(
    stillflow_cuda_license
    NAMES LICENSE EULA.txt
    HINTS "${CMAKE_CUDA_COMPILER_TOOLKIT_ROOT}"
    NO_DEFAULT_PATH NO_CACHE REQUIRED)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/../../../licenses/cuda/NOTICE.txt.in"
               "${CMAKE_CURRENT_BINARY_DIR}/NOTICE.txt" @ONLY)
install(
    FILES "${stillflow_cuda_license}"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/stillflow/cuda"
    RENAME LICENSE)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/NOTICE.txt" DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/stillflow/cuda")
set_target_properties(
    stillflow_backend_cuda
    PROPERTIES POSITION_INDEPENDENT_CODE ON
               CUDA_STANDARD 20
               CUDA_STANDARD_REQUIRED ON
               CUDA_RUNTIME_LIBRARY Static)
target_include_directories(stillflow_backend_cuda PUBLIC include)
target_link_libraries(
    stillflow_backend_cuda
    PUBLIC StillFlow::application
    PRIVATE StillFlow::interop)
target_compile_definitions(stillflow_backend_cuda PRIVATE STILLFLOW_CUDA_VERSION="${CMAKE_CUDA_COMPILER_VERSION}")
