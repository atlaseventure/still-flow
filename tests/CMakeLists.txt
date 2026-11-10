include(FetchContent)
FetchContent_Declare(
    Catch2
    URL https://github.com/catchorg/Catch2/archive/refs/tags/v3.16.1.tar.gz
    URL_HASH SHA256=d16bd1b8b2364918bd472cb216394acb93bde6784890df0d91bccfb3f28d777d
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(Catch2)
add_executable(stillflow_tests domain.cpp)
target_link_libraries(stillflow_tests PRIVATE StillFlow::application Catch2::Catch2WithMain)
target_compile_definitions(stillflow_tests
                           PRIVATE STILLFLOW_GOLDEN_FILE="${CMAKE_CURRENT_SOURCE_DIR}/golden/trajectory.txt")
if(STILLFLOW_BUILD_ADAPTERS)
    target_sources(stillflow_tests PRIVATE backends.cpp)
    target_link_libraries(stillflow_tests PRIVATE StillFlow::backend_cpu)
    if(TARGET StillFlow::backend_cuda)
        target_link_libraries(stillflow_tests PRIVATE StillFlow::backend_cuda)
        target_compile_definitions(stillflow_tests PRIVATE STILLFLOW_TEST_CUDA)
    endif()
endif()
add_test(NAME regression COMMAND stillflow_tests --abort)
set_tests_properties(regression PROPERTIES TIMEOUT 120)
if(STILLFLOW_BUILD_CLI)
    find_program(UV_EXECUTABLE uv REQUIRED)
    add_test(
        NAME prepare_video
        COMMAND
            "${UV_EXECUTABLE}" run --locked --group video python "${CMAKE_CURRENT_SOURCE_DIR}/prepare_video.py" --cli
            $<TARGET_FILE:stillflow> --source "${PROJECT_SOURCE_DIR}" --output "${CMAKE_CURRENT_BINARY_DIR}/video")
    set_tests_properties(prepare_video PROPERTIES FIXTURES_SETUP video TIMEOUT 180 WORKING_DIRECTORY
                                                  "${PROJECT_SOURCE_DIR}")
    add_executable(stillflow_video_tests video.cpp)
    target_link_libraries(stillflow_video_tests PRIVATE Catch2::Catch2WithMain)
    target_compile_definitions(stillflow_video_tests
                               PRIVATE STILLFLOW_VIDEO_METRICS="${CMAKE_CURRENT_BINARY_DIR}/video/metrics.txt")
    add_test(NAME video_regression COMMAND stillflow_video_tests --abort)
    set_tests_properties(video_regression PROPERTIES FIXTURES_REQUIRED video TIMEOUT 30)
endif()
