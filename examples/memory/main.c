#include "stillflow/stillflow.h"
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    sf_settings settings = sf_settings_default();
    sf_image image = sf_image_default();
    sf_error error = {0};
    sf_video video = {64, 64, 0, 30.0};
    sf_analyzer* analyzer = NULL;
    sf_plan* plan = NULL;
    sf_frame_plan* frame_plan = NULL;
    sf_renderer* renderer = NULL;
    sf_frame* frame = NULL;
    int result = EXIT_FAILURE;
    uint8_t pixels[64 * 64];
    uint32_t state = 42;
    for (size_t i = 0; i < sizeof(pixels); ++i) {
        state = state * 1664525u + 1013904223u;
        pixels[i] = (uint8_t)(state >> 24);
    }
    image.width = 64;
    image.height = 64;
    image.format = SF_GRAY8;
    image.plane_count = 1;
    image.planes[0].data = pixels;
    image.planes[0].size_bytes = sizeof(pixels);
    image.planes[0].row_stride = 64;
    if (sf_analyzer_create(&settings, &video, NULL, &analyzer, &error) != SF_OK)
        goto done;
    for (int i = 0; i < 8; ++i)
        if (sf_analyzer_push(analyzer, &image, (int64_t)i * 33333, 33333, &error) != SF_OK)
            goto done;
    for (;;) {
        sf_recovery_frame request = {0};
        sf_status status = sf_analyzer_next_recovery_frame(analyzer, &request, &error);
        if (status == SF_END)
            break;
        if (status != SF_OK)
            goto done;
        /* All frames in this example use the same pixels. */
        if (sf_analyzer_recover(analyzer, &image, request.pts_us, &error) != SF_OK)
            goto done;
    }
    if (sf_analyzer_finish(analyzer, &plan, &error) != SF_OK)
        goto done;
    if (sf_renderer_create(SF_BACKEND_AUTO, NULL, &renderer, &error) != SF_OK)
        goto done;
    for (size_t i = 0; i < 8; ++i) {
        sf_image output = {0};
        if (sf_plan_get_frame(plan, i, &frame_plan, &error) != SF_OK)
            goto done;
        if (sf_renderer_render(renderer, frame_plan, &image, &frame, &error) != SF_OK)
            goto done;
        if (sf_frame_get_image(frame, &output, &error) != SF_OK || output.width != 64 || !output.planes[0].data)
            goto done;
        sf_frame_destroy(frame);
        frame = NULL;
        sf_frame_plan_destroy(frame_plan);
        frame_plan = NULL;
    }
    puts("Analyzed and rendered 8 in-memory frames");
    result = EXIT_SUCCESS;
done:
    if (result != EXIT_SUCCESS)
        fputs(error.message, stderr);
    sf_frame_destroy(frame);
    sf_frame_plan_destroy(frame_plan);
    sf_renderer_destroy(renderer);
    sf_plan_destroy(plan);
    sf_analyzer_destroy(analyzer);
    return result;
}
