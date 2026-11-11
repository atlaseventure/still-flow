#include "stillflow/stillflow.h"
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    sf_settings settings = {0};
    settings.struct_size = sizeof(settings);
    sf_image image = {0};
    image.struct_size = sizeof(image);
    sf_error error = {0};
    error.struct_size = sizeof(error);
    sf_video video = {sizeof(video), 64, 64, 0, 30.0};
    sf_analyzer* analyzer = NULL;
    sf_plan* plan = NULL;
    sf_renderer* renderer = NULL;
    sf_frame* frame = NULL;
    int result = EXIT_FAILURE;
    uint8_t pixels[64 * 64];
    uint32_t state = 42;
    for (size_t i = 0; i < sizeof(pixels); ++i) {
        state = state * 1664525u + 1013904223u;
        pixels[i] = (uint8_t)(state >> 24);
    }
    if (sf_settings_default(&settings) != SF_OK || sf_image_default(&image) != SF_OK)
        goto done;
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
        sf_recovery_frame request = {sizeof(request), 0, 0, 0};
        if (sf_analyzer_next_recovery_frame(analyzer, &request, &error) != SF_OK)
            goto done;
        if (!request.available)
            break;
        /* All frames in this example use the same pixels. */
        if (sf_analyzer_recover(analyzer, &image, request.pts_us, &error) != SF_OK)
            goto done;
    }
    if (sf_analyzer_finish(analyzer, &plan, &error) != SF_OK)
        goto done;
    if (sf_renderer_create(NULL, &renderer, &error) != SF_OK)
        goto done;
    for (size_t i = 0; i < 8; ++i) {
        sf_image output = {0};
        output.struct_size = sizeof(output);
        if (sf_renderer_render(renderer, plan, i, &image, &frame, &error) != SF_OK)
            goto done;
        if (sf_frame_get_image(frame, &output, &error) != SF_OK || output.width != 64 || !output.planes[0].data)
            goto done;
        sf_frame_destroy(frame);
        frame = NULL;
    }
    puts("Analyzed and rendered 8 in-memory frames");
    result = EXIT_SUCCESS;
done:
    if (result != EXIT_SUCCESS)
        fputs(error.message, stderr);
    sf_frame_destroy(frame);
    sf_renderer_destroy(renderer);
    sf_plan_destroy(plan);
    sf_analyzer_destroy(analyzer);
    return result;
}
