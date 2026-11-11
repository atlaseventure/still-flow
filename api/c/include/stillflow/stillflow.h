#ifndef STILLFLOW_H
#define STILLFLOW_H
#include <stddef.h>
#include <stdint.h>
#define SF_API __attribute__((visibility("default")))
#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t sf_status;
enum {
    SF_OK = 0,
    SF_INVALID_ARGUMENT = 1,
    SF_INVALID_STATE = 2,
    SF_CANCELLED = 3,
    SF_OUT_OF_MEMORY = 4,
    SF_UNSUPPORTED = 5,
    SF_INTERNAL_ERROR = 6
};
enum { SF_GRAY8 = 0, SF_BGR8 = 1, SF_RGBA8 = 2, SF_NV12 = 3, SF_NV21 = 4, SF_I420 = 5 };
enum { SF_CROP_AUTO = 0, SF_CROP_FIXED = 1, SF_CROP_ADAPTIVE = 2 };
enum { SF_MOTION_TRANSLATION = 0, SF_MOTION_OPENCV = 1 };

typedef struct sf_analyzer sf_analyzer;
typedef struct sf_plan sf_plan;
typedef struct sf_renderer sf_renderer;
typedef struct sf_frame sf_frame;
typedef struct sf_error {
    size_t struct_size;
    char message[512];
} sf_error;
typedef struct sf_settings {
    size_t struct_size;
    int32_t analysis_width, features, bands, crop_mode, rolling_shutter, motion_estimator;
    double smooth_seconds, rs_seconds, crop, rs_strength;
} sf_settings;
typedef struct sf_video {
    size_t struct_size;
    int32_t width, height, horizontal_scan;
    double fps;
} sf_video;
typedef void (*sf_work_fn)(void* work_data, size_t index);
/* execute must call work once per index, join all workers, then return. All
   callbacks are synchronous, must not throw, and user_data outlives the handle. */
typedef struct sf_runtime {
    size_t struct_size;
    void* user_data;
    int32_t (*cancelled)(void* user_data);
    void (*log)(void* user_data, const char* message);
    void (*execute)(void* user_data, size_t count, sf_work_fn work, void* work_data);
} sf_runtime;
typedef struct sf_plane {
    size_t struct_size;
    const uint8_t* data;
    size_t size_bytes, row_stride;
} sf_plane;
typedef struct sf_image {
    size_t struct_size;
    int32_t width, height, format, plane_count;
    sf_plane planes[3];
    float kr, kb, chroma_x, chroma_y;
    int32_t full_range;
    /* Set both callbacks for a retained read lease; otherwise push copies data.
       Rendering is synchronous. Pixel contents remain immutable until release. */
    void* user_data;
    void (*retain)(void* user_data);
    void (*release)(void* user_data);
} sf_image;
typedef struct sf_plan_info {
    size_t struct_size;
    size_t frames;
    int32_t analysis_width, analysis_height;
    int64_t origin_us;
} sf_plan_info;

/* Initialize struct_size before calling defaults or passing a structure.
   Larger structures are accepted; unknown trailing fields are untouched.
   Handles are single-caller objects. Distinct handles may run concurrently. */
SF_API sf_status sf_settings_default(sf_settings* settings);
SF_API sf_status sf_image_default(sf_image* image);
SF_API const char* sf_build_description(void);
SF_API sf_status sf_analyzer_create(const sf_settings*, const sf_video*, const sf_runtime*, sf_analyzer**, sf_error*);
SF_API sf_status sf_analyzer_push(sf_analyzer*, const sf_image*, int64_t pts_us, int64_t duration_us, sf_error*);
/* Seals analysis. Replay each requested frame with its original timestamp;
   available is zero when recovery is complete. Requests increase by index.
   finish returns SF_INVALID_STATE while a recovery frame remains outstanding. */
typedef struct sf_recovery_frame {
    size_t struct_size;
    size_t index;
    int64_t pts_us;
    int32_t available;
} sf_recovery_frame;
SF_API sf_status sf_analyzer_next_recovery_frame(sf_analyzer*, sf_recovery_frame*, sf_error*);
SF_API sf_status sf_analyzer_recover(sf_analyzer*, const sf_image*, int64_t pts_us, sf_error*);
SF_API sf_status sf_analyzer_finish(sf_analyzer*, sf_plan**, sf_error*);
SF_API void sf_analyzer_destroy(sf_analyzer*);
SF_API sf_status sf_plan_get_info(const sf_plan*, sf_plan_info*, sf_error*);
SF_API void sf_plan_destroy(sf_plan*);
SF_API sf_status sf_renderer_create(const sf_runtime*, sf_renderer**, sf_error*);
/* Explicit backend selection: "cpu", "gles" (Android), or "auto".
   auto prefers the platform GPU and falls back to CPU on backend failures. */
SF_API sf_status sf_renderer_create_backend(const char* backend, const sf_runtime*, sf_renderer**, sf_error*);
typedef struct sf_renderer_info {
    size_t struct_size;
    char backend[32], device[256], fallback_reason[512];
    int32_t gpu;
} sf_renderer_info;
SF_API sf_status sf_renderer_get_info(const sf_renderer*, sf_renderer_info*, sf_error*);
/* Packed gray8/BGR8/RGBA8 render output. Feed YUV luma to analysis; convert
   chroma through a platform renderer when full-color YUV rendering is needed. */
SF_API sf_status sf_renderer_render(sf_renderer*, sf_plan*, size_t index, const sf_image*, sf_frame**, sf_error*);
SF_API void sf_renderer_destroy(sf_renderer*);
/* The returned view remains valid until sf_frame_destroy. */
SF_API sf_status sf_frame_get_image(const sf_frame*, sf_image*, sf_error*);
SF_API void sf_frame_destroy(sf_frame*);
#ifdef __cplusplus
}
#endif
#endif
