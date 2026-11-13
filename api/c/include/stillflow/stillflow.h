#ifndef STILLFLOW_H
#define STILLFLOW_H
#include <stddef.h>
#include <stdint.h>
#define SF_API __attribute__((visibility("default")))
#ifdef __cplusplus
extern "C" {
#endif

/* Use headers and binaries from the same build. Use *_default() for types that
   provide it; zero-initialize other descriptors. Required input
   and output pointers must be non-null; runtime and error pointers are optional.
   Handle outputs are null on failure/AGAIN/END. Error messages are cleared on
   every status-returning call. All destroy functions accept null.
   Handles have one caller at a time; independent handles may run concurrently.
   Immutable plans may be read concurrently. After cancellation or an internal
   failure, destroy the affected processing handle. */
typedef int32_t sf_status;
enum {
    SF_OK = 0,
    SF_AGAIN = 1, /* No input consumed/result produced; drain or provide more input. */
    SF_END = 2,   /* No further results in this phase. */
    SF_INVALID_ARGUMENT = -1,
    SF_INVALID_STATE = -2,
    SF_CANCELLED = -3,
    SF_OUT_OF_MEMORY = -4,
    SF_UNSUPPORTED = -5,
    SF_INTERNAL_ERROR = -6
};
typedef int32_t sf_pixel_format;
enum { SF_GRAY8 = 0, SF_BGR8 = 1, SF_RGBA8 = 2, SF_NV12 = 3, SF_NV21 = 4, SF_I420 = 5 };
typedef int32_t sf_crop_mode;
enum { SF_CROP_AUTO = 0, SF_CROP_FIXED = 1, SF_CROP_ADAPTIVE = 2 };
typedef int32_t sf_motion_estimator;
enum { SF_MOTION_TRANSLATION = 0, SF_MOTION_OPENCV = 1 };
typedef int32_t sf_backend;
enum { SF_BACKEND_AUTO = 0, SF_BACKEND_CPU = 1, SF_BACKEND_GLES = 2, SF_BACKEND_CUDA = 3 };

typedef struct sf_analyzer sf_analyzer;
typedef struct sf_plan sf_plan;
typedef struct sf_frame_plan sf_frame_plan;
typedef struct sf_stream sf_stream;
typedef struct sf_renderer sf_renderer;
typedef struct sf_frame sf_frame;
typedef struct sf_error {
    char message[512];
} sf_error;
typedef struct sf_settings {
    int32_t analysis_width, features, bands;
    sf_crop_mode crop_mode;
    int32_t rolling_shutter; /* Boolean fields accept only 0 or 1. */
    sf_motion_estimator motion_estimator;
    double smooth_seconds, rs_seconds, crop, rs_strength;
} sf_settings;
typedef struct sf_video {
    int32_t width, height, horizontal_scan;
    double fps;
} sf_video;
typedef void (*sf_work_fn)(void* work_data, size_t index);
/* execute calls work exactly once per index and joins before returning.
   Callbacks must not throw or reenter their processing handle. user_data lives
   until the handle is destroyed (or the file call returns). Cancellation and
   logging callbacks may run on worker threads and must be thread-safe. */
typedef struct sf_runtime {
    void* user_data;
    int32_t (*cancelled)(void* user_data);
    void (*log)(void* user_data, const char* message);
    void (*execute)(void* user_data, size_t count, sf_work_fn work, void* work_data);
} sf_runtime;
typedef struct sf_plane {
    const uint8_t* data;
    size_t size_bytes, row_stride;
} sf_plane;
/* YUV matrix coefficients, chroma sample position in luma pixels, and range. */
typedef struct sf_color_info {
    float kr, kb, chroma_x, chroma_y;
    int32_t full_range;
} sf_color_info;
typedef struct sf_image {
    int32_t width, height;
    sf_pixel_format format;
    int32_t plane_count;
    sf_plane planes[3];
    sf_color_info color;
    /* Both callbacks retain read-only storage until release; with neither,
       input is copied as needed. Input pixels remain unchanged during the call
       and, for retained storage, until release. Descriptor storage is borrowed
       only during the call. Output image views borrow their sf_frame's storage. */
    void* user_data;
    void (*retain)(void* user_data);
    void (*release)(void* user_data);
} sf_image;
typedef struct sf_plan_info {
    size_t frames;
    int32_t analysis_width, analysis_height;
    int64_t origin_us;
} sf_plan_info;
typedef struct sf_frame_info {
    size_t index; /* Input index among accepted frames. */
    int64_t pts_us, duration_us;
    double lookahead_seconds; /* Zero for offline plans. */
} sf_frame_info;
typedef struct sf_recovery_frame {
    size_t index;
    int64_t pts_us;
} sf_recovery_frame;

SF_API sf_settings sf_settings_default(void);
SF_API sf_image sf_image_default(void);
SF_API const char* sf_build_description(void);
SF_API sf_status sf_analyzer_create(const sf_settings*, const sf_video*, const sf_runtime*, sf_analyzer**, sf_error*);
/* Frames are full-sized, or gray proxies at the configured analysis dimensions.
   Presentation timestamps must increase; durations are nonnegative microseconds. */
SF_API sf_status sf_analyzer_push(sf_analyzer*, const sf_image*, int64_t pts_us, int64_t duration_us, sf_error*);
/* First call seals input. OK returns a replay request; END means recovery is
   complete. Replay requests in order with their original timestamps. */
SF_API sf_status sf_analyzer_next_recovery_frame(sf_analyzer*, sf_recovery_frame*, sf_error*);
SF_API sf_status sf_analyzer_recover(sf_analyzer*, const sf_image*, int64_t pts_us, sf_error*);
/* Requires recovery to be complete; consumes the analyzer's collected data. */
SF_API sf_status sf_analyzer_finish(sf_analyzer*, sf_plan**, sf_error*);
SF_API void sf_analyzer_destroy(sf_analyzer*);
SF_API sf_status sf_plan_get_info(const sf_plan*, sf_plan_info*, sf_error*);
/* Creates an immutable, independently owned frame plan, valid after its parent
   plan/analyzer/stream is destroyed. The renderer never changes it. */
SF_API sf_status sf_plan_get_frame(const sf_plan*, size_t index, sf_frame_plan**, sf_error*);
SF_API void sf_plan_destroy(sf_plan*);
SF_API sf_status sf_frame_plan_get_info(const sf_frame_plan*, sf_frame_info*, sf_error*);
#define SF_MAP_MAX_VERTICES 288
/* Output-resolution inverse map; offsets contain columns*rows interleaved x/y pairs. */
typedef struct sf_map {
    int32_t columns, rows;
    double origin_x, origin_y, step_x, step_y, grid_x, grid_y;
    double offsets[2 * SF_MAP_MAX_VERTICES];
} sf_map;
SF_API sf_status sf_frame_plan_get_map(const sf_frame_plan*, int32_t width, int32_t height, sf_map*, sf_error*);
SF_API void sf_frame_plan_destroy(sf_frame_plan*);

typedef struct sf_stream_settings {
    double lookahead_seconds;
    size_t max_buffered_frames;
    int32_t adaptive_tracking;
} sf_stream_settings;
SF_API sf_stream_settings sf_stream_settings_default(void);
SF_API sf_status sf_stream_create(const sf_settings*, const sf_stream_settings*, const sf_video*, const sf_runtime*,
                                  sf_stream**, sf_error*);
/* OK accepts one frame; AGAIN means full and consumes nothing. Drain after
   pushes, then retry the same frame or drop it. The host retains full-sized
   frames until their frame plan arrives. Capacity/shot boundaries can shorten
   lookahead. The library retains only bounded analysis history and statistics. */
SF_API sf_status sf_stream_push(sf_stream*, const sf_image*, int64_t pts_us, int64_t duration_us, sf_error*);
/* Seals input; call take_ready until END to drain. Repeated finish is harmless. */
SF_API sf_status sf_stream_finish(sf_stream*, sf_error*);
/* OK returns an owned frame plan, AGAIN needs more input, END is fully drained. */
SF_API sf_status sf_stream_take_ready(sf_stream*, sf_frame_plan**, sf_error*);
typedef struct sf_profile_entry {
    const char* stage; /* Borrowed for the duration of the visitor callback. */
    uint64_t count;
    double total_ms, min_ms, max_ms;
    double p50_ms, p95_ms, p99_ms;
} sf_profile_entry;
typedef void (*sf_profile_visitor)(void* user_data, const sf_profile_entry* entry);
/* Visits cumulative aggregates on the calling thread; storage is bounded by
   stage count. The visitor must not reenter the stream. */
SF_API sf_status sf_stream_get_profile(const sf_stream*, sf_profile_visitor, void* user_data, sf_error*);
SF_API void sf_stream_destroy(sf_stream*);
/* Process-wide; call before starting analysis workers. */
SF_API sf_status sf_opencv_set_threads(int32_t threads, sf_error*);

/* AUTO selects a compiled platform backend and falls back to CPU. Explicit
   unsupported backends return SF_UNSUPPORTED; invalid selectors return INVALID_ARGUMENT. */
SF_API sf_status sf_renderer_create(sf_backend, const sf_runtime*, sf_renderer**, sf_error*);
typedef struct sf_renderer_info {
    sf_backend backend;
    char device[256], fallback_reason[512];
    int32_t gpu;
} sf_renderer_info;
SF_API sf_status sf_renderer_get_info(const sf_renderer*, sf_renderer_info*, sf_error*);
/* Input dimensions match sf_video. Output retains the input format and frame
   plan timing. CPU supports gray8/BGR8/RGBA8 and NV12/NV21/I420 planes. */
SF_API sf_status sf_renderer_render(sf_renderer*, const sf_frame_plan*, const sf_image*, sf_frame**, sf_error*);
SF_API void sf_renderer_destroy(sf_renderer*);
SF_API sf_status sf_frame_get_info(const sf_frame*, sf_frame_info*, sf_error*);
/* View remains valid until sf_frame_destroy, independent of renderer/plan lifetime. */
SF_API sf_status sf_frame_get_image(const sf_frame*, sf_image*, sf_error*);
SF_API void sf_frame_destroy(sf_frame*);
#ifdef __cplusplus
}
#endif
#endif
