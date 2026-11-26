#ifndef STILLFLOW_DESKTOP_H
#define STILLFLOW_DESKTOP_H
#include "stillflow/stillflow.h"
#define SF_DESKTOP_API SF_API
#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t sf_audio_mode;
enum {
    SF_AUDIO_AUTO = 0, /* Copy compatible tracks; encode other tracks as AAC. */
    SF_AUDIO_COPY = 1, /* Fail on container-incompatible tracks. */
    SF_AUDIO_AAC = 2,
    SF_AUDIO_DROP = 3
};
typedef int32_t sf_file_mode;
enum { SF_FILE_OFFLINE = 0, SF_FILE_STREAM = 1 };
typedef int32_t sf_scan_axis;
enum { SF_SCAN_AUTO = 0, SF_SCAN_X = 1, SF_SCAN_Y = 2 };
typedef int32_t sf_decoder;
enum { SF_DECODER_AUTO = 0, SF_DECODER_CPU = 1, SF_DECODER_NVDEC = 2 };
typedef int32_t sf_encoder;
enum { SF_ENCODER_AUTO = 0, SF_ENCODER_OPENH264 = 1, SF_ENCODER_NVENC = 2 };
typedef struct sf_gyro_options {
    const char* config;                  /* stabilization.json or .gyroflow. Null selects visual stabilization. */
    const char* imu;                     /* Optional camera-frame IMU CSV override. */
    int32_t output_width, output_height; /* Both zero to use the configured dimensions. */
    double smoothness;                   /* 0..1, negative uses configuration. */
    double max_zoom;                     /* Ratio or percentage (>10), nonpositive uses configuration. */
    double zoom_window;                  /* Seconds, zero for static zoom, negative uses configuration. */
    double fov;                          /* Positive multiplier, nonpositive uses configuration. */
    double lens_correction;              /* 0..1, negative uses configuration. */
    double readout_ms;                   /* Negative uses configuration. */
    double time_offset_ms;               /* Added to camera-frame sample times. */
    sf_imu_integration integration;
} sf_gyro_options;
/* UTF-8 strings are borrowed until sf_stabilize_file returns. Optional paths
   may be null. Use the default function, then set input and output. */
typedef struct sf_file_options {
    sf_settings stabilization;
    const char *input, *output, *compare, *report;
    sf_file_mode mode;
    sf_stream_settings stream;
    sf_backend backend;
    sf_decoder decoder;
    sf_encoder encoder;
    sf_scan_axis scan_axis;
    sf_audio_mode audio;
    int32_t threads, crf, overwrite;
    const char* preset; /* NVENC speed name; ignored by OpenH264. */
    sf_gyro_options gyro;
} sf_file_options;
typedef struct sf_run_summary {
    size_t frames;
    double wall_ms, cpu_seconds, peak_rss_mib;
} sf_run_summary;
typedef int32_t sf_progress_stage;
enum {
    SF_PROGRESS_ANALYZE = 0,
    SF_PROGRESS_RECOVER = 1,
    SF_PROGRESS_PLAN = 2,
    SF_PROGRESS_RENDER = 3,
    SF_PROGRESS_FINALIZE = 4,
    SF_PROGRESS_COMPLETE = 5
};
/* Offline progress. Fraction is a monotonic stage-weighted work estimate.
   Only COMPLETE reaches 1, after all output/report publication. Analysis totals
   come from metadata; zero means unknown. Recovery counts scanned frames.
   Planning/finalization report boundaries. The event is borrowed during the
   callback, called synchronously on the sf_stabilize_file calling thread. */
typedef struct sf_progress_event {
    sf_progress_stage stage;
    int32_t stage_complete;
    size_t processed_frames, total_frames;
    int64_t processed_us, duration_us;
    double fraction;
} sf_progress_event;
/* Optional, zero-initializable. All callbacks use runtime.user_data. Log
   messages use runtime.log; progress is structured and independent of logging. */
typedef struct sf_file_callbacks {
    sf_runtime runtime;
    void (*progress)(void* user_data, const sf_progress_event* event);
} sf_file_callbacks;
SF_DESKTOP_API sf_file_options sf_file_options_default(void);
SF_DESKTOP_API const char* sf_desktop_build_description(void);
/* Blocking worker-thread operation. On failure summary is zero and no COMPLETE
   event is sent; error describes the failure. */
SF_DESKTOP_API sf_status sf_stabilize_file(const sf_file_options*, const sf_file_callbacks*, sf_run_summary*,
                                           sf_error*);
#ifdef __cplusplus
}
#endif
#endif
