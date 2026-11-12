#ifndef STILLFLOW_DESKTOP_H
#define STILLFLOW_DESKTOP_H
#include "stillflow/stillflow.h"
#define SF_DESKTOP_API SF_API
#ifdef __cplusplus
extern "C" {
#endif
typedef enum sf_audio_mode {
    SF_AUDIO_AUTO = 0, /* Copy container-compatible tracks; encode other tracks as AAC. */
    SF_AUDIO_COPY = 1, /* Fail if a track is incompatible with the output container. */
    SF_AUDIO_AAC = 2,
    SF_AUDIO_DROP = 3
} sf_audio_mode;
/* UTF-8 paths and strings are borrowed until sf_stabilize_file returns. */
typedef struct sf_file_options {
    size_t struct_size;
    sf_settings stabilization;
    const char *input, *output, *compare, *report;
    const char *scan_axis, *preset, *backend, *decoder, *encoder;
    int32_t threads, crf, audio, overwrite;
    int32_t realtime;
    double lookahead_seconds;
    size_t max_buffered_frames;
    int32_t audio_mode; /* sf_audio_mode; audio=0 always omits audio. */
} sf_file_options;
typedef struct sf_run_summary {
    size_t struct_size;
    size_t frames;
    double wall_ms, cpu_seconds, peak_rss_mib;
} sf_run_summary;
typedef enum sf_progress_stage {
    SF_PROGRESS_ANALYZE = 0,
    SF_PROGRESS_RECOVER = 1,
    SF_PROGRESS_PLAN = 2,
    SF_PROGRESS_RENDER = 3,
    SF_PROGRESS_FINALIZE = 4,
    SF_PROGRESS_COMPLETE = 5
} sf_progress_stage;
/* Offline progress. Fraction is a monotonic, stage-weighted estimate in [0,1],
   not a time estimate. Only COMPLETE reaches 1, after output/report publication.
   Analysis totals come from container metadata; zero means unknown. Recovery
   counts scanned frames. Planning/finalization report stage boundaries.
   The event is borrowed during the synchronous callback on the calling thread.
   Queue UI updates on the UI thread; do not reenter this file operation. */
typedef struct sf_progress_event {
    size_t struct_size;
    int32_t stage, stage_complete;
    size_t processed_frames, total_frames;
    int64_t processed_us, duration_us;
    double fraction;
} sf_progress_event;
typedef struct sf_progress {
    size_t struct_size;
    void* user_data;
    void (*message)(void* user_data, const char* text);
    void (*event)(void* user_data, const sf_progress_event* event);
} sf_progress;
SF_DESKTOP_API sf_status sf_file_options_default(sf_file_options*);
SF_DESKTOP_API const char* sf_desktop_build_description(void);
SF_DESKTOP_API sf_status sf_stabilize_file(const sf_file_options*, const sf_runtime*, const sf_progress*,
                                           sf_run_summary*, sf_error*);
#ifdef __cplusplus
}
#endif
#endif
