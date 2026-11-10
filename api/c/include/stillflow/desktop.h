#ifndef STILLFLOW_DESKTOP_H
#define STILLFLOW_DESKTOP_H
#include "stillflow/stillflow.h"
#define SF_DESKTOP_API SF_API
#ifdef __cplusplus
extern "C" {
#endif
/* UTF-8 paths and strings are borrowed until sf_stabilize_file returns. */
typedef struct sf_file_options {
    size_t struct_size;
    sf_settings stabilization;
    const char *input, *output, *compare, *report;
    const char *scan_axis, *preset, *backend, *decoder, *encoder;
    int32_t threads, crf, audio, overwrite;
} sf_file_options;
typedef struct sf_run_summary {
    size_t struct_size;
    size_t frames;
    double wall_ms, cpu_seconds, peak_rss_mib;
} sf_run_summary;
typedef struct sf_progress {
    size_t struct_size;
    void* user_data;
    void (*message)(void* user_data, const char* text);
} sf_progress;
SF_DESKTOP_API sf_status sf_file_options_default(sf_file_options*);
SF_DESKTOP_API const char* sf_desktop_build_description(void);
SF_DESKTOP_API sf_status sf_stabilize_file(const sf_file_options*, const sf_runtime*, const sf_progress*,
                                           sf_run_summary*, sf_error*);
#ifdef __cplusplus
}
#endif
#endif
