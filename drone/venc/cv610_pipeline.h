/* Hi3516CV610 capture graph: MIPI -> VI -> ISP -> VPSS, ending at a VPSS
 * channel the encoder binds to. VENC itself belongs to venc_core_cv610.c.
 *
 * Derived from waybeam_venc src/cv610_pipeline.c.
 * Copyright (c) 2023 OpenIPC, MIT License. */
#ifndef MABUR_VENC_CV610_PIPELINE_H
#define MABUR_VENC_CV610_PIPELINE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CV610_VPSS_GRP 0
#define CV610_VPSS_CHN 0

typedef struct {
	uint32_t out_width, out_height; /* encoded size: VPSS scales to it */
	int mirror, flip;
} Cv610PipelineConfig;

/* The sensor's one mode: what the capture side delivers. */
uint32_t cv610_pipeline_sensor_width(void);
uint32_t cv610_pipeline_sensor_height(void);
uint32_t cv610_pipeline_sensor_fps(void);

/* Brings the whole graph up, the ISP thread included. Returns 0 or -1; on -1
 * the caller still owes cv610_pipeline_stop(). */
int cv610_pipeline_start(const Cv610PipelineConfig *config);

/* Applies a new output size and orientation to a running graph. The sensor,
 * VI and ISP keep running, so a change costs no re-convergence of 3A. The
 * encoder must be unbound from the VPSS channel for the duration. */
int cv610_pipeline_reconfigure(const Cv610PipelineConfig *config);

/* Safe after a partial start. */
void cv610_pipeline_stop(void);

#ifdef __cplusplus
}
#endif
#endif
