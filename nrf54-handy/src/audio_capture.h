/*
 * Audio Capture Module for Voice Bridge BLE
 * PDM Microphone interface using Zephyr Audio Subsystem
 */

#ifndef AUDIO_CAPTURE_H
#define AUDIO_CAPTURE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize audio capture from PDM microphone
 *
 * @return 0 on success, negative errno on failure
 */
int audio_capture_init(void);

/**
 * Start audio capture
 *
 * @return 0 on success, negative errno on failure
 */
int audio_capture_start(void);

/**
 * Stop audio capture
 *
 * @return 0 on success, negative errno on failure
 */
int audio_capture_stop(void);

/**
 * Get audio data from capture buffer
 *
 * @param buffer Pointer to receive buffer pointer
 * @param size Pointer to receive data size
 * @return 0 on success, negative errno on failure
 */
int audio_capture_get_data(int16_t **buffer, size_t *size);

/**
 * Get frame size in bytes
 *
 * @return Frame size
 */
size_t audio_capture_get_frame_size(void);

/**
 * Get sample rate
 *
 * @return Sample rate in Hz
 */
uint32_t audio_capture_get_sample_rate(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_CAPTURE_H */
