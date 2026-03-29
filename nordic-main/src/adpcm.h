/*
 * ADPCM Codec for Voice Bridge BLE
 * IMA ADPCM 4-bit encoding/decoding
 */

#ifndef ADPCM_H
#define ADPCM_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ADPCM state structure */
typedef struct {
    int32_t predictor;
    int32_t step_size;
} adpcm_state_t;

/* Initialize ADPCM state */
void adpcm_init_state(adpcm_state_t *state);

/* Encode 16-bit PCM to 4-bit ADPCM */
size_t adpcm_encode(adpcm_state_t *state, const int16_t *input,
                    size_t num_samples, uint8_t *output);

/* Decode 4-bit ADPCM to 16-bit PCM */
size_t adpcm_decode(adpcm_state_t *state, const uint8_t *input,
                    size_t num_bytes, int16_t *output);

#ifdef __cplusplus
}
#endif

#endif /* ADPCM_H */
