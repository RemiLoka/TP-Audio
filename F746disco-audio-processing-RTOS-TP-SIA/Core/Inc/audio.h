/*
 * audio_processing.h
 *
 *  Created on: May 17, 2021
 *      Author: sydxrey
 */

#ifndef INC_AUDIO_H_
#define INC_AUDIO_H_

#include "stdint.h"
#include <string.h>
#include "arm_math.h"
typedef enum {
    GATE_CLOSED,
    GATE_OPENING,
    GATE_OPEN,
    GATE_CLOSING
} GateState;

typedef struct {
    float threshold;
    float attenuation;
    int attackTime;
    int releaseTime;
    int holdTime;
    int sampleRate;
    int counter;
    GateState state;
} NoiseGate;

void audioLoop();
void calculateFFT(int16_t *buff_in);
void initializeNoiseGate(NoiseGate *gate, float threshold, float attenuation, float attackTimeMs, float releaseTimeMs, float holdTimeMs, int sampleRate);
void processNoiseGate(NoiseGate *gate, int16_t *in, int16_t *out);

#endif /* INC_AUDIO_H_ */
