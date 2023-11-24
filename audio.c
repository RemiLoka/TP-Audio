/*
 * audio_processing.c
 *
 *  Created on: May 17, 2021
 *      Author: sydxrey
 *
 *
 * === Audio latency ===
 *
 * Receive DMA copies audio samples from the CODEC to the DMA buffer (through the I2S serial bus) as interleaved stereo samples
 * (either slots 0 and 2 for the violet on-board "LineIn" connector, or slots 1 and 3, for the pair of on-boards microphones).
 *
 * Transmit DMA copies audio samples from the DMA buffer to the CODEC again as interleaved stereo samples (in the present
 * implementation only copying to the headphone output, that is, to slots 0 and 2, is available).
 *
 * For both input and output transfers, audio double-buffering is simply implemented by using
 * a large (receive or transmit) buffer of size AUDIO_DMA_BUF_SIZE
 * and implementing half-buffer and full-buffer DMA callbacks:
 * - HAL_SAI_RxHalfCpltCallback() gets called whenever the receive (=input) buffer is half-full
 * - HAL_SAI_RxCpltCallback() gets called whenever the receive (=input) buffer is full
 *
 * As a result, one audio frame has a size of AUDIO_DMA_BUF_SIZE/2. But since one audio frame
 * contains interleaved L and R stereo samples, its true duration is AUDIO_DMA_BUF_SIZE/4.
 *
 * Example:
 * 		AUDIO_BUF_SIZE = 512 (=size of a stereo audio frame)
 * 		AUDIO_DMA_BUF_SIZE = 1024 (=size of the whole DMA buffer)
 * 		The duration of ONE audio frame is given by AUDIO_BUF_SIZE/2 = 256 samples, that is, 5.3ms at 48kHz.
 *
 * === interprocess communication ===
 *
 *  Communication b/w DMA IRQ Handlers and the main audio loop is carried out
 *  using the "audio_rec_buffer_state" global variable (using the input buffer instead of the output
 *  buffer is a matter of pure convenience, as both are filled at the same pace anyway).
 *
 *  This variable can take on three possible values:
 *  - BUFFER_OFFSET_NONE: initial buffer state at start-up, or buffer has just been transferred to/from DMA
 *  - BUFFER_OFFSET_HALF: first-half of the DMA buffer has just been filled
 *  - BUFFER_OFFSET_FULL: second-half of the DMA buffer has just been filled
 *
 *  The variable is written by HAL_SAI_RxHalfCpltCallback() and HAL_SAI_RxCpltCallback() audio in DMA transfer callbacks.
 *  It is read inside the main audio loop (see audioLoop()).
 *
 *  If RTOS is to used, Signals may be used to communicate between the DMA IRQ Handler and the main audio loop audioloop().
 *
 */

#include <audio.h>
#include <ui.h>
#include <stdio.h>
#include "string.h"
#include <math.h>
#include "cmsis_os.h"
#include "arm_math.h"
#include "bsp/disco_sai.h"
#include "bsp/disco_base.h"

extern SAI_HandleTypeDef hsai_BlockA2; // see main.c
extern SAI_HandleTypeDef hsai_BlockB2;
extern DMA_HandleTypeDef hdma_sai2_a;
extern DMA_HandleTypeDef hdma_sai2_b;

extern osThreadId defaultTaskHandle;
extern osThreadId uiTaskHandle;

// ---------- communication b/w DMA IRQ Handlers and the audio loop -------------

typedef enum {
	BUFFER_OFFSET_NONE = 0, BUFFER_OFFSET_HALF = 1, BUFFER_OFFSET_FULL = 2,
} BUFFER_StateTypeDef;
uint32_t audio_rec_buffer_state;




// ---------- DMA buffers ------------

// whole sample count in an audio frame: (beware: as they are interleaved stereo samples, true audio frame duration is given by AUDIO_BUF_SIZE/2)
#define AUDIO_BUF_SIZE   ((uint32_t)256)
/* size of a full DMA buffer made up of two half-buffers (aka double-buffering) */
#define AUDIO_DMA_BUF_SIZE   (2 * AUDIO_BUF_SIZE)

#define FFT_Length (AUDIO_BUF_SIZE / 2)

#define DELAY_BUFFER_SIZE 44100 // Pour un delay d'une seconde à 44.1kHz

#define WINDOW_SIZE 500

float sampleSum = 0.0f;
float absValue = 0.0f;


// DMA buffers are in embedded RAM:
int16_t buf_input[AUDIO_DMA_BUF_SIZE];
int16_t buf_output[AUDIO_DMA_BUF_SIZE];
int16_t *buf_input_half = buf_input + AUDIO_DMA_BUF_SIZE / 2;
int16_t *buf_output_half = buf_output + AUDIO_DMA_BUF_SIZE / 2;

int16_t delayBuffer[DELAY_BUFFER_SIZE];
int delayIndex = 0;

// ------------- scratch float buffer for long delays, reverbs or long impulse response FIR based on float implementations ---------

uint32_t scratch_offset = 0; // see doc in processAudio()
#define AUDIO_SCRATCH_SIZE   AUDIO_SCRATCH_MAXSZ_WORDS

//FFT
arm_rfft_fast_instance_f32 FFT_struct;
float32_t aFFT_Output_f32[FFT_Length];
float32_t aFFT_Input_f32[FFT_Length];


// ------------ Private Function Prototypes ------------

static void processAudio(int16_t*, int16_t*);
static void accumulateInputLevels();
static float readFloatFromSDRAM(int pos);
static void writeFloatToSDRAM(float val, int pos);
static void applyDelay(int16_t *in, int16_t *out, int delayTime, float feedback, float wetMix, float dryMix);
static void attenuation (int16_t *in, int16_t *out, float seuil_dB, float attenuation_dB);

void initializeNoiseGate(NoiseGate *gate, float threshold, float attenuation, float attackTimeMs, float releaseTimeMs, float holdTimeMs, int sampleRate);
void processNoiseGate(NoiseGate *gate, int16_t *in, int16_t *out);


// ----------- Local vars ------------

static int count = 0; // debug
double inputLevelL = 0;
double inputLevelR = 0;
double inputLevelL_cp = 0;
double inputLevelR_cp = 0;

// ----------- Functions ------------

/**
 * This is the main audio loop (aka infinite while loop) which is responsible for real time audio processing tasks:
 * - transferring recorded audio from the DMA buffer to buf_input[]
 * - processing audio samples and writing them to buf_output[]
 * - transferring processed samples back to the DMA buffer
 */


void audioLoop() {

	//uiDisplayBasic();

	arm_rfft_fast_init_f32(&FFT_struct, FFT_Length);

	/* Initialize SDRAM buffers */
	memset((int16_t*) AUDIO_SCRATCH_ADDR, 0, AUDIO_SCRATCH_SIZE * 2); // note that the size argument here always refers to bytes whatever the data type

	//audio_rec_buffer_state = BUFFER_OFFSET_NONE;

	// start SAI (audio) DMA transfers:
	startAudioDMA(buf_output, buf_input, AUDIO_DMA_BUF_SIZE);

	/* main audio loop */
	while (1) {

		/* calculate average input level over 20 audio frames */
		accumulateInputLevels();
		count++;
		if (count >= 20) {
			count = 0;
			inputLevelL *= 0.05;
			inputLevelR *= 0.05;

			inputLevelL_cp = inputLevelL;
			inputLevelR_cp = inputLevelR;

			//uiDisplayInputLevel(inputLevelL, inputLevelR);
			inputLevelL = 0.;
			inputLevelR = 0.;
		}
		osSignalWait(0x0001, osWaitForever);
		processAudio(buf_output, buf_input);
		calculateFFT(buf_output);

		// Permet d'attendre que la seconde trame DMA soit complètement rempli avant de procéder au process audio
		osSignalWait(0x0002, osWaitForever);
		processAudio(buf_output_half, buf_input_half);
		calculateFFT(buf_output_half);

		/* Wait until first half block has been recorded */
		//while (audio_rec_buffer_state != BUFFER_OFFSET_HALF) {
			//asm("NOP");
		//}
		//audio_rec_buffer_state = BUFFER_OFFSET_NONE;
		/*copy recorded 1st half block*/
		//processAudio(buf_output, buf_input);

		/*Wait until second half block has been recorded */
		//while (audio_rec_buffer_state != BUFFER_OFFSET_FULL) {
			//asm("NOP");
		//}
		//audio_rec_buffer_state = BUFFER_OFFSET_NONE;
		/* Copy recorded 2nd half block */
		//processAudio(buf_output_half, buf_input_half);

	}
}

void calculateFFT(int16_t *in){

	 for (int i = 0; i < FFT_Length; i++){
		 aFFT_Input_f32[i] = in[i];
	 }

	 arm_rfft_fast_f32(&FFT_struct, aFFT_Input_f32, aFFT_Output_f32, 0);
	 arm_cmplx_mag_f32(aFFT_Output_f32, aFFT_Input_f32, FFT_Length/2);
	 osSignalSet(uiTaskHandle, 0x0003);
 }




/*
 * Update input levels from the last audio frame (see global variable inputLevelL and inputLevelR).
 * Reminder: audio samples are actually interleaved L/R samples,
 * with left channel samples at even positions,
 * and right channel samples at odd positions.
 */
static void accumulateInputLevels() {

	// Left channel:
	uint32_t lvl = 0;
	for (int i = 0; i < AUDIO_DMA_BUF_SIZE; i += 2) {
		int16_t v = (int16_t) buf_input[i];
		if (v > 0)
			lvl += v;
		else
			lvl -= v;
	}
	inputLevelL += (double) lvl / AUDIO_DMA_BUF_SIZE / (1 << 15);

	// Right channel:
	lvl = 0;
	for (int i = 1; i < AUDIO_DMA_BUF_SIZE; i += 2) {
		int16_t v = (int16_t) buf_input[i];
		if (v > 0)
			lvl += v;
		else
			lvl -= v;
	}
	inputLevelR += (double) lvl / AUDIO_DMA_BUF_SIZE / (1 << 15);
	;
}

// --------------------------- Callbacks implementation ---------------------------

/**
 * Audio IN DMA Transfer complete interrupt.
 */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
	osSignalSet(defaultTaskHandle, 0x0001);
	//audio_rec_buffer_state = BUFFER_OFFSET_FULL;
	return;
}

/**
 * Audio IN DMA Half Transfer complete interrupt.
 */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
	osSignalSet(defaultTaskHandle, 0x0002);
	//audio_rec_buffer_state = BUFFER_OFFSET_HALF;
	return;
}

/* --------------------------- Audio "scratch" buffer in SDRAM ---------------------------
 *
 * The following functions allows you to use the external SDRAM as a "scratch" buffer.
 * There are around 7Mbytes of RAM available (~ 1' of stereo sound) which makes it possible to store signals
 * (either input or processed) over long periods of time for e.g. FIR filtering or long tail reverb's.
 */

/**
 * Read a 32 bit float from SDRAM at position "pos"
 */
static float readFloatFromSDRAM(int pos) {

	__IO float *pSdramAddress = (float*) AUDIO_SCRATCH_ADDR; // __IO is used to specify access to peripheral variables
	pSdramAddress += pos;
	//return *(__IO float*) pSdramAddress;
	return *pSdramAddress;

}

/**
 * Write the given 32 bit float to SDRAM at position "pos"
 */
static void writeFloatToSDRAM(float val, int pos) {

	__IO float *pSdramAddress = (float*) AUDIO_SCRATCH_ADDR;
	pSdramAddress += pos;
	//*(__IO float*) pSdramAddress = val;
	*pSdramAddress = val;


}

/**
 * Read a 16 bit integer from SDRAM at position "pos"
 */
static int16_t readInt16FromSDRAM(int pos) {

	__IO int16_t *pSdramAddress = (int16_t*) AUDIO_SCRATCH_ADDR;
	pSdramAddress += pos;
	//return *(__IO int16_t*) pSdramAddress;
	return *pSdramAddress;

}

/**
 * Write the given 16 bit integer to the SDRAM at position "pos"
 */
static void writeInt16ToSDRAM(int16_t val, int pos) {

	__IO int16_t *pSdramAddress = (int16_t*) AUDIO_SCRATCH_ADDR;
	pSdramAddress += pos;
	//*(__IO int16_t*) pSdramAddress = val;
	*pSdramAddress = val;

}

// --------------------------- AUDIO ALGORITHMS ---------------------------

/**
 * This function is called every time an audio frame
 * has been filled by the DMA, that is,  AUDIO_BUF_SIZE samples
 * have just been transferred from the CODEC
 * (keep in mind that this number represents interleaved L and R samples,
 * hence the true corresponding duration of this audio frame is AUDIO_BUF_SIZE/2 divided by the sampling frequency).
 *
 *
 */



static void applyDelay(int16_t *in, int16_t *out, int delayTime, float feedback, float wetMix, float dryMix) {
    for (int n = 0; n < AUDIO_BUF_SIZE; n += 2) {
        // Pour le canal gauche (échantillons pairs)
        int delayedSampleIndexL = (delayIndex - delayTime * 2 + DELAY_BUFFER_SIZE) % DELAY_BUFFER_SIZE;
        int16_t delayedSampleL = readInt16FromSDRAM(delayedSampleIndexL);
        int16_t newSampleL = in[n] + (delayedSampleL * feedback);
        out[n] = (int16_t)(in[n] * dryMix + delayedSampleL * wetMix);

        writeInt16ToSDRAM(newSampleL, delayIndex); // Stocker l'échantillon actuel pour le retard

        // Pour le canal droit (échantillons impairs)
        int delayedSampleIndexR = (delayIndex - delayTime * 2 + 1 + DELAY_BUFFER_SIZE) % DELAY_BUFFER_SIZE;
        int16_t delayedSampleR = readInt16FromSDRAM(delayedSampleIndexR);
        int16_t newSampleR = in[n+1] + (delayedSampleR * feedback);
        out[n+1] = (int16_t)(in[n+1] * dryMix + delayedSampleR * wetMix);

        writeInt16ToSDRAM(newSampleR, delayIndex + 1); // Stocker l'échantillon actuel pour le retard

        // Mise à jour de delayIndex pour la prochaine itération
        delayIndex = (delayIndex + 2) % DELAY_BUFFER_SIZE; // +2 car nous traitons des échantillons stéréo
    }
}

static void attenuation (int16_t *in, int16_t *out, float seuil_dB, float attenuation_dB) {

    float attenuation_factor = powf(10.0f, attenuation_dB / 20.0f);

    float seuil = powf(10.0f, seuil_dB / 20.0f);

    for (int n = 0; n < AUDIO_BUF_SIZE; n++) {

        if (fabs(in[n]) > seuil) {
            out[n] = (int16_t)(in[n] / attenuation_factor);
        } else {
            out[n] = in[n];
        }
    }
}

void initializeNoiseGate(NoiseGate *gate, float threshold, float attenuation, float attackTimeMs, float releaseTimeMs, float holdTimeMs, int sampleRate) {
	gate->threshold = threshold;
	gate->attenuation = powf(10.0f, attenuation / 20.0f);  // Conversion de dB en facteur linéaire
	gate->attackTime = (int)(attackTimeMs * sampleRate / 1000.0f);
	gate->releaseTime = (int)(releaseTimeMs * sampleRate / 1000.0f);
	gate->holdTime = (int)(holdTimeMs * sampleRate / 1000.0f);
	gate->sampleRate = sampleRate;
	gate->counter = 0;
	gate->state = GATE_CLOSED;
}


void processNoiseGate(NoiseGate *gate, int16_t *in, int16_t *out) {
    float energySum = 0.0f;
    float energyThreshold = gate->threshold * gate->threshold; // Threshold based on energy

    for (int n = 0; n < AUDIO_BUF_SIZE; n++) {
        // Calculate the energy of the current sample
        float currentEnergy = (float)in[n] * (float)in[n];
        energySum += currentEnergy;

        // Calculate the average energy over the window size
        if (n >= WINDOW_SIZE) {
            float avgEnergy = energySum / WINDOW_SIZE;
            energySum -= (float)in[n - WINDOW_SIZE] * (float)in[n - WINDOW_SIZE]; // Remove oldest energy

            switch (gate->state) {
                case GATE_CLOSED:
                    if (avgEnergy > energyThreshold) {
                        gate->state = GATE_OPENING;
                        gate->counter = gate->attackTime;
                    }
                    out[n] = (int16_t)(in[n] * gate->attenuation);
                    break;

                case GATE_OPENING:
                    if (gate->counter > 0) {
                        float linearGain = 1.0f - ((float)gate->counter / gate->attackTime) * (1.0f - gate->attenuation);
                        out[n] = (int16_t)(in[n] * linearGain);
                        gate->counter--;
                    } else {
                        gate->state = GATE_OPEN;
                        gate->counter = gate->holdTime;
                        out[n] = in[n];
                    }
                    break;

                case GATE_OPEN:
                    if (avgEnergy < energyThreshold) {
                        if (gate->counter > 0) {
                            gate->counter--;
                        } else {
                            gate->state = GATE_CLOSING;
                            gate->counter = gate->releaseTime;
                        }
                    }
                    out[n] = in[n];
                    break;

                case GATE_CLOSING:
                    if (gate->counter > 0) {
                        float linearGain = gate->attenuation + ((float)gate->counter / gate->releaseTime) * (1.0f - gate->attenuation);
                        out[n] = (int16_t)(in[n] * linearGain);
                        gate->counter--;
                    } else {
                        gate->state = GATE_CLOSED;
                        out[n] = (int16_t)(in[n] * gate->attenuation);
                    }
                    break;
            }
        } else {
            // Default processing for first few samples
            out[n] = in[n];
        }
    }
}



static void processAudio(int16_t *out, int16_t *in) {

    LED_On(); // for oscilloscope measurements...

    //for (int n = 0; n < AUDIO_BUF_SIZE; n++) out[n] = in[n] ;  //entrée==>sortie

    //applyDelay(in, out, 20000,0.5,0.1,0.1); //Le Delay

    //attenuation(in,out,300,-100);  // Atténuation naive

    NoiseGate myNoiseGate;
    initializeNoiseGate(&myNoiseGate, -80.0f, -120.0f, 10.0f, 10.0f, 250.0f, 44100);
    processNoiseGate(&myNoiseGate, in, out); // noisegate

    LED_Off();
}




