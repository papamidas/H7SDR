/*
 * fftwindows.h
 *
 *  Created on: 02.04.2021
 *      Author: papamidas DM1CR
 */

#ifndef INC_FFTWINDOWS_H_
#define INC_FFTWINDOWS_H_

#include <math.h> // for M_PI constant, cosf(), sqrtf(), fabsf()
#include "arm_math.h"

void bartlett_f32( float32_t * w, uint16_t winLen );
void blackman_f32( float32_t * w, uint16_t winLen );
void hamming_f32( float32_t * w, uint16_t winLen );
void vonHann_f32( float32_t * w, uint16_t winLen );
void kaiser_f32( float32_t * w, uint16_t beta, uint16_t winLen );

#endif /* INC_FFTWINDOWS_H_ */
