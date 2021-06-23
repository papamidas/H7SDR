/*
 * fftwindows.c
 *
 *  Created on: 02.04.2021
 *      Author: papamidas DM1CR
 */

#include "fftwindows.h"

void bartlett_f32( float32_t * w, uint16_t winLen )
{
	float32_t M = winLen;
	for( int n=0; n<winLen; n++) {
		w[n] = 2.0/(M-1)*((M-1)/2.0-fabsf(n - (M-1)/2.0));
	}
}

void blackman_f32( float32_t * w, uint16_t winLen )
{
	float M = winLen;
	for( int n=0; n<winLen; n++) {
        w[n] = 0.42 - 0.5 * cosf(2.0 * M_PI * n/M) + 0.08 * cosf(4 * M_PI * n/M);
	}
}

void hamming_f32( float32_t * w, uint16_t winLen )
{
	float M = winLen;
	for( int n=0; n<winLen; n++) {
        w[n] = 0.54 - 0.46 * cosf(2.0 * M_PI * n/(M-1));
	}
}

void vonHann_f32( float32_t * w, uint16_t winLen )
{
	float M = winLen;
	for( int n=0; n<winLen; n++) {
        w[n] = 0.5 - 0.5 * cosf(2.0 * M_PI * n/(M-1));
	}
}

// for computation of the Kaiser windos see, e.g.
// https://www.dsprelated.com/freebooks/sasp/Kaiser_Window.html

static float32_t besselI0_f32( float32_t x )
{
	 const int km = 25; // probably has to be increased for high beta values?
	 float64_t sigma = 0.0;
	 for (int k=1; k<km; k++) {
	     float64_t kfac = 1.0;
	     for(int n=1; n<=k; n++) {
	         kfac *= n;
	     }
	     float64_t x2powk = 1.0;
	     for(int n=0; n<k; n++) {
	         x2powk *= x/2.0;
	     }
	     sigma += x2powk*x2powk/kfac/kfac;
	 }
	 return((float32_t)sigma + 1.0);
}

void kaiser_f32( float32_t * w, uint16_t beta, uint16_t winLen )
{
 	float M = winLen;
 	for( int n=0; n<winLen; n++) {
        float32_t arg = beta * sqrtf(1 - ((n-M/2)/(M/2)) * ((n-M/2)/(M/2)));
        w[n] = besselI0_f32(arg)/besselI0_f32(beta);
 	}
}

