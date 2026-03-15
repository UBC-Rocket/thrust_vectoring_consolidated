#ifndef MATRIX_H
#define MATRIX_H

void normalize(float q[4]);

int inverse(float a[3][3], float inv[3][3]);

void transpose3x4_to_4x3(const float A[3][4], float AT[4][3]);

void transpose4x4(const float A[4][4], float AT[4][4]);

void transpose6x6(const float A[6][6], float AT[6][6]);

void transpose3x3(const float A[3][3], float AT[3][3]);

void transpose3x6_to_6x3(const float A[3][6], float AT[6][3]);

void transpose4x3_to_3x4(float in[4][3], float out[3][4]);

#endif