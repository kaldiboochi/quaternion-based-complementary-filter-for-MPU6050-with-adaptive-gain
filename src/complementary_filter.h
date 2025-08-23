//complementary_filter.h
#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "quaternions.h"
#include <math.h>

#define TH1 0.1f
#define TH2 0.2f

// Identity quaternion (no rotation)
static const struct quaternion identity_quaternion = { 1.0f, 0.0f, 0.0f, 0.0f };

/**
 * @brief Compute gain factor for accelerometer correction.
 * @param x Magnitude error (| ||a||−g|/g).
 * @return Interpolation factor in [0,1].
 */
float gainFactor(float x);

/**
 * @brief Prediction step: integrate gyro rates.
 * @param q     Pointer to orientation quaternion (global-to-local).
 * @param rate  Raw angular rates array of 3 floats (rad/s).
 * @param dt    Time step (s).
 */
void updateQuaternionFromRate(struct quaternion* q, float rate[3], float dt);

/**
 * @brief Correction step: use accelerometer to correct roll/pitch.
 * @param q    Pointer to orientation quaternion (global-to-local).
 * @param acc  Raw accelerometer vector array of 3 floats (m/s²).
 */
void updateQuaternionFromAcc(struct quaternion* q, float acc[3]);

#endif // COMPLEMENTARY_FILTER_H

