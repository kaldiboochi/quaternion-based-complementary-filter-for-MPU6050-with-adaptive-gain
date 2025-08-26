#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "quaternion.h"
#include <math.h>

#define CF_TH1 0.1f
#define CF_TH2 0.2f
#define CF_GRAVITY 9.807f

/** Adaptive gain from magnitude error */
float cf_gain(float err);

/** Integrate gyro: q ← q + 0.5·dt·ω⊗q */
void cf_update_from_gyro(struct quaternion* q, float rate[3], float dt);

/** Correct with accel */
void cf_update_from_accel(struct quaternion* q, float acc[3]);

/** Combined update */
void cf_update(struct quaternion* q, float rate[3], float acc[3], float dt);

#endif // COMPLEMENTARY_FILTER_H

