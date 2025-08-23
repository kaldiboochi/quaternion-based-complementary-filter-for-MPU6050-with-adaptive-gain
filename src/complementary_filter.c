// complementary_filter.c

#include "complementary_filter.h"
#include <math.h>

float gainFactor(float x) {
    if (x <= TH1) return 1.0f;
    if (x >= TH2) return 0.0f;
    return (TH2 - x) / TH1;
}

void updateQuaternionFromRate(struct quaternion* q, float rate[3], float dt) {
    struct quaternion omega, qDot, delta_q;
    pureQuat(rate, &omega);
    quatMul(&omega, q, &qDot);
    scalarMul(-0.5f * dt, &qDot, &delta_q);
    quatAdd(q, &delta_q, q);
    quatNormalise(q);
}

void updateQuaternionFromAcc(struct quaternion* q, float acc[3]) {
    struct quaternion qInv, delta_acc, A, B;
    float g_p[3];

    inverse(q, &qInv);
    quatrotate(&qInv, acc, g_p);

    delta_acc.w = sqrtf((g_p[2] + 1.0f) * 0.5f);
    float inv = 1.0f / sqrtf((g_p[2] + 1.0f) * 2.0f);
    delta_acc.x = -g_p[1] * inv;
    delta_acc.y = g_p[0] * inv;
    delta_acc.z = 0.0f;

    float eps = quatDot(&delta_acc, &identity_quaternion);
    if (eps >= 0.9f) {
        quatNormalise(&delta_acc);
    } else {
        float mag = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
        float g_ref = 9.807f;
        float em = fabsf(mag - g_ref) / g_ref;
        float alpha = gainFactor(em);
        scalarMul(1.0f - alpha, &identity_quaternion, &A);
        scalarMul(alpha, &delta_acc, &B);
        quatAdd(&A, &B, &delta_acc);
        quatNormalise(&delta_acc);
    }

    quatMul( q,&delta_acc, q);
    quatNormalise(q);
}

