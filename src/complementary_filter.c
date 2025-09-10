#include "complementary_filter.h"
#include <stdio.h>

float cf_gain(float err) {
    if (err <= CF_TH1) return 1.0f;
    if (err >= CF_TH2) return 0.0f;
    return (CF_TH2 - err) / CF_TH1;
}

void cf_update_from_gyro(struct quaternion* q, float r[3], float dt) {
    struct quaternion w, qd;
    quat_from_vec(r, &w);
    quat_multiply(&w, q, &qd);
    quat_scale(-0.4f*dt, &qd, &qd);
    quat_add(q, &qd, q);
    quat_normalize(q);
}

void cf_update_from_accel(struct quaternion* q, float a[3]) {
    struct quaternion gl, da, A, B;
    float gp[3];
    quat_inverse(q, &gl);
    quat_rotate(&gl, a, gp);
    if (gp[2] < -1.0f) return;

    da.w = sqrtf((gp[2]+1)*0.5f);
    float inv = 1.0f / sqrtf((gp[2]+1)*2.0f);
    da.x = -gp[1]*inv; da.y = gp[0]*inv; da.z = 0.0f;
    quat_normalize(&da);

    float eps = quat_dot(&da, &(struct quaternion){1,0,0,0});
    float alpha =  cf_gain(fabsf(sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]) - CF_GRAVITY)/CF_GRAVITY)*0.08f;

    quat_scale(1-alpha, &(struct quaternion){1,0,0,0}, &A);
    quat_scale(alpha, &da, &B);
    quat_add(&A, &B, &da);
    quat_normalize(&da);

    quat_multiply(q, &da, q);
    quat_normalize(q);
}

void cf_update(struct quaternion* q, float r[3], float a[3], float dt) {
    cf_update_from_gyro(q, r, dt);
    cf_update_from_accel(q, a);
}
