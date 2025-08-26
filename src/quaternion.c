#include "quaternion.h"

void quat_print(const struct quaternion* q) {
    printf("q: w=%.6f x=%.6f y=%.6f z=%.6f\n", q->w, q->x, q->y, q->z);
}

void quat_multiply(const struct quaternion* q1, const struct quaternion* q2, struct quaternion* r) {
    struct quaternion t;
    t.w = q1->w*q2->w - q1->x*q2->x - q1->y*q2->y - q1->z*q2->z;
    t.x = q1->w*q2->x + q1->x*q2->w + q1->y*q2->z - q1->z*q2->y;
    t.y = q1->w*q2->y - q1->x*q2->z + q1->y*q2->w + q1->z*q2->x;
    t.z = q1->w*q2->z + q1->x*q2->y - q1->y*q2->x + q1->z*q2->w;
    *r = t;
}

void quat_scale(float s, const struct quaternion* q, struct quaternion* r) {
    r->w = q->w * s; r->x = q->x * s;
    r->y = q->y * s; r->z = q->z * s;
}

float quat_norm_sq(const struct quaternion* q) {
    return q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z;
}

float quat_norm(const struct quaternion* q) {
    float ns = quat_norm_sq(q);
    QUAT_ASSERT(ns >= 0, "norm_sq=%.6f", ns);
    return sqrtf(ns);
}

void quat_normalize(struct quaternion* q) {
    float n = quat_norm(q);
    QUAT_ASSERT(n > 0, "norm=%.6f", n);
    quat_scale(1.0f / n, q, q);
}

void quat_add(const struct quaternion* a, const struct quaternion* b, struct quaternion* r) {
    r->w = a->w + b->w; r->x = a->x + b->x;
    r->y = a->y + b->y; r->z = a->z + b->z;
}

void quat_from_vec(const float v[3], struct quaternion* q) {
    q->w = 0; q->x = v[0]; q->y = v[1]; q->z = v[2];
}

void quat_inverse(const struct quaternion* q, struct quaternion* r) {
    float ns = quat_norm_sq(q);
    QUAT_ASSERT(ns > 0, "norm_sq=%.6f", ns);
    r->w =  q->w / ns; r->x = -q->x / ns;
    r->y = -q->y / ns; r->z = -q->z / ns;
}

void quat_rotate(const struct quaternion* q, const float v[3], float out[3]) {
    struct quaternion p, t, inv, res;
    quat_from_vec(v, &p);
    quat_inverse(q, &inv);
    quat_multiply(q, &p, &t);
    quat_multiply(&t, &inv, &res);
    out[0] = res.x; out[1] = res.y; out[2] = res.z;
}

float quat_dot(const struct quaternion* a, const struct quaternion* b) {
    return a->w*b->w + a->x*b->x + a->y*b->y + a->z*b->z;
}

