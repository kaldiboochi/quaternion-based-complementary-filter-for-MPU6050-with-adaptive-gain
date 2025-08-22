

#include "quaternions.h"

void quatMul(struct quaternion* result, const struct quaternion* q1, const struct quaternion* q2) {
    struct quaternion temp;
    temp.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    temp.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    temp.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    temp.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
    *result = temp;
}

float normSq(const struct quaternion* q) {
    return q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
}

float norm(const struct quaternion* q) {
    return sqrtf(normSq(q));
}

void inverse(const struct quaternion* q, struct quaternion* result) {
    float norm_sq = normSq(q);
    struct quaternion temp;
    temp.w = q->w / norm_sq;
    temp.x = -q->x / norm_sq;
    temp.y = -q->y / norm_sq;
    temp.z = -q->z / norm_sq;
    *result = temp;
}

