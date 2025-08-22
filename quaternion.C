

#include "quaternions.h"

void quaternion_multiply(struct Quaternion* q1, struct Quaternion* q2, struct Quaternion* result) {
    struct Quaternion temp;
    temp.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    temp.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    temp.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    temp.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
    *result = temp;
}


float norm(const struct Quaternion* q) {
    return sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
}

void inverse( struct Quaternion* q,struct Quaternion* result){
  temp.w = q->w
  temp.x = -1.0*q->x
  temp.y = -1.0*q->y
  temp.z = -1.0*q->z
}
