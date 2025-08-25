#include "quaternion.h"
#include <math.h>  

    
void quatPrint(const struct quaternion* q){
    printf("quaternion values: w:%.6f x:%.6f y:%.6f z:%.6f \n",q->w,q->x,q->y,q->z);
}

void quatMul(const struct quaternion* q2, const struct quaternion* q1, struct quaternion* result) {
    struct quaternion temp;
    temp.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    temp.x = q1->w * q2->x + q1->x * q2->w - q1->y * q2->z + q1->z * q2->y;
    temp.y = q1->w * q2->y + q1->x * q2->z + q1->y * q2->w - q1->z * q2->x;
    temp.z = q1->w * q2->z - q1->x * q2->y + q1->y * q2->x + q1->z * q2->w;
    *result = temp;
}

void scalarMul(float scalar, const struct quaternion* q, struct quaternion* res){
  res->w = q->w * scalar;
  res->x = q->x * scalar;
  res->y = q->y * scalar;
  res->z = q->z * scalar;
}

void quatNormalise(struct quaternion* q){
  float norm_ = norm(q);
  assert(norm_!=0);
  scalarMul(1.0f/norm_, q, q); 
}

void quatAdd(const struct quaternion* q1, const struct quaternion* q2, struct quaternion* q3){
  q3->w = q1->w + q2->w;
  q3->x = q1->x + q2->x;
  q3->y = q1->y + q2->y;
  q3->z = q1->z + q2->z;
}

float normSq(const struct quaternion* q) {
    return q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
}

float norm(const struct quaternion* q) {
    ASSERT_MSG(normSq(q)>0, "norm squared =%.6f  quaternion values: w:%.6f x:%.6f y:%.6f z:%.6f \n",normSq(q),q->w,q->x,q->y,q->z);
    return sqrtf(normSq(q));
}

void pureQuat(const float vec[3], struct quaternion* result){
  result->w = 0;
  result->x = vec[0];
  result->y = vec[1];
  result->z = vec[2];
}

void vecFromQuat(const struct quaternion* q, float result[3]){
  result[0] = q->x;
  result[1] = q->y;
  result[2] = q->z;
}

void inverse(const struct quaternion* q, struct quaternion* result) {
    float normsq = normSq(q);
    assert(normsq!=0);
    struct quaternion temp;
    temp.w = q->w / normsq;
    temp.x = -q->x / normsq;
    temp.y = -q->y / normsq;
    temp.z = -q->z / normsq;
    *result = temp;
}

void quatrotate(const struct quaternion* q, float vector[3], float rotated[3]) {
    struct quaternion vector_quaternion;
    pureQuat(vector, &vector_quaternion);

    struct quaternion rotated_vector_quaternion;
    struct quaternion qInv;
    inverse(q, &qInv);

    struct quaternion temp;
    quatMul(q, &vector_quaternion, &temp);
    quatMul(&temp, &qInv, &rotated_vector_quaternion);
    vecFromQuat(&rotated_vector_quaternion, rotated);
}

float quatDot(const struct quaternion* q1, const struct quaternion* q2){
  return q1->w * q2->w + q1->x * q2->x + q1->y * q2->y + q1->z * q2->z;
}

