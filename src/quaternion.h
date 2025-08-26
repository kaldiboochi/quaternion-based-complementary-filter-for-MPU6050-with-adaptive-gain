#ifndef QUATERNION_H
#define QUATERNION_H

#include <stdio.h>
#include <math.h>
#include <assert.h>

/** Enhanced assert with message */
#define QUAT_ASSERT(cond, fmt, ...) \
    do { \
        if (!(cond)) { \
            printf("ASSERT FAILED: %s\nFile: %s:%d\nMessage: " fmt "\n", \
                   #cond, __FILE__, __LINE__, ##__VA_ARGS__); \
            abort(); \
        } \
    } while (0)

/** Quaternion struct */
struct quaternion {
    float w, x, y, z;
};

/** Print quaternion */
void quat_print(const struct quaternion* q);

/** Multiply q1 * q2 -> result */
void quat_multiply(const struct quaternion* q1, const struct quaternion* q2, struct quaternion* result);

/** Scale quaternion by scalar */
void quat_scale(float s, const struct quaternion* q, struct quaternion* result);

/** Normalize quaternion in place */
void quat_normalize(struct quaternion* q);

/** Add q1 + q2 -> result */
void quat_add(const struct quaternion* q1, const struct quaternion* q2, struct quaternion* result);

/** Squared norm */
float quat_norm_sq(const struct quaternion* q);

/** Norm */
float quat_norm(const struct quaternion* q);

/** Build pure quaternion from vec[3] */
void quat_from_vec(const float vec[3], struct quaternion* result);

/** Rotate vector by quaternion */
void quat_rotate(const struct quaternion* q, const float vec[3], float out[3]);

/** Inverse quaternion */
void quat_inverse(const struct quaternion* q, struct quaternion* result);

/** Dot product */
float quat_dot(const struct quaternion* q1, const struct quaternion* q2);

#endif // QUATERNION_H

