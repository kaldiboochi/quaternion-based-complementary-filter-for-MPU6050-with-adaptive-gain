#ifndef QUATERNIONS_H
#define QUATERNIONS_H

#include <math.h>

/**
 * @brief Quaternion structure
 * 
 * Represents a quaternion with real part (w) and vector part (x, y, z)
 */
struct quaternion {
    float w;  // Real part (scalar)
    float x;  // i component (vector part)
    float y;  // j component (vector part)
    float z;  // k component (vector part)
};

/**
 * @brief Multiply two quaternions
 * 
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @param result Result of q1 * q2
 */
void quatMul(const struct quaternion* q1, const struct quaternion* q2, struct quaternion* result);

/**
 * @brief Multiply quaternion by scalar
 * 
 * @param scalar Scalar value
 * @param q Input quaternion
 * @param res Result quaternion
 */
void scalarMul(float scalar, const struct quaternion* q, struct quaternion* res);

/**
 * @brief Normalize quaternion to unit length
 * 
 * @param q Quaternion to normalize (modified in place)
 */
void quatNormalise(struct quaternion* q);

/**
 * @brief Add two quaternions
 * 
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @param q3 Result of q1 + q2
 */
void quatAdd(const struct quaternion* q1, const struct quaternion* q2, struct quaternion* q3);

/**
 * @brief Calculate squared norm of quaternion
 * 
 * @param q Input quaternion
 * @return Squared norm (w² + x² + y² + z²)
 */
float normSq(const struct quaternion* q);

/**
 * @brief Calculate norm (magnitude) of quaternion
 * 
 * @param q Input quaternion
 * @return Norm (sqrt(w² + x² + y² + z²))
 */
float norm(const struct quaternion* q);

/**
 * @brief Create pure quaternion from 3D vector
 * 
 * @param vec 3D vector [x, y, z]
 * @param result Pure quaternion [0, x, y, z]
 */
void pureQuat(const float vec[3], struct quaternion* result);

/**
 * @brief Extract vector part from quaternion
 * 
 * @param q Input quaternion
 * @param result 3D vector [x, y, z]
 */
void vecFromQuat(const struct quaternion* q, float result[3]);

/**
 * @brief Calculate inverse of quaternion
 * 
 * @param q Input quaternion
 * @param result Inverse quaternion
 */
void inverse(const struct quaternion* q, struct quaternion* result);

/**
 * @brief Rotate 3D vector using quaternion
 * 
 * Performs rotation: v' = q * v * q^(-1)
 * 
 * @param q Rotation quaternion
 * @param vector Input 3D vector
 * @param rotated Output rotated vector
 */
void quatrotate(const struct quaternion* q, float vector[3], float rotated[3]);

/**
 * @brief Calculate dot product of two quaternions
 * 
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @return Dot product (w1*w2 + x1*x2 + y1*y2 + z1*z2)
 */
float quatDot(const struct quaternion* q1, const struct quaternion* q2);

#endif /* QUATERNIONS_H */
