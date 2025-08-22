struct quaternion {
    // The real part of the quaternion
    float w;
    // The i component of the vector part
    float x;
    // The j component of the vector part
    float y;
    // The k component of the vector part
    float z;
};

void quaternion_multiply(struct Quaternion* q1, struct Quaternion* q2, struct Quaternion* result);
float norm(const struct Quaternion* q);
inverse( struct Quaternion* q,struct Quaternion* result);
