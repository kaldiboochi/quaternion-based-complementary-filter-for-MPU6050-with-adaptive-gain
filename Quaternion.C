void quaternion_multiply(const struct QuaternionC* q1, const struct QuaternionC* q2, struct QuaternionC* result) {
    struct QuaternionC temp;
    temp.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    temp.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    temp.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    temp.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
    *result = temp;
}
