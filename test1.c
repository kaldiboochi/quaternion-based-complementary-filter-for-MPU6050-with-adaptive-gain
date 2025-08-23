#include "complementary_filter.h"
#include "quaternion.h"
#include <stdio.h>

void test_basic_functions() {
    struct quaternion q = {1.0f, 0.0f, 0.0f, 0.0f}; // Identity
    float gyro[3] = {0.1f, 0.2f, 0.3f}; // Small rotation rates
    
    printf("Before: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n", q.w, q.x, q.y, q.z);
    updateQuaternionFromRate(&q, gyro, 0.01f); // 10ms timestep
    printf("After gyro: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n", q.w, q.x, q.y, q.z);
    
    float acc[3] = {0.0f, 0.0f, 9.807f}; // Perfect gravity
    updateQuaternionFromAcc(&q, acc);
    printf("After accel: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n", q.w, q.x, q.y, q.z);
}
int main(){
test_basic_functions();

return 0;
}
