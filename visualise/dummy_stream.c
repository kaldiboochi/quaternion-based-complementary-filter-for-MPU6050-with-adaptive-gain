#include "quaternion.h"
#include "complementary_filter.h"
#include <stdio.h>
#include <unistd.h>
#include <math.h>

int main() {
    // Initialize quaternion (identity - no rotation)
    struct quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};
    
    // Simulate IMU data
    float dt = 0.01f; // 10ms timestep
    int counter = 0;
    
    while(1) {
        // Simulate some gyro rates (sinusoidal motion for demo)
        float t = counter * dt;
        float gyro[3] = {
            0.5f * sinf(t * 0.1f),     // Roll rate
            0.3f * sinf(t * 0.15f),    // Pitch rate  
            0.2f * sinf(t * 0.2f)      // Yaw rate
        };
        
        // Simulate accelerometer (with some tilt)
        float acc[3] = {
            2.0f * sinf(t * 0.05f),    // Some x acceleration
            1.0f * sinf(t * 0.08f),    // Some y acceleration
            9.807f + 0.5f * sinf(t * 0.1f) // Mostly gravity with variation
        };
        
        // Update quaternion using your filter
        updateQuaternionFromRate(&q, gyro, dt);
        updateQuaternionFromAcc(&q, acc);
        
        // Output quaternion data for Python to read
        printf("%.6f,%.6f,%.6f,%.6f\n", q.w, q.x, q.y, q.z);
        fflush(stdout); // Force output
        
        // Sleep for 10ms (100Hz update rate)
        usleep(10000);
        counter++;
    }
    
    return 0;
}
