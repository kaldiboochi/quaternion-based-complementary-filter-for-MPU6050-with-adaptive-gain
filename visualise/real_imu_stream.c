#include "quaternion.h"
#include "complementary_filter.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

// Serial port configuration
int setup_serial_port(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Unable to open serial port");
        return -1;
    }
    
    // Configure serial port
    struct termios options;
    tcgetattr(fd, &options);
    
    // Set baud rate (115200)
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    
    // Configure 8N1 (8 data bits, no parity, 1 stop bit)
    options.c_cflag &= ~PARENB;        // No parity
    options.c_cflag &= ~CSTOPB;        // 1 stop bit
    options.c_cflag &= ~CSIZE;         // Clear data size bits
    options.c_cflag |= CS8;            // 8 data bits
    options.c_cflag |= CLOCAL | CREAD; // Enable receiver, local mode
    
    // Raw input mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    
    // Set timeouts
    options.c_cc[VMIN] = 0;   // Non-blocking read
    options.c_cc[VTIME] = 1;  // 0.1 second timeout
    
    // Apply settings
    tcsetattr(fd, TCSANOW, &options);
    
    // Flush any existing data
    tcflush(fd, TCIOFLUSH);
    
    printf("Serial port %s opened successfully!\n", device);
    return fd;
}

// Parse IMU data from serial line
int parse_imu_data(char* line, float* ax, float* ay, float* az, 
                   float* wx, float* wy, float* wz) {
    // Expected format: "ax,ay,az,wx,wy,wz\n"
    char* token;
    float values[6];
    int count = 0;
    
    // Remove newline
    char* newline = strchr(line, '\n');
    if (newline) *newline = '\0';
    
    // Parse comma-separated values
    token = strtok(line, ",");
    while (token != NULL && count < 6) {
        values[count] = atof(token);
        token = strtok(NULL, ",");
        count++;
    }
    
    if (count == 6) {
        *ax = values[0];
        *ay = values[1]; 
        *az = values[2];
        *wx = values[3];
        *wy = values[4];
        *wz = values[5];
        return 1; // Success
    }
    
    return 0; // Failed to parse
}

int main(int argc, char* argv[]) {
    // Check command line arguments
    if (argc != 2) {
        printf("Usage: %s <serial_device>\n", argv[0]);
        printf("Example: %s /dev/ttyACM0\n", argv[0]);
        return 1;
    }
    
    const char* device = argv[1];
    
    // Setup serial port
    int serial_fd = setup_serial_port(device);
    if (serial_fd < 0) {
        return 1;
    }
    
    // Initialize quaternion (identity - no rotation)
    struct quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};
    
    // Buffer for reading serial data
    char buffer[1024];
    char line_buffer[256];
    int line_pos = 0;
    
    float dt = 0.01f; // Assume 100Hz update rate
    
    printf("Reading IMU data from %s...\n", device);
    printf("Expected format: ax,ay,az,wx,wy,wz\\n\n");
    printf("Quaternion output format: w,x,y,z\n");
    printf("Press Ctrl+C to stop.\n\n");
    
    while (1) {
        // Read data from serial port
        int bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);
        
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            
            // Process each character
            for (int i = 0; i < bytes_read; i++) {
                char c = buffer[i];
                
                if (c == '\n' || c == '\r') {
                    // End of line - process the data
                    if (line_pos > 0) {
                        line_buffer[line_pos] = '\0';
                        
                        // Parse IMU data
                        float ax, ay, az, wx, wy, wz;
                        if (parse_imu_data(line_buffer, &ax, &ay, &az, &wx, &wy, &wz)) {
                            // Convert gyro from deg/s to rad/s
                            float gyro[3] = {wx * M_PI / 180.0f, 
                                           wy * M_PI / 180.0f, 
                                           wz * M_PI / 180.0f};
                            
                            // Accelerometer data (assuming in m/s²)
                            float accel[3] = {ax, ay, az};
                            
                            // Update quaternion using your complementary filter
                            updateQuaternionFromRate(&q, gyro, dt);
                            updateQuaternionFromAcc(&q, accel);
                            
                            // Output quaternion for visualization
                            printf("%.6f,%.6f,%.6f,%.6f\n", q.w, q.x, q.y, q.z);
                            fflush(stdout);
                        } else {
                            // Debug: show unparsed line
                            fprintf(stderr, "Could not parse: %s\n", line_buffer);
                        }
                        
                        line_pos = 0; // Reset line buffer
                    }
                } else if (line_pos < sizeof(line_buffer) - 1) {
                    // Add character to line buffer
                    line_buffer[line_pos++] = c;
                }
            }
        } else if (bytes_read < 0 && errno != EAGAIN) {
            perror("Error reading from serial port");
            break;
        }
        
        // Small delay to prevent excessive CPU usage
        usleep(1000); // 1ms
    }
    
    // Cleanup
    close(serial_fd);
    printf("Serial port closed.\n");
    return 0;
}
