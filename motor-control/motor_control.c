#include <stdio.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#define IPC_BUFFER_SIZE 2

#include "motor.h"

// Define GPIO pins for motor output and encoder input
#define GPIO_MOTOR_LEFT_PWM 19
#define GPIO_MOTOR_LEFT_DIR 20
#define GPIO_ENCODER_LEFT 23

#define GPIO_MOTOR_RIGHT_PWM 18
#define GPIO_MOTOR_RIGHT_DIR 17
#define GPIO_ENCODER_RIGHT 24

// Define motors
struct Motor leftMotor = {
    GPIO_MOTOR_LEFT_PWM,
    GPIO_MOTOR_LEFT_DIR,
    GPIO_ENCODER_LEFT
};

struct Motor rightMotor = {
    GPIO_MOTOR_RIGHT_PWM,
    GPIO_MOTOR_RIGHT_DIR,
    GPIO_ENCODER_RIGHT
};

// Define encoder tick handlers
void onLeftEncoderTick(int gpio, int level, uint32_t time) {
//    printf("Left encoder tick: level %d at %d us\n", level, time);
    motorOnEncoderTick(&leftMotor, level, time);
}

void onRightEncoderTick(int gpio, int level, uint32_t time) {
//    printf("Right encoder tick: level %d at %d us\n", level, time);
    motorOnEncoderTick(&rightMotor, level, time);
}



void setRatio(double theta, double radius) { 
//  countL = (int)abs((radius - robBased / 2) * theta / distOneTurn * countOneTurn);
//  countR = (int)abs((radius + robBased / 2) * theta / distOneTurn * countOneTurn);

}

// Handles interrupt signal (ctrl+C)
static volatile int running = 1;
void systemInterruptHandler(int dummy) {
    running = 0;
}

int main(int argc, char *argv[]) {
    // Assign signal handler
    signal(SIGINT, systemInterruptHandler);
    
    // Initialize motors
    motorInitialize(&leftMotor, onLeftEncoderTick);
    motorInitialize(&rightMotor, onRightEncoderTick);
    
    // Create and open non-blocking FIFO pipe for receiving motor commands
    int fileDescriptor;
    char* fifoPath = "/tmp/motor_control";
    int dataBuffer[IPC_BUFFER_SIZE];
    mkfifo(fifoPath, 0666);
    fileDescriptor = open(fifoPath, O_RDONLY | O_NONBLOCK);
    
    printf("File: %d\n", fileDescriptor);

    // Loop until program is terminated
    while (running) {
        // Get external input from pipe
        int bytesReceived = read(fileDescriptor, dataBuffer, IPC_BUFFER_SIZE);
        
        // Set left and right motor speeds if command is received
        if (bytesReceived > 0) {
            printf("Received: %d, %d\n", dataBuffer[0], dataBuffer[1]);
            motorSetDuty(&leftMotor, dataBuffer[0]);
            motorSetDuty(&rightMotor, dataBuffer[1]);
        }
    }
    
    // Close and remove FIFO pipe
    close(fileDescriptor);
    unlink(fifoPath);
    
    // Terminate motors and piGPIO
    motorTerminate(&leftMotor);
    motorTerminate(&rightMotor);
    gpioTerminate();
}
