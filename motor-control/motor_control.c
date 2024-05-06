#include <stdio.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#define IPC_BUFFER_SIZE 20

#include "motor.h"
#define VELOCITY_BALANCE 0.5

// Define GPIO pins for motor output and encoder input
#define GPIO_MOTOR_LEFT_PWM 19
#define GPIO_MOTOR_LEFT_DIR 20
#define GPIO_ENCODER_LEFT 24

#define GPIO_MOTOR_RIGHT_PWM 18
#define GPIO_MOTOR_RIGHT_DIR 17
#define GPIO_ENCODER_RIGHT 23

// Define motors
struct Motor leftMotor = {
    GPIO_MOTOR_LEFT_PWM,
    GPIO_MOTOR_LEFT_DIR,
    GPIO_ENCODER_LEFT,
    0
};

struct Motor rightMotor = {
    GPIO_MOTOR_RIGHT_PWM,
    GPIO_MOTOR_RIGHT_DIR,
    GPIO_ENCODER_RIGHT,
    1 // reversed
};

// Define encoder tick handlers
void onLeftEncoderTick(int gpio, int level, uint32_t time) {
//    printf("Left encoder tick: level %d at %d us\n", level, time);
    if (level) return;
    motorOnEncoderTick(&leftMotor, level, time);
}

void onRightEncoderTick(int gpio, int level, uint32_t time) {
//    printf("Right encoder tick: level %d at %d us\n", level, time);
    motorOnEncoderTick(&rightMotor, level, time);
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
    
    // Calibrate
    motorCalibrate(&leftMotor);
    motorCalibrate(&rightMotor);
    printf("Calibration result:\nMin power: %d, Start power: %d, Max speed: %f\n", leftMotor.powerMin, leftMotor.powerStart, leftMotor.speedMax);
    printf("Calibration result:\nMin power: %d, Start power: %d, Max speed: %f\n", rightMotor.powerMin, rightMotor.powerStart, rightMotor.speedMax);
    
    // Set left and right speedMax to minimum
    float speedMax = fmin(leftMotor.speedMax, rightMotor.speedMax);
    leftMotor.speedMax = speedMax;
    rightMotor.speedMax = speedMax;
    
    // Create and open non-blocking FIFO pipe for receiving motor commands
    int fileDescriptor;
    char* fifoPath = "/tmp/motor_control";
    int dataBuffer[IPC_BUFFER_SIZE];
    mkfifo(fifoPath, 0666);
    fileDescriptor = open(fifoPath, O_RDONLY | O_NONBLOCK);
    printf("File: %d\n", fileDescriptor);
    
    // Define timing variables
    float now = gpioTick();
    float lastUpdate = now;
    
    // Loop until program is terminated
    while (running) {
        // Get external input from pipe (non-blocking)
        int bytesReceived = read(fileDescriptor, dataBuffer, IPC_BUFFER_SIZE);
        
        // Set left and right motor speeds if command is received
        if (bytesReceived > 0) {
            motorSetVelocity(&leftMotor, leftMotor.speedMax*dataBuffer[0]/100);
            motorSetVelocity(&rightMotor, -rightMotor.speedMax*dataBuffer[1]/100);
        }
        
        // Calculate delta time
        now = gpioTick();
        float delta = (now - lastUpdate)/1000000.0;
        
        // Adjust speeds to maintain proper ratio
        if (leftMotor.velocityTarget != 0 && rightMotor.velocityTarget != 0) {
            float ratio = leftMotor.velocityTarget/rightMotor.velocityTarget;
            float leftSpeedError = rightMotor.velocityMeasured*ratio - leftMotor.velocityMeasured;
            float rightSpeedError = leftMotor.velocityMeasured/ratio - rightMotor.velocityMeasured;
            leftMotor.velocityOutput += leftSpeedError*VELOCITY_BALANCE*delta;
            rightMotor.velocityOutput -= rightSpeedError*VELOCITY_BALANCE*delta;
        }
        
        // Update motors
        motorUpdate(&leftMotor, delta);
        motorUpdate(&rightMotor, delta);
        
        lastUpdate = now;
        usleep(10000);
    }
    
    // Close and remove FIFO pipe
    close(fileDescriptor);
    unlink(fifoPath);
    
    // Terminate motors and piGPIO
    motorTerminate(&leftMotor);
    motorTerminate(&rightMotor);
    gpioTerminate();
}
