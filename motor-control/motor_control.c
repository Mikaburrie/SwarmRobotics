#include <stdio.h>
#include <signal.h>

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
    printf("Left encoder tick: level %d at %d us\n", level, time);
    motorOnEncoderTick(&leftMotor, level, time);
}

void onRightEncoderTick(int gpio, int level, uint32_t time) {
    printf("Right encoder tick: level %d at %d us\n", level, time);
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

    // Loop until program is terminated
    while (running) {
        // Set left and right motor speeds
        motorSetDuty(&leftMotor, 190);
        motorSetDuty(&rightMotor, 160);
    }
    
    // Terminate motors and piGPIO
    motorTerminate(&leftMotor);
    motorTerminate(&rightMotor);
    gpioTerminate();
}
