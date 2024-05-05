#include <stdio.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>

#define IPC_BUFFER_SIZE 20

#include "motor.h"

// Define GPIO pins for motor output and encoder input
#define GPIO_MOTOR_LEFT_PWM 19
#define GPIO_MOTOR_LEFT_DIR 20
#define GPIO_ENCODER_LEFT 24

#define GPIO_MOTOR_RIGHT_PWM 18
#define GPIO_MOTOR_RIGHT_DIR 17
#define GPIO_ENCODER_RIGHT 23

#define WHEEL_DIAM .065

#define COUNT_ONE_TURN  20
#define UPDATE_TIME 100

const float DIST_ONE_TURN = WHEEL_DIAM * 3.14;

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

long long timeInMilliseconds(void) {
    struct timeval tv;

    gettimeofday(&tv,NULL);
    return (((long long)tv.tv_sec)*1000)+(tv.tv_usec/1000);
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
    
  
    int lastLeftSpd = 0xFF;
    int lastRightSpd = 0xFF;
    int lastLeftCount = 0;
    int lastRightCount = 0;
    
    printf("File: %d\n", fileDescriptor);
    unsigned long lastUpdate = timeInMilliseconds();
    unsigned long currTime = lastUpdate;
    
    float alpha = 1.0;
    
    // Loop until program is terminated
    while (running) {
        // Get external input from pipe (non-blocking)
        int bytesReceived = read(fileDescriptor, dataBuffer, IPC_BUFFER_SIZE);
        
        // Set left and right motor speeds if command is received
        if (bytesReceived > 0) {
          
            printf("Received: %d, %d Left count:%d Right count:%d\n", dataBuffer[0], dataBuffer[1], leftMotor.distance, rightMotor.distance);
            
            
            
            
            motorSetDuty(&leftMotor, dataBuffer[0]);
            motorSetDuty(&rightMotor, dataBuffer[1]);
            
            
          
            
            
            
	    
        }
        
        currTime = timeInMilliseconds();
        if(currTime > lastUpdate + UPDATE_TIME) {
                 unsigned long timeBT = currTime - lastUpdate;
                 float lastRightSpeed = (((rightMotor.distance - lastRightCount) / COUNT_ONE_TURN) * DIST_ONE_TURN) / timeBT;
                 float lastLeftSpeed = (((leftMotor.distance - lastLeftCount) / COUNT_ONE_TURN) * DIST_ONE_TURN) / timeBT;
                 lastLeftCount = leftMotor.distance;
                 lastRightCount = rightMotor.distance;
                 lastUpdate = timeInMilliseconds();
                 
                 printf("left speed: %f right speed % f\n", lastLeftSpeed, lastRightSpeed);
                 
        } 
        
        //printf("%d\n", leftMotor.distance);
        //printf("%d\n", rightMotor.distance);
        
    }
    
    // Close and remove FIFO pipe
    close(fileDescriptor);
    unlink(fifoPath);
    
    // Terminate motors and piGPIO
    motorTerminate(&leftMotor);
    motorTerminate(&rightMotor);
    gpioTerminate();
}
