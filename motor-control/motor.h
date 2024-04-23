#include <pigpio.h>

// Struct containing motor settings and values
struct Motor {
    // GPIO pins for PWM, direction, and the encoder
    const int gpioPWM; // not to be confused with pigpio's gpioPWM() function
    const int gpioDir;
    const int gpioEncoder;
};

// Handles encoder interrupt
void motorOnEncoderTick(struct Motor* m, int level, uint32_t time) {
    // Handle tick
}

void motorInitialize(struct Motor* m, gpioISRFunc_t onEncoderTick) {
    // Initialize piGPIO if needed
    static int gpioInitialized = 0;
    if (!gpioInitialized) {
        // Initialize piGPIO with signal handlers disabled
        int cfg = gpioCfgGetInternals();
        cfg |= PI_CFG_NOSIGHANDLER;  // (1<<10)
        gpioCfgSetInternals(cfg);
        if (gpioInitialise() < 0) {
            printf("PiGPIO Initialization failed. Make sure to run with sudo\n");
            return;
        }
        
        // Set initialization flag
        gpioInitialized = 1;
    }
    
    // Setup PWM
    gpioSetMode(m->gpioPWM, PI_OUTPUT);
    gpioSetPWMfrequency(m->gpioPWM, 8000);
    gpioPWM(m->gpioPWM, 0);
    
    // Setup motor direction
    gpioSetMode(m->gpioDir, PI_OUTPUT);
    gpioWrite(m->gpioDir, 0);
    
    // Setup encoder interrupt
    gpioSetMode(m->gpioEncoder, PI_INPUT);
    gpioSetPullUpDown(m->gpioEncoder, PI_PUD_DOWN);
    gpioSetISRFunc(m->gpioEncoder, EITHER_EDGE, 0, onEncoderTick);
}

void motorSetDuty(struct Motor* m, int duty) {
    // Set PWM and direction
    int dir = duty < 0;
    gpioPWM(m->gpioPWM, duty*(dir ? -1 : 1));
    gpioWrite(m->gpioDir, dir);
}

void motorTerminate(struct Motor* m) {
    // Clear motor output and encoder interrupt
    gpioPWM(m->gpioPWM, 0);
    gpioWrite(m->gpioPWM, 0);
    gpioSetISRFunc(m->gpioEncoder, EITHER_EDGE, 0, NULL);
}
