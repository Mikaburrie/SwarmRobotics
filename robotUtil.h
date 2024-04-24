
// Distances in micrometers
#define WHEELBASE 150000
#define WHEEL_DIAMETER 62000
#define TICKS_PER_ROTATION 20
#define PI 3.14159
#define WHEEL_CIRCUMFERENCE PI*WHEEL_DIAMETER
#define DISTANCE_PER_TICK (WHEEL_CIRCUMFERENCE/TICKS_PER_ROTATION)

// Time in microseconds
#define MINIMUM_ENCODER_INTERVAL 5000UL
#define MAXIMUM_ENCODER_INTERVAL 200000UL

#define ENCODER_P 2
#define VELOCITY_BALANCE 0.5


// Definitions for Motor
struct Motor {
  int16_t encoderTick = 0;
  unsigned long encoderTime = 0;

  float velocityTarget = 0;
  float velocityOutput = 0;
  float velocityMeasured = 0;
  float speedMax = 0;

  uint8_t powerMin = 0;
  uint8_t powerStart = 0;
};

// Calculates powerStart, powerMin, and speedMax for a motor
void motorCalibrate(Motor* m) {
  // Set forward and reset encoder
  digitalWrite(m->pinForward, HIGH);
  digitalWrite(m->pinBackward, LOW);
  m->velocityMeasured = 0;

  // Increase power until motor moves and store result
  for (uint8_t power = 0; power < 255; power++) {
    if (m->velocityMeasured != 0) {
      m->powerStart = min(power + 10, 255);
      break;
    }
    analogWrite(m->pinPower, power);
    delay(16);
  }

  // Drive at full power and measure speed
  analogWrite(m->pinPower, 255);
  delay(1000);

  float speed = 0;
  for (uint8_t i = 0; i < 64; i++) {
    speed += m->velocityMeasured;
    delay(10);
  }

  m->speedMax = speed/64;

  // Decrease power until motor stops and store result
  for (uint8_t power = m->powerStart; power > 0; power--) {
    if ((micros() - m->encoderTime) > MAXIMUM_ENCODER_INTERVAL) {
      m->powerMin = power + 10;
      break;
    }
    analogWrite(m->pinPower, --power);
    delay(50);
  }

  m->velocityTarget = 0;
}

// Determines motor speed and adjusts output to match target
void motorUpdate(Motor* m, float delta) {
  // Assume motor stopped if no encoder tick occurs in a certain interval
  if ((micros() - m->encoderTime) > MAXIMUM_ENCODER_INTERVAL) m->velocityMeasured = 0;
  
  // Update PID loop (really just P)
  float error = m->velocityTarget - m->velocityMeasured;
  if (m->velocityTarget == 0) m->velocityOutput = 0;
  else m->velocityOutput += ENCODER_P*error*delta;

  // Limit speed range
  m->velocityOutput = max(-m->speedMax, min(m->velocityOutput, m->speedMax));

  // Calculate power output
  uint16_t power = (uint16_t) 255*abs(m->velocityOutput)/m->speedMax; // Speed -> power
  if (m->powerMin < power && power < m->powerStart && abs(m->velocityMeasured) < 0.01) power = m->powerStart; // Start push
  power = min(power, 255)*(power > m->powerMin); // Range

  // Set motor power
  digitalWrite(m->pinForward, m->velocityOutput > 0 ? HIGH : LOW);
  digitalWrite(m->pinBackward, m->velocityOutput <= 0 ? HIGH : LOW);
  analogWrite(m->pinPower, power);
}

// Determines speed of motor
void motorOnEncoderTick(Motor* m) {
  unsigned long now = micros();
  unsigned long delta = now - m->encoderTime;

  // Ignore tick if too fast
  if (delta < MINIMUM_ENCODER_INTERVAL) return;

  // Handle tick
  int8_t direction = (m->velocityOutput < 0 ? -1 : 1);
  m->velocityMeasured = DISTANCE_PER_TICK/delta*direction;
  m->encoderTick += direction;
  m->encoderTime = now;
}

// Sets the motor speed
void motorSetVelocity(Motor* m, float velocity) {
  float speed = abs(velocity);

  // Zero power if below minimum
  if (speed < (m->speedMax*m->powerMin/255)) {
    m->velocityTarget = 0;
    return;
  }

  // Limit to max speed and set
  if (speed > m->speedMax) velocity = m->speedMax*(velocity < 0 ? -1 : 1);
  m->velocityTarget = velocity;
}

