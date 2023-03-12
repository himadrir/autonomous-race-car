#include <Servo.h>
#include "CytronMotorDriver.h"

const int STEERING_SERVO_PIN = 2;
const float MIN_STEERING_ANGLE = 40;
const float MAX_STEERING_ANGLE = 140;

const float NEUTRAL_STEERING_ANGLE = 90.0;
const float NEUTRAL_THROTTLE = 0.0;

Servo steeringServo;

#define COMMAND_RATE 20 //hz

CytronMD motor(PWM_DIR, 3, 6);

const float wheel_circum = 0.26430197; //measured physically
const float wheel_x_distance = 0.28;
const float wheel_y_distance = 0.25;
