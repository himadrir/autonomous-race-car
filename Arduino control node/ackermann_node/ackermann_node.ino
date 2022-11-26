#include <Servo.h>
#include <math.h>

#include <ros.h>
#include <std_msgs/Float64.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Int32.h>

//_________________________________CAR SETUP VALUES______________________________________________
const float wheel_ratio = 0.0003096;    // 1 encoder tick to wheel rev ratio
const float wheel_circum = 0.26430197;   // in meters

const int STEERING_SERVO_PIN = 9;
const int ESC_SERVO_PIN = 10;
const int PIN_LED = 13;

const float NEUTRAL_THROTTLE = 90;
const float NEUTRAL_STEERING_ANGLE = 90;

const float MIN_THROTTLE = 60;             // min throttle (0-180)
const float MAX_THROTTLE = 102;            // max throttle (0-180)
const float UPHILL_THROTTLE = 108;

const float MIN_STEERING_ANGLE = 38;       // min steering angle (0-180)
const float MAX_STEERING_ANGLE = 145;      // max steering angle (0-180)

const float THROTTLE_FORWARD_OFFSET  = 7;  // Offset car's weight in throttle (m/s) * 10
const float THROTTLE_BACKWARD_OFFSET = 28; // Offset car's weight in throttle (m/s) * 10

const int   DELAY = 5;

Servo steeringServo;
Servo electronicSpeedController;
//_________________________________CAR SETUP VALUES______________________________________________

volatile float v = 0.0;
volatile long counter = 0;
volatile float servo_ang = NEUTRAL_STEERING_ANGLE;

bool LED_STATE = true;

// Forward Declaration
void ackermannCallback( const ackermann_msgs::AckermannDriveStamped & ackermann );
//encoder pub
std_msgs::Int32 encoder_tick_msg;
std_msgs::Float64 servo_ang_msg;


ros::Publisher encoder_ticks_pub("encoder_ticks", &encoder_tick_msg);
ros::Publisher servo_ang_pub("servo_angle", &servo_ang_msg);


// Throttle/Steering Command Subscriber
ros::Subscriber<ackermann_msgs::AckermannDriveStamped> ackermannSubscriber("/ackermann_cmd", &ackermannCallback);
ros::NodeHandle nodeHandle;

// Vehicle State
const int FORWARD    =  1;
const int STOP       =  0;
const int BACKWARD   = -1;
int lastVehicleState = STOP;

/**
 * Function:    ackermannCallback()
 * Input:       driveMsg - Ackermann ROS message (throttle, steering angle)
 * Description: This method is invoked whenever a new message appears in ROS regarding
 *              a new throttle/steering specification for the vehicle. This function will
 *              change the steering angle and throttle accordingly to match that of the
 *              subscribed ackermann message.
 */
//***********************************INTERRUPT CALLBACKS*********************************************
  void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
  counter++;
  }else{
  counter--;
  }
  }
   
  void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
  counter--;
  }else{
  counter++;
  }
  }
//**************************************************************************************************

void ackermannCallback( const ackermann_msgs::AckermannDriveStamped & ackermann )
{
  // Convert steering angle from radian to degree that fits in range of (0-180)
  float steering_angle = ackermann.drive.steering_angle * (180 / M_PI) + 90;
  
  // Get throttle in range of 0-180 and offset the weight
  float throttle = ackermann.drive.speed * 10 + 90;
  if (throttle > NEUTRAL_THROTTLE)
    throttle += THROTTLE_FORWARD_OFFSET;
  if (throttle < NEUTRAL_THROTTLE)
    throttle -= THROTTLE_BACKWARD_OFFSET;
  
  // Check for allowed min steering angle
  if( steering_angle < MIN_STEERING_ANGLE )
    steering_angle = MIN_STEERING_ANGLE;
    
  // Check for allowed max steering angle
  if( steering_angle > MAX_STEERING_ANGLE )
    steering_angle = MAX_STEERING_ANGLE;
    
  // Check for allowed min throttle
  if( throttle < MIN_THROTTLE )
    throttle = MIN_THROTTLE;
  
  // Check for allowed max throttle
  if( throttle > MAX_THROTTLE ) {
    // Check if up hill speed requested. request code: 999
    if( ackermann.drive.speed == 999)
      throttle = UPHILL_THROTTLE;
    else
      throttle = MAX_THROTTLE;
  }
  
  // Switches ESC to backward mode if necessary
  if( throttle <= NEUTRAL_THROTTLE - THROTTLE_BACKWARD_OFFSET ) {
    if( lastVehicleState != BACKWARD ) {
      electronicSpeedController.write( NEUTRAL_THROTTLE );
      delay(50);
      electronicSpeedController.write( MIN_THROTTLE );
      delay(50);
      electronicSpeedController.write( MIN_THROTTLE );
      delay(50);
      electronicSpeedController.write( NEUTRAL_THROTTLE );
      delay(50);
    }
  }
  
  // Update last vehicle's state
  if( throttle >= NEUTRAL_THROTTLE + THROTTLE_FORWARD_OFFSET )
    lastVehicleState = FORWARD;
  else if ( throttle <= NEUTRAL_THROTTLE - THROTTLE_BACKWARD_OFFSET )
    lastVehicleState = BACKWARD;
  else
    lastVehicleState = STOP;
  
  servo_ang = steering_angle;
  // Send out steering and throttle commands
  steeringServo.write( steering_angle );
  electronicSpeedController.write( throttle );
}

/**
 * Function:    setup()
 * Input:       None
 * Description: Run once at initialization. Subscribe to ROS's ackermann_msg
 *              and connect to steering and ESC servo.
 */
void setup() {

  pinMode(13, OUTPUT);
  
  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
  pinMode(3, INPUT_PULLUP); // internalเป็น pullup input pin 3
  attachInterrupt(digitalPinToInterrupt(2), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(3), ai1, RISING);
  
  // Connect steering and esc servo
  steeringServo.attach( STEERING_SERVO_PIN );
  electronicSpeedController.attach( ESC_SERVO_PIN );
  
  // Initialize node and subscribe to ackermann msg
  nodeHandle.getHardware()->setBaud(57600);
  nodeHandle.initNode();
  nodeHandle.advertise(servo_ang_pub);
  nodeHandle.advertise(encoder_ticks_pub);
  nodeHandle.subscribe(ackermannSubscriber);
  
  // Initialize steering and throttle to neutral
  steeringServo.write( NEUTRAL_STEERING_ANGLE );
  electronicSpeedController.write( NEUTRAL_THROTTLE );
  
  delay(1000);
  
  
}


void loop() {

 
  // Stop car if ROS connection lost
  if( !nodeHandle.connected() ) {
    steeringServo.write( NEUTRAL_STEERING_ANGLE );
    electronicSpeedController.write( NEUTRAL_THROTTLE );
  }
  
  
  encoder_tick_msg.data = counter;
  servo_ang_msg.data = servo_ang;

  
  encoder_ticks_pub.publish(&encoder_tick_msg);
  servo_ang_pub.publish(&servo_ang_msg);


  
  nodeHandle.spinOnce();
  delay(DELAY);
}
