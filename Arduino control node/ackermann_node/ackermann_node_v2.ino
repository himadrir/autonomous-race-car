#include "Arduino.h"

#include "ArduPID.h"
#include "encoder_h.h"
#include "car_setup.h"

#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
// #include <ackermann_msgs/AckermannDriveStamped.h>
// #include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

ArduPID pid;

void vel_callback(const geometry_msgs::TwistStamped& cmd_msg);


ros::NodeHandle nh;

std_msgs::Float32 vel_msg;
std_msgs::Int32 pid_msg;
std_msgs::Float32 theta_servo_msg;


ros::Publisher pid_output_pub("/pid_op", &pid_msg);
ros::Publisher vel_publisher("/curr_vel", &vel_msg);
ros::Publisher theta_pub("/theta_servo", &theta_servo_msg);

ros::Subscriber<geometry_msgs::TwistStamped> cmd_sub("/cmd_vel_", vel_callback);

float linear_vel_x = 0.0;
float angular_vel_z = 0.0;

unsigned long prev_command_time = 0;

float getVelocity(long deltaTicks_, int deltaTime_)
{

  return ((deltaTicks_ * enc_to_wheel_ratio * wheel_circum) / (deltaTime_*0.001));
  
}

float getPID(float ip_vel, float stpt){

  ip_ = ip_vel;

  setpoint_ = stpt;

  pid.compute();

  return op_;
}

void vel_callback(const geometry_msgs::TwistStamped& cmd_msg)
{

  f = 1;

  linear_vel_x = cmd_msg.twist.linear.x;

  angular_vel_z = cmd_msg.twist.angular.z;
  
  prev_command_time = millis();

}

int velocityToRPM(float v)
{

  return ((v*60) / wheel_circum);

}

void stop_car()
{

  linear_vel_x = 0;

  angular_vel_z = 0;

  return;
  // move_car(linear_vel_x, angular_vel_z);
}

void move_car(float vel_stpt, float theta)
{

  static int pwm = 0;

  if(vel_stpt < 0.0)
  {

    pwm = -(getPID(velocity, abs(vel_stpt)));

  }

  else 
  { 

    pwm = getPID(velocity, abs(vel_stpt));
  
  }

  if ( (vel_stpt == 0.0) && (pwm != 0) )
  {

    pwm = 0;

  }

  pid_msg.data = op_;

  motor.setSpeed(pwm); 

  float curr_steer_ang = getThetaFromSteer(steer(theta));

  theta_servo_msg.data = curr_steer_ang;

  return;

}

float steer(float steering_angle)
{
  
  float sa = constrain((steering_angle * (180 / M_PI) + 90), MIN_STEERING_ANGLE, MAX_STEERING_ANGLE);
  
  steeringServo.write(sa);
  
  return sa;  

}

float getThetaFromSteer(float sa_)
{
  return ((sa_ - 90) * (M_PI/180));
}



void isr1()
{
  if( digitalRead (isr_pin) && (micros()-debounce > 500) && digitalRead (isr_pin)) 
  { 

    debounce = micros(); // Stores the time to check that we don't count the bounce in the signal.

    counter++;

  } // Add the good pulse coming in.

  else ; 
         
}


  

void setup() 
{

  nh.getHardware()->setBaud(57600);
  nh.initNode();

  nh.advertise(vel_publisher);
  nh.advertise(pid_output_pub);
  nh.advertise(theta_pub);

  nh.subscribe(cmd_sub);
  // nh.subscribe(ackermannSubscriber);

  pinMode(13, OUTPUT);

  pinMode(isr_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(isr_pin), isr1, RISING);

  motor.setSpeed(0);

  pid.begin(&ip_, &op_, &setpoint_, kp_, ki_, kd_);


  steeringServo.attach(STEERING_SERVO_PIN);
  steeringServo.write(NEUTRAL_STEERING_ANGLE);

  while (!nh.connected()) 
  {
   nh.spinOnce();
  }

  nh.loginfo("BASE CONNECTED");
  delay(1000);

  
  /* 
  if(!nh.getParam("/pid/Kp", &kp_, 1)){
    kp_ = 0.085;
    nh.logwarn("default param used for Kp");
  }
  if(!nh.getParam("/pid/Ki", &ki_, 1)){
    ki_ = 0.08;
    nh.logwarn("default param used for Ki");
  }
  if(!nh.getParam("/pid/Kd", &kd_, 1)){
    kd_ = 0.015;
    nh.logwarn("default param used for Kd");
  }

  */

}

void loop() 
{

  static unsigned long prev_control_time = 0;

  long deltaTime = millis() - time_now;

  if(deltaTime >= 50)
  {

    noInterrupts();

    deltaCount = counter - prevCount;

    prevCount = counter;

    interrupts();

    velocity = getVelocity(deltaCount, deltaTime);

    time_now = millis();

  }

  if(!nh.connected())
  {

    stop_car();

  }

  // int pwm = getPID(velocity, linear_vel_x);

  if((millis() - prev_control_time) >= 1000/COMMAND_RATE)
  {

    move_car(linear_vel_x, angular_vel_z);

    vel_msg.data = velocity;

    vel_publisher.publish(&vel_msg);

    pid_output_pub.publish(&pid_msg);

    theta_pub.publish(&theta_servo_msg);

    prev_control_time = millis();

  }

  if((millis() - prev_command_time) >= 400)
  {

    stop_car();

  }


  if(f == 1)
  {
    nh.loginfo("callback called!");

    f = 0;

  }

  else 
  {

    nh.loginfo("no data received!");

  }

  nh.spinOnce();
 
}
