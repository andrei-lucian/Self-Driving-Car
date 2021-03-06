//The required includes for the motor controller
#include "Encoder.h"  //This library includes the encoder class, such that you can obtain the number of ticks counted so far.
#include "MsTimer2.h" //This library allows for timer interrupts
#include <ros.h>      //This includes the ROS overlay
#include "my_msgs/Vel.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Char.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include "PID.h" //This includes the PID class you will be using

//The folling defines defines constants required for interfacing with the hardware. Please don't edit this.
#define ENCA1 27
#define ENCA2 26
#define ENCB1 39
#define ENCB2 38

#define APHASE 29
#define PWMA 30
#define STBYA 28
#define BPHASE 36
#define PWMB 35
#define STBYB 37

//These defines define some constants
#define UPATE_RATE 20  //in ms. Hence actual rate (Hz) = 1 / (update_rate / 1000). So, for 20 ms => 50 Hz
#define PER_ROT 1200.0 // number of ticks per full wheel rotation
#define WHEEL_CIR 0.18 // The wheel circumference in meters. You might have to fine tune this for your vehicle
#define REPORT_COUNT 10
#define WHEEL_RADIUS 0.03 // Meters
#define TRACK 0.105       // Distance between the wheels

//The PID controller for the left and right wheel
PID PID_left(100, 3000, 1);
PID PID_right(100, 3000, 1);

//The encoders to read the number of encoder ticks for the left and right wheel
Encoder motor_left(ENCA1, ENCA2);
Encoder motor_right(ENCB1, ENCB2);

long last_time = millis();

float k_rho = 3;  // > 0
float k_alpha = 8;   // k_alpha > k_rho
float k_beta = -1.5; // < 0

float k_1 = 1;//3;  // > 0
float k_2 = 1;//8;  // > 0
float lamb_2 = 0.5; // > 0

// The nodehandle as you saw in the ROS intro documentation
ros::NodeHandle node_handle;

//This defines an enumeration. You can define your states here for Finite State Machine behaviour of your update loop.
enum State
{
  VELOCITY_CONTROL,
  LINEAR_POSITION_CONTROL,
  NON_LINEAR_POSITION_CONTROL,
};

State state = State::VELOCITY_CONTROL; //This is an example on how to set the current state to one of the values from the enumeration.
geometry_msgs::Pose2D target_point;
std_msgs::Char char_msg;

// ***** Publishers *********
// These publishers are provided to make visualisation prossible for tuning your PID controllers.
std_msgs::Float64 wheel_msg;
ros::Publisher left_wheel_publisher("left_vel", &wheel_msg);
ros::Publisher right_wheel_publisher("right_vel", &wheel_msg);
ros::Publisher target_point_publisher("target_point", &target_point);
ros::Publisher dbg_pub("dbg", &wheel_msg);
ros::Publisher state_publisher("state", &char_msg);

// **** Subscriber callback functions **********

//These callbacks receive PID values, and updates the PID controller with these new values;
void P_cb(std_msgs::Float32 const &msg)
{
  PID_left.P(msg.data);
  PID_right.P(msg.data);
}
void I_cb(std_msgs::Float32 const &msg)
{
  PID_left.I(msg.data);
  PID_right.I(msg.data);
}
void D_cb(std_msgs::Float32 const &msg)
{
  PID_left.D(msg.data);
  PID_right.D(msg.data);
}

// Set the left and right wheel velocities in rads/s based on the provided
// body velocities. Where translational is the forward speed in m/s and
// rotational the rotational speed around the z axis in rads/s.
void setTargetVelocity(double translational, double rotational)
{
  if (abs(rotational) > 31)
    rotational = 0;
  double left_wheel_vel = (2 * translational - TRACK * rotational) / (2 * WHEEL_RADIUS);
  double right_wheel_vel = (2 * translational + TRACK * rotational) / (2 * WHEEL_RADIUS);
  
  PID_left.set_target(left_wheel_vel);
  PID_right.set_target(right_wheel_vel);
}

// This callback receives robot target velocities from outside the Teensy.
void vel_cmd_cb(geometry_msgs::Twist const &msg)
{
  if (state != State::VELOCITY_CONTROL)
    return;

  double translational = msg.linear.x;
  double rotational = msg.angular.z;
  setTargetVelocity(translational, rotational);
  
  last_time = millis();
}

void pos_cmd_cb(geometry_msgs::Pose2D const &msg)
{
  //The received message contains coordinates of the target location in the robot frame of reference.
  //The mathematics behind position control, uses the robot position within the target-location frame of reference though.
  //The following mapping will map the coordinates from the robot frame of reference to the latter frame of reference:

  //express the target location in polar coordinates:
  double distance = sqrt(msg.x * msg.x + msg.y * msg.y);
  double angle = atan2(msg.y, msg.x);

  //Now map to the other frame of reference in polar coordinates:
  angle += PI;
  // and add the rotation theta to this frame of reference:
  // The robot angle within the target frame of reference:

  double theta = msg.theta * -1;
  // Rotate the new frame of reference with this angle:
  angle += theta;

  //So now the robot coordinates in the target frame of reference is:
  target_point.x = distance * cos(angle);
  target_point.y = distance * sin(angle);
  // and theta is the angle of the robot in the target frame of reference.
  // Use these coordinates for position control calculations
  target_point.theta = theta;

  state = State::LINEAR_POSITION_CONTROL;
}

// ***** The subscribers *********

ros::Subscriber<geometry_msgs::Pose2D> pose_sub("pos_cmd", pos_cmd_cb);
//ros::Subscriber<geometry_msgs::Pose2D> pose_sub_nl("pos_cmd_nl", pos_nl_cmd_cb);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", vel_cmd_cb);
ros::Subscriber<std_msgs::Float32> pid_p_sub("PID/P", P_cb);
ros::Subscriber<std_msgs::Float32> pid_i_sub("PID/I", I_cb);
ros::Subscriber<std_msgs::Float32> pid_d_sub("PID/D", D_cb);
//add your publisher/subscriber objects here

//This function is called after the Teensy is reprogrammed, or when powered on.
void setup()
{
  //This part initializes the ROS node, and tells ROS about the declared publishers and subscribers.
  node_handle.initNode();
  node_handle.advertise(dbg_pub);
  node_handle.advertise(state_publisher);
  node_handle.advertise(left_wheel_publisher);
  node_handle.advertise(right_wheel_publisher);
  node_handle.advertise(target_point_publisher);
  node_handle.subscribe(cmd_vel_sub);
  node_handle.subscribe(pose_sub);
  //node_handle.subscribe(pose_sub_nl);
  node_handle.subscribe(pid_p_sub);
  node_handle.subscribe(pid_i_sub);
  node_handle.subscribe(pid_d_sub);

  //This part sets the hardware I/O configuration
  pinMode(APHASE, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BPHASE, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBYA, OUTPUT);
  pinMode(STBYB, OUTPUT);
  analogWriteFrequency(PWMA, 10000);
  analogWriteFrequency(PWMB, 10000);
  analogWriteResolution(12);

  //This part sets the PID target velocities to 0
  PID_left.set_target(0);
  PID_right.set_target(0);

  //This part begins a serial communication
  Serial.begin(57600);

  //This part sets up the timed updates
  MsTimer2::set(UPATE_RATE, timed_update_callback);
  MsTimer2::start();

  //You can do some initialization below here in this function.
}

void timed_update_callback()
{
  //This if scope makes sure that if for some reason this function is not updated properly,
  // the PID's get instructions to stop the wheels.
  if (millis() - last_time > 500)
  {
    PID_left.set_target(0);
    PID_right.set_target(0);
    last_time = millis();
  }

  long new_left = motor_left.readAndReset();   //get the newest encoder values for the left wheel
  long new_right = motor_right.readAndReset(); //get the newest encoder values for the right wheel

  float vel_left = (float)new_left / PER_ROT * (1000.0 / UPATE_RATE) * 2 * PI;   // calculate the wheel velocity for the left wheel here in rads/s.
  float vel_right = (float)new_right / PER_ROT * (1000.0 / UPATE_RATE) * 2 * PI; // calculate the wheel velocity for the right wheel here in rads/s.

  double curr_v = (WHEEL_RADIUS / 2) * (vel_left + vel_right);
  double curr_omega = (WHEEL_RADIUS / TRACK) * (vel_right - vel_left);

  target_point.theta += curr_omega / (1000.0 / UPATE_RATE);
  target_point.x += curr_v * cos(target_point.theta) / (1000.0 / UPATE_RATE);
  target_point.y += curr_v * sin(target_point.theta) / (1000.0 / UPATE_RATE);

  double distance, beta, alpha, v, omega;

  distance = sqrt(target_point.x * target_point.x + target_point.y * target_point.y); // rho
  if (distance < 0.05) {
    char_msg.data = 'v';
    state_publisher.publish(&char_msg);
  }
  // beta = -atan2(-target_point.y, -target_point.x);
  // alpha = -beta - target_point.theta;
  
  alpha = -target_point.theta + atan2(-target_point.y, -target_point.x);
  while(alpha < -PI){
    alpha += 2*PI;
  }
  while(alpha > PI){
    alpha -= 2* PI;
  }
  beta = -target_point.theta - alpha;

  switch (state) {

    case State::VELOCITY_CONTROL:
    {
      break;
    }
    case State::LINEAR_POSITION_CONTROL:
    {
     
      if ((distance < 0.05) && (abs(target_point.theta) < 0.1)) {
        node_handle.loginfo("Done!");
        setTargetVelocity(0,0);
        state = State::VELOCITY_CONTROL;
      } 
      v = k_rho * distance;
      omega = k_alpha * alpha + k_beta * beta;

      double target = 0.1;
      double scale = target / v;
      v = target;
      omega = scale * omega;
      if (fabs(omega)> PI/2)
      {
       scale = 1.57/fabs(omega);
       omega *= scale;
        v *= scale;
      }
      setTargetVelocity(v, omega);

    break;
    }

    case State::NON_LINEAR_POSITION_CONTROL:
    {
      beta = -1 * beta;
    
      if ((distance < 0.05) && (abs(target_point.theta) < 0.2)) {
        node_handle.loginfo("Done!");
        setTargetVelocity(0,0);
        state = State::VELOCITY_CONTROL;
      } 
      v = k_1 * distance * cos(alpha);
      omega = k_1 * ( sin(alpha) / alpha ) * cos(alpha) * (alpha + lamb_2 * beta) + k_2 * alpha;

      double target = 0.2;
      double scale = target / v;
      v = target;
      omega = scale * omega;
      if (fabs(omega)> PI/2)
      {
        scale = 1.57/fabs(omega);
       omega *= scale;
        v *= scale;
      }
      setTargetVelocity(v, omega);

    break;
    }
  }

  float left_throttle = PID_left.update(vel_left);
  float right_throttle = PID_right.update(vel_right);
  drive_motors(left_throttle, right_throttle);

  // The following lines publish the calculated velocities. This is required for the visualisation of current velocities.

  wheel_msg.data = alpha;
  dbg_pub.publish(&wheel_msg);
  wheel_msg.data = distance;
  left_wheel_publisher.publish(&wheel_msg);
  wheel_msg.data = vel_right;
  right_wheel_publisher.publish(&wheel_msg);

  target_point_publisher.publish(&target_point);
}

//Handles communication with the SerialNode on the Jetson. Don't touch this function.
void loop()
{
  node_handle.spinOnce();
  delay(1);
}
