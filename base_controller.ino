#include <util/atomic.h>
#include <math.h>
 
#include <ros.h>
#include <geometry_msgs/Twist.h>

// Right motor pins (Arduino Mega)
const int RENCA {18};
const int RENCB {19};
const int RPWM {5};
const int RDIR {4};

// Left motor pins
const int LENCA {3};
const int LENCB {2};
const int LPWM {12};
const int LDIR {13};

// for velocity calculation
const float wheel_base {0.27};

//0.27- Wheel base =  Axle length 

//const float counts_per_rotation = 420.0f;
volatile long right_counter{0}, left_counter{0};

// for ros
ros::NodeHandle  nh;

void move_right_motor(float V,float omega){

  // convert to linear vel of wheel in cm/s
  float right_vel = (V + omega * (wheel_base/2)) * 100;

  // set direction
  bool dir = (right_vel < 0 )? true : false;
  digitalWrite(RDIR,dir);

  // Pulse Width Modulation
  // calculate signal using linear eq found emperically
  int pwm = 5.81 * right_vel + 7.56;   // proportional constant and y intercept to found manually 

  // clip pwm signal if is stopped
  pwm = ((V == 0) && (omega == 0))? 0 : pwm;

  // move motor
  analogWrite(RPWM, pwm);
}

void move_left_motor(float V, float omega){

  // convert to linear vel of wheel in cm/s
  float left_vel = (V - omega * (wheel_base/2)) * 100;

  // determine direction
  bool dir = (left_vel < 0 )? false : true;
  digitalWrite(LDIR,dir);

  // calculate signal using linear eq found emperically
  int pwm = 5.93 * left_vel + 6.71;    // proportional constant and y intercept to found manually 

  // clip pwm signal if is stopped
  pwm = ((V == 0) && (omega == 0))? 0 : pwm;

  // move motor
  analogWrite(LPWM, pwm);
  
}

void move_robot(const geometry_msgs::Twist& msg){

  float V = msg.linear.x;
  float omega = msg.angular.z;
  
  move_right_motor(V, omega);
  move_left_motor(V, omega);
  
}


// sub to cmd_vel
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", move_robot);

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(RDIR, OUTPUT);
  pinMode(RENCA, INPUT);
  pinMode(RENCB, INPUT);
  
  pinMode(LPWM, OUTPUT);
  pinMode(LDIR, OUTPUT);
  pinMode(LENCA, INPUT);
  pinMode(LENCB, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(RENCA), update_right_counter, RISING);
  attachInterrupt(digitalPinToInterrupt(LENCA), update_left_counter, RISING);  
//low to high - RISING trigger
  nh.initNode();
  nh.subscribe(sub);

}

void loop() {
  long read_right_counter{},read_left_counter{};
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    read_right_counter = right_counter;
    read_left_counter = left_counter;
  }

  nh.spinOnce(); //returns when ros node shut down
  delay(1);

  
}

void update_right_counter(){
  int encb = digitalRead(RENCB);
  if (encb > 0) --right_counter;
  else ++right_counter;
}

void update_left_counter(){
  int encb = digitalRead(LENCB);
  if (encb > 0) ++left_counter;
  else --left_counter;
}
