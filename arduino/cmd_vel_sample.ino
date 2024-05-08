#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// --- --- --- Encoder Part --- --- ---
// Encoder A pin definition 
const uint16_t AENCA = 35;        // Encoder A input A_C2(B)
const uint16_t AENCB = 34;        // Encoder A input A_C1(A)
// Encoder B pin definition   
const uint16_t BENCB = 16;        // Encoder B input B_C2(B)
const uint16_t BENCA = 27;        // Encoder B input B_C1(A)

//To calculate the number of transitions between high and low levels of a Hall sensor of the encoder within a given "interval" time (in milliseconds)
//If you are using RISING edge detection for initializing interrupts, you'll specifically count the number of transitions from low to high level
volatile long A_wheel_pulse_count  = 0;
volatile long B_wheel_pulse_count = 0;
void IRAM_ATTR A_wheel_pulse();
void IRAM_ATTR B_wheel_pulse();

//To calculate the period for calculating speed, you need to specify the interval at which you want to compute the speed
int interval = 100;

//The current time  
long currentMillis = 0;

// The reduction ratio of the motor means that the motor speed and the output shaft speed of the reduced motor are different.
// For example, with the DCGM3865 motor, with a reduction ratio of 1:42, it means that for every 42 revolutions of the motor, the output shaft completes 1 revolution.
// For each revolution of the output shaft, the motor needs to complete more revolutions as the reduction ratio increases. Typically, higher reduction ratios result in greater torque.
// Take the DCGM3865 motor (reduction ratio: 1：42) as an example:
double reduction_ratio = 42;

//Number of encoder lines, one revolution of the motor, the number of high and low level changes of one Hall sensor of the encoder
int ppr_num = 11;

// When the output shaft completes one revolution, the number of transitions observed in a single Hall sensor
double shaft_ppr = reduction_ratio * ppr_num;

// The callback function of the interrupt function, refer to the attachInterrupt() function later
// A_wheel_pulse function 
void IRAM_ATTR A_wheel_pulse() {
  if(digitalRead(AENCA)){
    A_wheel_pulse_count++;
  }
  else{
    A_wheel_pulse_count--;
  }
}
// B_wheel_pulse function 
void IRAM_ATTR B_wheel_pulse() {
  if(digitalRead(BENCA)){
    B_wheel_pulse_count++;
  }
  else{
    B_wheel_pulse_count--;
  }
}
// --- --- --- --- --- --- --- --- ---

// --- --- --- Motor Part --- --- ---
// The following defines how to control the ESP32 pin of TB6612
// Motor A
const uint16_t PWMA = 25;                           // Motor A PWM control  Orange
const uint16_t AIN1 = 21;        // Left backward   // Motor A input 1      Green 
const uint16_t AIN2 = 17;        // Left forward    // Motor A input 2      Brown
// Motor B 
const uint16_t BIN1 = 22;        // Right backward  // Motor B input 1       Yellow
const uint16_t BIN2 = 23;        // Right forward   // Motor B input 2       Purple
const uint16_t PWMB = 26;                           // Motor B PWM control   White

// PWM frequency of pins used for PWM outputs
int freq = 100000;

// Define PWM channel
int channel_A = 5;
int channel_B = 6;

// Defines PWM accuracy, when it is 8, the PWM value is 0-255 (2^8-1)
const uint16_t ANALOG_WRITE_BITS = 8;
// The maximum PWM value 
const uint16_t MAX_PWM = pow(2, ANALOG_WRITE_BITS)-1;
// The minimum PWM, due to the poor low-speed characteristics of DC motors, may not reach the motor's rotation threshold
const uint16_t MIN_PWM = MAX_PWM/5;

// Motor A control 
void channel_A_Ctrl(float pwmInputA){
  // Round the pwmInput value to the nearest integer
  int pwmIntA = round(pwmInputA);

  // If pwmInput is 0, it stops rotating  
  if(pwmIntA == 0){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    return;
  }

  // Determine the direction of rotation by determining the positive or negative pwmInput value
  if(pwmIntA > 0){
    // Left forward 
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    // constrain() function is for limiting the pwmIntA value between MIN_PWM and MAX_PWM  
    ledcWrite(channel_A, constrain(pwmIntA, MIN_PWM, MAX_PWM));
  }
  else{
    // Left backward
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(channel_A,-constrain(pwmIntA, -MAX_PWM, 0));
  }
}

// Motor B control 
void channel_B_Ctrl(float pwmInputB){
  int pwmIntB = round(pwmInputB);

  // If pwmInput is 0, it stops rotating 
  if(pwmIntB == 0){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    return;
  }

  if(pwmIntB > 0){
    // Right forward
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(channel_B, constrain(pwmIntB, 0, MAX_PWM));
  }
  else{
    // Right backward
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(channel_B,-constrain(pwmIntB, -MAX_PWM, 0));
  }
}
// --- --- --- --- --- --- --- --- ---

// --- --- --- Closed-loop Control  --- --- ---
// PID Controller Parameters  
double Kp = 0.05;   // Scale factor
double Ki = 0.05;   // Integral coefficient
double Kd = 0;   // Differentiation factor

// Target rotation speed and the actual rotation speed  
double targetSpeed_A = 100.0;  // Target rotation speed (Adjustable on request)  
double actualSpeed_A = 0.0;   // The actual rotation speed 
double targetSpeed_B = 100.0;  // The target rotation speed (Adjustable on request) 
double actualSpeed_B = 0.0;   // The actual rotation speed 

// PID controller variables 
double previousError_A = 0.0;
double integral_A = 0.0;
double previousError_B = 0.0;
double integral_B = 0.0;
// --- --- --- --- --- --- --- --- ---

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Function prototypes
void setMotorSpeed(int speedLeft, int speedRight);

void error_loop(){
  while(1){
    delay(100);
  }
}

// Twist message callback
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // Calculate motor speeds based on twist message
  float linear = msg->linear.x;
  float angular = msg->angular.z;

  // Calculate individual motor speeds
  actualSpeed_A = (linear - angular/2)*1000;
  actualSpeed_B = (linear + angular/2)*1000;

  if (actualSpeed_A > 0) {
  actualSpeed_A = actualSpeed_A + 40;
  } 
  if (actualSpeed_A < 0) {
  actualSpeed_A = actualSpeed_A - 40;
  }
  
  if (actualSpeed_B > 0) {
  actualSpeed_B = actualSpeed_B + 40;
  } 
  if (actualSpeed_B < 0) {
  actualSpeed_B = actualSpeed_B - 40;
  }
  
  // Set motor speeds
  channel_A_Ctrl(actualSpeed_A);
  channel_B_Ctrl(actualSpeed_B);
}


void setup() {
  //set_microros_transports();
  set_microros_wifi_transports("alswlwh", "1234567890", "192.168.97.165", 8888);

  // ----------------------------------------------------------------------------------- //
  // Set up the working mode of the encoder's related pins 
  pinMode(BENCB , INPUT_PULLUP);
  pinMode(BENCA , INPUT_PULLUP);
  pinMode(AENCB , INPUT_PULLUP);
  pinMode(AENCA , INPUT_PULLUP);
 
  // Set the interrupt and the corresponding callback function to call the B_wheel_pulse function when BEBCB changes from low to high (RISING)
  attachInterrupt(digitalPinToInterrupt(BENCB), B_wheel_pulse, RISING);
  // Set the interrupt and the corresponding callback function to call the A_wheel_pulse function when AEBCB changes from low to high (RISING) 
  attachInterrupt(digitalPinToInterrupt(AENCB), A_wheel_pulse, RISING);

  //Initialize the serial port, and the baud rate is 115200
  Serial.begin(115200);
  // Wait for serial port initialization to complete 
  while(!Serial){}

  // Setting the operating mode of the ESP32 pin used to control the TB6612FNG  
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // Setting the channel, frequency, and accuracy of the ESP32 pin used for PWM outputs  
  ledcSetup(channel_A, freq, ANALOG_WRITE_BITS);
  ledcAttachPin(PWMA, channel_A);

  ledcSetup(channel_B, freq, ANALOG_WRITE_BITS);
  ledcAttachPin(PWMB, channel_B);

  // The pin used to control rotation is placed at a low level, the motor stops rotating to avoid starting rotation immediately after initialization
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  // ----------------------------------------------------------------------------------- //

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "waverover", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "follower/cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  B_wheel_pulse_count = 0;
  A_wheel_pulse_count = 0;
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
