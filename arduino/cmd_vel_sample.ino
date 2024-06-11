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


const float L = 0.183;
double actualSpeed_A = 0.0;   // The actual rotation speed 
double actualSpeed_B = 0.0;   // The actual rotation speed 
float vcmd;
float wcmd;


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
const uint16_t MIN_PWM = -MAX_PWM;

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
    ledcWrite(channel_A,constrain(pwmIntA, MIN_PWM, MAX_PWM));
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
    ledcWrite(channel_B,constrain(pwmIntB, MAX_PWM, MIN_PWM));
  }
  else{
    // Right backward
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(channel_B,constrain(pwmIntB, MIN_PWM, MAX_PWM));
  }
}
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
  float vcmd = msg->linear.x;
  float wcmd = msg->angular.z;

  actualSpeed_A = vcmd + (wcmd*L/2);
  actualSpeed_B = vcmd - (wcmd*L/2);

  // Set motor speeds
  channel_A_Ctrl(actualSpeed_A);
  channel_B_Ctrl(actualSpeed_B);
}


void setup() {
  //set_microros_transports();
  set_microros_wifi_transports("alswlwh", "1234567890", "192.168.97.165", 8888);

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
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
