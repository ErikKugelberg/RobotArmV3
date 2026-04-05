#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#define MOTOR_LOOP_DELAY 30 // How often we send Gcode in ms, code needs around 1ms
#define SERVO_LOOP_DELAY 20 // How often we update servo positions in ms,  code needs around 1ms

#define MAX_STEPPER_SPEED 30 // degrees/second, this applies to all joints
#define MAX_SERVO_SPEED 90

// Define bit masks for each task's completion
#define TASK1_DONE_BIT (1 << 0)
#define TASK2_DONE_BIT (1 << 1)

// Define actuator limits
#define MOTOR7_H_LIMIT  450 // Gripper spin
#define MOTOR7_L_LIMIT  -450
#define MOTOR7_START    0

#define MOTOR6_H_LIMIT  630 // Gripper spin
#define MOTOR6_L_LIMIT  -630
#define MOTOR6_START    0

#define MOTOR5_H_LIMIT  1075 // Wrist
#define MOTOR5_L_LIMIT  -1075
#define MOTOR5_START    0     

#define MOTOR4_H_LIMIT  1800 // Wrist twist 
#define MOTOR4_L_LIMIT  -1800
#define MOTOR4_START    0

#define MOTOR3_H_LIMIT  880 // Z-axis, "elbow joint"
#define MOTOR3_L_LIMIT  -900 
#define MOTOR3_START    880

#define MOTOR2_H_LIMIT  900 // Y-axis, "shoulder"
#define MOTOR2_L_LIMIT  -900 
#define MOTOR2_START    -900

#define MOTOR1_H_LIMIT  900 // X-axis, "base rotation"
#define MOTOR1_L_LIMIT  -900  
#define MOTOR1_START    0  


// Struct for sending three angle values (in degrees) between controller and motor tasks
struct AngleStruct {
  int theta1;
  int theta2;
  int theta3; 
  int theta4; // For end-effector, if used 
  int cycles;
};

enum {
  MOTOR1,
  MOTOR2,
  MOTOR3,
  MOTOR4,
  MOTOR5,
  MOTOR6,
  MOTOR7,
  NUM_MOTORS  
};

// Define queues
extern QueueHandle_t servoQue;
extern QueueHandle_t stepperQue;

// Event group handle
extern EventGroupHandle_t eventGroup;


void serialMotorControl (void * par);

void pwmMotorControl (void * par);

#endif 
