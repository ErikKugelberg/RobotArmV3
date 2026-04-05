#include "motorControl.hpp"

#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

// Define Servo data
#define SERVO4_MIN_PULSEWIDTH_US     1303 // Minimum pulse width in microseconds
#define SERVO4_MID_PULSEWIDTH_US     1412
#define SERVO4_MAX_PULSEWIDTH_US     1534 // Maximum pulse width in microseconds

#define SERVO5_MIN_PULSEWIDTH_US     1000 // Minimum pulse width in microseconds
#define SERVO5_MID_PULSEWIDTH_US     1426
#define SERVO5_MAX_PULSEWIDTH_US     1900 // Maximum pulse width in microseconds

#define SERVO6_MIN_PULSEWIDTH_US     1000 // Minimum pulse width in microseconds
#define SERVO6_MID_PULSEWIDTH_US     1458
#define SERVO6_MAX_PULSEWIDTH_US     2000 // Maximum pulse width in microseconds
 
#define SERVO7_MIN_PULSEWIDTH_US     1000 // Minimum pulse width in microseconds
#define SERVO7_MID_PULSEWIDTH_US     1300
#define SERVO7_MAX_PULSEWIDTH_US     1600 // Maximum pulse width in microseconds 

#define SERVO4_PULSE_GPIO            5   // GPIO connects to the PWM signal line
#define SERVO5_PULSE_GPIO            15  // GPIO connects to the PWM signal line
#define SERVO6_PULSE_GPIO            2   // GPIO connects to the PWM signal line
#define SERVO7_PULSE_GPIO            18   // GPIO connects to the PWM signal line

#define RX_PIN 16
#define TX_PIN 17
#define UART_BAUD 115200  // Gcode baud

Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;

EventGroupHandle_t eventGroup;

QueueHandle_t servoQue = NULL;
QueueHandle_t stepperQue = NULL;

// PWM servo control
void pwmMotorControl (void * par) {
  int pos[4] = {MOTOR4_START,MOTOR5_START,MOTOR6_START,MOTOR7_START};  // Current servo position in degrees
  int dPos[4] = {MOTOR4_START,MOTOR5_START,MOTOR6_START,MOTOR7_START}; // Desired servo position in degrees
  int cyclesToNext = 0; // How many cycles to next desired position
  
  const int maxDiff = 10*MAX_SERVO_SPEED/(1000/SERVO_LOOP_DELAY); // Max steps we can take in one cycle
  AngleStruct inputs;
    
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  servo4.setPeriodHertz(50);      // Standard 50hz servo
  servo5.setPeriodHertz(50);      // Standard 50hz servo
  servo6.setPeriodHertz(50);      // Standard 50hz servo
  servo7.setPeriodHertz(50);      // Standard 50hz servo

  servo4.attach(SERVO4_PULSE_GPIO, SERVO4_MIN_PULSEWIDTH_US, SERVO4_MAX_PULSEWIDTH_US);
  servo5.attach(SERVO5_PULSE_GPIO, SERVO5_MIN_PULSEWIDTH_US, SERVO5_MAX_PULSEWIDTH_US);
  servo6.attach(SERVO6_PULSE_GPIO, SERVO6_MIN_PULSEWIDTH_US, SERVO6_MAX_PULSEWIDTH_US);
  servo7.attach(SERVO7_PULSE_GPIO, SERVO7_MIN_PULSEWIDTH_US, SERVO7_MAX_PULSEWIDTH_US);

  servo4.setTimerWidth(16); // 16 bit pwm timer resolution
  servo5.setTimerWidth(16);
  servo6.setTimerWidth(16);
  servo7.setTimerWidth(16);

  vTaskDelay(1000/portTICK_PERIOD_MS);
  
  if (pos[MOTOR4-3] <= 0) {
    servo4.writeMicroseconds(map(pos[MOTOR4-3], MOTOR4_L_LIMIT, 0, SERVO4_MIN_PULSEWIDTH_US, SERVO4_MID_PULSEWIDTH_US));
  } else {
    servo4.writeMicroseconds(map(pos[MOTOR4-3], 0, MOTOR4_H_LIMIT, SERVO4_MID_PULSEWIDTH_US, SERVO4_MAX_PULSEWIDTH_US));
  }
  if (pos[MOTOR5-3] <= 0) {
    servo5.writeMicroseconds(map(pos[MOTOR5-3], MOTOR5_L_LIMIT, 0, SERVO5_MIN_PULSEWIDTH_US, SERVO5_MID_PULSEWIDTH_US));
  } else {
    servo5.writeMicroseconds(map(pos[MOTOR5-3], 0, MOTOR5_H_LIMIT, SERVO5_MID_PULSEWIDTH_US, SERVO5_MAX_PULSEWIDTH_US));
  }
  if (pos[MOTOR6-3] <= 0) {
    servo6.writeMicroseconds(map(pos[MOTOR6-3], MOTOR6_L_LIMIT, 0, SERVO6_MIN_PULSEWIDTH_US, SERVO6_MID_PULSEWIDTH_US));
  } else {
    servo6.writeMicroseconds(map(pos[MOTOR6-3], 0, MOTOR6_H_LIMIT, SERVO6_MID_PULSEWIDTH_US, SERVO6_MAX_PULSEWIDTH_US));
  }
  if (pos[MOTOR7-3] <= 0) {
    servo7.writeMicroseconds(map(pos[MOTOR7-3], MOTOR7_L_LIMIT, 0, SERVO7_MIN_PULSEWIDTH_US, SERVO7_MID_PULSEWIDTH_US));
  } else {
    servo7.writeMicroseconds(map(pos[MOTOR7-3], 0, MOTOR7_H_LIMIT, SERVO7_MID_PULSEWIDTH_US, SERVO7_MAX_PULSEWIDTH_US));
  }
  
  long long before = millis();
  while (1) {
    while (uxQueueMessagesWaiting(servoQue) != 0) { // Make sure to get the latest message
      xQueueReceive(servoQue, &inputs, 0);
      dPos[0] = inputs.theta1;
      dPos[1] = inputs.theta2;
      dPos[2] = inputs.theta3;
      dPos[3] = inputs.theta4;
      cyclesToNext = inputs.cycles;
    }
    
    if ((pos[MOTOR4-3] != dPos[MOTOR4-3]) || (pos[MOTOR5-3] != dPos[MOTOR5-3]) || (pos[MOTOR6-3] != dPos[MOTOR6-3]) || (pos[MOTOR7-3] != dPos[MOTOR7-3])) {
      for (int i = 0;i < 4;i++) {
        if (cyclesToNext <= 1) {
          pos[i] = dPos[i];
        } else {
          pos[i] += (dPos[i] - pos[i])/cyclesToNext;
        }
      }
      cyclesToNext--;
      
      float pulse;
      if (pos[MOTOR4-3] <= 0) {
          pulse = (float)(pos[MOTOR4-3] - MOTOR4_L_LIMIT) / (0 - MOTOR4_L_LIMIT) * (SERVO4_MID_PULSEWIDTH_US - SERVO4_MIN_PULSEWIDTH_US) + SERVO4_MIN_PULSEWIDTH_US;
      } else {
          pulse = (float)(pos[MOTOR4-3]) / MOTOR4_H_LIMIT * (SERVO4_MAX_PULSEWIDTH_US - SERVO4_MID_PULSEWIDTH_US) + SERVO4_MID_PULSEWIDTH_US;
      }
      servo4.writeMicroseconds((int)pulse);
      if (pos[MOTOR5-3] <= 0) {
          pulse = (float)(pos[MOTOR5-3] - MOTOR5_L_LIMIT) / (0 - MOTOR5_L_LIMIT) * (SERVO5_MID_PULSEWIDTH_US - SERVO5_MIN_PULSEWIDTH_US) + SERVO5_MIN_PULSEWIDTH_US;
      } else {
          pulse = (float)(pos[MOTOR5-3]) / MOTOR5_H_LIMIT * (SERVO5_MAX_PULSEWIDTH_US - SERVO5_MID_PULSEWIDTH_US) + SERVO5_MID_PULSEWIDTH_US;
      }
      servo5.writeMicroseconds((int)pulse);
      if (pos[MOTOR6-3] <= 0) {
          pulse = (float)(pos[MOTOR6-3] - MOTOR6_L_LIMIT) / (0 - MOTOR6_L_LIMIT) * (SERVO6_MID_PULSEWIDTH_US - SERVO6_MIN_PULSEWIDTH_US) + SERVO6_MIN_PULSEWIDTH_US;
      } else {
          pulse = (float)(pos[MOTOR6-3]) / MOTOR6_H_LIMIT * (SERVO6_MAX_PULSEWIDTH_US - SERVO6_MID_PULSEWIDTH_US) + SERVO6_MID_PULSEWIDTH_US;
      }
      servo6.writeMicroseconds((int)pulse);
      if (pos[MOTOR7-3] <= 0) {
          pulse = (float)(pos[MOTOR7-3] - MOTOR7_L_LIMIT) / (0 - MOTOR7_L_LIMIT) * (SERVO7_MID_PULSEWIDTH_US - SERVO7_MIN_PULSEWIDTH_US) + SERVO7_MIN_PULSEWIDTH_US;
      } else {
          pulse = (float)(pos[MOTOR7-3]) / MOTOR7_H_LIMIT * (SERVO7_MAX_PULSEWIDTH_US - SERVO7_MID_PULSEWIDTH_US) + SERVO7_MID_PULSEWIDTH_US;
      }
      servo7.writeMicroseconds((int)pulse);
      //Serial.println(String(servo4.readMicroseconds()) + "   " + String(servo5.readMicroseconds()) + "   " + String(servo6.readMicroseconds()));
    } 

    if ((pos[MOTOR4-3] == dPos[MOTOR4-3]) && (pos[MOTOR5-3] == dPos[MOTOR5-3]) && (pos[MOTOR6-3] == dPos[MOTOR6-3]) && (pos[MOTOR7-3] == dPos[MOTOR7-3])) {
      xEventGroupSetBits(eventGroup, TASK2_DONE_BIT); // Notify that position is reached
    }
    
    int del = constrain((SERVO_LOOP_DELAY-(millis()-before)),0,SERVO_LOOP_DELAY);
    vTaskDelay(del/portTICK_PERIOD_MS);
    before = millis();
    //Serial.println("Stepper loop: " + String(del));
    //Serial.println(uxTaskGetStackHighWaterMark( NULL ));
  }
}

// Serial motor grbl control, send new positions based on current speed
void serialMotorControl (void * par) {  
  int pos[3] = {MOTOR1_START, MOTOR2_START, MOTOR3_START};    // Current position
  int dPos[3] = {MOTOR1_START, MOTOR2_START, MOTOR3_START}; // Desired position
  int cyclesToNext = 0; // How many cycles to next desired position
  AngleStruct inputs;
  int deltaPos[3];                                            // Keeps track of how much each position has changed
  float actSpd[3];                                            // Actual speed, for feed calculation
  const int maxDiff = 10*MAX_STEPPER_SPEED/(1000/MOTOR_LOOP_DELAY); // Max steps we can take in one cycle
  
  Serial2.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  vTaskDelay(100/portTICK_PERIOD_MS);
  
  // Set up correct settings on arduino uno
  Serial2.println("$RST=$");          // Reset to default settings
  vTaskDelay(100/portTICK_PERIOD_MS);
  
  // X-Axis settings
  Serial2.println("$100=20");         // nr steps to go 1 degree in X
  vTaskDelay(100/portTICK_PERIOD_MS); 
  Serial2.println("$110=5000");       // max rate 1 degrees/min in X
  vTaskDelay(100/portTICK_PERIOD_MS); 
  Serial2.println("$120=300");        // acceleration 1 degrees/s^2 in X
  vTaskDelay(100/portTICK_PERIOD_MS); 

  // Y-Axis settings
  Serial2.println("$101=20");         // nr steps to go 1 degree in Y
  vTaskDelay(100/portTICK_PERIOD_MS); 
  Serial2.println("$111=5000");       // max rate 1 degrees/min in Y
  vTaskDelay(100/portTICK_PERIOD_MS); 
  Serial2.println("$121=300");        // acceleration 1 degrees/s^2 in Y
  vTaskDelay(100/portTICK_PERIOD_MS); 

  // Z-Axis settings
  Serial2.println("$102=12.7");         // nr steps to go 1 degree in Z
  vTaskDelay(100/portTICK_PERIOD_MS); 
  Serial2.println("$112=5000");       // max rate 1 degrees/min in Z
  vTaskDelay(100/portTICK_PERIOD_MS); 
  Serial2.println("$122=300");        // acceleration 1 degrees/s^2 in Z
  vTaskDelay(100/portTICK_PERIOD_MS); 
  
  Serial2.println("G92 X" + String(MOTOR1_START/10) + " Y" + String(-MOTOR2_START/10) + " Z" + String(-MOTOR3_START/10));   // Set starting position
  vTaskDelay(100/portTICK_PERIOD_MS); 

  // Empty serial respone
  while (Serial2.available() > 0) {
    String incoming  = Serial2.readString();
    Serial.println(incoming);
  }

  long long before = millis();
  while (1) { 
    // Get wanted speeds from controller input
    while (uxQueueMessagesWaiting(stepperQue) != 0) { // Make sure to get the latest message
      xQueueReceive(stepperQue, &inputs, 0);
      dPos[MOTOR1] = inputs.theta1;
      dPos[MOTOR2] = inputs.theta2;
      dPos[MOTOR3] = inputs.theta3;
      cyclesToNext = inputs.cycles;
    }

    // Check if the desired positions have changed
    if ((pos[MOTOR1] != dPos[MOTOR1]) || (pos[MOTOR2] != dPos[MOTOR2]) || (pos[MOTOR3] != dPos[MOTOR3])) {
      String posCmd = "G01";

      // Calculate new motor positions based on loop time and speed
      for (int i = 0;i < 3;i++) {
        if (cyclesToNext <= 1) {
          deltaPos[i] = dPos[i] - pos[i];
        } else {
          //deltaPos[i] = (dPos[i] - pos[i])/cyclesToNext;
          deltaPos[i] = (dPos[i] - pos[i] + (cyclesToNext / 2)) / cyclesToNext;
        }
        pos[i] += deltaPos[i];
      }
      cyclesToNext--;

      // Add positions to gcode command if they have changed
      if (deltaPos[MOTOR1] != 0){
        actSpd[MOTOR1] = abs((float)deltaPos[MOTOR1]*6000/MOTOR_LOOP_DELAY); // degrees/minute ????
        posCmd += " X";
        posCmd += String((float)pos[MOTOR1] / 10, 1);
      } else {
        actSpd[MOTOR1] = 0;
      }

      if (deltaPos[MOTOR2] != 0){
        actSpd[MOTOR2] = abs((float)deltaPos[MOTOR2]*6000/MOTOR_LOOP_DELAY); // degrees/minute
        posCmd += " Y";
        posCmd += String((float)(-pos[MOTOR2]) / 10, 1); // invert coordinate
      } else {
        actSpd[MOTOR2] = 0;
      }

      if (deltaPos[MOTOR3] != 0){
        actSpd[MOTOR3] = abs((float)deltaPos[MOTOR3]*6000/MOTOR_LOOP_DELAY); // degrees/minute 
        posCmd += " Z";
        posCmd += String((float)(-pos[MOTOR3]) / 10, 1);
      } else {
        actSpd[MOTOR3] = 0;
      }

      // Calculate feed rate based on new positions
      float totSpd = sqrt(actSpd[MOTOR1]*actSpd[MOTOR1] + actSpd[MOTOR2]*actSpd[MOTOR2] + actSpd[MOTOR3]*actSpd[MOTOR3]);
      if (totSpd != 0) {
        posCmd += " F";
        posCmd += String(totSpd, 0);
        Serial2.println(posCmd);  // Send the command
        Serial.println(posCmd);
      }
    }

    if ((pos[MOTOR1] == dPos[MOTOR1]) && (pos[MOTOR2] == dPos[MOTOR2]) && (pos[MOTOR3] == dPos[MOTOR3])) {
      xEventGroupSetBits(eventGroup, TASK1_DONE_BIT); // Notify
    }
    
    int del = constrain((MOTOR_LOOP_DELAY-(millis()-before)),0,MOTOR_LOOP_DELAY);
    vTaskDelay(del/portTICK_PERIOD_MS);
    before = millis();
    //Serial.println("Stepper loop: " + String(del));
    //Serial.println(uxTaskGetStackHighWaterMark( NULL ));
  }
}
