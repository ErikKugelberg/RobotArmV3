#include "calculations.hpp"

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "communication.hpp"
#include "motorControl.hpp"

#define CONTROL_LOOP_DELAY 60 // How often we calculate desired joint angles based on input, code needs around 6ms

#define MAX_PATH_STEPS 100
#define JOG_JOINT_SPEED 30 // degrees/second
#define JOG_CARTESIAN_TRANS_SPEED 10 // mm/second
#define JOG_CARTESIAN_ROT_SPEED 10 // deg/second
#define PATH_CARTESIAN_TRANS_SPEED 5 // mm/second
#define PATH_CARTESIAN_ROT_SPEED 5 // deg/second
#define PATH_JOINT_SPEED 15

#define DISCONNECT_MS 500
#define SERIAL_DEBUG false
#define STICK_DEADZONE 1
#define SWITCH1_MASK 0x01
#define SWITCH2_MASK 0x02
#define BUTTON1_MASK 0x04
#define BUTTON2_MASK 0x08
#define BUTTON3_MASK 0x10

//#define WRIST_LENGTH 68.5 // distance to tool holder
#define WRIST_LENGTH 145.5f // distance to gripper

struct Move {
  uint8_t type;
  float values[7];
};

enum {
  UNINITIALIZED, 
  JMOVE,
  LMOVE,
  NUM_MODES
};

// From joint space to operational space
void forwardKinematics (int* q, float* x) {
  float c1 = cos((float)q[0]*PI/1800.0);
  float s1 = sin((float)q[0]*PI/1800.0);
  float c2 = cos((float)q[1]*PI/1800.0-PI/2.0);
  float s2 = sin((float)q[1]*PI/1800.0-PI/2.0);
  float c3 = cos((float)q[2]*PI/1800.0);
  float s3 = sin((float)q[2]*PI/1800.0);
  float c4 = cos((float)q[3]*PI/1800.0);
  float s4 = sin((float)q[3]*PI/1800.0);
  float c5 = cos((float)q[4]*PI/1800.0);
  float s5 = sin((float)q[4]*PI/1800.0);
  float c6 = cos((float)q[5]*PI/1800.0);
  float s6 = sin((float)q[5]*PI/1800.0);

  // not matching rotation with baseframe
  float r11 = s6*(c4*s1 - s4*(c1*c2*c3 - c1*s2*s3)) - c6*(s5*(c1*c2*s3 + c1*c3*s2) - c5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3)));
  float r21 = - c6*(s5*(c2*s1*s3 + c3*s1*s2) + c5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3))) - s6*(c1*c4 + s4*(c2*c3*s1 - s1*s2*s3));
  float r31 = s4*s6*(c2*s3 + c3*s2) - c6*(s5*(c2*c3 - s2*s3) + c4*c5*(c2*s3 + c3*s2));
  float r32 = s6*(s5*(c2*c3 - s2*s3) + c4*c5*(c2*s3 + c3*s2)) + c6*s4*(c2*s3 + c3*s2);
  float r33 = c4*s5*(c2*s3 + c3*s2) - c5*(c2*c3 - s2*s3);
  

  x[0] = 150.0*c1*c2 - 13.4*s1 - WRIST_LENGTH*c5*(c1*c2*s3 + c1*c3*s2) - WRIST_LENGTH*s5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3)) + 49.0*c1*c2*c3 - 110.0*c1*c2*s3 - 110.0*c1*c3*s2 - 49.0*c1*s2*s3;  // x
  x[1] = 13.4*c1 + 150.0*c2*s1 - WRIST_LENGTH*c5*(c2*s1*s3 + c3*s1*s2) + WRIST_LENGTH*s5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3)) + 49.0*c2*c3*s1 - 110.0*c2*s1*s3 - 110.0*c3*s1*s2 - 49.0*s1*s2*s3;  // y
  x[2] = 110.0*s2*s3 - 110.0*c2*c3 - 49.0*c2*s3 - 49.0*c3*s2 - 150.0*s2 - WRIST_LENGTH*c5*(c2*c3 - s2*s3) + WRIST_LENGTH*c4*s5*(c2*s3 + c3*s2);                                                 // z 
  
  x[3] = atan2(r21,r11);  // Yaw (alpha)
  x[4] = asin(-r31);      // Pitch (beta) 
  x[5] = atan2(r32,r33);  // Roll (gamma)
}


// Gradient Descent to calculate first three angles of inverse kinematics
void gradientDescent(float x_target, float y_target, float z_target, float &theta1, float &theta2, float &theta3) {
  float learning_rate = 0.000008; // Small learning rate
  float tolerance = 0.0000001;      // Convergence criteria
  float fx, fy, fz, error;

  for (int i = 0; i < 1000; i++) {  // Max iterations
    float c1 = cos(theta1), c2 = cos(theta2), c3 = cos(theta3);
    float s1 = sin(theta1), s2 = sin(theta2), s3 = sin(theta3);
    
    // reduce computation
    float c1c2 = c1*c2;
    float c2s1 = c2*s1;
    float c2c3 = c2*c3;
    float c3s2 = c3*s2;
    float c1c2s3 = c1c2*s3;
    float c1c2c3 = c1c2*c3;
    float c1c3s2 = c1*c3s2;
    float c1s2s3 = c1*s2*s3;
    float c2s1s3 = c2s1*s3;
    float c3s1s2 = c3*s1*s2;
    float s1s2s3 = s1*s2*s3;
    float c2c3s1 = c2c3*s1;

    fx = 150.0*c1c2 - 13.4*s1 + 49.0*c1c2c3 - 110.0*c1c2s3 - 110.0*c1c3s2 - 49.0*c1s2s3;
    fy = 13.4*c1 + 150.0*c2s1 + 49.0*c2c3s1 - 110.0*c2s1s3 - 110.0*c3s1s2 - 49.0*s1s2s3;
    fz = 110.0*s2*s3 - 110.0*c2c3 - 49.0*c2*s3 - 49.0*c3s2 - 150.0*s2;
    
    // Calculate error
    float ex = x_target - fx;
    float ey = y_target - fy;
    float ez = z_target - fz;
    error = ex * ex + ey * ey + ez * ez;  // Sum of squared errors
    
    if (error < tolerance) {
      //Serial.println(i);
      break; 
    }  // Stop if error is small enough

    // Compute partial derivatives (you need to implement these in the model function)
    float dfx_dtheta1 = 110.0*c2s1s3 - 150.0*c2s1 - 13.4*c1 + 110.0*c3s1s2 + 49.0*s1s2s3 - 49.0*c2c3s1;
    float dfx_dtheta2 = 110.0*c1s2s3 - 150.0*c1*s2 - 110.0*c1c2c3 - 49.0*c1c2s3 - 49.0*c1c3s2;
    float dfx_dtheta3 = 110.0*c1s2s3 - 110.0*c1c2c3 - 49.0*c1c2s3 - 49.0*c1c3s2;
    float dfy_dtheta1 = 150.0*c1c2 - 13.4*s1 - 49.0*c1s2s3 + 49.0*c1c2c3 - 110.0*c1c2s3 - 110.0*c1c3s2;
    float dfy_dtheta2 = 110.0*s1s2s3 - 49.0*c2s1s3 - 49.0*c3s1s2 - 150.0*s1*s2 - 110.0*c2c3s1;
    float dfy_dtheta3 = 110.0*s1s2s3 - 49.0*c3s1s2 - 49.0*c2s1s3 - 110.0*c2c3s1;
    float dfz_dtheta1 = 0;
    float dfz_dtheta2 = 110.0*c2*s3 - 49.0*c2c3 - 150.0*c2 + 110.0*c3s2 + 49.0*s2*s3;
    float dfz_dtheta3 = 110.0*c2*s3 - 49.0*c2c3 + 110.0*c3s2 + 49.0*s2*s3;

    // Update theta1
    theta1 += learning_rate * (ex * dfx_dtheta1 + ey * dfy_dtheta1 + ez * dfz_dtheta1);
    // Update theta2
    theta2 += learning_rate * (ex * dfx_dtheta2 + ey * dfy_dtheta2 + ez * dfz_dtheta2);
    // Update theta3
    theta3 += learning_rate * (ex * dfx_dtheta3 + ey * dfy_dtheta3 + ez * dfz_dtheta3);
    //Serial.println("Ex: " + String(ex,1) + "   Ey: " + String(ey,1) + "   Ez: " + String(ez,1));
  }
}

boolean withinRange(float x, float a, float b) {
  return ((x >= a) && (x <= b));
}

// Calculate desired joint angles based on operational space configuration
// q_des in degree, prevQ in radians
boolean inverseKinematics(int* q_des, float* x, bool sendFB) {
  // Kinematic decoupling
  float calculatedQ[6];
  // Calculate centerpoint of first 3 axis:
  float xc[3];
  float prevQ[6] = {(float)(q_des[MOTOR1]*PI/1800.0), (float)(q_des[MOTOR2]*PI/1800.0), (float)(q_des[MOTOR3]*PI/1800.0), (float)(q_des[MOTOR4]*PI/1800.0), (float)(q_des[MOTOR5]*PI/1800.0), (float)(q_des[MOTOR6]*PI/1800.0)};
  
  //  yaw = x[3], pitch = x[4], roll = x[5]
  float sa = sin(x[3]), sb = sin(x[4]), sg = sin(x[5]);
  float ca = cos(x[3]), cb = cos(x[4]), cg = cos(x[5]);
  xc[0] = x[0] - WRIST_LENGTH*(ca*sb*cg+sa*sg);  //x
  xc[1] = x[1] - WRIST_LENGTH*(sa*sb*cg-ca*sg);  //y
  xc[2] = x[2] - WRIST_LENGTH*(cb*cg);           //z

 // Initial guess for theta1, theta2, theta3, use previous values
  calculatedQ[MOTOR1] = prevQ[MOTOR1];
  calculatedQ[MOTOR2] = prevQ[MOTOR2] - PI/2.0;
  calculatedQ[MOTOR3] = prevQ[MOTOR3];
  
  // Solve using gradient descent
  gradientDescent(xc[0], xc[1], xc[2], calculatedQ[MOTOR1], calculatedQ[MOTOR2], calculatedQ[MOTOR3]);

  float c1 = cos(calculatedQ[MOTOR1]), c2 = cos(calculatedQ[MOTOR2]), c3 = cos(calculatedQ[MOTOR3]);
  float s1 = sin(calculatedQ[MOTOR1]), s2 = sin(calculatedQ[MOTOR2]), s3 = sin(calculatedQ[MOTOR3]);

  // Now to solve the final three joints of the wrist:
  
  // theta 5 has two possible answers, pick the one closest to previous position
  float r23 = (sa*sg + ca*cg*sb)*(c1*c2*s3 + c1*c3*s2) - (ca*sg - cg*sa*sb)*(c2*s1*s3 + c3*s1*s2) + cb*cg*(c2*c3 - s2*s3);
  float r22 = (ca*cg + sa*sb*sg)*(c2*s1*s3 + c3*s1*s2) - (cg*sa - ca*sb*sg)*(c1*c2*s3 + c1*c3*s2) + cb*sg*(c2*c3 - s2*s3);
  float r33 = c1*(ca*sg - cg*sa*sb) + s1*(sa*sg + ca*cg*sb);
  float r13 = (sa*sg + ca*cg*sb)*(c1*c2*c3 - c1*s2*s3) - (ca*sg - cg*sa*sb)*(c2*c3*s1 - s1*s2*s3) - cb*cg*(c2*s3 + c3*s2);
  
  float t5_1 = acos(-(r23)), t5_2 = -t5_1;
  if (abs(t5_1 - prevQ[MOTOR5]) < abs(t5_2 - prevQ[MOTOR5])) {
    calculatedQ[MOTOR5] = t5_1;
  } else {
    calculatedQ[MOTOR5] = t5_2;  
  }

  
  float t4 = acos(-(r13)/sin(calculatedQ[MOTOR5]));
  if (!isnan(t4)) { 

    if (r33 < 0) {
      t4 *= -1;
    }
    calculatedQ[MOTOR4] = t4;
  } else {
    // SINGULARITY
    calculatedQ[MOTOR4] = prevQ[MOTOR4];
  }

  float t6_1 = asin(((ca*cg + sa*sb*sg)*(c2*s1*s3 + c3*s1*s2) - (cg*sa - ca*sb*sg)*(c1*c2*s3 + c1*c3*s2) + cb*sg*(c2*c3 - s2*s3))/sin(calculatedQ[4]));
  if (!isnan(t6_1)) {
    calculatedQ[MOTOR6] = t6_1;
  } else {
    // SINGULARITY
    calculatedQ[MOTOR6] = prevQ[MOTOR6];
  }

  // Remove offset needed during inverse calculations
  calculatedQ[MOTOR2] += PI/2.0;

  for (int i = 0;i < 6;i++) {
    calculatedQ[i] *= (1800.0/PI);
  }

  if (sendFB) {
    // Prepare the output for transmission, translate to degrees
    FeedBack fb;
    fb.X = x[0];
    fb.Y = x[1];
    fb.Z = x[2];
    fb.YAW = x[3]*180.0/PI;
    fb.PITCH = x[4]*180.0/PI;
    fb.ROLL = x[5]*180.0/PI;
    // Send the output array over ESP-NOW
    sendData(&fb, sizeof(fb));  // We pass the array and the number of elements
  }
  
  if (!withinRange(calculatedQ[0], (float)MOTOR1_L_LIMIT, (float)MOTOR1_H_LIMIT)) {
    Serial.println("error: joint 1 out of range");
    return false;
  }
  if (!withinRange(calculatedQ[1], MOTOR2_L_LIMIT, MOTOR2_H_LIMIT)) {
    Serial.println("error: joint 2 out of range");
    return false;
  }
  if (!withinRange(calculatedQ[2], MOTOR3_L_LIMIT, MOTOR3_H_LIMIT)) {
    Serial.println("error: joint 3 out of range");
    return false;
  }
  if (!withinRange(calculatedQ[3], MOTOR4_L_LIMIT, MOTOR4_H_LIMIT)) {
    Serial.println("error: joint 4 out of range");
    return false;
  }
  if (!withinRange(calculatedQ[4], MOTOR5_L_LIMIT, MOTOR5_H_LIMIT)) {
    Serial.println("error: joint 5 out of range");
    return false;
  }
  if (!withinRange(calculatedQ[5], MOTOR6_L_LIMIT, MOTOR6_H_LIMIT)) {
    Serial.println("error: joint 6 out of range");
    return false;
  }
  
  for (int i = 0;i < 6;i++) {
    q_des[i] = (int)round(calculatedQ[i]);
  }
  return true;
}

// Send new position data to motor control tasks
void writePosition(QueueHandle_t servoQ, QueueHandle_t stepperQ, int* q_des, int* prevQ, int maxSpeed) {  
  int maxServoDiff;
  int maxStepperDiff;
  if (maxSpeed != -1) {
    maxServoDiff = maxSpeed*SERVO_LOOP_DELAY/100; // Max steps we can take in one cycle
    maxStepperDiff = maxSpeed*MOTOR_LOOP_DELAY/100; // Max steps we can take in one cycle
  } else {
    maxServoDiff = MAX_SERVO_SPEED*SERVO_LOOP_DELAY/100; // Max steps we can take in one cycle
    maxStepperDiff = MAX_STEPPER_SPEED*MOTOR_LOOP_DELAY/100; // Max steps we can take in one cycle
  }
  
  float maxCycles = 0;
  float n = 0;
  for (int i = 0; i < 3;i++) {
    n = abs(q_des[i]-prevQ[i])/maxStepperDiff;
    if (maxCycles < n) {
      maxCycles = n;
    }
    n = abs(q_des[i+3]-prevQ[i+3])/maxServoDiff;
    if (maxCycles < n) {
      maxCycles = n;
    }
  }
  // Special case for gripper
  n = abs(q_des[6]-prevQ[6])/maxServoDiff;
  if (maxCycles < n) {
    maxCycles = n;
  }
  
  AngleStruct stepperAngles;
  stepperAngles.theta1 = q_des[MOTOR1];
  stepperAngles.theta2 = q_des[MOTOR2];
  stepperAngles.theta3 = q_des[MOTOR3];
  stepperAngles.cycles = ((int)maxCycles)+1;
  xQueueSend(stepperQ,&stepperAngles, 0);

  AngleStruct servoAngles;
  servoAngles.theta1 = q_des[MOTOR4];
  servoAngles.theta2 = q_des[MOTOR5];
  servoAngles.theta3 = q_des[MOTOR6];
  servoAngles.theta4 = q_des[MOTOR7];
  servoAngles.cycles = ((int)maxCycles)+1;
  xQueueSend(servoQ,&servoAngles, 0);

  memcpy(prevQ,q_des,7*sizeof(int)); // Save previous position
} 

// Wait for notification that joints have reached the previous desired angles
void waitForJointsFunc() {
  EventBits_t bits = xEventGroupWaitBits(
            eventGroup,                  // The event group handle
            TASK1_DONE_BIT | TASK2_DONE_BIT, // Wait for both bits to be set
            pdTRUE,                      // Clear the bits on exit
            pdTRUE,                      // Wait for both bits (AND logic)
            portMAX_DELAY);              // Wait indefinitely
}

// Calculate desired joint angles based on user input
void jointCalculator (void * par) { 
  servoQue = xQueueCreate(2,sizeof(AngleStruct));
  stepperQue = xQueueCreate(2,sizeof(AngleStruct));

  // Initialize the event group
  eventGroup = xEventGroupCreate();

  vTaskDelay(2000/portTICK_PERIOD_MS);

  int maxSpeed = -1;

  //Move* path = (Move*)malloc(100 * sizeof(Move));  // allocate memory
  Move path[MAX_PATH_STEPS];
  unsigned int currentMove = 0;
  unsigned int totalMoves = 0;
  boolean onPath = false;
  
  long long localLastInput = 0;
  InputStruct inputs = {255,255,255,255,255,255,255,255,255};
  int diffs[7] = {0,0,0,49,49,49,49};  // Recalculated inputs as diffs from 0 to convert into speeds later
  int q_des[7] = {0,0,0,0,0,0,0};       // Desired joint angles in degrees
  int prevQ[7] = {MOTOR1_START,MOTOR2_START,MOTOR3_START, MOTOR4_START,MOTOR5_START,MOTOR6_START,MOTOR7_START};  // Previous position in degrees
  float x[6] = {0};         // Current position in cartesian space, angles in radians
  boolean linearMotion = false;
  boolean rotationalMotion = false;
  float x_des[6] = {0};
  boolean newCartesianPath = true;
  int nrCartesianCycles = 0;

  int cnt = 0;
  
  writePosition(servoQue, stepperQue, q_des, prevQ, maxSpeed);
  waitForJointsFunc();   // Wait for notification that joints have reached the previous desired angles
  
  long long before = millis();
  while (1) {

    inputs.switches &= ~(BUTTON1_MASK | BUTTON2_MASK | BUTTON3_MASK); // Reset buttons so they only activate once
    
    while (uxQueueMessagesWaiting(inputQue) != 0) { // Make sure to get the latest message
      localLastInput = millis();
      xQueueReceive(inputQue, &inputs, 0);
    } 
    
    if (SERIAL_DEBUG) {
      Serial.print("Last Packet Recv Data: "); 
        for (int i = 0;i < 9;i++) {
          //Serial.print(localInputs[i]);
          Serial.print(" ");
        }
        Serial.println("");
    }

    diffs[0] = 0;
    diffs[1] = 0;
    diffs[2] = 0;

    // Check if data is being received from transmitter
    if ((millis() - localLastInput) < DISCONNECT_MS) {
      diffs[0] = -(inputs.jsR_X - 40);
      if ((inputs.jsR_X == 255) || (abs(diffs[0]) < STICK_DEADZONE))  {   
            diffs[0] = 0;     
      } 

      diffs[1] = inputs.jsR_Y - 40;
      if ((inputs.jsR_Y == 255) || (abs(diffs[1]) < STICK_DEADZONE))  {   
            diffs[1] = 0;     
      } 

      diffs[2] = inputs.jsL_Y - 40;
      if ((inputs.jsL_X == 255) || (abs(diffs[2]) < STICK_DEADZONE))  {   
            diffs[2] = 0;     
      }

      if (inputs.pot0 != 255) {
        diffs[3] = inputs.pot0;
      }

      if (inputs.pot1 != 255) {
        diffs[4] = inputs.pot1;
      }

      if (inputs.pot2 != 255) {
        diffs[5] = inputs.pot2;
      }

      if (inputs.pot3 != 255) {
        diffs[6] = inputs.pot3;
      }

      if (inputs.switches & BUTTON3_MASK) {
        path[0].type = UNINITIALIZED; // Remove path
        currentMove = 0;
        totalMoves = 0;
        newCartesianPath = true;
      }
      
      if (inputs.switches & BUTTON2_MASK) {
        /* Couldn't make LMOVE work reliably
        if (inputs.switches & SWITCH1_MASK) {
          path[totalMoves].type = LMOVE;
          for (int i = 0;i < 3;i++) {
            path[totalMoves].values[i] = (int)round((x[i]*10));
            path[totalMoves].values[i+3] = (int)round((x[i+3]*1000));
          }
          path[totalMoves].values[6] = q_des[6];
        } else {
          path[totalMoves].type = JMOVE;
          memcpy(path[totalMoves].values, q_des, 7*sizeof(int));
        }
        */
        path[totalMoves].type = JMOVE;
        memcpy(path[totalMoves].values, q_des, 7*sizeof(int));
        if (totalMoves < (MAX_PATH_STEPS-1)) {
          totalMoves++;
        }     
      }

      if (inputs.switches & SWITCH2_MASK) {
        if (path[0].type != UNINITIALIZED) {
          if (!onPath || (currentMove >= totalMoves)) { // Check if there is a saved path
            onPath = true;
            currentMove = 0;
          }
        } else {
          onPath = false;
        }
      } else {
        onPath = false;
      }

      if (onPath) {
        if (path[currentMove].type == JMOVE) {
          memcpy(q_des, path[currentMove].values, 7*sizeof(int));
          currentMove++;
          maxSpeed = PATH_JOINT_SPEED;
          newCartesianPath = true;
        
        } else if (path[currentMove].type == LMOVE) {
          q_des[6] = path[currentMove].values[6]; // Gripper angle
          if (newCartesianPath) {
            newCartesianPath = false;
            float biggestCartesianTransDiff = 0;  
            float biggestCartesianAngDiff = 0;   
            forwardKinematics(q_des,x); // calculate starting point
            for (int i = 0;i < 3;i++) {
              x_des[i] = (float)(path[currentMove].values[i])/10.0;
              if (abs(x_des[i]-x[i]) > biggestCartesianTransDiff) {
                biggestCartesianTransDiff = abs(x_des[i]-x[i]);
              }
              x_des[i+3] = (float)(path[currentMove].values[i+3])/1000.0;
              if (abs(x_des[i+3]-x[i+3]) > biggestCartesianAngDiff) {
                biggestCartesianAngDiff = abs(x_des[i+3]-x[i+3]);
              }
            }
            float maxCyclesRotational = (int)(biggestCartesianAngDiff*180*1000.0/((float)(PI*PATH_CARTESIAN_ROT_SPEED*CONTROL_LOOP_DELAY)))+1;
            float maxCyclesTranslational = (int)(biggestCartesianTransDiff*1000.0/((float)(PATH_CARTESIAN_TRANS_SPEED*CONTROL_LOOP_DELAY)))+1;
            nrCartesianCycles = max(maxCyclesRotational, maxCyclesTranslational);
            //Serial.println(nrCartesianCycles);
          }
          float xOld[6];
          memcpy(xOld,x,6*sizeof(float));
          if (nrCartesianCycles <= 1) {
            memcpy(x,x_des,6*sizeof(float));
            newCartesianPath = true;
            currentMove++;
          } else {
            for (int i = 0;i < 6;i++) {
              x[i] += ((x_des[i]-x[i])/((float)nrCartesianCycles));
            }
            nrCartesianCycles--;
          }
          if (!inverseKinematics(q_des, x, false)) { // q_des is returned in degrees*10, if angles are within range
            memcpy(x,xOld,6*sizeof(float));
          } 
        }
        
      } else { // NOT ON PATH
        if (inputs.switches & BUTTON1_MASK) {
          if (inputs.switches & SWITCH1_MASK) {
              rotationalMotion = !rotationalMotion;
          } else {
            for (int i = 0;i < 6;i++) { // Return to home
              q_des[i] = 0;
            }
            linearMotion = false;
            maxSpeed = PATH_JOINT_SPEED;
          }
        } else {
          if (inputs.switches & SWITCH1_MASK) {
            if (!linearMotion) {
                forwardKinematics(q_des,x); // takes normal q, no offsets 
                linearMotion = true;
            }
              
            bool sendFB = false;
            if (cnt >= 100/MOTOR_LOOP_DELAY) {
              sendFB = true;
              cnt = 0;
            }
            float factor = JOG_CARTESIAN_TRANS_SPEED*CONTROL_LOOP_DELAY/40000.0; // Map diff to translation speed
            int i = 0;
       
            if (rotationalMotion) {
              i = 3;
              factor = JOG_CARTESIAN_ROT_SPEED*CONTROL_LOOP_DELAY/2291831.2; // Map diff to rotation speed (/40/1000/(180/pi))
            }
            x[i] += (float)diffs[1]*factor;
            x[i+1] += (float)diffs[0]*factor;
            x[i+2] += (float)diffs[2]*factor;
            if (!inverseKinematics(q_des, x, sendFB)) { // q_des is returned in degrees*10, if angles are within range
              x[i] -= (float)diffs[1]*factor;
              x[i+1] -= (float)diffs[0]*factor;
              x[i+2] -= (float)diffs[2]*factor;
            }
  
            cnt++;
          } else {
            //Serial.println("ooga");
            linearMotion = false;
            float factor = JOG_JOINT_SPEED*CONTROL_LOOP_DELAY/4000.0; // Map diff to speed
            q_des[MOTOR1] = constrain(q_des[MOTOR1] + diffs[MOTOR1]*factor, MOTOR1_L_LIMIT, MOTOR1_H_LIMIT);
            q_des[MOTOR2] = constrain(q_des[MOTOR2] + diffs[MOTOR2]*factor, MOTOR2_L_LIMIT, MOTOR2_H_LIMIT);
            q_des[MOTOR3] = constrain(q_des[MOTOR3] + diffs[MOTOR3]*factor, MOTOR3_L_LIMIT, MOTOR3_H_LIMIT);
            q_des[MOTOR4] = constrain(map(diffs[MOTOR4], 0, 200 ,MOTOR4_L_LIMIT, MOTOR4_H_LIMIT), MOTOR4_L_LIMIT, MOTOR4_H_LIMIT);
            q_des[MOTOR5] = constrain(map(diffs[MOTOR5], 0, 200 ,MOTOR5_L_LIMIT, MOTOR5_H_LIMIT), MOTOR5_L_LIMIT, MOTOR5_H_LIMIT);
            q_des[MOTOR6] = constrain(map(diffs[MOTOR6], 0, 200 ,MOTOR6_L_LIMIT, MOTOR6_H_LIMIT), MOTOR6_L_LIMIT, MOTOR6_H_LIMIT);
          }
          q_des[MOTOR7] = constrain(map(diffs[MOTOR7], 0, 200 ,MOTOR7_L_LIMIT, MOTOR7_H_LIMIT), MOTOR7_L_LIMIT, MOTOR7_H_LIMIT);
        }
      }

      writePosition(servoQue, stepperQue, q_des, prevQ, maxSpeed);
      maxSpeed = -1;
      xEventGroupClearBits(eventGroup, TASK1_DONE_BIT | TASK2_DONE_BIT);
      waitForJointsFunc();
    } 
    
    
    int del = constrain((CONTROL_LOOP_DELAY-(millis()-before)),0,CONTROL_LOOP_DELAY);
    vTaskDelay(del/portTICK_PERIOD_MS);
    before = millis();
    //Serial.println(uxTaskGetStackHighWaterMark( NULL ));
    //Serial.println("Control loop: " + String(del));
  }
}
