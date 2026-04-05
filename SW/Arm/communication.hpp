#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern QueueHandle_t inputQue;

// Data structure in feedback to controller
struct FeedBack {
  float X;
  float Y;
  float Z;
  float YAW;
  float PITCH;
  float ROLL;
};

// Data structure for controller input
struct InputStruct {
  uint8_t jsL_Y; // Joystick left, x-axis
  uint8_t jsL_X; // Joystick left, y-axis
  uint8_t jsR_Y; // Joystick right, x-axis
  uint8_t jsR_X; // Joystick right, y-axis
  uint8_t pot0;  // Potentiometer 0
  uint8_t pot1;  // Potentiometer 1
  uint8_t pot2;  // Potentiometer 2
  uint8_t pot3;  // Potentiometer 3
  uint8_t switches;  // Represents buttons and switches
};

void espNow (void * par);

void sendData(FeedBack* output, size_t num_elements);

#endif 
