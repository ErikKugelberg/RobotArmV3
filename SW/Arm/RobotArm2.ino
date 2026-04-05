// must install: ESP dev board, ESP32servo
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "communication.hpp"
#include "motorControl.hpp"
#include "calculations.hpp"

void setup(void) {
  Serial.begin(115200); // Debugging Serial
  vTaskDelay(100/portTICK_PERIOD_MS);

  // Create Queues for cross-thread communication
  inputQue = xQueueCreate(5,sizeof(InputStruct));
  servoQue = xQueueCreate(2,sizeof(AngleStruct));
  stepperQue = xQueueCreate(2,sizeof(AngleStruct));

  // Create tasks
  int prio = 1;
  xTaskCreate(espNow, "espNow", 3*1024, NULL, prio++, NULL);
  xTaskCreate(jointCalculator, "jointCalculator", 10*1024, NULL, prio++, NULL);
  xTaskCreate(serialMotorControl, "serialMotorControl", 2*1024, NULL, prio++, NULL);
  xTaskCreate(pwmMotorControl, "pwmMotorControl", 2*1024, NULL, prio++, NULL);
}

void loop () {
  vTaskDelay(1000/portTICK_PERIOD_MS);
}
