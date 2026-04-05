#include "communication.hpp"

#include <WiFi.h>
#include <esp_now.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define ESP_NOW_CHANNEL 1

QueueHandle_t inputQue = NULL;

// [DEFAULT] robotarm ESP32 Board MAC Address: ac:67:b2:c0:54:f8
uint8_t masterAddress[] = {0x80, 0x64, 0x6f, 0xc5, 0x0f, 0x30};
esp_now_peer_info_t peerInfo;

// Add the master as a peer
void addPeer() {
  //memset(&peerInfo, 0, sizeof(esp_now_peer_info_t));
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.channel = ESP_NOW_CHANNEL;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add master as a peer.");
  } else {
    Serial.println("Master added as a peer successfully.");
  }
}

// callback when data is recv from Master, transmitter sends 10 times a second
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  InputStruct inputs;
  inputs.pot0 = *(data+0); 
  inputs.pot1 = *(data+1); 
  inputs.pot2 = *(data+2); 
  inputs.pot3 = *(data+3); 
  inputs.jsL_Y = *(data+4); 
  inputs.jsL_X = *(data+5); 
  inputs.jsR_Y = *(data+6); 
  inputs.jsR_X = *(data+7); 
  inputs.switches = *(data+8); 
  xQueueSend(inputQue,&inputs, 0);
}

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
  }
  addPeer();
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

// Function to send data back to the master
void sendData(FeedBack* output, size_t num_elements) {

  // Send the data over ESP-NOW
  esp_err_t result = esp_now_send(masterAddress, (const uint8_t*)output, num_elements);

  if (result == ESP_OK) {
    //Serial.println("Reply sent successfully");
  } else {
    // Add additional debug information
    if (result == ESP_ERR_ESPNOW_NOT_INIT) {
      Serial.println("ESP-NOW is not initialized.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
      Serial.println("Invalid argument passed.");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
      Serial.println("Internal error.");
    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
      Serial.println("Out of memory.");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
      Serial.println("Peer not found.");
    } else if (result == ESP_ERR_ESPNOW_IF) {
      Serial.println("Invalid interface.");
    }
  }
}

void espNow (void * par) {  
  InitESPNow();
  
  while (1) { 
    vTaskDelay(1000/portTICK_PERIOD_MS);
    //Serial.println(uxTaskGetStackHighWaterMark( NULL ));
  }
}
