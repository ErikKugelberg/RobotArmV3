#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Wire.h>
#include <FaBoLCD_PCF8574.h>
#include <esp_now.h>
#include <WiFi.h>

// Pinout: https://raw.githubusercontent.com/AchimPieters/esp32-homekit-camera/master/Images/ESP32-38%20PIN-DEVBOARD.png
#define POT1_PIN 34 //ADC6
#define POT2_PIN 35 //ADC7
#define POT3_PIN 33 //ADC4
#define POT4_PIN 32 //ADC5

#define JSLY_PIN 25 //ADC18
#define JSLX_PIN 26 //ADC19
#define JSRY_PIN 27 //ADC17
#define JSRX_PIN 14 //ADC16

#define BUTTON1_PIN 5
#define BUTTON2_PIN 18
#define BUTTON3_PIN 19

#define SWITCH1_PIN 16
#define SWITCH2_PIN 17

#define I2C_SDA 21
#define I2C_SCL 22

#define SWITCH1_MASK 0x01
#define SWITCH2_MASK 0x02
#define BUTTON1_MASK 0x04
#define BUTTON2_MASK 0x08
#define BUTTON3_MASK 0x10

#define BUTTON_DISPLAY_MS 1000
#define BUTTON_DEBOUNCE_MS 500
#define STICK_DEADZONE 4

// Global copy of slave
#define ESP_NOW_CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

struct FeedBack {
    float X;
    float Y;
    float Z;
    float YAW;
    float PITCH;
    float ROLL;
};

// [DEFAULT] ESP32 transmitter Board MAC Address: 80:64:6f:c5:0f:30
uint8_t slaveAddress[] = {0xac, 0x67, 0xb2, 0xc0, 0x54, 0xf8}; // ESPnow mac address
esp_now_peer_info_t peerInfo;

uint8_t inputs[9];

long buttonMillis = 0;
long button1Millis = 0;
long button2Millis = 0;
long button3Millis = 0;

boolean sendButton1 = false, sendButton2 = false, sendButton3 = false;

FaBoLCD_PCF8574 lcd(0x27); // LCD address

void IRAM_ATTR ISR_BUTTON1() {
  if ((millis() - button1Millis) > BUTTON_DEBOUNCE_MS) {
    sendButton1 = true;
    button1Millis = millis();
    
    // Do shit
    buttonMillis = millis();
  }
}

void IRAM_ATTR ISR_BUTTON2() {
  if ((millis() - button2Millis) > BUTTON_DEBOUNCE_MS) {
    sendButton2 = true;
    button2Millis = millis();
    
    // Do shit
    buttonMillis = millis();
  }
}

void IRAM_ATTR ISR_BUTTON3() {
  if ((millis() - button3Millis) > BUTTON_DEBOUNCE_MS) {
    sendButton3 = true;
    button3Millis = millis();
    
    // Do shit
    buttonMillis = millis();
  }
}

// Callback function when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  FeedBack receivedData;
  memcpy(&receivedData, data, sizeof(receivedData));
  Serial.println("X: " + String(receivedData.X, 1) + "mm    Y: " + String(receivedData.Y,1) + "mm    Z: " + String(receivedData.Z,1) + "mm    Yaw: " + String(receivedData.YAW,1) + "°    Pitch: " + String(receivedData.PITCH,1) + "°    Roll: " + String(receivedData.ROLL,1) + "°");  // Assuming data is a string
}

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin();

  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // InitESPNow();
    //ESP.restart();
    return;
  }

  // Register peer
  memcpy(peerInfo.peer_addr, slaveAddress, 6);
  peerInfo.channel = ESP_NOW_CHANNEL;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

// send data
void sendData() {
  const uint8_t *peer_addr = peerInfo.peer_addr;
  //Serial.print("Sending... ");
  esp_err_t result = esp_now_send(peer_addr, &inputs[0], sizeof(inputs));
  //Serial.print("Send Status: ");
  if (result == ESP_OK) {
    //Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

/*
// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
*/

void readADC2 () {
  WiFi.mode(WIFI_OFF);
  pinMode(JSLY_PIN, INPUT);
  pinMode(JSLX_PIN, INPUT);
  pinMode(JSRY_PIN, INPUT);
  pinMode(JSRX_PIN, INPUT);
  
  inputs[4] = (uint8_t)map(analogRead(JSLY_PIN),0,4095,0,80);
  inputs[5] = (uint8_t)map(analogRead(JSLX_PIN),0,4095,0,80);
  inputs[6] = (uint8_t)map(analogRead(JSRY_PIN),0,4095,0,80);
  inputs[7] = (uint8_t)map(analogRead(JSRX_PIN),0,4095,0,80);
  for (int i = 0;i < 4;i++) {
    if (abs(inputs[4+i]-40) < STICK_DEADZONE) {
      inputs[4+i] = 40;
    } 
  }
  WiFi.mode(WIFI_STA);
}

void updateDisplay() {
  lcd.setCursor(0,0);
  for (int i = 0;i < 8;i++) {
    int value = 0;
    if (i < 4) {
      value = map(inputs[i],0,200,0,99);
    } else {
      value = inputs[i];
    }
    String s = "";
    if (value < 10) {
      s += "0";
    }
    s += String(value);
    //Serial.print(s + " , ");
    lcd.print(s);
    if ((i != 3) && (i != 7)) {
      lcd.print(" ");
    }
    if (i == 3) {
      long t = millis();
      if ((t - button1Millis) < BUTTON_DISPLAY_MS) {
        lcd.print(" 1 ");
      } else if ((t - button2Millis) < BUTTON_DISPLAY_MS){
        lcd.print(" 2 ");
      } else if ((t - button3Millis) < BUTTON_DISPLAY_MS){
        lcd.print(" 3 ");
      } else {
        lcd.print("   ");
      }

      if ((inputs[8] & SWITCH1_MASK) && SWITCH1_MASK) {
        lcd.print("S1");
      } else {
        lcd.print("  ");
      }
      if ((inputs[8] & SWITCH2_MASK) && SWITCH2_MASK) {
        lcd.setCursor(14,1);
        lcd.print("S2");
      } else {
        lcd.setCursor(14,1);
        lcd.print("  ");
      }
      lcd.setCursor(0,1);
    }
  }
  //Serial.println("");
}

void readValues () {
  inputs[0] = (uint8_t)map(analogRead(POT1_PIN),0,4095,0,200);
  inputs[1] = (uint8_t)map(analogRead(POT2_PIN),4095,415,0,200);
  inputs[2] = (uint8_t)map(analogRead(POT3_PIN),0,4095,0,200);
  inputs[3] = (uint8_t)map(analogRead(POT4_PIN),0,4095,0,200);
  /*
  inputs[4] = (uint8_t)map(analogRead(JSLX_PIN),0,4095,0,10);
  inputs[5] = (uint8_t)map(analogRead(JSLY_PIN),0,4095,0,10);
  inputs[6] = (uint8_t)map(analogRead(JSRX_PIN),0,4095,0,10);
  inputs[7] = (uint8_t)map(analogRead(JSRY_PIN),0,4095,0,10);
  */
  readADC2();
  if (sendButton1) {
    sendButton1 = false;
    inputs[8] |=  BUTTON1_MASK;
  } else {
    inputs[8] &= ~BUTTON1_MASK;
  }

  if (sendButton2) {
    sendButton2 = false;
    inputs[8] |=  BUTTON2_MASK;
  } else {
    inputs[8] &= ~BUTTON2_MASK;
  }

  if (sendButton3) {
    sendButton3 = false;
    inputs[8] |=  BUTTON3_MASK;
  } else {
    inputs[8] &= ~BUTTON3_MASK;
  }
  
  if (!digitalRead(SWITCH1_PIN)) {
    inputs[8] |=  SWITCH1_MASK;
  } else {
    inputs[8] &= ~SWITCH1_MASK;
  }

  if (!digitalRead(SWITCH2_PIN)) {
    inputs[8] |=  SWITCH2_MASK;
  } else {
    inputs[8] &= ~SWITCH2_MASK;
  }
}

void setup() {
  Serial.begin(115200);
  InitESPNow();
  
  analogSetAttenuation((adc_attenuation_t)ADC_ATTEN_DB_11); // set range to 0-3.3V
  
  pinMode(POT1_PIN, INPUT);
  pinMode(POT2_PIN, INPUT);
  pinMode(POT3_PIN, INPUT);
  pinMode(POT4_PIN, INPUT);
  /*
  pinMode(JSLX_PIN, INPUT);
  pinMode(JSLY_PIN, INPUT);
  pinMode(JSRX_PIN, INPUT);
  pinMode(JSRY_PIN, INPUT);
  */
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);

  pinMode(SWITCH1_PIN, INPUT_PULLUP);
  pinMode(SWITCH2_PIN, INPUT_PULLUP);

  attachInterrupt(BUTTON1_PIN, ISR_BUTTON1, FALLING);
  attachInterrupt(BUTTON2_PIN, ISR_BUTTON2, FALLING);
  attachInterrupt(BUTTON3_PIN, ISR_BUTTON3, FALLING);

  Wire.begin(I2C_SDA,I2C_SCL);
  lcd.begin(16, 2);
}

void loop() {
  readValues();
  updateDisplay();
  sendData();
  delay(100);
}
