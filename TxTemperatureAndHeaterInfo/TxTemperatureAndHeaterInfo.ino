//KYGM Services 2024
//Temperature Reader & Heater Switcher

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "/home/leperry/Desktop/Arduino/DogStatus/HeaterPayload.h"
//Arduino IDE limitation - relative pathing does not work for files outside of the directory of 
//the sketch :(

#include <dht11.h>
#define DHT11PIN 4

using namespace std;

//Definition of screen for debug purposes in the field
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

HeaterPayload heaterPayload;

dht11 DHT11;

esp_now_peer_info_t peerInfo;
//B4 addr: E4:65:B8:25:41:B4
uint8_t macAddrToSendMsgTo[] = {0xE4, 0x65, 0xB8, 0x25, 0x41, 0xB4};
//EC addr:  {0xD4, 0x8A, 0xFC, 0xA5, 0xEA, 0xEC};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.print("\nSent to: ");
  Serial.print("mac_addr");
}

void setup() 
{
  Serial.begin(115200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, macAddrToSendMsgTo, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  //Read the DHT11 sensor first:
  int discard = DHT11.read(DHT11PIN);

  float tempf = ((float)DHT11.temperature * 1.8)+32;

  Serial.print("Temp in f: ");
  Serial.println(tempf, 2);
  char* messageOnOled;
  heaterPayload.Temperature = (double)tempf;

  esp_err_t result = esp_now_send(macAddrToSendMsgTo, (uint8_t *) &heaterPayload, sizeof(heaterPayload));


  //messageOnOledStr = string("Temp c: ") + (float)DHT11.temperature string("\nHumidity: ") + (float)DHT11.humidity;
  
  //formatMessage((float)DHT11.temperature, (float)DHT11.humidity, messageOnOled, sizeof(messageOnOled));

  printTextOnFirstLine(messageOnOled);
  delay(500);
}

void formatMessage(float value1, float value2, char* messageBase, size_t bufferSize) {
  snprintf(messageBase, bufferSize, "Temperature F: %.2f, Humidity: %.2f", value1, value2);
}

void printTextOnFirstLine(const char* text)
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(1, 10);
  // Display static text
  display.println(text);
  display.display();
}


