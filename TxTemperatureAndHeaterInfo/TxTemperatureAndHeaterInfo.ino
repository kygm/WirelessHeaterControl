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

HeaterPayload outgoingHeaterPayload;
HeaterPayload incomingHeaterPayload;

dht11 DHT11;

const uint8_t HEATER_RELAY_PIN = 5;
bool commandHeaterOn = false;

esp_now_peer_info_t peerInfo;
//B4 addr: E4:65:B8:25:41:B4
uint8_t macAddrToSendMsgTo[] = {0xE4, 0x65, 0xB8, 0x25, 0x41, 0xB4};
//EC addr:  {0xD4, 0x8A, 0xFC, 0xA5, 0xEA, 0xEC};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingHeaterPayload, incomingData, sizeof(incomingHeaterPayload));

  Serial.println(incomingHeaterPayload.StartHeater);
  commandHeaterOn = incomingHeaterPayload.StartHeater;

  Serial.print("Rx success! \n");
}

void setup() 
{
  Serial.begin(115200);

  pinMode(HEATER_RELAY_PIN, OUTPUT);

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

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
  //First handle rx data
  if(commandHeaterOn)
  {
    digitalWrite(HEATER_RELAY_PIN, HIGH);
    Serial.println("Commanded high");
  }
  else
  {
    digitalWrite(HEATER_RELAY_PIN, LOW);
    Serial.println("Commanded low");
  }

  //Read the DHT11 sensor first:
  int discard = DHT11.read(DHT11PIN);

  float tempf = ((float)DHT11.temperature * 1.8)+32;

  Serial.print("Tempf sent: ");
  Serial.println(tempf, 2);

  outgoingHeaterPayload.Temperature = (double)tempf;

  esp_err_t result = esp_now_send(macAddrToSendMsgTo, (uint8_t *) &outgoingHeaterPayload, sizeof(outgoingHeaterPayload));

  delay(500);
}


