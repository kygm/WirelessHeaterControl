#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>

#include <Adafruit_SSD1306.h>

#include "/home/leperry/Desktop/Arduino/DogStatus/HeaterPayload.h"
//Arduino IDE limitation - relative pathing does not work for files outside of the directory of 
//the sketch :(

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const uint8_t BUTTON_PIN = 13;
bool startHeaterCommanded = false;
bool isFirstRun = true;


uint8_t macAddrToSendMsgTo[] = {0xD4, 0x8A, 0xFC, 0xA5, 0xEA, 0xEC};

HeaterPayload outgoingHeaterPayload;
HeaterPayload incomingHeaterPayload;

float rxIncomingTemp;
bool rxIsHeaterOn;
bool rxStartHeater;

//incomingReadings = incomingHeaterPaylod for my own ref
String success;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingHeaterPayload, incomingData, sizeof(incomingHeaterPayload));
  Serial.print("Bytes received: ");
  Serial.println(len);
  //kygm workaround for temp difference
  incomingHeaterPayload.Temperature -= 8;

  Serial.println(incomingHeaterPayload.Temperature);

  //Need to do this to handle when the heater is on and this device loses power/is shut off.
  //otherwise since startHeaterCommand is initialized to 0, the heater will be immediately shut off
  //on first tx
  if(isFirstRun)
  {
    startHeaterCommanded = incomingHeaterPayload.StartHeater;
    isFirstRun = false;
  }

  Serial.print("Rx success! \n");
}

void TOGGLE_HEATER_START_ISR()
{
  startHeaterCommanded = !startHeaterCommanded;
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT);
  attachInterrupt(BUTTON_PIN, TOGGLE_HEATER_START_ISR, RISING);
  // Init OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
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
  // get the status of Trasnmitted packet
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
 
int numberOfWaitCycles = 0;
const int NUMBER_CYCLE_TO_WAIT_FOR = 5;

void loop() {
  
  //Waiting five wait cycles to get a response from the heater device in case this device
  //was restarted while the heater device was commanded on 
  if(isFirstRun && numberOfWaitCycles <= NUMBER_CYCLE_TO_WAIT_FOR)
  {
    numberOfWaitCycles++;
    displayInitialization();

    //resetting to wait another five cycles for a response. Otherwise the device 
    //shows zeros as data
    if(numberOfWaitCycles == NUMBER_CYCLE_TO_WAIT_FOR)
      numberOfWaitCycles = 0;
  }
  else
  {
    outgoingHeaterPayload.StartHeater = startHeaterCommanded;

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(macAddrToSendMsgTo, (uint8_t *) &outgoingHeaterPayload, sizeof(outgoingHeaterPayload));
    
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }

    updateDisplay();
  }
  delay(1000);
}

void displayInitialization()
{
  display.clearDisplay();
  display.setCursor(0, 5);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("Initializing...");
  display.println("Waiting on a response from the heater");
  display.println();

  display.print("Attempt count: ");
  display.print(numberOfWaitCycles);
  display.print("/");
  display.print(NUMBER_CYCLE_TO_WAIT_FOR);
  display.display();
}

void updateDisplay(){

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0, 5);
  display.println("  KYGM Services LLC");

  display.setCursor(0, 15);
  display.print("Incoming Data");

  display.setCursor(0, 25);
  display.print("- Temperature: ");
  display.print((int)incomingHeaterPayload.Temperature);
  display.cp437(true);
  display.write(248);
  display.print("F");

  display.setCursor(0, 35);
  display.print("- Heater On:   ");
  display.print(incomingHeaterPayload.IsHeaterRunning);

  display.setCursor(0, 45);
  display.print("Outgoing Data");

  display.setCursor(0, 55);
  display.print("- Command On:  ");
  display.print(startHeaterCommanded);
  display.display();
  
}