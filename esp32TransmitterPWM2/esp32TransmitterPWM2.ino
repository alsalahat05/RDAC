//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Two-Way communication ESP32 to Control LED
//----------------------------------------Load libraries
#include <esp_now.h>
#include <WiFi.h>
//----------------------------------------

//----------------------------------------


//uint8_t broadcastAddress[] = {0xCC, 0xDB, 0xA7, 0x92, 0xAB, 0x40};
uint8_t broadcastAddress[] = {0x34, 0x5F, 0x45, 0xAA, 0xF1, 0x8C}; // Release setup

int LED_State_Send = 0; //--> Variable to hold the data to be transmitted to control the LEDs on the paired ESP32.

String success; //--> Variable to store if sending data was successful




unsigned dutyCheck = 0;
unsigned freqCheck = 0;

volatile unsigned long riseTime = 0;
volatile unsigned long fallTime = 0;
volatile unsigned long highTime = 0;
volatile unsigned long lowTime = 0;

const int pwmPin = 14; // Connect PWM signal to GPIO14


void IRAM_ATTR handlePWM() {
  bool level = digitalRead(pwmPin);
  unsigned long now = micros();

  if (level == HIGH) {
    riseTime = now;
    lowTime = riseTime - fallTime;
  } else {
    fallTime = now;
    highTime = fallTime - riseTime;
  }
}

  uint32_t period = 0;
  float dutyCycle = 0.0;


//----------------------------------------Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int led;
} struct_message_send;

struct_message send_Data; // Create a struct_message to send data.

//----------------------------------------

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
  Serial.println(">>>>>");
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ VOID SETUP
void setup() {
  Serial.begin(115200);



  pinMode(pwmPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(pwmPin), handlePWM, CHANGE);











  
  WiFi.mode(WIFI_STA); //--> Set device as a Wi-Fi Station

  //----------------------------------------Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  //----------------------------------------
  
  //----------------------------------------Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  //----------------------------------------
  
  //----------------------------------------Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  //----------------------------------------
  
  //----------------------------------------Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  //----------------------------------------
  
  //esp_now_register_recv_cb(OnDataRecv); //--> Register for a callback function that will be called when data is received
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void loop() {


noInterrupts();
  unsigned long high = highTime;
  unsigned long low = lowTime;
  interrupts();
if (high > 0 && low > 0) {
    float period = high + low;
    float frequency = 1000000.0 / period;      // microseconds to Hz
    float dutyCycle = (high * 100.0) / period; // %

    Serial.print("Frequency: ");
    Serial.print(frequency);
    Serial.print(" Hz, Duty Cycle: ");
    Serial.print(dutyCycle);
    Serial.println(" %");
     dutyCheck = dutyCycle >= 7 && dutyCycle <= 11.1;
     freqCheck = frequency>45.0 && frequency<61;
    
  } else {
    Serial.println("Waiting for PWM signal...");
  }
//Serial.println(freqCheck);


  if(dutyCheck && freqCheck) {
      LED_State_Send = 1;
      //LED_State_Send = 89; // Y
      }
    else {
      LED_State_Send = 0;
      //LED_State_Send = 78; // N
      }

  
////..................................................................

  if(LED_State_Send) {

    send_Data.led = LED_State_Send;

    Serial.println();
    Serial.print(">>>>> ");
    Serial.println("Send data");
  
    //----------------------------------------Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &send_Data, sizeof(send_Data));
     
    if (result == ESP_OK) {
      Serial.println("Sent with success");
      Serial.println("LED_State_Send");
      Serial.println(LED_State_Send);
    }
    else {
      Serial.println("Error sending the data");
      Serial.println("LED_State_Send");
      Serial.println(LED_State_Send);
    }

    delay(1000);
  }
  //----------------------------------------
}
