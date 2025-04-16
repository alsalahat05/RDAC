//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Two-Way communication ESP32 to Control LED
//----------------------------------------Load libraries
#include <esp_now.h>
#include <WiFi.h>
//----------------------------------------

//----------------------------------------Defines PIN Button and PIN LED.
#define Relay_Pin   15
#define BTN_Pin   0
//----------------------------------------


int LED_State_Receive; //--> Variable to receive data to control the LEDs on the ESP32 running this code.

//----------------------------------------Structure example to send data
//Must match the receiver structure

typedef struct struct_message {
    int led;
} struct_message_send;


struct_message receive_Data; // Create a struct_message to receive data.
//----------------------------------------


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Callback when data is received
void OnDataRecv(const esp_now_recv_info * mac, const unsigned char *incomingData, int len) {
  memcpy(&receive_Data, incomingData, sizeof(receive_Data));
  Serial.println();
  Serial.println("<<<<< Receive Data:");
  Serial.print("Bytes received: ");
  Serial.println(len);
  LED_State_Receive = receive_Data.led;
  Serial.print("Receive Data: ");
  Serial.println(LED_State_Receive);
  Serial.println("<<<<<");

  if(LED_State_Receive){
    digitalWrite(Relay_Pin, !LED_State_Receive);
    delay(1000);
    digitalWrite(Relay_Pin, 1);
  }else{
    digitalWrite(Relay_Pin, 1);
  }
  

  
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ VOID SETUP
void setup() {

  Serial.begin(115200);
  delay(10);

  pinMode(Relay_Pin, OUTPUT);
  digitalWrite(Relay_Pin, 1);

  
  WiFi.mode(WIFI_STA); //--> Set device as a Wi-Fi Station
  delay(1000);

  //----------------------------------------Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }



  esp_now_register_recv_cb(OnDataRecv); //--> Register for a callback function that will be called when data is received
}

void loop() {}
