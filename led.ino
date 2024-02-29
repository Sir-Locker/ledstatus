#include <esp_now.h>
#include <WiFi.h>
#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
#endif
const int led_red = 5;
const int led_green = 17;
const int led_blue = 16;
const int buttonpin = 36;
int statusled = 0;
int pcard = 0;

uint8_t MacAddress4[] = {0x3C, 0x61, 0x05, 0x03, 0xCA, 0x04};
uint8_t MacAddress3[] = {0x24, 0x6F, 0x28, 0x28, 0x17, 0x1C};
uint8_t MacAddress2[] = {0xE8, 0x68, 0xE7, 0x23, 0x82, 0x1C};


typedef struct card_status{
  int cardstat; // 0 ผิด 1 ถูก
}card_status;

typedef struct led_status{
  int statuss; // 0 close 1 true 2 worng 3register
}led_status;

typedef struct servo_struct{
  int servo_status;//1 = lock, 0 = unlock
} servo_struct;

servo_struct servoled;
led_status led;
card_status card; 

esp_now_peer_info_t peerInfo1;
esp_now_peer_info_t peerInfo2;
esp_now_peer_info_t peerInfo3;

bool compareMac(const uint8_t * a, uint8_t * b){
  for(int i=0;i<6;i++){
    if(a[i]!=b[i])
      return false;    
  }
  return true;
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  //receive data
  if(compareMac(mac_addr,MacAddress4)){
     memcpy(&led, incomingData, sizeof(led));
     statusled = led.statuss;
     if(statusled == 0){
        analogWrite(5,0);
        analogWrite(17,0);
        analogWrite(16,0);
       }else if(statusled == 1){
        analogWrite(5,0);
        analogWrite(17,255);
        analogWrite(16,0);
       }else if(statusled == 2 ){
        analogWrite(5,255);
        analogWrite(17,0);
        analogWrite(16,0);
       }else if(statusled == 3){
        analogWrite(5,0);
        analogWrite(17,0);
        analogWrite(16,255);
       }
  }
   if(compareMac(mac_addr,MacAddress3)){
      Serial.println("IN");
      memcpy(&servoled, incomingData, sizeof(servoled));
      statusled = servoled.servo_status;
      if(statusled == 1){
        analogWrite(5,0);
        analogWrite(17,0);
        analogWrite(16,0);
      }
      if(statusled == 0){
        analogWrite(5,0);
        analogWrite(17,255);
        analogWrite(16,0);
      }
   }

    if(compareMac(mac_addr,MacAddress2)){
      Serial.println("revice card status");
      memcpy(&card, incomingData, sizeof(card));
      pcard = card.cardstat;
      if(pcard == 1){
        Serial.println("Pass is Correct");
      }
      if(pcard == 0){
        Serial.println("wongpass");
      }
   }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
 
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  memcpy(peerInfo1.peer_addr, MacAddress4, 6);
  peerInfo1.channel = 0;  
  peerInfo1.encrypt = false;
  if (esp_now_add_peer(&peerInfo1) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(peerInfo2.peer_addr, MacAddress3, 6);
  peerInfo2.channel = 0;  
  peerInfo2.encrypt = false;
  if (esp_now_add_peer(&peerInfo2) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  pinMode(led_red, OUTPUT);
  pinMode(led_blue,OUTPUT);
  pinMode(led_green,OUTPUT);
  pinMode(buttonpin,INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
}
