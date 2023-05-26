//https://aws.amazon.com/ko/blogs/compute/building-an-aws-iot-core-device-using-aws-serverless-and-an-esp32/

#include <pgmspace.h>
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"

#include "heltec.h"
#include "SPIFFS.h"
#include <WebServer.h>

#define BAND    923E6
#define TRIGGER_PIN 13 // trigger pin GPIO13 (D13)

// AWS
const char AWS_IOT_ENDPOINT[] = "your end point"; // ex) xxxxxxxxx.iot.ap-northeast-2.amazonaws.com
WiFiClientSecure net = WiFiClientSecure();
//MQTTClient client = MQTTClient(256);
MQTTClient client = MQTTClient(30000);

class Lora {
  public:
    String mac;
    int rssi=-200; //-80 보다 크면 아주 양호 -80~-90 양호  -90~-100 불량  -100보다 작음 나쁨
    int layer=0;
};
Lora loraC; // lora Client
Lora loraF; // lora From
Lora loraT; // lora To

char ssid[40] = "";
char password[50] = "";
char email[50] = "";
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);
String ipAct="";

int type=999; // 999=lora gate way  
const char* outTopic = "youtTopic"; // 이름이 중복되지 않게 설정 기록
const char* inTopic = "youtTopic"; // 이름이 중복되지 않게 설정 기록
char clientName[30] = {0};  // setup 함수에서 자동생성
char msg[500];
String sMac="",sEmail="",sMsg="";
String linkMsg=""; //1분마다 Client 접속을 위한 msg 보냄
int mqttConnected=0; // 1=연결 0=끊김
char mac[20];  //mac address
int bootMode=0; //0:station  1:AP
int nWifi;

unsigned long previousMillis = 0;     
const long interval = 60000; 

WebServer server(80);


void connectAWS();
void messageHandler(String &topic, String &payload);
void publishMessage();
void setup();
void onReceive(int packetSize);
void factoryDefault();//
void GoHome();//
void handleRoot();//
void handleWifiSave();//
void HeltecSetup();
void linkLora();
void readConfig();//
void saveConfig();//
String sensWifi(int in);//
void setup();

//LORA로 메세지 받으면 동작
// 받은 메세지는 mqtt 통신으로 와이파이 서버로 전송
void onReceive(int packetSize) {
  if(packetSize <=0)
    return;
  loraF.rssi=LoRa.packetRssi();
  String packet ="";
  for (int i = 0; i < packetSize; i++) { packet += (char) LoRa.read(); }
  packet.toCharArray(msg, packet.length()+1);
  //Serial.println(packet);

  StaticJsonDocument<200> doc;
  deserializeJson(doc, packet);
  String macTo = doc["macTo"];
  int dire = doc["dire"];
  //const char* message = doc["message"];

  //Serial.println(msg);
  if((macTo == loraC.mac) && dire==-1) {
    client.publish(outTopic, msg);
  }
}

void linkLora() {
  DynamicJsonDocument doc(1024);
  doc["mac"] = loraC.mac;
  doc["dire"]   = 1;
  doc["macFrom"] = loraC.mac;
  doc["type"] = type;
  doc["layer"] = 0;

  linkMsg="";
  serializeJson(doc, linkMsg); 
  sMsg=linkMsg;
  for(int i=0;i<5;i++) {
    //Serial.println(sMsg);
    sendLora();
    delay(200);
  }
}

void sendLora() {
  LoRa.beginPacket();
  LoRa.setTxPower(14,RF_PACONFIG_PASELECT_PABOOST);
  LoRa.print(sMsg);
  LoRa.endPacket();
}

void bootWifiAp() {
  bootMode=1; //0:station  1:AP
  /* Soft AP network parameters */
  Serial.println("AP Mode");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, netMsk);
  char i2rMac[30];
  sMac="i2r-"+loraC.mac;
  sMac.toCharArray(i2rMac, sMac.length()+1);
  WiFi.softAP(i2rMac, "");
  ipAct=WiFi.softAPIP().toString();
  delay(500); // Without delay I've seen the IP address blank
  Serial.print("AP IP address: ");
  Serial.println(ipAct);
  displayOled(1);
}

void bootWifiStation() {
  //referance: https://www.arduino.cc/en/Reference/WiFiStatus
  //WL_NO_SHIELD:255 WL_IDLE_STATUS:0 WL_NO_SSID_AVAIL:1 WL_SCAN_COMPLETED:2
  //WL_CONNECTED:3 WL_CONNECT_FAILED:4 WL_CONNECTION_LOST:5 WL_DISCONNECTED:6
  //WiFi 연결
  bootMode=0; //0:station  1:AP
  Serial.println("Station Mode");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    //공장리셋
    if ( digitalRead(TRIGGER_PIN) == LOW ) 
      factoryDefault();
  }
  ipAct=WiFi.localIP().toString();
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(ipAct);
  displayOled(2);
}

void HeltecSetup() {
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  LoRa.setSpreadingFactor(7);           // ranges from 6-12,default 7 see API docs
  Heltec.display->init();
  Heltec.display->flipScreenVertically();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  displayOled(0);
}

void displayOled(int no) {
  Heltec.display->clear();
  if(no==0) { //lora 시작
    Heltec.display->drawString(0, 0, "Lola Gateway");
    Heltec.display->drawString(0, 20, "Start");
  }
  else if(no==1) { //AP Mode
    Heltec.display->drawString(0, 0, "Set WiFi ssid:");
    Heltec.display->drawString(0, 20, sMac);
    Heltec.display->drawString(0, 40, "ip: 192.168.4.1");
  }
  else if(no==2) { // Station Mode로 접속됨
    Heltec.display->drawString(0, 0, "Gateway OK");
    Heltec.display->drawString(0, 10, loraC.mac);
    Heltec.display->drawString(0, 30, "ip "+String(ipAct));
    if(mqttConnected==1)
      Heltec.display->drawString(0, 40, "AWS Connected");
    else
      Heltec.display->drawString(0, 40, "AWS fail");
  }
  else if(no==3) {
    Heltec.display->drawString(0, 0, "Reconnecting Mqtt ...");
  }
  else if(no==4) {
    Heltec.display->drawString(0, 40, "AWS Connected");
  }
  Heltec.display->display();
}

void connectAWS()
{
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.begin(AWS_IOT_ENDPOINT, 8883, net);

  // Create a message handler
  client.onMessage(messageHandler);

  Serial.print("Connecting to AWS IOT");
  reconnect();
}

// mqtt 통신에 지속적으로 접속한다.
void reconnect() {
  if(WiFi.status() != WL_CONNECTED)
    return;
  // Loop until we're reconnected
  //while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    //if (client.connect(clientName)) {
    if (client.connect(clientName)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish(outTopic, "Reconnected");
      // ... and resubscribe
      client.subscribe(inTopic);
      mqttConnected=1;
    } else {
      Serial.print("failed, rc=");
      Serial.println(" try again in 5 seconds");
      mqttConnected=0;
      // Wait 5 seconds before retrying
      delay(5000);
    }
    displayOled(2);
  //}
}

void publishMessage()
{
  StaticJsonDocument<200> doc;
  doc["time"] = millis();
  //doc["sensor_a0"] = analogRead(0);
  //doc["sensor"] = "00000000001111111111222222222233333333334444444444555555555556666666666677777777777888888888888999999999999990000000000000011111111112222222222333333333344444444445555555555566666666666777777777778888888888889999999999999900000000000000111111111122222222223333333333444444444455555555555666666666667777777777788888888888899999999999999";
  doc["sensor"] = "test";
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client

  client.publish(outTopic, jsonBuffer);
}

//서버에서 mqtt 통신으로 전달받은 메세지
void messageHandler(String &topic, String &payload) {
  //Serial.println("incoming: " + topic + " - " + payload);
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  String macIn = doc["mac"];
  String sAdd=",\"macFrom\":\""+loraC.mac+"\",\"dire\":1,\"layer\":0}";
  sMsg=String(payload);
  sMsg.replace("}",sAdd);
  //Serial.println(sMsg);
  sendLora();
}

void setup() {
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  HeltecSetup();

  Serial.println("mac address");
  //이름 자동으로 생성
  uint64_t chipid;
  chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
  loraC.mac=String(((uint16_t)(chipid>>32)),HEX)+String(((uint32_t)chipid),HEX); 
  loraC.mac.toCharArray(clientName,loraC.mac.length()+1);
  Serial.println(loraC.mac);
  //Serial.println(clientName);
  
  readConfig();
  if(ssid[0]==0)
    bootWifiAp();
  else {
    bootWifiStation();
    connectAWS();
  }
  
  if(bootMode ==0) {
    linkLora();
  }
  else {
    server.on("/", handleRoot);
    server.on("/wifisave", handleWifiSave);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("HTTP server started");
    nWifi = WiFi.scanNetworks();
    Serial.println(nWifi);
  }
}

//1초 마다 실행되는 시간함수
void tick() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sMsg=linkMsg;
    //Serial.println(sMsg);
    sendLora();
  }
}

void loop() {
  if(bootMode==1)
    server.handleClient();
  else {
    if (!client.connected()) {
      reconnect();
    }
    else
      client.loop();
    onReceive(LoRa.parsePacket());
    tick();
  }
  //공장리셋
  if ( digitalRead(TRIGGER_PIN) == LOW ) 
    factoryDefault();
}
