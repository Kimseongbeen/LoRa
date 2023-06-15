/* 프로토콜 설명
PC에서 오는 프로토콜
  mac: 이 프로그램이 동작하는 CPU mac address
  macObj:메세지 전송할 특정한 lora 주소  
  func:명령번호
lora 에서 추가되는 프로토콜
  layer: 서버는 0이고 lora를 거쳐서 전달할 때 +1 씩 증가한다.
  dire: 1=서버에서 lora로 전송 -1=lora 에서 서버로 전송
  macFrom:말단방향 lora 로부터 전송받은 lora mac 주소
  macTo: 서버방향르로 전송할 lora mac 주소
*/
#include <Wire.h>  
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <ArduinoJson.h>
#include "CRC.h"
#include "ca.h"
#include "SPIFFS.h"
#include "HT_SSD1306Wire.h"
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

SSD1306Wire  displayOled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst


#define RF_FREQUENCY                                923000000 // Hz

#define TX_OUTPUT_POWER                             5        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 200 // Define the payload size here
#define TRIGGER_PIN 37 // trigger pin GPIO37
#define board_ID 0

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void display(int no);
void callback(char* topic, byte* payload, unsigned int length);
void displayAct(int no);
void readConfig();
void saveConfig();
void factoryDefault();
void bootWifiStation();
void tick();
void reconnect();
typedef enum
{
    LOWPOWER,
    STATE_RX,
    STATE_TX
}States_t;

int16_t txNumber;
States_t state;
bool sleepMode = false;
int16_t Rssi,rxSize;
bool triggerState = false;
class Lora {
  public:
    String mac;
    int dire=0;
    int rssi=-200; //-80 보다 크면 아주 양호 -80~-90 양호  -90~-100 불량  -100보다 작음 나쁨
    int layer=100;
};
Lora loraC; // lora Client
Lora loraF; // lora onReceive 함수에서 From 임시로 전달받은 로라, 통시방향 관계없이 전달된 값
Lora loraT; // lora To 나에게 링크되어 메세지 서버쪽으로 전달할 로라

//String packet;
int packSize = 0;
String msgTo;


unsigned int counter = 0;
unsigned long previousMillis = 0;     
const long interval = 5000; 
char mac[20];  //mac address
int mqttConnected=0; // 1=연결 0=끊김

float temp=0.,humi=0.,pres=0.;

//json을 위한 설정
StaticJsonDocument<200> doc;
DeserializationError error;
JsonObject root;

int type=999; // 999=lora gate way  
char incomingPacket[255]; // buffer for incoming packets
/****** WiFi Connection Details *******/
char ssid[40] = "mecha1203";
char password[50] = "mecha1203";
/******* MQTT Broker Connection Details *******/
//const char* mqtt_server = "24500ab070ec4a1893114b95c15b0373.s2.eu.hivemq.cloud"; //브로커 주소
const char* mqtt_server = "broker.mqtt-dashboard.com";
//const char* mqtt_username = "hivemq.webclient.1686627684792";
//const char* mqtt_password = "xd39.0f,TC1oUw?ZiEN#";
const int mqtt_port =1883;
const char* outTopic = "/LoRa/outTopic"; // 이름이 중복되지 않게 설정 기록
const char* inTopic = "/LoRa/inTopic"; // 이름이 중복되지 않게 설정 기록
char clientName[30] = {0};  // setup 함수에서 자동생성
char ipMqtt[40]="";
char msg[500];

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
    pinMode(TRIGGER_PIN, INPUT_PULLUP);
    Serial.begin(115200);

    Serial.println("mac address");
    //이름 자동으로 생성
    uint64_t chipid;
    chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
    loraC.mac=String(((uint16_t)(chipid>>32)),HEX)+String(((uint32_t)chipid),HEX);  
    loraC.mac.toCharArray(clientName,loraC.mac.length()+1);
    Serial.println(loraC.mac);
    bootWifiStation();

    Mcu.begin();
    txNumber=0;
    Rssi=0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
    state=STATE_RX;

    displayOled.init();
    displayOled.setFont(ArialMT_Plain_10);
    displayOled.setTextAlignment(TEXT_ALIGN_LEFT);
    displayAct(0);

    readConfig();

    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
}

void loop()
{
  tick();
  switch(state)
  {
    case STATE_TX:
      txNumber++;
      msgTo.toCharArray(txpacket,msgTo.length());
      Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
      state=LOWPOWER;
      break;
    case STATE_RX:
      Serial.println("into RX mode");
      Radio.Rx( 0 );
      state=LOWPOWER;
      break;
    case LOWPOWER:
      Radio.IrqProcess( );
      break;
    default:
      break;
  }


 if (digitalRead(TRIGGER_PIN) == LOW && !triggerState) {
    triggerState = true;  // Set the trigger state to true
    msgTo = "{\"mac\":"+loraC.mac+",\"func\":\"ffffffff\",\"ID\":"+board_ID;
    state = STATE_TX;
    client.publish(outTopic, msgTo.c_str());
    msgTo = "";
  }
  else if (digitalRead(TRIGGER_PIN) == HIGH && triggerState) {
    triggerState = false;  // Reset the trigger state to false
  }
    if (!client.connected())
    reconnect();
  client.loop();

}

void bootWifiStation() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

// Store the MQTT server IP if not already saved
  if (String(ipMqtt).length() < 2) {
    String ip = WiFi.localIP().toString();
    ip.toCharArray(ipMqtt, sizeof(ipMqtt));
    saveConfig();
  }
}


// mqtt 통신에 지속적으로 접속한다.
void reconnect() {
  if(WiFi.status() != WL_CONNECTED)
    return;
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientName)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish(outTopic, "Reconnected");
      // ... and resubscribe
      client.subscribe(inTopic);
      mqttConnected=1;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      mqttConnected=0;
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



void OnTxDone( void )
{
  Serial.print("TX done......");
  state=STATE_RX;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.print("TX Timeout......");
    state=STATE_TX;
}


void displayAct(int no) {
  displayOled.clear();
  displayOled.drawString(0, 0, "Lola Gateway");
  displayOled.drawString(0, 10, "mac: "+loraC.mac);
  if(no==0) { //lora 시작
    displayOled.drawString(0, 20, "Start");
  }
  else if(no==1) {
    displayOled.drawString(0, 0, "Lora Start");
  }
  else if(no==2) {
    displayOled.drawString(0, 20, "connected "+String(counter));
  }
  else if(no==3) {
    displayOled.drawString(0, 20, "mqtt connect fail");
  }
  else if(no==4) {
    displayOled.drawString(0, 20, "no Mqtt server IP");
  }
  else if(no==5) {
    displayOled.drawString(0, 20, "Lora Reset");
  }
  displayOled.display();
}


void tick() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
//    //서버로 전송 테스트
//    if(client.connected()) {
//      Serial.println(counter++);
//      sprintf(msg, "%d", counter);
//      client.publish(outTopic, msg);
//      //client.publish(outTopic, "test");
//    }
  }  
}

//LORA로 메세지 받으면 동작
//전달할 데이타중 조건에 맞으면 loraT로 SPIFF에 저장
//새로이 전달되어온 데이타는 loraF에 저장
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
  Rssi=rssi; //쓰는 이유 모름
  rxSize=size;
  memcpy(rxpacket, payload, size );
  rxpacket[size]='\0';
  Radio.Sleep( );
  state=STATE_RX;
  loraF.rssi=rssi; // 맞음
  displayAct(3);
  String packet1(rxpacket);
  
  String packet = packet1 + "}";
  deserializeJson(doc,packet);
  root = doc.as<JsonObject>();
  Serial.println(packet);
  int layerIn = root["layer"];
  int Cindex = root["index"];     //data3 = (arr[7] << 8) + arr[6];
  int dire = root["dire"];
  if(layerIn!=0){
    if( Cindex == 0 || dire == 1 ) return; //node들의 데이터가 0일때 끝 받지 않는다.
  }
  String macTo = root["macTo"];
  if((macTo == loraC.mac) && dire==-1) {
    client.publish(outTopic, rxpacket);
    Serial.println("Send Data to Server");
  }
  int func = root["func"];
  if(func == 2){
    String sResetD="{\"macTo\":"+macTo+",\"mac\":"+loraC.mac+"\"macFrom\":\""+loraC.mac+"\",\"func\":255\"dire\":1,\"layer\":0."; //마지막. 데이터가 마지막이 없어짐
    msgTo=sResetD;
    Serial.println(msgTo);  
    state=STATE_TX; // func2를 요구하는 mac, 
  }
}

// 와이파이 통신에서 문자가 들어오면 이 함수의 payload 배열에 저장된다.
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String sMsg="";
  for (int i = 0; i < length; i++) {
    //Serial.print((char)payload[i]);
    sMsg+=(char)payload[i];
  }
  Serial.println(sMsg);
  
  // json {}로 수신되어야 한며 }를 추가 메세지를 붙여서 보낸다.
  deserializeJson(doc,payload);
  root = doc.as<JsonObject>();
  String macIn = root["mac"];
  String sAdd=",\"mac\":"+loraC.mac+"\"macFrom\":\""+loraC.mac+"\",\"dire\":1,\"layer\":0."; //마지막. 데이터가 마지막이 없어짐
  sMsg.replace("}",sAdd);
  msgTo=sMsg;
  Serial.println(msgTo);
  state=STATE_TX;
}
