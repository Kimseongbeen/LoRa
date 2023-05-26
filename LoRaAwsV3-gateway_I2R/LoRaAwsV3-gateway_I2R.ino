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
aws 연결 참조 사이트 : 
  https://aws.amazon.com/ko/blogs/compute/building-an-aws-iot-core-device-using-aws-serverless-and-an-esp32/
MQTTClient. : Search MQTT, and install the latest version by Joel Gaehwiler.
*/
#include <pgmspace.h>
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <Wire.h>  
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <ArduinoJson.h>
#include "SPIFFS.h"
#include "HT_SSD1306Wire.h"
#include <WiFi.h>


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

// AWS
const char AWS_IOT_ENDPOINT[] = "a35y3l4o97ieag-ats.iot.us-east-1.amazonaws.com";
WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(256);
//MQTTClient client = MQTTClient(30000);

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void display(int no);
//void callback(char* topic, byte* payload, unsigned int length);
void displayAct(int no);
void readConfig();
void saveConfig();
void factoryDefault();
void bootWifiStation();
void reconnect();
void connectAWS();
void messageHandler(String &topic, String &payload);
const unsigned long ONE_DAY = 24UL * 60UL * 60UL * 1000UL; // One day in milliseconds
const unsigned long SIX_HOURS = 6UL * 60UL * 60UL * 1000UL; // Six hours in milliseconds
const unsigned long THREE_HOURS = 3UL * 60UL * 60UL * 1000UL; // Six hours in milliseconds
const unsigned long ONE_MINUTES = 1UL * 60UL * 1000UL; // Six hours in milliseconds
unsigned long startTime = 0;
int WiFi_State = 0;

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

char ssid[40] = "와이파이 이름";
char password[50] = "와이파이 비번";
int type=999; // 999=lora gate way  
const char* outTopic = "yourOutTopic"; // 이름이 중복되지 않게 설정 기록
const char* inTopic = "yourInTopic"; // 이름이 중복되지 않게 설정 기록
char clientName[30] = {0};  // setup 함수에서 자동생성
//char ipMqtt[40]="";
char msg[500];
String sMsg="";


void setup() {
    pinMode(TRIGGER_PIN, INPUT_PULLUP);
    Serial.begin(115200);
    readConfig();
    delay(1000);
    startTime = millis(); // Store the start time
    Mcu.begin();
    displayOled.init();
    displayOled.setFont(ArialMT_Plain_10);
    displayOled.setTextAlignment(TEXT_ALIGN_LEFT);
    Serial.println("mac address");
    //이름 자동으로 생성
    uint64_t chipid;
    chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
    loraC.mac=String(((uint16_t)(chipid>>32)),HEX)+String(((uint32_t)chipid),HEX);  
    loraC.mac.toCharArray(clientName,loraC.mac.length()+1);
    Serial.println(loraC.mac);
    Serial.println(clientName);
    bootWifiStation();
    connectAWS();

    //Mcu.begin();
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

    
    displayAct(0);
}

//mqtt로 데이터 들어왔을때 TX 출력
void messageHandler(String &topic, String &payload) {
  String sMsg=String(payload);
  // json {}로 수신되어야 한며 }를 추가 메세지를 붙여서 보낸다.
  deserializeJson(doc,payload);
  root = doc.as<JsonObject>();
  String macIn = root["mac"];
  String sAdd=",\"mac\":\""+loraC.mac+"\",\"dire\":1,\"layer\":0}";
  sMsg.replace("}",sAdd);
  msgTo=sMsg;
  Serial.println(msgTo);
  state=STATE_TX;
}

void connectAWS()
{
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
  client.begin(AWS_IOT_ENDPOINT, 8883, net);
  client.onMessage(messageHandler);
}

void loop()
{
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;

  if (elapsedTime >= THREE_HOURS && WiFi_State == 1)
  {
    Serial.println("Rebooting...");
    ESP.restart();
  }
   if (!client.connected()) {
    reconnect();
  } else {
    client.loop();
  }
  
  if ( digitalRead(TRIGGER_PIN) == LOW ){ 
    factoryDefault();
  }
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
}

void bootWifiStation() {
  Serial.println("Station Mode");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    displayAct(8);
    delay(500);
    Serial.print(".");
    //공장리셋
    if ( digitalRead(TRIGGER_PIN) == LOW ) 
      factoryDefault();
  }
  displayAct(9);
  WiFi_State = 1;
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP().toString());
  delay(1000);
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    displayAct(6);
    if (client.connect(clientName)) {
      Serial.println("Connected to MQTT broker");
      displayAct(2);
      client.subscribe(inTopic);
      // MQTT 연결 성공 시 추가적인 작업을 수행할 수 있습니다.
    } else {
      Serial.print("Failed to connect to MQTT broker, retrying in 1 second...");
      delay(1000);
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
  displayOled.drawString(0, 0, "Lora Gateway");
  displayOled.drawString(0, 10, "mac: "+loraC.mac);
  if(no==0) { //lora 시작
    displayOled.drawString(0, 20, "Start");
  }
  else if(no==1) {
    displayOled.drawString(0, 0, "Lora Start");
  }
  else if(no==2) {
    displayOled.drawString(0, 20, "mqtt connected ");
    displayOled.drawString(0, 40, "waitng packet...");
  }
  else if(no==3) {
    displayOled.drawString(0, 20, "mqtt connect fail..");
  }
  else if(no==4) {
    displayOled.drawString(0, 20, "no Mqtt server IP");
  }
  else if(no==5) {
    displayOled.drawString(0, 20, "Lora Reset");
  }
  else if(no==6) {
    displayOled.drawString(0, 20, "mqtt connecting...");
  }
  else if(no==7) {
    displayOled.drawString(0, 20, "data in");
  }
  else if(no==8) {
    displayOled.drawString(0, 20, "wifi connecting...");
  }
  else if(no==9) {
    displayOled.drawString(0, 20, "wifi connected");
  }
  displayOled.display();
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
  Rssi=rssi;
  rxSize=size;
  memcpy(rxpacket, payload, size );
  rxpacket[size]='\0';
  Radio.Sleep( );
  state=STATE_RX;
  loraF.rssi=rssi;
  displayAct(7);
  
  //Serial.println(rxpacket);
  //if((macTo == loraC.mac) && dire==-1) {
  reconnect();
  client.publish(outTopic, rxpacket);
  //}
}
