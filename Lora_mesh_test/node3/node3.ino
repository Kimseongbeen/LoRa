#include <Wire.h>  
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <ArduinoJson.h>
#include "CRC.h"
#include "SPIFFS.h"
#include "heltec.h"
#include "HT_SSD1306Wire.h"


SSD1306Wire  displayOled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

#define TRIGGER_PIN 37 // trigger pin 13

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

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
char lastRxPacket[BUFFER_SIZE];
static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void display(int no);
void tick();
void changeLoraTo();
void crd16Rtu();
void serialEvent();
void tickMeasure();
typedef enum
{
    LOWPOWER,
    STATE_RX,
    STATE_TX
}States_t;

int16_t txNumber=0;
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
int msgType=0; //0=measure 1=전달(packet 을 lora로 보냄)
int checkCount=0;
String TestName = "node3";
unsigned int counter = 0;
unsigned long previousMillis = 0;     
unsigned long interval = random(100, 200) * 100;  // 출력 간격(10~20분)
String inputString = "";         // 받은 문자열
char mac[20];  //mac address
//같은신호 방지.1
//rxpacket last_packet;
//
//Packet create_packet() {
//  String packet = "";
//  for(int i=0;i<4;i++) {
//    packet += String(random(0, 10));
//  }
//  return rxpacket;
//}
//
//void send_packet(Packet packet) {
//  int packet_size = packet.length();
//  Radio.Send((uint8_t*)packet.c_str(), packet_size);
//}
//json을 위한 설정
StaticJsonDocument<200> doc;
DeserializationError error;
JsonObject root;

void setup() {
  Serial.begin(9600);
  //Serial2.begin(9600);
  //Serial2.begin(4800, SERIAL_8N1, 6, 5);
      Serial.println("mac address");
    //이름 자동으로 생성
    uint64_t chipid;
    chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
    loraC.mac=String(((uint16_t)(chipid>>32)),HEX)+String(((uint32_t)chipid),HEX);  
    Serial.println(loraC.mac);

    Mcu.begin();
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
}

void loop() {
  tick();
  switch(state){
    case STATE_TX:
      //Serial.println(msgTo);
      txNumber++;
      msgTo.toCharArray(txpacket,msgTo.length());
      Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
      msgType=0; 
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

void tick() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    interval = random(100, 200) * 100; //10~20초
    if(msgType==0) {
      tickMeasure();
      state=STATE_TX;
      displayAct(2);
//    displayOled.drawString(0, 20, "Humidity :"+String(humi)+" %");
//    displayOled.drawString(0, 40, "Temperature :"+String(temp)+" °C");      
      //counter++;
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
// 메세지 전달할 mac rssi 현재 lora의 layer 저장
void changeLoraTo() {
  //if(macFrom.length() !=12) return;
  loraT.mac="";
  loraT.mac+=loraF.mac;
  loraT.rssi=loraF.rssi;
  loraC.layer=loraF.layer+1;
  saveConfig();  
}


void displayAct(int no) {
  displayOled.clear();
  displayOled.drawString(0, 0, "Lola Client");
  displayOled.drawString(0, 10, "mac: "+loraC.mac);
  if(no==0) { //lora 시작
    displayOled.drawString(0, 20, "Start");
  }
  else if(no==1) {
    displayOled.drawString(0, 0, "Lora Start");
  }
  else if(no==2) {
    displayOled.drawString(0, 20, "TestName :"+TestName);
    displayOled.drawString(0, 40, "checkCount :"+String(checkCount));
  }
  else if(no==3) {
    displayOled.drawString(0, 20, "rxpacket");
    displayOled.drawString(0, 40, rxpacket);
  }
  else if(no==4) {
    displayOled.drawString(0, 20, "Out Out");
  }
  else if(no==5) {
    displayOled.drawString(0, 20, "Lora Reset");
  }
  displayOled.display();
}

void tickMeasure() {
  //++로 각 신호에 번호를 붙여서 통신이 된다면 그 값을 저장하여 얼마나 누락이 됐는지 확인한다.
  checkCount++;
////새로운 패킷 만들기
//Packet packet = create_packet();
//if (packet != last_packet) {
//// 이전 패킷과 다르면 전송
//send_packet(packet);
//last_packet = packet;
//}
  //측정값 로라로 전송
  DynamicJsonDocument doc(1024);
  doc["mac"] = loraC.mac;
//  doc["dire"]   = -1;
//  doc["macTo"] = loraT.mac;
//  doc["macFrom"] = loraF.mac;
//  Serial.println(loraF.mac);
  //doc["macFrom"] = loraC.mac;
  doc["name"] = TestName;
//  doc["layer"] = loraC.layer;
//  doc["rssi"] = loraF.rssi;
  doc["count"] = checkCount;
//  doc["humi"] = humi;
//  doc["temp"] = temp;
  msgTo="";
  serializeJson(doc, msgTo);
}

//LORA로 메세지 받으면 동작
//전달할 데이타중 조건에 맞으면 loraT로 SPIFF에 저장
//새로이 전달되어온 데이타는 loraF에 저장
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ){
  Rssi=rssi;
  rxSize=size;
  memcpy(rxpacket, payload, size );
  rxpacket[size]='\0';
  Radio.Sleep( );
  state=STATE_RX; 
  // 수신된 패킷이 마지막으로 수신된 패킷과 동일한지 확인합니다
  if (memcmp(rxpacket, lastRxPacket, size) == 0) {
    // If the packets are the same, don't proceed with processing
    return;
  }  

  // 패킷이 다른 경우 새 패킷을 lastRxPacket에 저장하고 계속합니다
  memcpy(lastRxPacket, rxpacket, size);  
  Serial.println(rxpacket); //rxpacket = {"mac":"106580fa12f4","type":13,"state":1,"mac":"dc6481fa12f4","dire":1,"layer":0
  loraF.rssi=rssi;
  displayAct(3);
  
  String packet1(rxpacket);
  
  String packet = packet1 + "}";
  deserializeJson(doc,packet);
  root = doc.as<JsonObject>();
  String macObj = root["macObj"];
  String macC = root["mac"];
  String macFrom = root["macFrom"];
  String macTo = root["macTo"];
  int dire = root["dire"];
  int layerIn = root["layer"];
  int func = root["func"];
//  Serial.println("macC:"+macC);
//  Serial.println("macFrom:"+macFrom);
//  Serial.println("macTo:"+macTo);
//  Serial.println("dire:"+String(dire));
//  Serial.println("layerIn:"+String(layerIn));
/*loraF.mac = macC;
//if(macFrom == "") {
//  loraF.mac = macC;
//} else {
//  loraF.mac = macFrom;
//}

  Serial.println("loraF.mac:"+loraF.mac);
  //dire = -1 은 상위 노드로 올라감, dire = 1 하위 노드로 내려감
  loraF.dire = dire;
  Serial.println("loraF.dire:"+String(loraF.dire));
  //loraF.layer(데이터 출처의 레이어층 = 들어온 값의 layer)
  loraF.layer = layerIn;
  Serial.println("loraF.layer:"+String(loraF.layer));
  //loraT.mac이 비어있으면 전송된 데이터를 loraT.mac으로 입력
  if( (loraT.mac.length()<6) && (loraF.rssi > -90)) {
   changeLoraTo();
  }
//신호가 상위 layer한테 전달
  if(dire==-1) {
    //본체mac값이 상대가 보낸 macTo랑 같고 내layer가 0 이 아니라면
    if(loraC.mac==macTo && loraC.layer!=0){
      packet.replace("\"layer\":"+String(layerIn),"\"layer\":"+String(loraC.layer));
      packet.replace("\"macFrom\":\""+macFrom,"\"macFrom\":\""+loraF.mac);
      packet.replace("\"macTo\":\""+macTo,"\"macTo\":\""+loraT.mac);
      packet.replace("\"mac\":\""+macC,"\"mac\":\""+loraC.mac);
      msgTo="";
      msgTo+=packet;
      msgType=1;
      Serial.println("Bridge "+msgTo);
      state=STATE_TX;
    }
  }
//신호가 하위 layer한테 전달
  if(dire==1) {
    //입력된 macTo 가 없을 때 macFrom 을 macTo로 입력
    // 더 좋은 수신호가 있을 때 macTo로 입력
    if((loraF.rssi-loraT.rssi) > 10 && (layerIn-loraC.layer) <= -1) {
      changeLoraTo();
    }
    // 메세지 전달 자기와 링크된 로라의 msg만 전달한다.
    if(loraF.mac==loraT.mac && macObj!=loraC.mac){
      //delay(random(0,3000));
      packet.replace("\"layer\":"+String(layerIn),"\"layer\":"+String(loraC.layer));
      packet.replace("\"macFrom\":\""+loraF.mac,"\"macFrom\":\""+loraC.mac);
      msgTo="";
      msgTo+=packet;
      msgType=1;
      Serial.println("Bridge(+전달) "+msgTo);
      state=STATE_TX;
    }
    //명령수행
    //if(macObj==loraC.mac|| macObj=="ffffffffffff"){
    if(macObj==loraC.mac){
      Serial.println("msgFrom: "+packet);
      displayAct(4);
      if(func==255)
        factoryDefault();
    }
  } */
}
