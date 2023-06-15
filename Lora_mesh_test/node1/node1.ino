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
#define RF_FREQUENCY                                923000000 // Hz 라디오가 작동하는 주파수(Hertz 단위)입니다 923MHz

#define TX_OUTPUT_POWER                             5        // dBm 값이 높을수록 범위가 증가하지만 전력 소비도 증가합니다.

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,LoRa 신호의 대역폭, 대역폭이 높을수록 데이터 속도는 빨라지지만 범위는 줄어듭니다.
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12] 확산 계수가 높을수록 범위는 증가하지만 데이터 속도는 감소합니다.
#define LORA_CODINGRATE                             2         // [1: 4/5, LoRa 신호의 부호화율
                                                              //  2: 4/6, 코딩 속도가 높을수록 데이터 중복성이 증가하여 간섭에 대한 저항성이 향상
                                                              //  3: 4/7, 되지만 데이터 속도는 감소합니다.
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx LoRa 패킷의 프리앰블 길이 프리앰블은 수신기가 송신기와 동기화할 수 있도록 각 패킷의 시작 부분에서 전송되는 일련의 기호
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols 수신기가 송신기에서 기호를 기다리는 최대 시간입니다.
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false     //true일경우 payload 고정
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 200 // Define the payload size here

#define board_ID 1

typedef enum
{
    LOWPOWER,
    STATE_RX,
    STATE_TX
}States_t;
  
States_t state;
bool sleepMode = false;
int16_t Rssi,rxSize;

class Lora {
  public:
    String mac;
    int dire=0;
    int rssi=-200; //-80 보다 크면 아주 양호 -80~-90 양호  -90~-100 불량  -100보다 작음 나쁨
    int layer=100;
    int ID;
    int index;
};
Lora loraC; // lora Client
Lora loraF; // lora onReceive 함수에서 From 임시로 전달받은 로라, 통시방향 관계없이 전달된 값
Lora loraT; // lora To 나에게 링크되어 메세지 서버쪽으로 전달할 로라
class Lora IDdata[200];
byte received_ID[200] = { 0, };
byte tickmeasureS = 0;
//String packet;
int packSize = 0;
String msgTo;
int msgType=0; //0=measure 1=전달(packet 을 lora로 보냄)
int checkCount=0;

unsigned long previousMillis = 0;     
unsigned long interval = random(100, 200) * 100;  // 출력 간격(10~20분)

char mac[20];  //mac address
//json을 위한 설정
StaticJsonDocument<200> doc;
DeserializationError error;
JsonObject root;
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
char lastRxPacket[BUFFER_SIZE];
static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void display(int no);
void tick();
void changeLoraTo(String macFrom,int rssiFrom, int layerNew);
void crd16Rtu();
void serialEvent();
void tickMeasure();

void setup() {
setup_function();
}

void loop() {
  tick();
  switch(state){
    case STATE_TX:
    if(loraT.mac.length()<6) return;
    displayAct(10);
      msgTo.toCharArray(txpacket,msgTo.length());
      Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
      msgType=0; 
      VextON();
      state=LOWPOWER;
      break;
    case STATE_RX:
      Radio.Rx( 0 );
      state=LOWPOWER;
      break;
    case LOWPOWER:
      Radio.IrqProcess( );
      VextOFF();
      break;
    default:
      break;
  }
  //공장리셋
  if ( digitalRead(TRIGGER_PIN) == LOW ) 
    factoryDefault();
}


void tick() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    interval = random(300, 800) * 100;
    if(loraT.mac.length()>6) { //데이터 저장될때 
    tickMeasure(); // 내 기기의 데이터 전송,
      byte num_of_received_sensor = 0;  
      for (int i = 0; i < 200; i++) {   //수신한 센서 갯수 계산
          if (received_ID[i] == 0) break; //수신한 센서 중 최신화된 값들 전송,
          num_of_received_sensor++;
      }
      Serial.println("통신하고있는 모듈 수 : "+String(num_of_received_sensor));
      //delay(30);
      displayAct(2);
    }
  }  
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
    //displayOled.drawString(0, 20, "TestName :"+TestName);
    //displayOled.drawString(0, 40, "index :"+index);
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
  else if(no==10) {  // 통신테스트 때 사용
    displayOled.drawString(0, 20, loraC.mac+"  layer "+String(loraC.layer));
    //displayOled.drawString(0, 10, "index  "+index);
    displayOled.drawString(0, 30, "to    "+loraT.mac);
    displayOled.drawString(0, 40, "from  "+loraF.mac);
  }
  displayOled.display();
}
void tickMeasure() {
  
  //측정값 로라로 전송
  DynamicJsonDocument doc(1024);
  doc["mac"] = loraC.mac;
  doc["dire"]   = -1;
  doc["macTo"] = loraT.mac;
  doc["macFrom"] = loraC.mac;
  doc["layer"] = loraC.layer;
  doc["rssi"] = loraF.rssi;
  doc["ID"] = board_ID;
  doc["index"] = checkCount;
  msgTo="";
  serializeJson(doc, msgTo);
  //++로 각 신호에 번호를 붙여서 통신이 된다면 그 값을 저장하여 얼마나 누락이 됐는지 확인한다.
  checkCount++;
  state=STATE_TX;
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
  loraF.rssi=rssi;
  displayAct(3);
  
  String packet1(rxpacket);
  
  String packet = packet1 + "}";
  deserializeJson(doc,packet);
  root = doc.as<JsonObject>();
  Serial.println(packet);
  bool save_flag = 1; //저장할지말지 변수
  int save_index = 0;
  
  int Cindex = root["index"];     
  int layerIn = root["layer"];    
  // node와 게이트웨이의 차이점 layer, dire두가지 인데 layer = 0이고 dire가 1
  //1. node한테 들어오는 데이터 2.게이트웨이에서 들어오는 데이터
  if (layerIn != 0 && Cindex == 0) return;
  
  String macIn = root["mac"];
  String macFrom = root["macFrom"];
  String macTo = root["macTo"];
  int dire = root["dire"];
  int func = root["func"];
  int ID = root["ID"];              //ID = arr[1];

  save_flag = 1; //저장할지말지 변수
  save_index = 0;
  for (int i = 0; i < 200; i++) {
    //수신된 ID 목록에 지금 수신한 ID가 있고 수신한 인덱스가 더 오래된것일 때 저장안함
    if (received_ID[i] == ID && (IDdata[ID].index >= Cindex)) {
      save_flag = 0;
      if (abs(int(Cindex - IDdata[ID].index)) > 255) {   //2바이트 숫자 넘어가면 저장함
        save_flag = 1;
      }
    }
  }
  //내 보드아이디와 같다면 저장안함, 내 데이터가 다시 나한테 온 경우,
  if (board_ID == ID) {
    save_flag = 0;
  }
  if (save_flag == 1) {   //데이터 저장
    IDdata[ID].ID = ID;
    IDdata[ID].index = Cindex;
  Serial.print("save_ID : " + String(ID));
  Serial.println("||save_index : " + String(Cindex));
    tickmeasureS = 1;
    for (int i = 0; i < 200; i++) {   
      if (received_ID[i] == 0) break; 
      if (received_ID[i] == ID) break; 
      save_index++; 
    }
    received_ID[save_index] = ID; 
  }
  loraF.mac = "";
  loraF.mac += macFrom;
  loraF.dire = dire;
  loraF.layer = layerIn;
  Serial.print("func:"+String(func));
  Serial.println(" layer:"+String(layerIn));
  if( func == 1 && layerIn == 0) {
  changeLoraTo(loraF.mac, loraF.rssi, layerIn+1);
  }
  if( (loraT.mac.length()<6) && (loraF.rssi > -130)) {
   changeLoraTo(loraF.mac, loraF.rssi, layerIn+1);
  }
//데이터 위로 올라감.
  if(dire==-1) {
    //본체mac값이 상대가 보낸 macTo랑 같고 내layer가 0 이 아니라면 layerIn(들어온layer 2) > loraC.layer(내layer 3)
    if(macTo==loraC.mac && loraC.layer!=0 && tickmeasureS == 1 && layerIn > loraC.layer){
      packet.replace("\"layer\":"+String(layerIn),"\"layer\":"+String(loraC.layer));
      packet.replace("\"macFrom\":\""+macFrom,"\"macFrom\":\""+loraC.mac);
      packet.replace("\"macTo\":\""+macTo,"\"macTo\":\""+loraT.mac);
      msgTo="";
      msgTo+=packet;
      msgType=1;
      Serial.println("Bridge(-전달) "+msgTo);
      state=STATE_TX;
    }
    tickmeasureS = 0;
  }
//신호가 하위 layer한테 전달
  if(dire==1) {
    //입력된 macTo 가 없을 때 macFrom 을 macTo로 입력
    // 더 좋은 수신호가 있을 때 macTo로 입력
    if((loraF.rssi-loraT.rssi) > 10 && (layerIn-loraC.layer) <= -1) {
      changeLoraTo(loraF.mac, loraF.rssi, layerIn+1);
    }
    // 메세지 전달 자기와 링크된 로라의 msg만 전달한다.
    if(loraF.mac==loraT.mac && macIn!=loraC.mac){
      packet.replace("\"layer\":"+String(layerIn),"\"layer\":"+String(loraC.layer+1));
      packet.replace("\"macFrom\":\""+loraF.mac,"\"macFrom\":\""+loraC.mac);
      msgTo="";
      msgTo+=packet;
      msgType=1;
      Serial.println("Bridge(+전달) "+msgTo);
      state=STATE_TX;
    }
    //명령수행
    if(macTo==loraC.mac || macIn=="ffffffff"){
      Serial.println("reset msg: "+packet);
      displayAct(4);
      if(func==255)
        factoryDefault();
    }
  } 
}
