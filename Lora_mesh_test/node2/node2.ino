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

#define board_ID 2
#define test_led1 35

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
//String TestName = "node1";
unsigned int counter = 0;
unsigned long previousMillis = 0;     
unsigned long interval = random(100, 200) * 100;  // 출력 간격(10~20분)
String inputString = "";         // 받은 문자열
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
    displayAct(10);
      msgTo.toCharArray(txpacket,msgTo.length());
      Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
      msgType=0; 
      state=LOWPOWER;
      break;
    case STATE_RX:
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
    interval = random(100, 200) * 100;
    if(msgType==0) {
    tickMeasure(); // 내 기기의 데이터 전송,
      byte num_of_received_sensor = 0;  
      for (int i = 0; i < 200; i++) {   //수신한 센서 갯수 계산
          if (received_ID[i] == 0) break; //수신한 센서 중 최신화된 값들 전송,
          num_of_received_sensor++;
      }
      Serial.println("통신하고있는 모듈 수 : "+String(num_of_received_sensor));
//      delay(30);
//      for (int i = 0; i < num_of_received_sensor; i++) { //수신한 센서 갯수만큼 반복
//          {
//          state=STATE_TX;
//          received_ID[i] = 0;
//          delay(30);
//      }     
//}
      displayAct(2);
    }
  }  
}

void OnTxDone( void )
{
  state=STATE_RX;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
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
    displayOled.drawString(0, 0, loraC.mac+"  layer "+String(loraC.layer));
    //displayOled.drawString(0, 10, "index  "+index);
    displayOled.drawString(0, 20, "to    "+loraT.mac);
    displayOled.drawString(0, 30, "from  "+loraF.mac);
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
  //doc["name"] = TestName;
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
  //Serial.print("받은페킷: ");
  //Serial.println(rxpacket); //rxpacket = {"mac":"106580fa12f4","type":13,"state":1,"mac":"dc6481fa12f4","dire":1,"layer":0
  loraF.rssi=rssi;
  displayAct(3);
  
  String packet1(rxpacket);
  
  String packet = packet1 + "}";
  deserializeJson(doc,packet);
  root = doc.as<JsonObject>();
  
  static bool led_state = 0;
  bool save_flag = 1; //저장할지말지 변수
  int save_index = 0;
  if (led_state == 1) led_state = 0;    //데이터 수신하면 LED 토글
  else led_state = 1;
  digitalWrite(test_led1, led_state);
  
  String macIn = root["mac"];
  String macFrom = root["macFrom"];
  String macTo = root["macTo"];
  int dire = root["dire"];
  int layerIn = root["layer"];
  int func = root["func"];
  int ID = root["ID"];              //ID = arr[1];
  int Cindex = root["index"];     //data3 = (arr[7] << 8) + arr[6];

  save_flag = 1; //저장할지말지 변수
  save_index = 0;
  for (int i = 0; i < 200; i++) {
    //수신된 ID 목록에 지금 수신한 ID가 있고 수신한 인덱스가 더 오래된것일 때 저장안함
    if (received_ID[i] == ID && (IDdata[ID].index >= Cindex)) {
      save_flag = 0;
      /*이는 두 인덱스 간의 숫자가 너무 크면 새로운 패킷으로 간주하고 저장하기 위함입니다. 
       * 이 조건이 필요한 이유는 인덱스가 0부터 200까지 범위로 돌고 있는데, 
       * 만약 이전에 199의 인덱스를 가진 데이터를 받았고, 지금 받은 데이터의 인덱스가 3이라면, 
       * 숫자가 순환했기 때문에 이전 데이터보다 새로운 것이며 이를 저장해야 합니다.
      */
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
    for (int i = 0; i < 200; i++) {   //저장한거 확인하는 배열에
      if (received_ID[i] == 0) break; //0이면 빈공간
      if (received_ID[i] == ID) break; //같은게 있으면 저장안함
      save_index++; //저장한거 확인하는 배열에 저장할 인덱스
    }
    received_ID[save_index] = ID; //received_ID[0] = 1; received_ID[1] = 2; received_ID[2] = 3;
  }

  loraF.mac = "";
  loraF.mac += macFrom;
  loraF.dire = dire;
  loraF.layer = layerIn;
  
  if( (loraT.mac.length()<6) && (loraF.rssi > -90)) {
   changeLoraTo();
  }
//신호가 상위 layer한테 전달
  if(dire==-1) {
    //본체mac값이 상대가 보낸 macTo랑 같고 내layer가 0 이 아니라면
    if(loraC.mac==macTo && loraC.layer!=0 && tickmeasureS == 1){
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
      changeLoraTo();
    }
    // 메세지 전달 자기와 링크된 로라의 msg만 전달한다.
    if(loraF.mac==loraT.mac && macIn!=loraC.mac && tickmeasureS == 1){
      packet.replace("\"layer\":"+String(layerIn),"\"layer\":"+String(loraC.layer));
      packet.replace("\"macFrom\":\""+loraF.mac,"\"macFrom\":\""+loraC.mac);
      msgTo="";
      msgTo+=packet;
      msgType=1;
      Serial.println("Bridge(+전달) "+msgTo);
      state=STATE_TX;
    }
    //명령수행
    if(macIn==loraC.mac){
      Serial.println("msgFrom: "+packet);
      displayAct(4);
      if(func==255)
        factoryDefault();
    }
    tickmeasureS = 0;
  } 
}
