#include <ArduinoJson.h>
#include "CRC.h"
#include "heltec.h"
#include <SoftwareSerial.h>
#include "SPIFFS.h"

#define BAND    923E6
#define TRIGGER_PIN 13 // trigger pin GPIO13 (D13)
SoftwareSerial mySerial(23, 17); // RX,TX

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

String packet;
int packSize = 0;
String msgTo;
int msgType=0; //0=measure 1=전달packet 을 lora로 보냄

int type=8; 

unsigned int counter = 0;
unsigned long previousMillis = 0;     
const long interval = 10000; 
char mac[20];  //mac address
String inputString = "";         // 받은 문자열

float temp=0.,humi=0.,pres=0.;

void onReceive(int packetSize);
void changeLoraTo(String macFrom,int rssiFrom, int layerNew);
void displayOled(int no);
void sendLora();
void HeltecSetup();
void setup();
void tick();
void tickMeasure();

//json을 위한 설정
StaticJsonDocument<200> doc;
DeserializationError error;
JsonObject root;

//LORA로 메세지 받으면 동작
//전달할 데이타중 조건에 맞으면 loraT로 SPIFF에 저장
//새로이 전달되어온 데이타는 loraF에 저장
void onReceive(int packetSize) {
  if(packetSize <=0)
    return;
  String packet ="";
  for (int i = 0; i < packetSize; i++) { packet += (char) LoRa.read(); }
  Serial.println("receive "+packet);
  //데이터의 출처의 신호는 = 받은 packet의 Rssi
  loraF.rssi=LoRa.packetRssi();
  
  //displayOled(3);

  deserializeJson(doc,packet);
  root = doc.as<JsonObject>();
  String macIn = root["mac"];
  String macFrom = root["macFrom"];
  String macTo = root["macTo"];
  int dire = root["dire"];
  int layerIn = root["layer"];
  int func = root["func"];
  //dire = 1 이 되는순간 , 게이트웨이에서 신호 보낼때만 1
  loraF.mac = "";
  loraF.mac += macFrom;
  loraF.dire = dire;
  loraF.layer = layerIn;

  //loraT.mac이 비어있으면 전송된 데이터를 loraT.mac으로 입력
  if((loraT.mac.length()<6) && (loraF.rssi > -90)) {
    changeLoraTo();
  }

  if(dire==-1) {
    if(loraC.mac==macTo && loraC.layer!=0){
      packet.replace("\"layer\":"+String(layerIn),"\"layer\":"+String(loraC.layer));
      packet.replace("\"macFrom\":\""+macFrom,"\"macFrom\":\""+loraC.mac);
      packet.replace("\"macTo\":\""+macTo,"\"macTo\":\""+loraT.mac);
      msgTo="";
      msgTo+=packet;
      msgType=1;
      Serial.println("Bridge(-전달) "+msgTo);
      sendLora(); //msgTo 전송, 보내는 부분에 넣어야 할까?
      msgType=0;
    }
  }

  if(dire==1) {
    //입력된 macTo 가 없을 때 macFrom 을 macTo로 입력
    // 더 좋은 수신호가 있을 때 macTo로 입력
    if((loraF.rssi-loraT.rssi) > 10 && (layerIn-loraC.layer) <= -1) {
      changeLoraTo();
    }
    // 메세지 전달 자기와 링크된 로라의 msg만 전달한다.
    if(loraF.mac==loraT.mac && macIn!=loraC.mac){
      //
      packet.replace("\"layer\":"+String(layerIn),"\"layer\":"+String(loraC.layer));
      packet.replace("\"macFrom\":\""+loraF.mac,"\"macFrom\":\""+loraC.mac);
      msgTo="";
      msgTo+=packet;
      msgType=1;
      Serial.println("Bridge(+전달) "+msgTo);
      sendLora();
      msgType=0;
    }
    //명령수행
    //if(macIn==loraC.mac|| macIn=="ffffffffffff"){
    if(macIn==loraC.mac){
      Serial.println("msgFrom: "+packet);
      displayOled(4);
      if(func==255)
        factoryDefault();
    }
  }
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

void setup() {
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  HeltecSetup();
  Serial1.begin(9600, SERIAL_8N1, 23, 17);
  displayOled(0);
  //Serial.begin(115200); //선언하면 에러 발생
  //while (!Serial);

  Serial.println("mac address");
  //이름 자동으로 생성
  uint64_t chipid;
  chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
  loraC.mac=String(((uint16_t)(chipid>>32)),HEX)+String(((uint32_t)chipid),HEX);  
  Serial.println(loraC.mac);

  readConfig();

  Serial.println("init ok");
  displayOled(1);
  //factoryDefault();
}

void HeltecSetup() {
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  LoRa.setSpreadingFactor(7);           // ranges from 6-12,default 7 see API docs
  Heltec.display->init();
  Heltec.display->flipScreenVertically();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);

}

void displayOled(int no) {
  Heltec.display->clear();
  if(no==0) { //lora 시작
    Heltec.display->drawString(0, 0, "Lola Client");
    Heltec.display->drawString(0, 20, "Start");
  }
  else if(no==1) {
    Heltec.display->drawString(0, 0, "Lora Start");
  }
  else if(no==2) {
    Heltec.display->drawString(0, 0, loraC.mac+"  layer "+String(loraC.layer));
    //Heltec.display->drawString(0, 10, "layer  "+String(loraC.layer));
    Heltec.display->drawString(0, 10, "temperature  "+String(temp));
    Heltec.display->drawString(0, 20, "humidity  "+String(humi));
    Heltec.display->drawString(0, 30, "prresure  "+String(pres));
  }
  else if(no==3) {
    Heltec.display->drawString(0, 20, packet);
  }
  else if(no==4) {
    Heltec.display->drawString(0, 20, "Out Out");
  }
  else if(no==5) {
    Heltec.display->drawString(0, 0, "Lora Reset");
  }
  else if(no==10) {  // 통신테스트 때 사용
    Heltec.display->drawString(0, 0, loraC.mac+"  layer "+String(loraC.layer));
    Heltec.display->drawString(0, 10, "counter  "+String(counter));
    Heltec.display->drawString(0, 20, "to    "+loraT.mac);
    Heltec.display->drawString(0, 30, "from  "+loraF.mac);
    Heltec.display->drawString(0, 40, "temp "+String(temp)+"   humi "+String(humi));
  }
  Heltec.display->display();
}

void loop() {
  onReceive(LoRa.parsePacket());
  tick();
  serial1Event();
  
  //공장리셋
  if ( digitalRead(TRIGGER_PIN) == LOW ) 
    factoryDefault();
}

//1초 마다 실행되는 시간함수
void tick() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if(msgType==0) {
      tickMeasure();
    }
    msgType=0;
    //displayOled(2);
    displayOled(10);
    counter++;
  }  
}

//로라통신으로 측정된 값을 보낸다.
void sendLora() {
  if(loraT.mac.length()<6)
    return;
    
  LoRa.beginPacket();
  LoRa.setTxPower(14,RF_PACONFIG_PASELECT_PABOOST);
  Serial.println("");
  Serial.println("to "+loraT.mac);
  Serial.println("from "+loraF.mac);
  Serial.println("send "+msgTo);
  Serial.println("");
  LoRa.print(msgTo);
  LoRa.endPacket();
  /*
  * LoRa.setTxPower(txPower,RFOUT_pin);
  * txPower -- 0 ~ 20
  * RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
  *   - RF_PACONFIG_PASELECT_PABOOST -- LoRa single output via PABOOST, maximum output 20dBm
  *   - RF_PACONFIG_PASELECT_RFO     -- LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
  */
}

void tickMeasure() {
  //온도 습도 측정
  crd16Rtu();
  
  //측정값 로라로 전송
  //temp=0.,humi=0.,pres=0.;
  DynamicJsonDocument doc(1024);
  doc["mac"] = loraC.mac;
  doc["dire"]   = -1;
  doc["macTo"] = loraT.mac;
  doc["macFrom"] = loraC.mac;
  doc["type"] = type;
  doc["layer"] = loraC.layer;
  doc["rssi"] = loraF.rssi;
  
  doc["count"] = counter;
  doc["temp"] = temp;
  doc["humi"] = humi;
  doc["pres"] = pres;
  msgTo="";
  serializeJson(doc, msgTo); 
  
  sendLora();
}

void serial1Event() {
  if(Serial1.available() == false)
    return;
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
    //Serial.print(inChar,HEX);
    // add it to the inputString:
    inputString += inChar;
  }
  //Serial.println("");
  if(inputString.length() >= 11) {
    String ss="";
    ss=((float)inputString.charAt(3)*255+(float)inputString.charAt(4))/100-40;
    temp=ss.toFloat();
    Serial.println("온도 : "+ss+" 도"); 
    ss=((float)inputString.charAt(5)*255+(float)inputString.charAt(6))/100;
    humi=ss.toFloat();
    Serial.println("습도 : "+ss+" %");
    ss=((float)inputString.charAt(7)*255+(float)inputString.charAt(8))/10;
    pres=ss.toFloat();
    Serial.println("압력 : "+ss+" hpa");
    inputString="";
    Serial.println("");
  }
}

// 아두이노에서 RS485 출력을 내보낸다.
void crd16Rtu() {
  //char str[24] =  {0x00,0x03,0x00,0x01,0x00,0x01,0x00,0x00,0x00};  //Read station number
  char str[24] =  {0xff,0x03,0x00,0x00,0x00,0x03,0x00,0x00,0x00};  //Read Date
  String s;
  int si,sj,len;

  len=6;
  
  uint8_t * data = (uint8_t *) &str[0];
  si=crc16(data, len, 0x8005, 0xFFFF, 0x0000, true,  true  );
  sj=si&0xff;
  str[len]=sj;
  sj=si>>8;
  str[len+1]=sj;

  //Serial.println("");
  for(int i=0;i<len+2;i++) {
    Serial1.print(str[i]);
    //Serial.print(" ");
    //Serial.print((int)str[i],HEX);
  }
}
