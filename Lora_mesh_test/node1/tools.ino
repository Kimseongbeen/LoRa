void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
  
}

void VextOFF(void) //Vext default OFF
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

// 메세지 전달할 mac rssi 현재 lora의 layer 저장
void changeLoraTo(String macFrom,int rssiFrom, int layerNew) {
  //if(macFrom.length() !=12) return;
  loraT.mac="";
  loraT.mac+=loraF.mac;
  loraT.rssi=loraF.rssi;
  loraC.layer=loraF.layer+1;
  saveConfig();  
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

void readConfig() {
  SPIFFS.begin();
  File f = SPIFFS.open("/config.txt", "r");
  if (!f) {
      Serial.println("file open failed");
  }  Serial.println("====== Reading from SPIFFS file =======");
  String mac;
  loraT.mac=f.readStringUntil('\n');
  loraT.mac.replace(0x0a,0x00);
  loraT.mac.remove(loraT.mac.length()-1);
  loraT.rssi=f.readStringUntil('\n').toInt();
  loraC.layer=f.readStringUntil('\n').toInt();
  f.close();
  SPIFFS.end();
  Serial.println("macTo: "+loraT.mac);
  Serial.println("rssiTo: "+String(loraT.rssi));
  Serial.println("layer: "+String(loraC.layer));
}

void saveConfig() {
  SPIFFS.begin();
  // open file for writing
  File f = SPIFFS.open("/config.txt", "w");
  if (!f) {
      Serial.println("file open failed");
  }
  f.println(loraT.mac);
  f.println(String(loraT.rssi));
  f.println(String(loraC.layer));
  f.close();
  SPIFFS.end();
  Serial.println("Save SPIFFS");
  delay(1000);
  ESP.restart();
  delay(1000);
}


// trigger pin 0(D3) 2(D4)
void factoryDefault() {
    Serial.println("AP mode as Factory Deafault");
    Serial.println("Please wait over 3 min");
    //displayOled(5);
    SPIFFS.format();
    delay(1000);
    ESP.restart();
    delay(1000);
}
