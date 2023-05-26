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
  // Next lines have to be done ONLY ONCE!!!!!When SPIFFS is formatted ONCE you can comment these lines out!!
  //Serial.println("Please wait 30 secs for SPIFFS to be formatted");
  //SPIFFS.format();

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
  delay(random(0,4000));
  ESP.restart();
  delay(1000);
}

// trigger pin 0(D3) 2(D4)
void factoryDefault() {
    Serial.println("AP mode as Factory Deafault");
    Serial.println("Please wait over 3 min");
    displayOled(5);
    SPIFFS.format();
    delay(1000);
    ESP.restart();
    delay(1000);
}
