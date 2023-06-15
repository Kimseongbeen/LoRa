void setup_function(){
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  Serial.println("mac address");
  
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
