
byte Key[8] = { 0x20, 0x95, 0x09, 0x4B, 0x39, 0x00, 0x00, 0x00 };
int iButtonPort = 2;
void setup() {
  // put your setup code here, to run once:
  pinMode(iButtonPort, OUTPUT);
}

void loop() {
  Emulate(Key);
  delay(100);

}


void SendBitMetacom(bool Bit) {

    //берем период за 150 мкс
    digitalWrite(iButtonPort, 1);  //низкий уровень тока

    if (Bit) delayMicroseconds(80);  //1 это удержание 2/3 периода
    else delayMicroseconds(20);

    digitalWrite(iButtonPort, 0);
    if (Bit) delayMicroseconds(20);
    else delayMicroseconds(80);
  }



  void Emulate(byte Key[8]) {

    pinMode(iButtonPort, OUTPUT);
    //Бит синхронизации и стартовое слово
    digitalWrite(iButtonPort, 0);
    delayMicroseconds(150);
    SendBitMetacom(0);
    SendBitMetacom(1);
    SendBitMetacom(0);

    byte bInd = 0;
    for (int i = 1; i < 5; i++) {
     // Serial.print(Key[i], HEX);

      for (int bInd = 0; bInd < 8; bInd++) {
        bool bit = (((Key[i]) & (0x1 << (7 - bInd))) != 0);  //получаем бит
        SendBitMetacom(bit);
       // Serial.print(bit);
      }
     // Serial.println(" ");
    }
  }