
//#include "user_interface.h"
#include <driver/adc.h>
#include "AnalogReadFast.h"
#include <WiFi.h>


#define FreqGen 1

//settings
#define rfidUsePWD 0    // ключ использует пароль для изменения
#define rfidPWD 123456  // пароль для ключа
#define rfidBitRate 2  // Скорость обмена с rfid в kbps


class EM_Marine {

    enum emkeyTypes { keyUnknown,
                    keyDallas,
                    keyTM2004,
                    keyCyfral,
                    keyMetacom,
                    keyEM_Marine };  // тип оригинального ключа

  emkeyTypes keyType;


  enum emRWType { rwUnknown,
                  TM01,
                  RW1990_1,
                  RW1990_2,
                  TM2004,
                  T5557,
                  EM4305 };  // тип болванки

  emRWType keyRWType;

 public: int calcAverageU() {
    unsigned long sum = 512;
    for (byte i = 0; i < 255; i++) {
      delayMicroseconds(10);
      sum += adc1_get_raw(ADC1_CHANNEL_0);
    }
    sum = sum >> 8;
    return sum;
  }

  /*uint16_t analogRead()  //pin просто для совместимости
  {
    uint16_t res;                       // значения которые считываются ацп, может быть [1, 65535]
    system_adc_read_fast(&res, 1, 10);  //костыльное решение проблемы со временем, но работает за 17мкс, а не 80 (значения, колво, adc_clk_div)
    return res;
  }*/


  byte ttACompVork(int aver = 200, unsigned long timeOut = 7000, byte accuracy = 50) {  // pulse 0 or 1 or -1 if timeout
     byte AcompState, AcompInitState;
    unsigned long tEnd = micros() + timeOut;
  //  Serial.println(adc1_get_raw(ADC1_CHANNEL_0));
    AcompInitState = (adc1_get_raw(ADC1_CHANNEL_0) - aver>accuracy)  ; // читаем флаг компаратора
      do {
      AcompState = (adc1_get_raw(ADC1_CHANNEL_0) - aver>accuracy);  // читаем флаг компаратора
        if (AcompState != AcompInitState) {
        //delayMicroseconds(1000 / (rfidBitRate * 4));    // 1/4 Period on 2 kBps = 125 mks
        delayMicroseconds(10); //128
        AcompState = (adc1_get_raw(ADC1_CHANNEL_0) - aver>accuracy) ;         // читаем флаг компаратора
        //delayMicroseconds(1000 / (rfidBitRate * 2));  // 1/2 Period on 2 kBps = 250 mks
        delayMicroseconds(195); //256
        return AcompState;
      }
    }
    while (micros() < tEnd)
      ;
    return 2;  //таймаут, компаратор не сменил состояние
  }


unsigned long tOld =0;
  byte ttAComp(int aver = 200, int period  = 230, unsigned long timeOut = 1000 ) {  // pulse 0 or 1 or -1 if timeout
    aver+=30;
    bool AcompState, AcompInitState;
    unsigned long tEnd = micros() + timeOut;
 
    AcompInitState = (adc1_get_raw(ADC1_CHANNEL_0) > aver)  ; // читаем флаг компаратора
    do {
      AcompState = (adc1_get_raw(ADC1_CHANNEL_0) > aver);  // читаем флаг компаратора
      if (AcompState != AcompInitState) {
        tEnd = micros() + period; // (230)256 - 23 - 10
        delayMicroseconds(1000 / (rfidBitRate * 6));    // 1/4 Period on 2 kBps = 125 mks
       // delayMicroseconds(10); //128
        AcompState = (adc1_get_raw(ADC1_CHANNEL_0) > aver) ;         // читаем флаг компаратора
        // delayMicroseconds(1000 / (rfidBitRate * 2));  // 1/2 Period on 2 kBps = 250 mks
        // delayMicroseconds(220); //256
      /*  Serial.print("t = ");
        Serial.println(tEnd - micros());*/
        if (AcompState == AcompInitState) continue; //проверка на ошибку
        delayMicroseconds((tEnd - micros()));
        return AcompState;
      }
      //delayMicroseconds(10);
    }
    while (micros() < tEnd)
      ;
    return 2;  //таймаут, компаратор не сменил состояние
  }

int PeriodCalc(int aver = 200)
{
  aver+=30;
  byte count = 0;
  unsigned long tStart = micros();
  unsigned long tEnd = micros() + 1000000;
  bool AcompState, AcompInitState;

  AcompInitState = (adc1_get_raw(ADC1_CHANNEL_0) > aver)  ; // читаем флаг компаратора
  while(count<128 && micros()<tEnd)
  {
      AcompState = (adc1_get_raw(ADC1_CHANNEL_0) > aver);  // читаем флаг компаратора
      if (AcompState != AcompInitState) count++;
      
      AcompInitState = AcompState;
  }

  return (micros() - tStart)>>6;
}


bool AcompState, AcompInitState;
unsigned long tEnd;

 byte ttAComp2(int aver = 200, unsigned long timeOut = 600, byte accuracy = 50) {  // pulse 0 or 1 or -1 if timeout
  // Serial.print("d =");
    //Serial.println(adc1_get_raw(ADC1_CHANNEL_0) - aver);
    tEnd = micros() + timeOut;
    AcompInitState = (adc1_get_raw(ADC1_CHANNEL_0) - aver > accuracy)  ; // читаем флаг компаратора
    do 
    {
      AcompState = (adc1_get_raw(ADC1_CHANNEL_0) - aver > accuracy);  // читаем флаг компаратора
      if (AcompState != AcompInitState) 
      {
        //delayMicroseconds(1000 / (rfidBitRate * 2));  // 1/2 Period on 2 kBps = 250 mks
        delayMicroseconds(220); //256
       // tOld = micros(); 
        return AcompState;

        //delayMicroseconds(80); //128
       // AcompState = ((adc1_get_raw(ADC1_CHANNEL_0) - aver) > accuracy) ;         // читаем флаг компаратора
        //delayMicroseconds(1000 / (rfidBitRate * 2));  // 1/2 Period on 2 kBps = 250 mks
       // delayMicroseconds(100); //128
        /*while(((adc1_get_raw(ADC1_CHANNEL_0) - aver) > accuracy) == AcompState);

        return AcompState;*/

        //delayMicroseconds(200); //256
       // return AcompState;
      }
    }
    while (micros() < tEnd);
    return 2;  //таймаут, компаратор не сменил состояние
  }

byte rfidData[5];  
 bool vertEvenCheck(byte* buf) {  // проверка четности столбцов с данными
    byte k;
    k = 1 & buf[1] >> 6 + 1 & buf[1] >> 1 + 1 & buf[2] >> 4 + 1 & buf[3] >> 7 + 1 & buf[3] >> 2 + 1 & buf[4] >> 5 + 1 & buf[4] + 1 & buf[5] >> 3 + 1 & buf[6] >> 6 + 1 & buf[6] >> 1 + 1 & buf[7] >> 4;
    if (k & 1) return false;
    k = 1 & buf[1] >> 5 + 1 & buf[1] + 1 & buf[2] >> 3 + 1 & buf[3] >> 6 + 1 & buf[3] >> 1 + 1 & buf[4] >> 4 + 1 & buf[5] >> 7 + 1 & buf[5] >> 2 + 1 & buf[6] >> 5 + 1 & buf[6] + 1 & buf[7] >> 3;
    if (k & 1) return false;
    k = 1 & buf[1] >> 4 + 1 & buf[2] >> 7 + 1 & buf[2] >> 2 + 1 & buf[3] >> 5 + 1 & buf[3] + 1 & buf[4] >> 3 + 1 & buf[5] >> 6 + 1 & buf[5] >> 1 + 1 & buf[6] >> 4 + 1 & buf[7] >> 7 + 1 & buf[7] >> 2;
    if (k & 1) return false;
    k = 1 & buf[1] >> 3 + 1 & buf[2] >> 6 + 1 & buf[2] >> 1 + 1 & buf[3] >> 4 + 1 & buf[4] >> 7 + 1 & buf[4] >> 2 + 1 & buf[5] >> 5 + 1 & buf[5] + 1 & buf[6] >> 3 + 1 & buf[7] >> 6 + 1 & buf[7] >> 1;
    if (k & 1) return false;
    if (1 & buf[7]) return false;
    //номер ключа, который написан на корпусе
    rfidData[0] = (0b01111000 & buf[1]) << 1 | (0b11 & buf[1]) << 2 | buf[2] >> 6;
    rfidData[1] = (0b00011110 & buf[2]) << 3 | buf[3] >> 4;
    rfidData[2] = buf[3] << 5 | (0b10000000 & buf[4]) >> 3 | (0b00111100 & buf[4]) >> 2;
    rfidData[3] = buf[4] << 7 | (0b11100000 & buf[5]) >> 1 | 0b1111 & buf[5];
    rfidData[4] = (0b01111000 & buf[6]) << 1 | (0b11 & buf[6]) << 2 | buf[7] >> 6;
    return true;
  }

  bool readEM_Marie(byte* buf) {
    int aver = calcAverageU();
    unsigned long tEnd = millis() + 150;
    byte ti;
    byte j = 0, k = 0;
    for (int i = 0; i < 64; i++) {  // читаем 64 bit
      int b = micros();
      ti = ttAComp(aver);
      /*Serial.print("Timeb = ");
      Serial.print(micros()-b);
      Serial.print(" Rez ");
      Serial.println(ti);*/
      if (ti == 2) break;          //timeout
      if ((ti == 0) && (i < 9)) {  // если не находим 9 стартовых единиц - начинаем сначала
        if (millis() > tEnd) {
          ti = 2;
          break;
        }  //timeout
        i = -1;
        j = 0;
        continue;
      }
      if ((i > 8) && (i < 59)) {  //начиная с 9-го бита проверяем контроль четности каждой строки
        if (ti) k++;              // считаем кол-во единиц
        if ((i - 9) % 5 == 4) {   // конец строки с данными из 5-и бит,
         if (k & 1) {            //если нечетно - начинаем сначала
            i = -1;
            j = 0;
            k = 0;
            continue;
          }
          k = 0;
        }
      }
      if (ti) bitSet(buf[i >> 3], 7 - j);
      else bitClear(buf[i >> 3], 7 - j);
      j++;
      if (j > 7) j = 0;
    }
    if (ti == 2) return false;  //timeout


            Serial.println("//////////EM1////////////");

            for (byte i = 9; i < 64; i++) {
                Serial.print(bitRead(buf[i>>3], 7-i%8));
                if((i-8)%5==0) Serial.print(" : ");
              }
            Serial.println("");


    return vertEvenCheck(buf);
  }


bool readEM_Marie2(byte* buf) {
  int aver = calcAverageU();
    unsigned long tEnd = millis() + 50;
    byte ti;
    //ждем 9 адяниц
    
    byte validate;
    byte ones = 0;
        //int period = PeriodCalc(aver)>>1;
    /*Serial.print("period = ");
    Serial.println(period);*/
   

      while(millis() < tEnd)
      {

          ti = ttAComp(aver) ;

          if (ti == 1) ones++;
          else ones = 0;          

          
          if(ones == 9) break;
      }
    // Serial.println();
      if(ones != 9) 
      {
        ones = 0;
        
       return false; //не нашли
      }
    
      
  //  Serial.println("Nashel");
    //читаем 10 групп по 4 бита данных и 1 бит чётности на каждую группу
    for(int i=0;i<10;i++)
    {
      validate = 0;
      for(int j=0;j<5;j++)
      { 
        
      //  Serial.print(ti);
        if (ttAComp(aver))
        {
          bitSet(buf[ones>>3], 7-(ones%8));
          validate+=1; //Считаем кол-во едениц для контроля четности
        } 
        else bitClear(buf[ones>>3], 7-(ones%8));
        ones++;
      }
    // Serial.print(" : ");
    // if(validate&1) return false; //не четно
    }
  // Serial.println("");



  //Наконец, есть 4 бита контрольной суммы и последний стоповый бит, который всегда равен нулю.
  for(int j=0;j<5;j++)
  {    
    if (ttAComp(aver))bitSet(buf[ones>>3], 7-ones%8);
    else bitClear(buf[ones>>3], 7-ones%8);
    ones++;
  }

  //добавляем 9 едениц в начало
  buf[0] = 255;
  bitSet(buf[1], 7);


  for (byte i = 0; i < 8; i++) {
      Serial.print(buf[i], BIN);
      Serial.print(" : ");
    }
  Serial.println("");

  for (byte i = 9; i < 64; i++) {
      Serial.print(bitRead(buf[i>>3], 7-i%8));
      if((i-8)%5==0) Serial.print(" : ");
    }
  Serial.println("");
  return vertEvenCheck(buf); // ну пока так
}


byte addr[8];
byte keyID[8];

void rfidACsetOn(int pwmpin){
  //включаем генератор 125кГц
  pinMode(pwmpin, OUTPUT);
  ledcAttach(pwmpin, 125000, 4);
  ledcWrite(pwmpin, 8); // 50% скважность (128 из 255)
}

  bool searchEM_Marine(bool copyKey = true) {
    // Serial.println("searchEM_Marine");
    //  byte gr = digitalRead(G_Led);
    bool rez = false;
    rfidACsetOn(FreqGen);  // включаем генератор 125кГц и компаратор
    delay(6);       //13 мс длятся переходные прцессы детектора
    readEM_Marie(addr);

    if (!readEM_Marie2(addr)) {
      if (!copyKey) digitalWrite(FreqGen, LOW);  // отключаем шим
//      digitalWrite(G_Led, gr);
      return rez;
    }
    rez = true;
    keyType = keyEM_Marine;
    for (byte i = 0; i < 8; i++) {
      if (copyKey) keyID[i] = addr[i];
      Serial.print(addr[i], HEX);
      Serial.print(":");
    }
    Serial.print(F(" ( id "));
    Serial.print(rfidData[0]);
    Serial.print(" key ");
    unsigned long keyNum = (unsigned long)rfidData[1] << 24 | (unsigned long)rfidData[2] << 16 | (unsigned long)rfidData[3] << 8 | (unsigned long)rfidData[4];
    Serial.print(keyNum);
    Serial.println(F(") Type: EM-Marie "));
    if (!copyKey) digitalWrite(FreqGen, LOW);  // отключаем шим
   // digitalWrite(G_Led, gr);
    return rez;
  }

  void TxBitRfid(byte data) {
    if (data & 1) delayMicroseconds(54 * 8);
    else delayMicroseconds(24 * 8);
    rfidGap(19 * 8);  //write gap
  }

  void TxByteRfid(byte data) {
    for (byte n_bit = 0; n_bit < 8; n_bit++) {
      TxBitRfid(data & 1);
      data = data >> 1;  // переходим к следующему bit
    }
  }

  void rfidGap(unsigned int tm) {
    digitalWrite(FreqGen, LOW);  // отключаем шим
    delayMicroseconds(tm);
    rfidACsetOn(FreqGen);
    // TCCR2A |= _BV(COM2A0);  // Включить ШИМ COM2A (pin 11)
  }

  bool T5557_blockRead(byte* buf) {
    byte ti;
    byte j = 0, k = 0;
    for (int i = 0; i < 33; i++) {  // читаем стартовый 0 и 32 значащих bit
      ti = ttAComp(2000);
      if (ti == 2) break;  //timeout
      if ((ti == 1) && (i == 0)) {
        ti = 2;
        break;
      }             // если не находим стартовый 0 - это ошибка
      if (i > 0) {  //начиная с 1-го бита пишем в буфер
        if (ti) bitSet(buf[(i - 1) >> 3], 7 - j);
        else bitClear(buf[(i - 1) >> 3], 7 - j);
        j++;
        if (j > 7) j = 0;
      }
    }
    if (ti == 2) return false;  //timeout
    return true;
  }

  bool sendOpT5557(byte opCode, unsigned long password = 0, byte lockBit = 0, unsigned long data = 0, byte blokAddr = 1) {
    TxBitRfid(opCode >> 1);
    TxBitRfid(opCode & 1);  // передаем код операции 10
    if (opCode == 0b00) return true;
    // password
    TxBitRfid(lockBit & 1);  // lockbit 0
    if (data != 0) {
      for (byte i = 0; i < 32; i++) {
        TxBitRfid((data >> (31 - i)) & 1);
      }
    }
    TxBitRfid(blokAddr >> 2);
    TxBitRfid(blokAddr >> 1);
    TxBitRfid(blokAddr & 1);  // адрес блока для записи
    delay(4);                 // ждем пока пишутся данные
    return true;
  }

  bool write2rfidT5557(byte* buf) {
    bool result;
    unsigned long data32;
    delay(6);
    for (byte k = 0; k < 2; k++) {  // send key data
      data32 = (unsigned long)buf[0 + (k << 2)] << 24 | (unsigned long)buf[1 + (k << 2)] << 16 | (unsigned long)buf[2 + (k << 2)] << 8 | (unsigned long)buf[3 + (k << 2)];
      rfidGap(30 * 8);                         //start gap
      sendOpT5557(0b10, 0, 0, data32, k + 1);  //передаем 32 бита ключа в blok k
      Serial.print('*');
      delay(6);
    }
    delay(6);
    rfidGap(30 * 8);  //start gap
    sendOpT5557(0b00);
    delay(4);
    result = readEM_Marie(addr);
    digitalWrite(FreqGen, LOW);  // отключаем шим

    for (byte i = 0; i < 8; i++)
      if (addr[i] != keyID[i]) {
        result = false;
        break;
      }
    if (!result) {
      Serial.println(F(" The key copy faild"));
      //OLED_printError(F("The key copy faild"));
      //Sd_ErrorBeep();
    } else {
      Serial.println(F(" The key has copied successesfully"));
      // OLED_printError(F("The key has copied"), false);
      //  Sd_ReadOK();
      delay(2000);
    }
    //digitalWrite(R_Led, HIGH);
    return result;
  }

  emRWType getRfidRWtype() {
    unsigned long data32, data33;
    byte buf[4] = { 0, 0, 0, 0 };
    rfidACsetOn(FreqGen);                  // включаем генератор 125кГц и компаратор
    delay(13);                      //13 мс длятся переходные процессы детектора
    rfidGap(30 * 8);                //start gap
    sendOpT5557(0b11, 0, 0, 0, 1);  //переходим в режим чтения Vendor ID
    if (!T5557_blockRead(buf)) return rwUnknown;
    data32 = (unsigned long)buf[0] << 24 | (unsigned long)buf[1] << 16 | (unsigned long)buf[2] << 8 | (unsigned long)buf[3];
    delay(4);
    rfidGap(20 * 8);                                                  //gap
    data33 = 0b00000000000101001000000001000000 | (rfidUsePWD << 4);  //конфиг регистр 0b00000000000101001000000001000000
    sendOpT5557(0b10, 0, 0, data33, 0);                               //передаем конфиг регистр
    delay(4);
    rfidGap(30 * 8);                //start gap
    sendOpT5557(0b11, 0, 0, 0, 1);  //переходим в режим чтения Vendor ID
    if (!T5557_blockRead(buf)) return rwUnknown;
    data33 = (unsigned long)buf[0] << 24 | (unsigned long)buf[1] << 16 | (unsigned long)buf[2] << 8 | (unsigned long)buf[3];
    sendOpT5557(0b00, 0, 0, 0, 0);  // send Reset
    delay(6);
    if (data32 != data33) return rwUnknown;
    Serial.print(F(" The rfid RW-key is T5557. Vendor ID is "));
    Serial.println(data32, HEX);
    return T5557;
  }


  bool write2rfid() {
    bool Check = true;
    if (searchEM_Marine(false)) {
      for (byte i = 0; i < 8; i++)
        if (addr[i] != keyID[i]) {
          Check = false;
          break;
        }           // сравниваем код для записи с тем, что уже записано в ключе.
      if (Check) {  // если коды совпадают, ничего писать не нужно
                    // digitalWrite(R_Led, LOW);
        Serial.println(F(" it is the same key. Writing in not needed."));
        // OLED_printError(F("It is the same key"));
        // Sd_ErrorBeep();
        //digitalWrite(R_Led, HIGH);
        delay(1000);
        return false;
      }
    }
    emRWType rwType = getRfidRWtype();  // определяем тип T5557 (T5577) или EM4305
    if (rwType != rwUnknown) Serial.print(F("\n Burning rfid ID: "));
    switch (rwType) {
      case T5557:
        return write2rfidT5557(keyID);
        break;  //пишем T5557
      //case EM4305: return write2rfidEM4305(keyID); break;                  //пишем EM4305
      case rwUnknown: break;
    }
    return false;
  }

  void Emulate(byte buf[8]) {
    //TCCR2A &=0b00111111; // отключаем шим
    digitalWrite(FreqGen, LOW);  // отключаем шим
    //FF:A9:8A:A4:87:78:98:6A
    delay(20);
    for (byte k = 0; k < 10; k++) {
      for (byte i = 0; i < 8; i++) {
        for (byte j = 0; j < 8; j++) {
          if (1 & (buf[i] >> (7 - j))) {
            pinMode(FreqGen, INPUT);
            delayMicroseconds(250);
            pinMode(FreqGen, OUTPUT);
            delayMicroseconds(250);
          } else {
            pinMode(FreqGen, OUTPUT);
            delayMicroseconds(250);
            pinMode(FreqGen, INPUT);
            delayMicroseconds(250);
          }
        }
      }
      // delay(1);
    }
  }
};


void AnalogTest()
{
  byte AcompInitState;
  uint32_t t0 = micros();
  
  for (size_t i = 0; i < 1000; i++) {
   AcompInitState = ((adc1_get_raw(ADC1_CHANNEL_0) - 600) > 200) ; 
  }
  uint32_t t1 = micros();
  Serial.print("1000 system_adc_read() calls took [us]: ");
  Serial.println(t1 - t0);
  //Serial.println( analogRead(2));

   t0 = micros();
  for (size_t i = 0; i < 1000; i++) {
   // analogReadFast0();
    adc1_get_raw(ADC1_CHANNEL_0);
    
  }
  t1 = micros();
  Serial.print("1000 analogRead() calls took [us]: ");
  Serial.println(t1 - t0);
  Serial.println(adc1_get_raw(ADC1_CHANNEL_0));
}

int aver;
EM_Marine EmMarine;
void setup() {
    //WiFi.mode(WIFI_OFF);
   // analogSetup();
    //analogReadResolution(9); 
    ///analogSetAtten(ADC_0db); // ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
    //analogSetWidth(10);
    //analogSetCycles(4);
    //analogSetClockp(10);
  //analogSetCycles(1);
   // analogSetClockDiv(10);

   // Отключите WiFi для уменьшения помех
  WiFi.mode(WIFI_OFF);
  
  // Увеличьте частоту CPU
 // setCpuFrequencyMhz(160);
  
  // Настройте АЦП
  //analogReadResolution(9); // 9 бит вместо 12
  adc1_config_width(ADC_WIDTH_BIT_12);
//adc1_config_width(0); 
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_6); // Укажите ваш канал
 adc1_config_channel_atten(ADC1_CHANNEL_4,ADC_ATTEN_DB_11);

  Serial.begin(921600);
  Serial.println("dddd");
  // put your setup code here, to run once:
  //pinMode(FreqGen, OUTPUT);
  
  Serial.println("dddd");
 EmMarine.rfidACsetOn(FreqGen);
  aver = EmMarine.calcAverageU();
  Serial.println(aver);

  AnalogTest();
  AnalogTest();
  AnalogTest();

  for(int i=0;i<200;i++)
{
  Serial.print(micros());
  Serial.print(' ');
  Serial.println(adc1_get_raw(ADC1_CHANNEL_0));
}

}



void loop() {
/*
for(int i=0;i<200;i++)
{
  a[i] = adc1_get_raw(ADC1_CHANNEL_0);
  Serial.print(micros());
  Serial.print(' ');
  Serial.println(a[i]);
}
delay(3000);*/
  // AnalogTest();
 //Serial.println(adc1_get_raw(ADC1_CHANNEL_0));
  // put your main code here, to run repeatedly:
  aver = EmMarine.calcAverageU();
  Serial.print("aver = ");
  Serial.println(aver);

  int period = EmMarine.PeriodCalc(aver);
  Serial.print("period = ");
  Serial.println(period);
  EmMarine.searchEM_Marine();


  //Serial.println(EmMarine.analogReadFast());
// int j = EmMarine.ttAComp(aver);
 // if(j!=2) Serial.println(j);

 // Serial.println(adc1_get_raw(ADC1_CHANNEL_0));
}
