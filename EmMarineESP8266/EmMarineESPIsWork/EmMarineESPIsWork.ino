// ОБЯЗАТЕЛЬНО ПОСТАВИТЬ ЧАСТОТУ 160MHz!!!! 
#include "user_interface.h"




#define FreqGen D4

//settings
#define rfidUsePWD 0    // ключ использует пароль для изменения
#define rfidPWD 123456  // пароль для ключа
#define rfidBitRate 2   // Скорость обмена с rfid в kbps
class EM_Marine {
public:
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

  uint16_t analogReadFast()  //pin просто для совместимости
  {
    uint16_t res;                       // значения которые считываются ацп, может быть [1, 65535]
    system_adc_read_fast(&res, 1, 10);  //костыльное решение проблемы со временем, но работает за 17мкс, а не 80 (значения, колво, adc_clk_div)
    return res;
  }

  int calcAverageU() {
    unsigned long sum = 60;
    for (byte i = 0; i < 255; i++) {
      delayMicroseconds(50);
      sum += analogReadFast();
    }
    sum = sum >> 8;
    return sum;
  }



  byte BitRead2(int aver = 200, int period  = 230, unsigned long timeOut = 600 ) {  // pulse 0 or 1 or -1 if timeout
    aver+=30;
    bool AcompState, AcompInitState;
    unsigned long tEnd = micros() + timeOut;
 
    AcompInitState = (analogReadFast() > aver)  ; // читаем флаг компаратора
    do {
      AcompState = (analogReadFast() > aver);  // читаем флаг компаратора
      if (AcompState != AcompInitState) {
        tEnd = micros() + period; // (230)256 - 23 - 10
        delayMicroseconds(1000 / (rfidBitRate * 6));    // 1/4 Period on 2 kBps = 125 mks
       // delayMicroseconds(10); //128
        AcompState = (analogReadFast() > aver) ;         // читаем флаг компаратора
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



//aver - среднее значение на порту A0
byte BitRead(int aver = 500, unsigned long timeOut = 7000) {  // pulse 0 or 1 or -1 if timeout
    byte AcompState, AcompInitState;
    unsigned long tEnd = micros() + timeOut;
    AcompInitState = (analogReadFast() > aver);  // читаем флаг компаратора
    do {
      AcompState = (analogReadFast() > aver);  // читаем флаг компаратора
      if (AcompState != AcompInitState) {
        //delayMicroseconds(1000 / (rfidBitRate * 4));    // 1/4 Period on 2 kBps = 125 mks

        delayMicroseconds(10);                   // 1/4 Period on 2 kBps = 125 mks
        AcompState = (analogReadFast() > aver);  // читаем флаг компаратора
        //delayMicroseconds(1000 / (rfidBitRate * 2));  // 1/2 Period on 2 kBps = 250 mks

        delayMicroseconds(200);  // 1/2 Period on 2 kBps = 250 mks
        return AcompState;
      }
    } while (micros() < tEnd);
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
    unsigned long tEnd = millis() + 50;
    byte ti;
    //ждем 9 адяниц
    
    byte validate;
    byte ones = 0;

      while(millis() < tEnd)
      {
          ti = BitRead(aver) ;
          if (ti == 1) ones++;
          else ones = 0;
          if(ones == 9) break;
      }
      if(ones != 9)  return false; //не нашли

    //читаем 10 групп по 4 бита данных и 1 бит чётности на каждую группу
    for(int i=0;i<10;i++)
    {
      validate = 0;
      for(int j=0;j<5;j++)
      { 
        if (BitRead(aver))
        {
          bitSet(buf[ones>>3], 7-(ones%8));
          validate+=1; //Считаем кол-во едениц для контроля четности
        } 
        else bitClear(buf[ones>>3], 7-(ones%8));
        ones++;
      }
    // if(validate&1) return false; //не четно
    }

  //Наконец, есть 4 бита контрольной суммы и последний стоповый бит, который всегда равен нулю.
  for(int j=0;j<5;j++)
  {    
    if (BitRead(aver))bitSet(buf[ones>>3], 7-ones%8);
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


//Оригинальная функция чтения от Alex Malow
  bool readEM_Marie2(byte* buf) {
    int aver = calcAverageU();
   /* Serial.print("Aver = ");
    Serial.println(aver);*/
    unsigned long tEnd = millis() + 100;
    byte ti;
    byte j = 0, k = 0;
 //   Serial.print("Ti = ");
    for (int i = 0; i < 64; i++) {  // читаем 64 bit
      ti = BitRead(aver - 10);

    //  Serial.print(ti);
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
    /*for (int i = 0; i < 8; i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }*/
    Serial.println();
    Serial.println("//////////EM1////////////");

            for (byte i = 9; i < 64; i++) {
                Serial.print(bitRead(buf[i>>3], 7-i%8));
                if((i-8)%5==0) Serial.print(" : ");
              }
            Serial.println("");

    if (ti == 2) return false;  //timeout
    return vertEvenCheck(buf);
  }


  byte addr[8];
  byte keyID[8];

  void rfidACsetOn() {
    //включаем генератор 125кГц
    // pinMode(FreqGen, OUTPUT);
    pinMode(FreqGen, OUTPUT);  //Инициализируйте pin как выходной
    // analogWriteRange(qap);
    analogWriteResolution(4);
    analogWriteFreq(125000);
    analogWrite(FreqGen, 10);
  }

  bool searchEM_Marine(bool copyKey = true) {
    Serial.println("searchEM_Marine");
    //  byte gr = digitalRead(G_Led);
    bool rez = false;
    rfidACsetOn();  // включаем генератор 125кГц и компаратор
    delay(6);       //13 мс длятся переходные прцессы детектора
    if (!readEM_Marie(addr)) {
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
            digitalWrite(FreqGen, LOW);  // отключаем шим
            delayMicroseconds(250);
          } else {
            pinMode(FreqGen, OUTPUT);
            digitalWrite(FreqGen, LOW);  // отключаем шим
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

int aver;
EM_Marine EmMarine;
void setup() {
  // put your setup code here, to run once:
  pinMode(FreqGen, OUTPUT);
  Serial.begin(921600);
  Serial.println("dddd");
  EmMarine.rfidACsetOn();
  aver = EmMarine.calcAverageU();
  Serial.println(aver);
}

void loop() {
  // put your main code here, to run repeatedly:
   EmMarine.searchEM_Marine();
  //Serial.println(EmMarine.analogReadFast());
// int j = EmMarine.ttAComp(aver);
 // if(j!=2) Serial.println(j);
}
