#include <OneWire.h>
#include <GyverOLED.h>
#include <EncButton.h>
#include <EEPROM.h>
#include "OneWireHub.h"
#include "DS2401.h"  // Serial Number

#define iButtonPort 2

//#define emul d2
//#define read A1


//byte key[8] = {0x01, 0x6A, 0xC6, 0x21, 0xB4, 0x00, 0x00, 0x52}; // 52 0 0 B4 21 C6 6A 1

//{0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x2F}; //metakom
//{0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x2F}; //metakom
//{0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3D}; //cyfral
//{0x01, 0xBE, 0x40, 0x11, 0x5A, 0x36, 0x00, 0xE1}; //visit
// {0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x9B}; //metakom 2003



constexpr uint8_t pin_led{ 13 };
constexpr uint8_t pin_onewire{ 2 };

auto hub = OneWireHub(pin_onewire);
auto ds1990A = DS2401(0x01, 0x6A, 0xC6, 0x21, 0xB4, 0x00, 0x00);  //{ 0x01, 0x6A, 0xC6, 0x21, 0xB4, 0x00, 0x00, 0x52 };

OneWire ibutton(iButtonPort);
GyverOLED<SSH1106_128x64> oled;
EncButton eb(5, 4, 3);

///////////ЭКРАН////////////
void PrintMessageDisplay(String message, int pos = 2) {
  oled.setCursor(0, pos);
  oled.print(message);
  oled.update();
}
void ClearDisplay() {
  oled.clear();  // очистка
  oled.home();   // курсор в 0,0
  oled.print("MultyKey 1.0.0");
}

void PrintEEPROMDisplay(int EEPROMpos = 0, int displayPos = 3)  //Печать из еепрома на экран (память экономим)
{
  ClearDisplay();
  int ind = 0;
  for (int i = 0 + EEPROMpos; i < 8 + EEPROMpos; i++) {  // Запускаем цикл печати данных из массива
    Serial.print(EEPROM[i], HEX);                        // Печатаем нужный байт в шестнадцатиричном виде
    Serial.print(" ");
    oled.setCursor(ind, 3);
    oled.print(String(EEPROM[i], HEX));
    ind += String(EEPROM[i], HEX).length() * 8;
    Serial.println(ind);
  }
  Serial.println();
  oled.update();
}

void ShowMenu(String Menu[], int len, int select) {
  for (int i = 0; i < len; i++) {
    String message = "";
    if (i == select) message = '>' + Menu[i];
    else message = ' ' + Menu[i];
    oled.setCursor(0, i + 2);
    oled.print(message);
  }
  oled.update();
}

int selectMenu = 0;
int MenuHandler(String Menu[], int len) {
  ClearDisplay();
  ShowMenu(Menu, len, selectMenu);
  while (true) {
    //Serial.println("menu");
    //Serial.println(selectMenu);
    if (eb.tick()) {
      if (eb.left()) selectMenu -= 1;
      if (eb.right()) selectMenu += 1;
      if (selectMenu < 0) selectMenu = len - 1;
      if (selectMenu >= len) selectMenu = 0;

      ShowMenu(Menu, len, selectMenu);
      if (eb.press()) return selectMenu;
    }

    //delay(10);
  }
}



void setup() {

  //  dsSlave.debug = true;
  // dsSlave.fullCommand = false;

  oled.init();  // инициализация
  Serial.begin(115200);
  oled.clear();  // очистка
  oled.home();   // курсор в 0,0
  oled.print("MultyKey 1.0.0");
}

void loop() {

  String mainMenu[3] = { "Эмулировать", "Считать", "Записать" };
  switch (MenuHandler(mainMenu, 3)) {
    case 0:
    //  emulateKey();
      break;
    case 1:
      ReadingKeyMenu();
      break;
    case 2:
       WritingKeyMenu();
      break;
    default:
      // statements
      break;
  }
}


void ReadingKeyMenu() {
  ClearDisplay();
  while (true)  // Чтение происходит на протяжении 5 секунд
  {
    PrintMessageDisplay("Reading....", 1);
    if (readKey()) {
      PrintEEPROMDisplay();
    }


    eb.tick();
    if (eb.press()) return;
  }
}

void WritingKeyMenu() {
  ClearDisplay();
  PrintMessageDisplay("Writing....", 1);
  PrintEEPROMDisplay();
  while (true)  // Чтение происходит на протяжении 5 секунд
  {
    WriteKey();
    eb.tick();
    if (eb.press()) return;
  }
}

////////ЭМУЛЯЦИЯ///////


void emulateKey() {

  ClearDisplay();
  Serial.println("Emulating.....");
  PrintMessageDisplay("Emulating.....");
  //  dsSlave.setRom(key);
  hub.attach(ds1990A);  // always online
  delay(10);
  while (true)  // Эмуляция выполняется на протяжении 7 секунд
  {
    // dsSlave.waitForRequest(false);  // Сама эмуляция
    hub.poll();
    // dsSlave.sendData(key,8);
    // Serial.print("Error - "); Serial.println(dsSlave.errno);
    // Serial.print("ErrorLib - "); Serial.println(dsSlave.errnoLib);
    // PrintMessageDisplay("Error - " + String(dsSlave.errno), 3);
    // PrintMessageDisplay("ErrorLib - " + String(dsSlave.errnoLib), 4);
    eb.tick();
    if (eb.press()) {
      hub.detach(ds1990A);
      return;
    }
  }
}


///ЧТЕНИЕ И ЗАПИСЬ
void WriteKey() {
  
  // send reset
  ibutton.skip();
  ibutton.reset();
  // send 0xD1
  ibutton.write(0xD1);
  // send logical 0
  digitalWrite(iButtonPort, LOW);
  pinMode(iButtonPort, OUTPUT);
  delayMicroseconds(60);
  pinMode(iButtonPort, INPUT);
  digitalWrite(iButtonPort, HIGH);
  delay(10);

  // Hardcode here your desired ID //
  // 01 D5 9F DC 02 00 00 96
  ibutton.skip();
  ibutton.reset();
  ibutton.write(0xD5);
  for (byte x = 0; x < 8; x++) {
    writeByte(EEPROM[x]);
    Serial.print('*');
  }
  Serial.print('\n');
  ibutton.reset();
  // send 0xD1
  ibutton.write(0xD1);
  //send logical 1
  digitalWrite(iButtonPort, LOW);
  pinMode(iButtonPort, OUTPUT);
  delayMicroseconds(10);
  pinMode(iButtonPort, INPUT);
  digitalWrite(iButtonPort, HIGH);
  delay(10);
}

int writeByte(byte data) {
  int data_bit;
  for (data_bit = 0; data_bit < 8; data_bit++) {
    if (data & 1) {
      digitalWrite(iButtonPort, LOW);
      pinMode(iButtonPort, OUTPUT);
      delayMicroseconds(60);
      pinMode(iButtonPort, INPUT);
      digitalWrite(iButtonPort, HIGH);
      delay(10);
    } else {
      digitalWrite(iButtonPort, LOW);
      pinMode(iButtonPort, OUTPUT);
      pinMode(iButtonPort, INPUT);
      digitalWrite(iButtonPort, HIGH);
      delay(10);
    }
    data = data >> 1;
  }
  return 0;
}

bool readKey() {
  byte key[8];
  if (ibutton.search(key)) {  // Если устройство подключено - считываем
    for (int i = 0; i < 8; i++) {
      EEPROM[i] = key[i];
    }
    ibutton.reset_search();  // Сбрасываем устройство
    return true;
  }
  return false;
}
