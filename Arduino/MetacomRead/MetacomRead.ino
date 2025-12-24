const byte z = 73;   // размер массива
int t2[73];    // длительность низкий уровень тока или напряжение на Rcч >  400
int t3[73];    // длительность высокий уровень тока или напряжение на Rcч <  400
void setup()
{
	Serial.begin(9600);
	ADCSRA &= B11111000; // очистить три младших бита
	ADCSRA |= B00000100; // установить в них комбинацию 100 что дает делитель 16 и тактовую частоту АЦП 1МГц
}
byte halfT;
//29 10 52 1C    101001  10000  1010010  11100
int calcAverage() {
    unsigned long sum = 512;
    unsigned int preADCH = 0, j = 0;
    for (byte i = 0; i < 255; i++) {
      delayMicroseconds(10);
      sum += analogRead(A0);
    }
    sum = sum >> 8;
    unsigned long tSt = micros();
    for (byte i = 0; i < 255; i++) {
      delayMicroseconds(4);
      int A0 = analogRead(A0);
      if (((A0 > sum) && (preADCH < sum)) | ((A0 < sum) && (preADCH > sum))) {
        j++;
        preADCH = A0;
      }
    }
    halfT = (byte) ((micros() - tSt) / j);
    Serial.print("Aver=");
  Serial.print(sum);
  Serial.print("  Half=");
  Serial.println(halfT);
    return sum;
  }


void loop()
{
  
	while(analogRead(0)>1000) yield();; // ждем пока нет ключа
  int aver = calcAverage();
	delay(30);
	// читаем
	memset(t2,0,z); memset(t3,0,z); // обнуляем массив
	unsigned long t=0;
	
	for(int i=0; i<z; i++)
	{
		t= micros();
		//while(analogRead(0)>250);
    while(analogRead(0)>aver  );

		t2[i]= micros() - t; // длительность начало тайм слота  низкий уровень тока
		
		t= micros();
	//	while(analogRead(0)<250);
    while(analogRead(0)< aver  );
		t3[i]= micros() - t;	// длительность окончание тайм слота  высокий уровень тока
	}
	
// для отладки наглядность бля
	
	for(int i=0; i<z; i++)
	{
		//
		Serial.print("t2[");
		Serial.print(i);
		Serial.print("]= ");
		Serial.print(t2[i]);
		//
		Serial.print("          ");
		Serial.print("t3= ");
		Serial.print(t3[i]);
		Serial.print("     ");
		Serial.print("t2+t3= ");
		Serial.print(t3[i]+t2[i]);
		if(   t2[i]>    (t3[i]+t2[i])/2  && t3[i]<200 || t2[i] > 60 & t3[i]>200      )
		{
			Serial.println("  1");
		}
		else
		Serial.println("  0");
		//
	}
	Serial.println("konetc");


	byte Sinhro = 0; // ищем синхробит
	for(int i=0; i<z; i++)
	{
		if (t3[i]>200 )
		{
			if (i<37)
			{
				Sinhro =i+1;
        Serial.print("Sinhro=");
        Serial.println(Sinhro);
				break; // нашли синхро бит? выходим из цикла
			}
			
		}
	}
	
	
	byte Key[35];
	
	byte q = Sinhro+35 ;
	byte m=0;
	
	for (Sinhro ; Sinhro < q ; Sinhro++)
	{
		if  (  t2[Sinhro] >= (t3[Sinhro]+t2[Sinhro])/2  && t3[Sinhro]<200 || t2[Sinhro] > 60 & t3[Sinhro]>200 )
		Key[m]=1;
		else
		Key[m]=0;
		m++;
	}
	
  for(int i=0;i<35;i++)
  {
    Serial.print(Key[i]);
    if((i-2)%8==0 || i==2) Serial.print(" ");
  }
  Serial.println();

	bool er=1;
	for (int i=0; i<4; i++)
	{
		
		byte Valid = ( Key[3 +(i<<3)]	+ Key[4 + (i<<3)]	+ Key[5 + (i<<3)]	+ Key[6 + (i<<3)]	+ Key[7 + (i<<3)]
		+ Key[8 + (i<<3)]	+ Key[9 + (i<<3)]	) % 2;
		

    if ( Valid == Key[10 +(i*8)] )
			{
				Serial.print("byte N= ");
				Serial.print(i+1);
				Serial.println("valid ");	
			}
			else
			{
			Serial.print("byte N= ");
			Serial.print(i+1);
			Serial.println("ERROR ");	
			}
		
		
	}
	if (er || 1)
	{
		
		
		if ( Key[2]<<2  |  Key[1]<<1  | Key[0] == 2 )
		Serial.println("-METACOM-  K1233KT2");
		
		Serial.print("KEY HEX = ");
		for(int i=0; i < 4 ; i++)
		{
      byte b = 0;
      for(int j=0;j<8;j++)
      {       
        if (Key[3+j+i*8] == 1) {
            bitSet(b, 7-j);
          }       
      }
      Serial.print(b, HEX);
      Serial.print(" ");
			/*byte n= i<<3;
			//byte as = ( Key[9+n]<<6 | Key[8+n]<<5 | Key[7+n]<<4 | Key[6+n]<<3 | Key[5+n]<<2 | Key[4+n]<<1 | Key[3+n] );
			Serial.print( Key[10+n]<<6 | Key[9+n]<<6 | Key[8+n]<<5 | Key[7+n]<<4 | Key[6+n]<<3
			| Key[5+n]<<2 | Key[4+n]<<1 | Key[3+n]  , HEX);
			Serial.print(" ");*/
		}
	}

	Serial.println("  ");
	Serial.println("  ");
	delay(500);


	
}