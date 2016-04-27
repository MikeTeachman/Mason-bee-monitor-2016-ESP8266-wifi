#include "AD7746.h"

long AD7746_readValue(void) 
{
  long ret = 0;
  char status = 0;

  status= AD7746_readRegister(AD7746_REGISTER_STATUS);

  // wait until a conversion is done
  while (!(status & _BV(0))) 
  {
    status= AD7746_readRegister(AD7746_REGISTER_STATUS);
  }

  unsigned long value =  AD7746_readLong(AD7746_REGISTER_CAP_DATA);

  // we have read one byte too much, now we have to get rid of it
  value >>=8;
  ret =  value;
  return ret;
}      


void AD7746_writeRegister(unsigned char r, unsigned char v)
{
  Wire.beginTransmission(AD7746_I2C_ADDRESS);
  Wire.write(r);
  Wire.write(v);
  Wire.endTransmission();
}

void AD7746_writeInteger(unsigned char r, unsigned int v) 
{
  AD7746_writeRegister(r,(unsigned byte)v);
  AD7746_writeRegister(r+1,(unsigned byte)(v>>8));
}

unsigned char AD7746_readRegister(unsigned char r)
{
  unsigned char v;
  Wire.beginTransmission(AD7746_I2C_ADDRESS);
  Wire.write(r);  // register to read
  Wire.endTransmission();

  Wire.requestFrom(AD7746_I2C_ADDRESS, 1); // read a byte
  while(Wire.available()==0) 
  {
    // waiting
  }
  v = Wire.read();
  return v;
}

void AD7746_readRegisters(unsigned char r, unsigned int numberOfBytes, unsigned char buffer[])
{
  unsigned char v;
  Wire.beginTransmission(AD7746_I2C_ADDRESS);
  Wire.write(r);  // register to read
  Wire.endTransmission();

  Wire.requestFrom(AD7746_I2C_ADDRESS, numberOfBytes); // read a byte
  char i = 0;
  while (i<numberOfBytes) 
  {
    while(!Wire.available()) 
    {
      // waiting
    }
    buffer[i] = Wire.read();
    i++;
  }
}

unsigned int AD7746_readInteger(unsigned char r) 
{
  union {
    char data[2];
    unsigned int value;
  } 
  byteMappedInt;

  byteMappedInt.value = 0;

  Wire.beginTransmission(AD7746_I2C_ADDRESS); // begin read cycle
  Wire.write(0); //pointer to first cap data register
  Wire.endTransmission(); // end cycle
  //after this, the address pointer is set to 0 - since a stop condition has been sent

  Wire.requestFrom(AD7746_I2C_ADDRESS,r+2); // reads 2 bytes plus all bytes before the register

  while (!Wire.available()==r+2) 
  {
    ; //wait
  }

  for (int i=r+1; i>=0; i--) 
  {
    uint8_t c = Wire.read();
    if (i<2) 
    {
      byteMappedInt.data[i]= c;
    }
  }

  return byteMappedInt.value;

}

unsigned long AD7746_readLong(unsigned char r) 
{
  union 
  {
    char data[4];
    unsigned long value;
  } 
  byteMappedLong;

  byteMappedLong.value = 0L;

  Wire.beginTransmission(AD7746_I2C_ADDRESS); // begin read cycle
  Wire.write(0); //pointer to first data register
  Wire.endTransmission(); // end cycle
  //the data pointer is reset anyway - so read from 0 on

  Wire.requestFrom(AD7746_I2C_ADDRESS,r+4); // reads 2 bytes plus all bytes before the register

  while (!Wire.available()==r+4) 
  {
    ; //wait
  }
  
  for (int i=r+3; i>=0; i--) 
  {
    uint8_t c = Wire.read();
    if (i<4) 
    {
      byteMappedLong.data[i]= c;
    }
  }

  return byteMappedLong.value;
}

void AD7746_displayStatus(void) 
{
  unsigned char data[18];
  AD7746_readRegisters(0,18,data);
  
  Serial.println(F("\nAD7746 Registers:"));
  Serial.print(F("Status (0x0): "));
  Serial.println(data[0],BIN);
  Serial.print(F("Cap Data (0x1-0x3): "));
  Serial.print(data[1],BIN);
  Serial.print(F("."));
  Serial.print(data[2],BIN);
  Serial.print(F("."));
  Serial.println(data[3],BIN);
  Serial.print(F("VT Data (0x4-0x6): "));
  Serial.print(data[4],BIN);
  Serial.print(F("."));
  Serial.print(data[5],BIN);
  Serial.print(F("."));
  Serial.println(data[6],BIN);
  Serial.print(F("Cap Setup (0x7): "));
  Serial.println(data[7],BIN);
  Serial.print(F("VT Setup (0x8): "));
  Serial.println(data[8],BIN);
  Serial.print(F("EXC Setup (0x9): "));
  Serial.println(data[9],BIN);
  Serial.print(F("Configuration (0xa): "));
  Serial.println(data[10],BIN);
  Serial.print(F("Cap Dac A (0xb): "));
  Serial.println(data[11],BIN);
  Serial.print(F("Cap Dac B (0xc): "));
  Serial.println(data[12],BIN);
  Serial.print(F("Cap Offset (0xd-0xe): "));
  Serial.print(data[13],BIN);
  Serial.print(F("."));
  Serial.println(data[14],BIN);
  Serial.print(F("Cap Gain (0xf-0x10): "));
  Serial.print(data[15],BIN);
  Serial.print(F("."));
  Serial.println(data[16],BIN);
  Serial.print(F("Volt Gain (0x11-0x12): "));
  Serial.print(data[17],BIN);
  Serial.print(F("."));
  Serial.println(data[18],BIN);
}

