/*  Mason Bee Capacitive Sensor

v0.1
- started with mason2016_v1_6 (final version used in 2016 Arduino Uno unit)
- added TCA9548 I2C Mux to select the AD7746 CDCs
v0.2
- added DS3231 Real Time Clock module 
v0.3
- added SD Card Module
v0.4
- added status LED
v0.5
- added auto CAPDAC adjustment
v0.6
- added ESP6266 cloud support to ThingSpeak
v1.0
- enhanced ThingSpeak support:  all cap data and bee count data is pushed to cloud DB
*/
#include <stdio.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <Fat16.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MCP9808.h>   // temperature sensor
#include <Adafruit_HTU21DF.h>   // humidity sensor
#include <Adafruit_BMP280.h>    // barometer pressure sensor
#include <Adafruit_TSL2591.h>   // light sensor
#include "AD7746.h"

#define ENV_SENSORS_PRESENT 1  // set to 1 when environmental sensors are present
#define CAP_SENSORS_PRESENT 1  // set to 1 when capacitive sensors are present
#define SDCARD_MODULE_PRESENT 1  // set to 1 when the SD card module is present
#define RTC_MODULE_PRESENT 1  // set to 1 when the DS3231 RTC module is present

#define NUMBER_OF_AD7746_DEVICES 4
#define NUMBER_OF_NESTING_TUBES (NUMBER_OF_AD7746_DEVICES * 2)
#define LOGGING_PERIOD_IN_SECS 5
#define THINGSPEAK_CAPDATA_PERIOD_IN_SECS 300
#define THINGSPEAK_COUNTDATA_PERIOD_IN_SECS 3600
#define NUM_INTERVALS_TO_RESET 3
#define CDC_WAIT_BEFORE_READING_IN_MS 300
#define I2C_EXPANDER_ADDR 0x70

// Wifi Information
#define SSID (F("==uid=="))
#define PASS (F("==pw=="))
#define IP (F("184.106.153.149")) // ThingSpeak IP Address: 184.106.153.149

// Software Serial 
#define SOFTWARE_SERIAL_RXPIN  2
#define SOFTWARE_SERIAL_TXPIN  3
#define FIRST_CHAR_TIMEOUT_IN_MS 200UL
#define INTERBYTE_TIMEOUT_IN_MS 10UL

#define DI_MAINTENANCE_MODE 5  // pin 5: low = bee block in maintenance mode, high = normal operation
#define D0_STATUS_LED 6  // pin 6: status LED

typedef struct
{
  float tempInDegC;
  float humidityInRh;
  float pressureInHpa;
  float lightInLux;
   
} envSensorReadings_t;

typedef struct 
{
  uint8_t cdcCapChannelSetup;    // Capacitive channel setup register
  uint8_t cdcExcChannelSetup;    // Capacitive channel excitation setup register
  
} cdcChannelConfig_t;

typedef enum
{
  thingspeakState_Idle = 0,
  thingspeakState_Cap1,
  thingspeakState_Cap2,
  thingspeakState_Cap3,
  thingspeakState_Cap4,
  thingspeakState_BeeCount1,
  thingspeakState_BeeCount2,
  thingspeakState_BeeCount3,
  thingspeakState_BeeCount4,
     
} thingspeakState_t;

const cdcChannelConfig_t cdcConfig[] = 
{
  {0xC0, 0x23},  // nesting tube #1
  {0x80, 0x0B},  // nesting tube #2
  {0xC0, 0x23},  // nesting tube #3
  {0x80, 0x0B},  // nesting tube #4
  {0xC0, 0x23},  // nesting tube #5
  {0x80, 0x0B},  // nesting tube #6
  {0xC0, 0x23},  // nesting tube #7
  {0x80, 0x0B},  // nesting tube #8 
};



const int SdCardModuleChipSelect = 10;  // for the SD  module, we use digital pin 10 for the SD module Chip Select (CS) line
const uint16_t EEPROMaddrForResetCount=0;
const uint16_t EEPROMaddrForCapdacConfig=2;  // non volatile storage for auto adjust CAPDAC config
const uint32_t CapUpperThreshold = 16000000;
const uint32_t CapLowerThreshold =  1000000;
const uint8_t CapdacIncrement = 0x10;
const uint8_t CapdacMaxValue = 0x7F;
const uint8_t CapdacMinValue = 0x00;

SdCard card;
Fat16 logfile;
Fat16 capdacConfigChangeFile;
Fat16 beeDetectConfigFile;
RTC_DS3231 rtc; // Real Time Clock 
SoftwareSerial espSerial( SOFTWARE_SERIAL_RXPIN, SOFTWARE_SERIAL_TXPIN ); // RX, TX 
uint32_t unixtime;
uint32_t timeOfLastRun = 0;
uint8_t wdtCount = NUM_INTERVALS_TO_RESET; // number of intervals before unit will force a reset (total time = number of intervals x 8 seconds)
uint32_t beeDetectThreshold = 10000;  // used to detect bee activity
int32_t capSensorReadings[NUMBER_OF_NESTING_TUBES];
uint32_t oldCapReadings[NUMBER_OF_NESTING_TUBES];
uint8_t cdcDacChannelSetup[NUMBER_OF_NESTING_TUBES];  // CAPDAC settings 
envSensorReadings_t envSensorReadings;
thingspeakState_t thingspeakState = thingspeakState_Idle;
boolean firstTimeStartup = true;
uint16_t beeCountThisHour[NUMBER_OF_NESTING_TUBES];
boolean pendingBeeCountPushToThingspeak = false;

#if ENV_SENSORS_PRESENT == 1
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
Adafruit_BMP280 bmp;
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); 
#else
uint16_t simulatedSensorData = 0;
#endif

void setup()
{
  // Watchdog timer setup.  
  noInterrupts();
  MCUSR  &= ~_BV(WDRF);
  WDTCSR  =  _BV(WDCE) | _BV(WDE);              // WDT change enable
  WDTCSR  =  _BV(WDIE) | _BV(WDP3) | _BV(WDP0); // Interrupt enable, 8 sec.
  interrupts();  
  
  Wire.begin(); // sets up i2c for operation
  Serial.begin(9600); // set up baud rate for serial
  espSerial.begin(9600);

  Serial.println();Serial.println();Serial.println(F("====== Starting Native Bee Monitoring Unit ======"));
  
  #if SDCARD_MODULE_PRESENT == 1
  initSdCard();
  readDacConfigFromEEPROM();
  readbeeDetectThresholdFromSdCard();
  #endif

  #if ENV_SENSORS_PRESENT == 1
  tempsensor.begin();
  htu.begin();
  bmp.begin();
  tsl.begin();
  // Configure the gain and integration time for the TSL2591 light sensor
  configureSensor();
  #endif
  
  pinMode(DI_MAINTENANCE_MODE, INPUT_PULLUP);   
  pinMode(D0_STATUS_LED, OUTPUT); 
  
  #if CAP_SENSORS_PRESENT == 1
  Serial.println(F("Initializing CDCs"));

  for (uint8_t i=0; i<NUMBER_OF_AD7746_DEVICES; i++)
  {
     cdcInit(i);
  }
  #endif 
    
  #if CAP_SENSORS_PRESENT == 0
  Serial.println(F("running in Capacitive sensor simulation mode"));
  #endif 
  
  #if ENV_SENSORS_PRESENT == 0
  Serial.println(F("running in Environmental sensor simulation mode"));
  #endif 

  #if SDCARD_MODULE_PRESENT == 0
  Serial.println(F("logging to SDCARD disabled"));
  #endif 

  #if RTC_MODULE_PRESENT == 0
  Serial.println(F("running in RTC simulation mode"));
  #endif 
 
  Serial.println(F("done setup()"));

  espReset();
  espSerial.println(F("ATE0")); // no echo
  delay(1000);
  readFromEsp();
  connectWiFi();

  clearBeeCounters();

  wdtCount = NUM_INTERVALS_TO_RESET;  // restore watchdog count
}

void loop()
{
  #if RTC_MODULE_PRESENT == 1
  DateTime timeNow;
  timeNow = rtc.now();
  unixtime = timeNow.unixtime();
  #else
  unixtime = millis() / 1000;
  #endif
  
  if (((unixtime % LOGGING_PERIOD_IN_SECS) == 0) && (timeOfLastRun != unixtime))
  {
    uint32_t startLoop = millis();
    Serial.print(F("unixtime = "));
    Serial.println(unixtime);    
    
    // -------------------------------------------- 
    //            Read Capacitive Sensors 
    // --------------------------------------------
    
    #if CAP_SENSORS_PRESENT == 1
    for (uint8_t nestingTubeNum = 0; nestingTubeNum < NUMBER_OF_NESTING_TUBES; nestingTubeNum++)
    {
      cdcEnable(nestingTubeNum);   
      delay(CDC_WAIT_BEFORE_READING_IN_MS);
      capSensorReadings[nestingTubeNum] = cdcRead(nestingTubeNum);
      cdcDisable(nestingTubeNum); 
    } 

    #else
    delay(2000);   
    #endif 

    detectBeeActivity();
 
    // -------------------------------------------- 
    //            Read Environmental Sensors 
    // --------------------------------------------
    
    #if ENV_SENSORS_PRESENT == 1
    // --- HTU21DF Humidity Sensor ---
    envSensorReadings.humidityInRh = htu.readHumidity();
     
    // --- MCP9808 Temperature Sensor ---
    envSensorReadings.tempInDegC = tempsensor.readTempC();  

    // --- BMP280 Pressure/Temperature Sensor ---
    envSensorReadings.pressureInHpa = bmp.readPressure()/100.0; // hPa

    // --- TSL2591 Light Sensor ---
    sensors_event_t lightEvent;
    tsl.getEvent(&lightEvent);
    envSensorReadings.lightInLux = lightEvent.light; // lux
    #else
    // substitute fake values when the sensors are not present
    envSensorReadings.tempInDegC = simulatedSensorData++;  
    envSensorReadings.humidityInRh = simulatedSensorData++;
    envSensorReadings.pressureInHpa = simulatedSensorData++; // hPa
    envSensorReadings.lightInLux = simulatedSensorData++; // lux
    
    Serial.println(F("delay for simulated env sensor read"));
    delay(100);  // simulates delay in reading all the environmental sensors
    #endif

    // -------------------------------------------- 
    //  Write data to SD Card 
    // --------------------------------------------
    
    #if SDCARD_MODULE_PRESENT == 1
    logBeeData(timeNow);
    #endif

    // -------------------------------------------- 
    //  Adjust CAPDAC config for any channels that are out of range 
    // --------------------------------------------

    capRangeAdjust(timeNow);

    Serial.print(F("Maintenance Mode = "));
    Serial.println(isMaintMode());
    printSensorData();
 
    // -------------------------------------------- 
    //  Push sensor and bee count data to Thingspeak cloud
    // --------------------------------------------
  
    if (((unixtime % THINGSPEAK_CAPDATA_PERIOD_IN_SECS) == 0) && (thingspeakState == thingspeakState_Idle))
    {
      thingspeakState = thingspeakState_Cap1;
    }

    if (((unixtime % THINGSPEAK_COUNTDATA_PERIOD_IN_SECS) == 0))
    {
      pendingBeeCountPushToThingspeak = true;
    }

    if (thingspeakState != thingspeakState_Idle)
    {
      sendDataToThingspeak();
    }

    flashLed(D0_STATUS_LED, 1, 50);
    timeOfLastRun = unixtime;  
    Serial.print(F("------ loop duration[ms] = "));   
    Serial.println(millis()-startLoop);

    wdtCount = NUM_INTERVALS_TO_RESET;  // restore watchdog count
  }
}

void cdcInit(uint8_t cdcIndex)
{
  selectCdcByChip(cdcIndex);
  Serial.print(F("\tInit CDC: "));
  Serial.println(cdcIndex);
  Wire.beginTransmission(AD7746_I2C_ADDRESS); // start i2c cycle
  Wire.write(AD7746_RESET_ADDRESS); // reset the AD7746 device
  Wire.endTransmission(); // ends i2c cycle
  
  delay(10);  // wait a bit for AD7746 to reboot
  
  AD7746_writeRegister(AD7746_REGISTER_CONFIGURATION, 0x39);  // 109.6 ms conversion time 
  AD7746_writeRegister(AD7746_REGISTER_CAP_OFFSET, 0x00);  
}

void cdcEnable(uint8_t tubeIndex)
{
  selectCdcByTube(tubeIndex);

  AD7746_writeRegister(AD7746_REGISTER_EXC_SETUP, cdcConfig[tubeIndex].cdcExcChannelSetup);  
  AD7746_writeRegister(AD7746_REGISTER_CAP_SETUP, cdcConfig[tubeIndex].cdcCapChannelSetup);
  
  AD7746_writeRegister(AD7746_REGISTER_CAP_DAC_A, _BV(7) | cdcDacChannelSetup[tubeIndex]);    
}

void cdcDisable(uint8_t tubeIndex)
{
  selectCdcByTube(tubeIndex);

  AD7746_writeRegister(AD7746_REGISTER_EXC_SETUP, 0x03);  // turn off excitation for both channels
  AD7746_writeRegister(AD7746_REGISTER_CAP_SETUP, 0x00); 
}

uint32_t cdcRead(uint8_t tubeIndex)
{
  selectCdcByTube(tubeIndex);
  return AD7746_readValue();
}

void selectCdcByTube(uint8_t tubeIndex)
{
  tcaSelect(NUMBER_OF_AD7746_DEVICES - tubeIndex/2 - 1);
  delay(10);
}

void selectCdcByChip(uint8_t chipIndex)
{
  tcaSelect(chipIndex);
  delay(10);  
}

uint8_t isMaintMode(void)
{
  if (digitalRead(DI_MAINTENANCE_MODE) == 1)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

#if SDCARD_MODULE_PRESENT == 1
void initSdCard(void)
{
  // initialize the SD card
  Serial.print(F("Initializing SD card..."));
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!card.begin(SdCardModuleChipSelect))
  {
    error("Card failed, or not present");
  }
  Serial.println(F("card initialized."));

  // initialize a FAT16 volume
  if (!Fat16::init(&card)) error("Fat16::init");
  
  //
  // create a new sensor data log file
  //
  char name[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    name[6] = i/10 + '0';
    name[7] = i%10 + '0';
    // O_CREAT - create the file if it does not exist
    // O_EXCL - fail if the file exists
    // O_WRITE - open for write only
    if (logfile.open(name, O_CREAT | O_EXCL | O_WRITE))break;
  }
  if (!logfile.isOpen()) error ("create");

  Serial.print(F("Logging to: "));
  Serial.println(name);

  logfile.println(F("datetime[unixtime],date,time,cap_ch1,cap_ch2,cap_ch3,cap_ch4,cap_ch5,cap_ch6,cap_ch7,cap_ch8,temp[degC],humidity[%RH],pressure[hPa],light[lux],maintMode"));
  
  //
  // create file to log changes to CAPDAC config
  //
  // O_CREAT - create the file if it does not exist
  // O_EXCL - fail if the file exists
  // O_WRITE - open for write only
  capdacConfigChangeFile.open("CAPDAC.CSV", O_CREAT | O_WRITE);
  
  if (!capdacConfigChangeFile.isOpen()) error ("create");

  Serial.print(F("Logging CAPDAC config changes to: "));
  Serial.println(F("CAPDAC.CSV"));

  //
  // connect to RTC
  //
  
  Wire.begin();
  if (!rtc.begin())
  {
    logfile.println(F("RTC failed"));
    Serial.println(F("RTC failed"));
  }
}

void readDacConfigFromEEPROM(void)
{
  for (uint8_t tubeIndex=0; tubeIndex<NUMBER_OF_NESTING_TUBES; tubeIndex++)
  {
    cdcDacChannelSetup[tubeIndex] = EEPROM.read(EEPROMaddrForCapdacConfig+tubeIndex);
    Serial.println(cdcDacChannelSetup[tubeIndex], HEX);
  }
}

void writeDacConfigToEEPROM(uint8_t tubeIndex, uint8_t capdac)
{
  EEPROM.write(EEPROMaddrForCapdacConfig+tubeIndex, capdac);
}

void capRangeAdjust(DateTime timeNow)
{
  for (uint8_t tubeIndex=0; tubeIndex<NUMBER_OF_NESTING_TUBES; tubeIndex++)
  {
    uint8_t capdacRoomAvailable;
    uint8_t capdacSettingPreAdjust = cdcDacChannelSetup[tubeIndex];

    // check if sensor is above the acceptable working range
    if (capSensorReadings[tubeIndex] > CapUpperThreshold)
    {
      // approaching max sensor range.  increase CAPDAC.  
      // first check for CAPDAC room available
      capdacRoomAvailable = CapdacMaxValue - cdcDacChannelSetup[tubeIndex];

      if (capdacRoomAvailable >= CapdacIncrement)
      {
        // enough room for a full increment
        cdcDacChannelSetup[tubeIndex] = cdcDacChannelSetup[tubeIndex] + CapdacIncrement;
      }
      else
      {
        // NOT enough room for a full increment, set to max value 
        cdcDacChannelSetup[tubeIndex] = CapdacMaxValue;
      }
    }
    // check if sensor is below the acceptable working range
    else if (capSensorReadings[tubeIndex] < CapLowerThreshold)
    {
      // approaching min sensor range.  decrease CAPDAC
      // first check for CAPDAC room available
      capdacRoomAvailable = cdcDacChannelSetup[tubeIndex] - CapdacMinValue;
      if (capdacRoomAvailable >= CapdacIncrement)
      {
        // adjust CAPDAC config
        cdcDacChannelSetup[tubeIndex] = cdcDacChannelSetup[tubeIndex] - CapdacIncrement;
      }
      else
      {
        // NOT enough room for a full increment, set to min value 
        cdcDacChannelSetup[tubeIndex] = CapdacMinValue;
      }
    } 

    // log any CAPDAC changes
    if (capdacSettingPreAdjust != cdcDacChannelSetup[tubeIndex])
    {
      writeDacConfigToEEPROM(tubeIndex, cdcDacChannelSetup[tubeIndex]);
      writeCapdacChangeToFile(timeNow, tubeIndex, cdcDacChannelSetup[tubeIndex]);
      Serial.print(F("Adjusting CAPDAC to 0x"));
      Serial.println(cdcDacChannelSetup[tubeIndex], HEX);
    }
  } // end for
}

void readbeeDetectThresholdFromSdCard(void)
{
  if (beeDetectConfigFile.open("BEEHOUSE.TXT", O_READ)) 
  {
    Serial.println(F("Opened BEEHOUSE.TXT"));
  } 
  else
  {
    Serial.println(F("file.open failed for BEEHOUSE.TXT"));
  }

  // threshold from the file
  char buf[12];  // up to 10 digits for DEC number, plus CR/LF
 
  beeDetectConfigFile.read(buf, sizeof(buf));
  sscanf (buf,"%lu",&beeDetectThreshold);
  Serial.print(F("beeDetectThreshold = "));
  Serial.println(beeDetectThreshold);
  
  beeDetectConfigFile.close();
}

void error(char *str)
{
  Serial.print(F("error: "));
  Serial.println(str);

  while (1);
}


void logBeeData(DateTime timeNow)
{
  logfile.print(timeNow.unixtime());
  logfile.print(F(", "));
  logfile.print(timeNow.year(), DEC);
  logfile.print(F("/"));
  logfile.print(timeNow.month(), DEC);
  logfile.print(F("/"));
  logfile.print(timeNow.day(), DEC);
  logfile.print(F(", "));
  logfile.print(timeNow.hour(), DEC);
  logfile.print(F(":"));
  logfile.print(timeNow.minute(), DEC);
  logfile.print(F(":"));
  logfile.print(timeNow.second(), DEC);
  logfile.print(F(", "));

  for (uint8_t nestingTubeNum = 0; nestingTubeNum < NUMBER_OF_NESTING_TUBES; nestingTubeNum++)
  {
    logfile.print(capSensorReadings[nestingTubeNum]);
    logfile.print(F(", ")); 
  } 

  logfile.print(envSensorReadings.tempInDegC);  
  logfile.print(F(", "));  
  logfile.print(envSensorReadings.humidityInRh);  
  logfile.print(F(", "));
  logfile.print(envSensorReadings.pressureInHpa);   
  logfile.print(F(", "));
  logfile.print(envSensorReadings.lightInLux);  

  logfile.print(F(", "));
  logfile.println(isMaintMode());  
    
  logfile.sync();
}

void writeCapdacChangeToFile(DateTime timeNow, uint8_t tubeIndex, uint8_t capdac)
{
  capdacConfigChangeFile.print(timeNow.unixtime());
  capdacConfigChangeFile.print(F(", "));
  capdacConfigChangeFile.print(timeNow.year(), DEC);
  capdacConfigChangeFile.print(F("/"));
  capdacConfigChangeFile.print(timeNow.month(), DEC);
  capdacConfigChangeFile.print(F("/"));
  capdacConfigChangeFile.print(timeNow.day(), DEC);
  capdacConfigChangeFile.print(F(", "));
  capdacConfigChangeFile.print(timeNow.hour(), DEC);
  capdacConfigChangeFile.print(F(":"));
  capdacConfigChangeFile.print(timeNow.minute(), DEC);
  capdacConfigChangeFile.print(F(":"));
  capdacConfigChangeFile.print(timeNow.second(), DEC);
  capdacConfigChangeFile.print(F(", "));
  
  capdacConfigChangeFile.print(tubeIndex);
  capdacConfigChangeFile.print(F(", ")); 
  capdacConfigChangeFile.println(capdac);  
    
  capdacConfigChangeFile.sync();
}

#endif  // end of #if SDCARD_MODULE_PRESENT == 1

void printSensorData(void)
{
  Serial.println(F("--------------"));

  for (uint8_t nestingTubeNum = 0; nestingTubeNum < NUMBER_OF_NESTING_TUBES; nestingTubeNum++)
  {
    Serial.print(F("0x"));
    Serial.println(capSensorReadings[nestingTubeNum], HEX);
  } 
  
  Serial.print(F("tempInDegC: "));
  Serial.println(envSensorReadings.tempInDegC);    
  Serial.print(F("humidity %RH: "));
  Serial.println(envSensorReadings.humidityInRh);      
  Serial.print(F("pressure hpa: "));
  Serial.println(envSensorReadings.pressureInHpa);        
  Serial.print(F("light lux: "));
  Serial.println(envSensorReadings.lightInLux);      

  Serial.print(F("maint mode: "));
  Serial.println(isMaintMode());    
}

void tcaSelect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(I2C_EXPANDER_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void flashLed(uint8_t ledIdent, uint8_t numberFlashes, uint8_t delayBetweenFlashesInMs)
{
  for (uint8_t i=0; i<numberFlashes; i++)
  {
    digitalWrite(ledIdent, HIGH);   
    delay(delayBetweenFlashesInMs);
    digitalWrite(ledIdent, LOW);   
    delay(delayBetweenFlashesInMs);
  }
}

void espReset(void)
{
  // reset
  digitalWrite(4, LOW);
  pinMode(4, OUTPUT);
  Serial.println(F("reset ESP8266"));
  delay(1000);
  digitalWrite(4, HIGH);
  delay(4000);
}  

boolean connectWiFi()
{
  espSerial.println(F("AT+CWMODE_CUR=1")); //WiFi STA mode 
  delay(1000);
  readFromEsp();
  
  //Connect to Router with AT+CWJAP="SSID","Password";
  // Check if connected with AT+CWJAP?
  String cmd=F("AT+CWJAP_CUR=\""); // Join access point
  cmd+=SSID;
  cmd+="\",\"";
  cmd+=PASS;
  cmd+="\"";
  espSerial.println(cmd);
  delay(5000);
  readFromEsp();

  espSerial.println(F("AT+CIPMUX=0")); // Set Single connection
  delay(1000);
  readFromEsp();
}

void readFromEsp(void)
{
  uint32_t timeout = millis() + FIRST_CHAR_TIMEOUT_IN_MS;
  while (timeout > millis())
  {
    if (espSerial.available() > 0)
    {
      Serial.write(espSerial.read());
      timeout = millis() + INTERBYTE_TIMEOUT_IN_MS;
     }
  }  
}

//
// Send data to Thingspeak cloud database
//
// use a state machine to incrementally interact with the ESP8266
// this approach makes sure that the cap reading and logging period is never overrun
//

void sendDataToThingspeak(void)
{
  switch (thingspeakState)
  {
    case thingspeakState_Cap1:
      connectToThingspeak();
      thingspeakState = thingspeakState_Cap2;
      break;
      
    case thingspeakState_Cap2:
      setCapDataLengthForThingspeak();
      thingspeakState = thingspeakState_Cap3;
      break;
      
    case thingspeakState_Cap3:
      sendCapDataToThingspeak();
      thingspeakState = thingspeakState_Cap4;        
      break;
      
    case thingspeakState_Cap4:
      receiveBufferedEspSerialData();
      if (pendingBeeCountPushToThingspeak == true)
      {
        thingspeakState = thingspeakState_BeeCount1;        
        pendingBeeCountPushToThingspeak = false; 
      }
      else
      {
        thingspeakState = thingspeakState_Idle;       
      }
      break;

    case thingspeakState_BeeCount1:
      connectToThingspeak();
      thingspeakState = thingspeakState_BeeCount2;
      break;
      
    case thingspeakState_BeeCount2:
      setCountDataLengthForThingspeak();
      thingspeakState = thingspeakState_BeeCount3;
      break;
      
    case thingspeakState_BeeCount3:
      sendCountDataToThingspeak();
      thingspeakState = thingspeakState_BeeCount4;        
      break;
      
    case thingspeakState_BeeCount4:
      receiveBufferedEspSerialData();
      thingspeakState = thingspeakState_Idle;       
      break;
    
    default:
      thingspeakState = thingspeakState_Idle;         
      break;
  }
}

void connectToThingspeak(void)
{
  // ESP8266 Client
  String cmd = F("AT+CIPSTART=\"TCP\",\""); // Setup TCP connection
  cmd += IP;
  cmd += F("\",80");
  espSerial.println(cmd);
}

void setCapDataLengthForThingspeak(void)
{  
  readFromEsp();
  
  String cmd = F("GET /update?key=*****************");

  for (uint8_t nestingTubeNum = 0; nestingTubeNum < NUMBER_OF_NESTING_TUBES; nestingTubeNum++)
  {
    uint8_t tn=nestingTubeNum+1;
    cmd+=F("&field");
    cmd+=tn;
    cmd+="=";
    cmd+=capSensorReadings[nestingTubeNum];
  } 

  cmd+=F("\r\n");
  espSerial.print(F("AT+CIPSEND="));
  espSerial.println(cmd.length());
}

void sendCapDataToThingspeak(void)
{ 
  readFromEsp();

  String cmd = F("GET /update?key=******************");

  for (uint8_t nestingTubeNum = 0; nestingTubeNum < NUMBER_OF_NESTING_TUBES; nestingTubeNum++)
  {
    uint8_t tn=nestingTubeNum+1;
    cmd+=F("&field");
    cmd+=tn;
    cmd+="=";
    cmd+=capSensorReadings[nestingTubeNum];
  } 

  cmd+=F("\r\n");
  espSerial.print(cmd);
}

void setCountDataLengthForThingspeak(void)
{  
  readFromEsp();
  
  String cmd = F("GET /update?key=*****************");

  for (uint8_t nestingTubeNum = 0; nestingTubeNum < NUMBER_OF_NESTING_TUBES; nestingTubeNum++)
  {
    uint8_t tn=nestingTubeNum+1;
    cmd+=F("&field");
    cmd+=tn;
    cmd+="=";
    cmd+=beeCountThisHour[nestingTubeNum];
  } 

  cmd+=F("\r\n");
  espSerial.print(F("AT+CIPSEND="));
  espSerial.println(cmd.length());
}

void sendCountDataToThingspeak(void)
{ 
  readFromEsp();

  String cmd = F("GET /update?key=4TC16GTY4LD30HLR");  

  for (uint8_t nestingTubeNum = 0; nestingTubeNum < NUMBER_OF_NESTING_TUBES; nestingTubeNum++)
  {
    uint8_t tn=nestingTubeNum+1;
    cmd+=F("&field");
    cmd+=tn;
    cmd+="=";
    cmd+=beeCountThisHour[nestingTubeNum];
  } 

  cmd+=F("\r\n");
  espSerial.print(cmd);
  clearBeeCounters();
}

void receiveBufferedEspSerialData(void)  
{
  readFromEsp();  
}  

void clearBeeCounters(void)
{
  for (uint8_t nestingTubeNum = 0; nestingTubeNum < NUMBER_OF_NESTING_TUBES; nestingTubeNum++)
  {
    beeCountThisHour[nestingTubeNum] = 0;
  }
}

void detectBeeActivity (void)
{
  if (firstTimeStartup == false)
  {
    // look for bees entering tube, indicated by an increase in capacitance
    for (uint8_t nestingTubeNum = 0; nestingTubeNum < NUMBER_OF_NESTING_TUBES; nestingTubeNum++)
    {
      if (capSensorReadings[nestingTubeNum]  > (oldCapReadings[nestingTubeNum] + beeDetectThreshold))
      {
        #if 1
        Serial.print(F("newCap["));
        Serial.print(nestingTubeNum);
        Serial.print(F("]:"));
        Serial.println(capSensorReadings[nestingTubeNum]);
        Serial.print(F("oldCap["));
        Serial.print(nestingTubeNum);
        Serial.print(F("]:"));
        Serial.println(oldCapReadings[nestingTubeNum]);
        #endif

        // only count bee activity in normal operation mode
        if (isMaintMode() == 0)
        {
          beeCountThisHour[nestingTubeNum]++;
        }
      }
    }
  }
  else
  {
    // need to wait until there are old cap readings before detecting bees entering or exiting
    #if 1
    Serial.println(F("First time start"));       
    #endif
    
    firstTimeStartup = false;
  }
    
  memcpy(&oldCapReadings, &capSensorReadings, sizeof(capSensorReadings));
}

//
// reset function designed specifically for Arduino Pro Mini boot sector
//
void(* resetFunc) (void) = 0; //declare reset function @ address 0

ISR(WDT_vect) 
{ 
  // Watchdog interrupt @ 8 sec. interval
  // Decrement watchdog counter...
  // If it reaches zero force a hw reset
  
  if(!--wdtCount) 
  { 
    uint8_t resetCount = EEPROM.read(EEPROMaddrForResetCount);
    resetCount++;
    EEPROM.write(EEPROMaddrForResetCount, resetCount);

    resetFunc();  //call reset
  }
}


