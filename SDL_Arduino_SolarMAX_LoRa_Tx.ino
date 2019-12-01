// SDL_Arduino_SolarMAX_LoRa_Tx
// SwitchDoc Labs November 2019
//
#define TXDEBUG
//#undef TXDEBUG
#include <JeeLib.h>

#include "MemoryFree.h"

#define LED 13

#define SOFTWAREVERSION 5

// WIRELESSID is changed if you have more than one unit reporting in the same area.  It is coded in protocol as WIRELESSID*10+SOFTWAREVERSION
// WIRELESSID of 8 is SolarMAX LiPo   BatV < 8V
// WIRELESSID of 10 is SolarMAX LeadAcid   BatV > 8V
// #define WIRELESSID 8
int WirelessID = 8;

// Number of milliseconds between data out - set 1000 or or 30000 or 60000 if you are using DS3231
//#define SLEEPCYCLE 30000

// WIRELESSID is changed if you have more than one unit reporting in the same area.  It is coded in protocol as WIRELESSID*10+SOFTWAREVERSION
//#define WIRELESSID 2
// Number of milliseconds between data out - set 1000 or or 30000 or 60000 if you are using DS3231
#define SLEEPCYCLE 14500
//#define SLEEPCYCLE 14000

#include "Crc16.h"

//Crc 16 library (XModem)
Crc16 crc;

ISR(WDT_vect) {
  Sleepy::watchdogEvent();
}
#include <SoftwareSerial.h>
#include <RH_RF95.h>

#include <avr/sleep.h>
#include <avr/power.h>
#include "SDL_Arduino_INA3221.h"
#include "XAdafruit_INA219.h"
#include "XClosedCube_HDC1080.h";

SDL_Arduino_INA3221 INA3221;

XAdafruit_INA219 INA219;  // for USB PowerCentral

// the three channels of the INA3221 named for INA3221 Solar Power Controller channels (www.switchdoc.com)
#define LIPO_BATTERY_CHANNEL 1
#define SOLAR_CELL_CHANNEL 2
#define OUTPUT_CHANNEL 3

#define ENABLE_RADIO 5


XClosedCube_HDC1080 hdc1080;

#define WATCHDOG_1 8
#define WATCHDOG_2 9

// Number of milliseconds between data out - set 1000 or or 30000 or 60000 if you are using DS3231


// Note:  If you are using a External WatchDog Timer, then you should set the timer to exceed SLEEPCYCLE by at least 10%.  If you are using the SwitchDoc Labs Dual WatchDog, SLEEPCYCLE must be less than 240 seconds
//        or what you set the WatchDog timeout interval.

// Note:  If you are using the alarm DS3231 system, you only have the choice of every second or every minute, but you can change this to some degree:


/*
  Values for Alarm 1

  ALM1_EVERY_SECOND -- causes an alarm once per second.
  ALM1_MATCH_SECONDS -- causes an alarm when the seconds match (i.e. once per minute).
  ALM1_MATCH_MINUTES -- causes an alarm when the minutes and seconds match.
  ALM1_MATCH_HOURS -- causes an alarm when the hours and minutes and seconds match.
  ALM1_MATCH_DATE -- causes an alarm when the date of the month and hours and minutes and seconds match.
  ALM1_MATCH_DAY -- causes an alarm when the day of the week and hours and minutes and seconds match.
  Values for Alarm 2

  ALM2_EVERY_MINUTE -- causes an alarm once per minute.
  ALM2_MATCH_MINUTES -- causes an alarm when the minutes match (i.e. once per hour).
  ALM2_MATCH_HOURS -- causes an alarm when the hours and minutes match.
  ALM2_MATCH_DATE -- causes an alarm when the date of the month and hours and minutes match.
  ALM2_MATCH_DAY -- causes an alarm when the day of the week and hours and minutes match.

*/
#include <SoftwareSerial.h>

SoftwareSerial SSerial(6, 7); // TX, RX



#define COMSerial SSerial
#define ShowSerial Serial

RH_RF95<SoftwareSerial> rf95(COMSerial);

unsigned long MessageCount = 0;

unsigned long badAM2315Reads = 0;






#include "avr/pgmspace.h"
#include <Time.h>
#include <TimeLib.h>
#include "DS3232RTC.h"

#include <Wire.h>

// Note:   We found a long term reliablity problem with the AM2315 (took weeks to manifest), so we went to our High Reliablity AM2315 and modified it to work on the Mini Pro LP.
#include "SDL_ESP8266_HR_AM2315.h"

typedef enum  {

  NO_INTERRUPT,
  IGNORE_INTERRUPT,
  SLEEP_INTERRUPT,
  RAIN_INTERRUPT,
  ANEMOMETER_INTERRUPT,
  ALARM_INTERRUPT,
  REBOOT
} wakestate;


// Device Present State Variables

bool INA3221_Present;
bool DS3231_Present;
bool USBPowerCentral_Present;
bool HDC1080_Present;

byte byteBuffer[200]; // contains string to be sent to RX unit

// State Variables
byte Protocol;
long TimeStamp;

float InsideTemperature;
float InsideHumidity;
float BatteryVoltage;
float BatteryCurrent;
float LoadVoltage;
float LoadCurrent;
float SolarPanelVoltage;
float SolarPanelCurrent;
float AuxA;
float AuxB;
int protocolBufferCount;



wakestate wakeState;  // who woke us up?

bool Alarm_State_1;
bool Alarm_State_2;

long nextSleepLength;





int convert4ByteLongVariables(int bufferCount, long myVariable)
{

  int i;

  union {
    long a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++)
  {
    byteBuffer[bufferCount] = thing.bytes[i];
    bufferCount++;
  }
  return bufferCount;

}

int convert4ByteFloatVariables(int bufferCount, float myVariable)
{
  int i;

  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++)
  {
    byteBuffer[bufferCount] = thing.bytes[i];


    bufferCount++;
  }

  return bufferCount;
}


int convert2ByteVariables(int bufferCount, int myVariable)
{


  union {
    int a;
    unsigned char bytes[2];
  } thing;

  thing.a = myVariable;


  byteBuffer[bufferCount] = thing.bytes[0];
  bufferCount++;
  byteBuffer[bufferCount] = thing.bytes[1];
  bufferCount++;
#if defined(TXDEBUG)
  Serial.println(F("-------"));
  Serial.println(thing.bytes[0]);
  Serial.println(thing.bytes[1]);
  Serial.println(F("------"));
#endif
  return bufferCount;

}

int convert1ByteVariables(int bufferCount, int myVariable)
{


  byteBuffer[bufferCount] = (byte) myVariable;
  bufferCount++;
  return bufferCount;

}

int checkSum(int bufferCount)
{
  unsigned short checksumValue;
  // calculate checksum
  checksumValue = crc.XModemCrc(byteBuffer, 0, 59);
#if defined(TXDEBUG)
  Serial.print(F("crc = 0x"));
  Serial.println(checksumValue, HEX);
#endif

  byteBuffer[bufferCount] = checksumValue >> 8;
  bufferCount++;
  byteBuffer[bufferCount] = checksumValue & 0xFF;
  bufferCount++;

  return bufferCount;
}

int buildProtocolString()
{

  int bufferCount;


  bufferCount = 0;

  byteBuffer[bufferCount] = 0xAB;
  bufferCount++;
  byteBuffer[bufferCount] = 0x66;
  bufferCount++;
  bufferCount = convert1ByteVariables(bufferCount, Protocol);
  bufferCount = convert4ByteLongVariables(bufferCount, TimeStamp / 100);
  bufferCount = convert2ByteVariables(bufferCount, 0);
  bufferCount = convert4ByteFloatVariables(bufferCount, 0.0);
  bufferCount = convert4ByteLongVariables(bufferCount, 0);
  bufferCount = convert4ByteFloatVariables(bufferCount, 0.0);
  bufferCount = convert4ByteFloatVariables(bufferCount, LoadVoltage);


  bufferCount = convert4ByteFloatVariables(bufferCount, InsideTemperature);
  bufferCount = convert4ByteFloatVariables(bufferCount, InsideHumidity);

  bufferCount = convert4ByteFloatVariables(bufferCount, BatteryVoltage);
  bufferCount = convert4ByteFloatVariables(bufferCount, BatteryCurrent);
  bufferCount = convert4ByteFloatVariables(bufferCount, LoadCurrent);
  bufferCount = convert4ByteFloatVariables(bufferCount, SolarPanelVoltage);
  bufferCount = convert4ByteFloatVariables(bufferCount, SolarPanelCurrent);

  bufferCount = convert4ByteFloatVariables(bufferCount, AuxA);
  bufferCount = convert4ByteLongVariables(bufferCount, MessageCount);
  protocolBufferCount = bufferCount + 2;
  //     bufferCount = convert1ByteVariables(bufferCount, protocolBufferCount);
  bufferCount = checkSum(bufferCount);




  return bufferCount;


}



void printStringBuffer()
{
  int bufferLength;

  bufferLength = protocolBufferCount;
  int i;
  for (i = 0; i < bufferLength; i++)
  {
    Serial.print(F("i="));
    Serial.print(i);
    Serial.print(F(" | "));
    Serial.println(byteBuffer[i], HEX);
  }

}


// DS3231 Library functions



const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

void printDigits(int digits) {
  // utility function for digital clock display: prints an leading 0

  if (digits < 10)
    Serial.print(F("0"));
  Serial.print(digits);
}

void digitalClockDisplay() {

  tmElements_t tm;
  RTC.read(tm);

  // digital clock display of the time
  printDigits(tm.Hour);
  Serial.print(F(":"));
  printDigits(tm.Minute);
  Serial.print(F(":"));
  printDigits(tm.Second);
  Serial.print(F(" "));
  Serial.print(tm.Day);
  Serial.print(F("/"));
  Serial.print(tm.Month);
  Serial.print(F("/"));
  Serial.print(tm.Year + 1970);
  Serial.println();
}

void return2Digits(char returnString[], char *buffer2, int digits)
{
  if (digits < 10)
    sprintf(returnString, "0%i", digits);
  else
    sprintf(returnString, "%i", digits);

  strcpy(returnString, buffer2);


}



void ResetWatchdog()
{


  digitalWrite(WATCHDOG_1, LOW);
  delay(200);
  digitalWrite(WATCHDOG_1, HIGH);

#if defined(TXDEBUG)
  Serial.println(F("Watchdog1 Reset - Patted the Dog"));
#endif

}


#define ENABLE 4
#define CONTROL 3

bool statusUSBPowerCentral = false;

void setupUSBPowerCentral()
{
  digitalWrite(CONTROL, LOW);
  digitalWrite(ENABLE, HIGH);
  pinMode(CONTROL, OUTPUT);
  pinMode(ENABLE, OUTPUT);

}

//USB PowerCentral turn on and off

void USBPowerCentralOff()
{
  digitalWrite(CONTROL, LOW);
  digitalWrite(ENABLE, HIGH);
  statusUSBPowerCentral = false;
#if defined(TXDEBUG)
  Serial.println(F("USB PowerCentral OFF"));
#endif

}

void USBPowerCentralOn()
{

  digitalWrite(CONTROL, HIGH);
  digitalWrite(ENABLE, HIGH);
  statusUSBPowerCentral = true;
#if defined(TXDEBUG)
  Serial.println(F("USB PowerCentral ON"));
#endif
}


void setup()
{
  Serial.begin(115200);    // TXDEBUGging only


  // shut off USB PowerCentral for power check.
  setupUSBPowerCentral();
  USBPowerCentralOff();

  Serial.println(F("SolarMAX WXLink Tx"));

  if (!rf95.init())
  {
    Serial.println(F("init failed"));
    while (1);
  }

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  //rf95.setTxPower(13, false);

  rf95.setFrequency(434.0);
  rf95.setTxPower(13);
  int Bw31_25Cr48Sf512 = 2;

  rf95.setModemConfig(RH_RF95<SoftwareSerial>::ModemConfigChoice(Bw31_25Cr48Sf512));
  //rf95.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);
  //rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);



  //rf95.printRegisters();

  //SoftSerial.begin(9600);


  Serial.print(F("Software Version:"));
  Serial.println(SOFTWAREVERSION);

  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  // setup initial values of variables

  wakeState = REBOOT;
  Alarm_State_1 = false;
  Alarm_State_2 = false;
  nextSleepLength = SLEEPCYCLE;



  TimeStamp = 0;

  InsideTemperature = 0.0;
  InsideHumidity = 0.0;
  BatteryVoltage = 0.0;
  BatteryCurrent = 0.0;
  LoadCurrent = 0.0;
  SolarPanelVoltage = 0.0;
  SolarPanelCurrent = 0.0;
  AuxA = 0.0;
  AuxB = 0.0;

  // protocol 1



  pinMode(WATCHDOG_1, OUTPUT);
  digitalWrite(WATCHDOG_1, HIGH);
  // Just to turn off LED on _1_
  //pinMode(WATCHDOG_2, OUTPUT);
  //digitalWrite(WATCHDOG_2, HIGH);

  Wire.begin();



  // Now set up Transmitter off
  digitalWrite(ENABLE_RADIO, HIGH);
  pinMode(ENABLE_RADIO, OUTPUT);


  // test for DS3231 Present
  DS3231_Present = false;

  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  if (timeStatus() != timeSet)
  {
    Serial.println(F("DS3231 Not Present"));
    DS3231_Present = false;
  }
  else
  {
    Serial.println(F("DS3231 Present"));
    digitalClockDisplay();
    DS3231_Present = true;


    uint8_t readValue;

  }

  uint8_t s = RTC.readRTC(RTC_STATUS);
  Serial.print(F("RTC_STATUS="));
  Serial.println(s, BIN);
  s = RTC.readRTC(RTC_CONTROL);
  Serial.print(F("RTC_CONTROL="));
  Serial.println(s, BIN);


  // test for INA3221_Present
  INA3221_Present = false;



  int MIDNumber;
  INA3221.wireReadRegister(0xFE, &MIDNumber);
  Serial.print(F("Manuf ID:   0x"));
  Serial.print(MIDNumber, HEX);
  Serial.println();
  if (MIDNumber != 0x5449)
  {
    INA3221_Present = false;
    Serial.println(F("INA3221 Not Present"));
  }
  else
  {
    INA3221_Present = true;

    BatteryVoltage = INA3221.getBusVoltage_V(LIPO_BATTERY_CHANNEL);
    BatteryCurrent = -INA3221.getCurrent_mA(LIPO_BATTERY_CHANNEL);

    SolarPanelVoltage = INA3221.getBusVoltage_V(SOLAR_CELL_CHANNEL);
    SolarPanelCurrent = -INA3221.getCurrent_mA(SOLAR_CELL_CHANNEL);

    Serial.println("");
    Serial.print(F("Battery Load Voltage:  ")); Serial.print(BatteryVoltage); Serial.println(F(" V"));
    Serial.print(F("Battery Current:       ")); Serial.print(BatteryCurrent); Serial.println(F(" mA"));
    Serial.println("");

    Serial.print(F("Solar Panel Voltage:   ")); Serial.print(SolarPanelVoltage); Serial.println(F(" V"));
    Serial.print(F("Solar Panel Current:   ")); Serial.print(SolarPanelCurrent); Serial.println(F(" mA"));
    Serial.println("");

    // Now Set Wireless ID

    if (BatteryVoltage < 8.0)
    {
      WirelessID = 8;   // SolarMAX LiPo
    }
    else
    {
      WirelessID = 10;   // SolarMAX Lead Acid (12V Battery)
    }

    Protocol = WirelessID * 10 + SOFTWAREVERSION;




    Serial.print(F("Wireless ID:"));
    Serial.println(WirelessID);

    Serial.println("");
  }


  // Now do the power up USB PowerCentral board check


  // now do the power up check on start.
  //
  //  if BV > 12V then turn on
  //
  //
#if defined(TXDEBUG)
  Serial.print(F("Battery Load Voltage:  ")); Serial.print(BatteryVoltage); Serial.println(F(" V"));
#endif
  if (WirelessID == 10)
  {
    if (BatteryVoltage > 12.0)
    {
      Serial.print(F("Battery Load Voltage:  ")); Serial.print(BatteryVoltage); Serial.println(F(" V"));
      USBPowerCentralOn();
    }
    else
    {
#if defined(TXDEBUG)
      Serial.println(F("USB PowerCentral NOT turned on"));
#endif
    }
  }


  if (WirelessID == 8)
  {
    if (BatteryVoltage > 3.0)
    {
      USBPowerCentralOn();
    }
    else
    {
#if defined(TXDEBUG)
      Serial.println(F("USB PowerCentral NOT turned on"));
#endif
    }
  }

#if defined(TXDEBUG)
  Serial.print(F("USBPowerCentral Status:"));
  Serial.println(statusUSBPowerCentral);
#endif

  // Look for INA219 on Power Central

  // default on 0x45

  INA219.begin();

  int Config;
  INA219.wireReadRegister(0x00, &Config);
  Serial.print(F("INA219 Config:   0x"));
  Serial.print(Config, HEX);
  Serial.println();
  USBPowerCentral_Present = false;


  if (Config == 0x399F)   // This will change with configuration changes
  {
    USBPowerCentral_Present = true;
    Serial.println("USB PowerCentral Present");


    float shuntvoltage = 0;
    float busvoltage = 0;

    float power_mW = 0;

    shuntvoltage = INA219.getShuntVoltage_mV();
    busvoltage = INA219.getBusVoltage_V();
    LoadCurrent = INA219.getCurrent_mA();
    power_mW = INA219.getPower_mW();
    LoadVoltage = busvoltage + (shuntvoltage / 1000);

    Serial.print("USBPowerCentral"); Serial.println();

    Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
    Serial.print("Load Voltage:  "); Serial.print(LoadVoltage); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(LoadCurrent); Serial.println(" mA");
    Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
    Serial.println("");
  }
  else
  {
    Serial.println("USB PowerCentral Present");
  }

  // Check for HDC1080
  HDC1080_Present = false;
  hdc1080.begin(0x40);

  Serial.print("Device ID ID=0x");
  int devID;
  devID = hdc1080.readDeviceId();
  Serial.println(devID, HEX); // 0x5449 ID of Texas Instruments

  if (devID == 0x1050)
  {
    Serial.println("HDC1080 Present");
    HDC1080_Present = true;
    Serial.print("T=");
    InsideTemperature = hdc1080.readTemperature();
    Serial.print(InsideTemperature);
    Serial.print("C, RH=");
    InsideHumidity = hdc1080.readHumidity();
    Serial.print(InsideHumidity);
    Serial.println("%");
  }
  else
  {
    Serial.println("HDC1080 Not Present");
    HDC1080_Present = false;
  }

}




void loop()
{

  if (DS3231_Present)
  {
    RTC.get();
#if defined(TXDEBUG)
    digitalClockDisplay();
#endif
  }
  // Only send if source is SLEEP_INTERRUPT
#if defined(TXDEBUG)
  Serial.print(F("wakeState="));
  Serial.println(wakeState);
#endif


  if ((wakeState == SLEEP_INTERRUPT) || (Alarm_State_1 == true)  || (Alarm_State_2 == true))
  {

    wakeState = NO_INTERRUPT;
    Alarm_State_1 = false;
    Alarm_State_2 = false;

    Serial.print(F("MessageCount="));
    Serial.println(MessageCount);

    if (HDC1080_Present)
    {

      InsideTemperature = hdc1080.readTemperature();
      InsideHumidity = hdc1080.readHumidity();


      Serial.print(F("Inside Temperature (C): ")); Serial.println(InsideTemperature);
      Serial.print(F("Inside Humidity (%RH): ")); Serial.println(InsideHumidity);

    }



    TimeStamp = millis();

    // if INA3221 present, read charge data

    if (INA3221_Present)
    {


      BatteryVoltage = INA3221.getBusVoltage_V(LIPO_BATTERY_CHANNEL);
      BatteryCurrent = -INA3221.getCurrent_mA(LIPO_BATTERY_CHANNEL);

      SolarPanelVoltage = INA3221.getBusVoltage_V(SOLAR_CELL_CHANNEL);
      SolarPanelCurrent = -INA3221.getCurrent_mA(SOLAR_CELL_CHANNEL);

      Serial.println("");
      Serial.print(F("LIPO_Battery Load Voltage:  ")); Serial.print(BatteryVoltage); Serial.println(F(" V"));
      Serial.print(F("LIPO_Battery Current:       ")); Serial.print(BatteryCurrent); Serial.println(F(" mA"));
      Serial.println("");

      Serial.print(F("Solar Panel Voltage:   ")); Serial.print(SolarPanelVoltage); Serial.println(F(" V"));
      Serial.print(F("Solar Panel Current:   ")); Serial.print(SolarPanelCurrent); Serial.println(F(" mA"));
      Serial.println("");


    }

    if (USBPowerCentral_Present)
    {



      float shuntvoltage;
      float busvoltage;


      shuntvoltage = INA219.getShuntVoltage_mV();
      busvoltage = INA219.getBusVoltage_V();
      LoadCurrent = INA219.getCurrent_mA();

      LoadVoltage = busvoltage + (shuntvoltage / 1000);

      Serial.print("USBPowerCentral"); Serial.println();



      Serial.print("Load Voltage:  "); Serial.print(LoadVoltage); Serial.println(" V");
      Serial.print("Current:       "); Serial.print(LoadCurrent); Serial.println(" mA");

      Serial.println("");

    }

    // write out the current protocol to message and send.
    int bufferLength;
    bufferLength = buildProtocolString();

    Serial.println(F("----------Sending packet----------"));

    /*
      /// turn radio on
      digitalWrite(ENABLE_RADIO, LOW);
      delay(100);
      SoftSerial.write((uint8_t *)byteBuffer, bufferLength);
    */
    // Send a message

    rf95.send(byteBuffer, bufferLength);
    Serial.println(F("----------After Sending packet----------"));
    if (!rf95.waitPacketSent(6000))
    {
      Serial.println(F("Timeout on transmission"));
      // re-initialize board
      if (!rf95.init())
      {
        Serial.println(F("init failed"));
        while (1);
      }

      // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

      // The default transmitter power is 13dBm, using PA_BOOST.
      // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
      // you can set transmitter powers from 5 to 23 dBm:
      //rf95.setTxPower(13, false);

      rf95.setFrequency(434.0);
      int Bw31_25Cr48Sf512 = 2;

      rf95.setModemConfig(RH_RF95<SoftwareSerial>::ModemConfigChoice(Bw31_25Cr48Sf512));
      // rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);

      rf95.setTxPower(13);

      //rf95.printRegisters();
      Serial.println(F("----------Board Reinitialized----------"));


    }
    else
    {
      Serial.println(F("----------Packet Sent.  Sleeping Now----------"));
      rf95.sleep();
    }
    Serial.println(F("----------After Wait Sending packet----------"));
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);

    Serial.print(F("freeMemory()="));
    Serial.println(freeMemory());
    Serial.print(F("bufferlength="));
    Serial.println(bufferLength);

    for (int i = 0; i < bufferLength; i++) {
      Serial.print(" ");
      if (byteBuffer[i] < 16)
      {
        Serial.print(F("0"));
      }
      Serial.print(byteBuffer[i], HEX);           //  write buffer to hardware serial port
    }
    Serial.println();

    // Now do the power up USB PowerCentral board check


    // now do the power up check.
    //
    // Do not power up computer until:
    //  Battery Voltage > 13V OR Solar Current > 100mA


#if defined(TXDEBUG)
    Serial.print(F("USB PowerUp Check - Wireless ID:"));
    Serial.println(WirelessID);
    Serial.print(F("Battery Load Voltage:  ")); Serial.print(BatteryVoltage); Serial.println(F(" V"));
    Serial.print(F("Solar Current:  ")); Serial.print(SolarPanelCurrent); Serial.println(F(" mA"));
#endif
    if (WirelessID == 10)
    {
      if ((BatteryVoltage > 12.8) || (SolarPanelCurrent > 100.0))
      {
        Serial.print(F("Battery Load Voltage:  ")); Serial.print(BatteryVoltage); Serial.println(F(" V"));
        USBPowerCentralOn();
      }
      else
      {
#if defined(TXDEBUG)
        Serial.println(F("USB PowerCentral NOT turned on"));
#endif
      }
    }

    if (WirelessID == 8)
    {
      if ((BatteryVoltage > 3.0) && (SolarPanelCurrent > 100.0))  
      {
        USBPowerCentralOn();
      }
      else
      {
#if defined(TXDEBUG)
        Serial.println(F("USB PowerCentral NOT turned on"));
#endif
      }
    }
    #if defined(TXDEBUG)
  Serial.print(F("USBPowerCentral Status:"));
  Serial.println(statusUSBPowerCentral);
#endif



    MessageCount++;



  }





  if (DS3231_Present == false)
  {
    if (wakeState != REBOOT)
      wakeState = SLEEP_INTERRUPT;
    long timeBefore;
    long timeAfter;
    timeBefore = millis();
#if defined(TXDEBUG)
    Serial.print(F("timeBeforeSleep="));
    Serial.println(timeBefore);
#endif
    delay(100);
    // This is what we use for sleep if DS3231 is not present
    if (DS3231_Present == false)
    {
      //Sleepy::loseSomeTime(nextSleepLength);
      for (long i = 0; i < nextSleepLength / 16; ++i)
        Sleepy::loseSomeTime(16);

      wakeState = SLEEP_INTERRUPT;

#if defined(TXDEBUG)
      Serial.print(F("Awake now: "));
#endif
      timeAfter = millis();
#if defined(TXDEBUG)
      Serial.print(F("timeAfterSleep="));
      Serial.println(timeAfter);

      Serial.print(F("SleepTime = "));
      Serial.println(timeAfter - timeBefore);

      Serial.print(F("Millis Time: "));
#endif
      long time;
      time = millis();
#if defined(TXDEBUG)
      //prints time since program started
      Serial.println(time / 1000.0);
      Serial.print(F("2wakeState="));
      Serial.println(wakeState);
#endif
    }
  }

  if (DS3231_Present == true)
  {
    // use DS3231 Alarm to Wake up
#if defined(TXDEBUG)
    Serial.println(F("Using DS3231 to Wake Up"));
#endif
    delay(50);
    Sleepy::powerDown ();

    if (RTC.alarm(ALARM_1))
    {
      Serial.println(F("ALARM_1 Found"));
      wakeState = ALARM_INTERRUPT;
      Alarm_State_1 = true;


    }

    if (RTC.alarm(ALARM_2))
    {
      Serial.println(F("ALARM_2 Found"));
      wakeState = ALARM_INTERRUPT;
      Alarm_State_2 = true;


    }

  }




  // Pat the WatchDog
  ResetWatchdog();


}
