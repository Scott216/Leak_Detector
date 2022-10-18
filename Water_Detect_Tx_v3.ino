/*
Wireless Water Detector Trasnmitter
Board: Arduino Pro Min 3.3v 

NOTE: After panStamp wakes up from LowPower.powerDown() Serial.print() doesn't work

Battery usage - used uCurrent to measure sleep current
awake current is 6.0 mA
atmega sleep only is 1.7 mA
atmega + cc1101 sleep is 0.11 mA

To Do:
Redo PCB with switch to select address 
 
Inputs:
 Wet/Dry status from sponge
 Temperature from OneWire DS18B20
 Status button - when pressed RGB LED will display status
 Address selector rotary dip switch - future
 
Outputs:
  PWM for LED

Data to Transmit (10 bytes):
 byte 0:   ID of wireless sensor sending data
 byte 1:   Minutes since last transmission of data (max 255)
 byte 2:   Wet/Dry Status. Wet = true, Dry = false
 byte 3-4: Temperature
 Byte 5-6: Battery voltage (mV)
 Byte 7:   RSSI - absolute value, it's normally a negative number.  This Rx sketch determines this
 Byte 8:   Spare
 Byte 9:   Checksum
 

=== I/O ===
D3 - Status LED, Blu, PWM
D4 - Water detector - sponge with op-amp
D5 - Status LED, Grn, PWM
D6 - Status LED, Red, PWM
D8 - 1Wire temperature sensor input
D9 - Status button input
A0, A1, A2, A3 - BCD switch for Tx address selection.  Each sensor should have a different address so Rx knows which sensor sent it

 
Change Log:
07/04/14 v2.00 - Made ledColor an enum, added spare data field and checksum
02/26/15 v2.01 - Moved voltage calibration from main sketch to this one. Changed some variable names, formatting. Support for 3rd wireless transmitter
                 Started adding support for BCD address switch.  Added code to save voltage calibration to EEPROM
03/08/15 v2.02 - Upgraded to work with latest panStamp API library. Lots of formatting and comment changes.
                 Removed readVcc() function and replaced with panstamp.getVcc().  Removed unused files: Water_detector_Tx_Library.h/cpp
                 Changed addresses of panStamps transmitters to start at 0 - bacause address ID is used in as array element number
03/09/15 v2.03 - Trying to get sketch to compile in Xcode
03/22/15 v2.04 - added print statements to voltage calibration section
11/02/20 v3.00 - Not using panStamp libraries anymore.  They haven't been kept up to date. Instead I'm using https://github.com/LSatan/SmartRC-CC1101-Driver-Lib
                 Moved voltage calibration read/write to its own function.  Got code to read battery voltage
                 Increased antenna length to 1/2 wave (6.8" for 433 MHz). Changed frequence to 433, it's a lot stronger signal than 868
11/05/20 v3.10 - Changed RF packet structure
11/08/20 v3.11 - added #define PRINT_DEBUG because Serial printing doesn't work after waking up from sleep

*/

#define VERSION  "3.11"
// #define PRINT_DEBUG  - comment this out for normal sleep mode.  If you need to print out info, then uncomment
 

byte g_Tx_Address = 0;  // Master Bath = 0, Guest = 1, First Floor = 2

const bool SAVE_VOLTAGE_CALIB = false; // set to true to save voltage calibration offset (g_volt_calibration[]) to EEPROM.  Just do this once right after calibration offset is calculated with a voltmeter
  
  //  Voltage calibration is the mV difference between what the program reads as battery voltage vs voltmeter measured voltage 
  //  Master bath = -71, Guest bath = -40, First Floor bath = 0
  int16_t g_volt_calibration[] = {-71, -36, 0, 0, 0, 0, 0, 0} ; 
  

#include <EEPROM.h>                   // voltage calibration offset is saved to EEPROM http://www.arduino.cc/en/Reference/EEPROM
#include "OneWire.h"                  // http://www.pjrc.com/teensy/td_libs_OneWire.html
#include "DallasTemperature.h"        // http://milesburton.com/Main_Page?title=Dallas_Temperature_Control_Library
#include <ELECHOUSE_CC1101_SRC_DRV.h> // https://github.com/LSatan/SmartRC-CC1101-Driver-Lib
#include "LowPower.h"                 // https://github.com/rocketscream/Low-Power
  

#define WET LOW                  // When digital input from sponge/Op-Amp is LOW, the sponge is wet
#define TEMPERATURE_PRECISION 9  // sets precision of DS18B20 1-wire temperature sensor

const byte PIN_LED_RED =          6;  // RGB LED Red, PWM pin
const byte PIN_LED_GRN =          5;  // RGB LED Green, PWM pin
const byte PIN_LED_BLU =          3;  // RGB LED Blue, PWM pin
const byte PIN_STATUS_BUTTON =    9;  // Status pushbutton input pin
const byte PIN_SPONGE  =          4;  // Input pin from op-amp connected to the sponge
const byte PIN_TEMPERATURE =      8;  // Data pin for 1-Wire temperature sensor
const byte PIN_ADDR_BCD1    =    A0;  // Rotary BCD switch pin 1
const byte PIN_ADDR_BCD2    =    A0;  // Rotary BCD switch pin 2
const byte PIN_ADDR_BCD4    =    A0;  // Rotary BCD switch pin 4
const byte PIN_ADDR_BCD8    =    A0;  // Rotary BCD switch pin 8


const byte DATA_LENGTH =                10;  // number of bytes in wireless packet going to receiver
const byte CHECKSUM_BYTE = DATA_LENGTH - 1;  // packet checksum (last byte in packet)
const byte EEPROM_ADDR_CALIB =         100;  // Starting EEPROM address used to store for voltage calibration offset.  Offset is calculate manually by comparing actual voltage from voltmeter to voltage detected by program
                                             // 1st byte is status whether or not calibration has been saved, next two byts are lsb and msb of calibaration amount

// Color settings for LED
// Green = volts ok, yellow = volts getting low, red = low volts, blue = water detected
enum ledColor_t { GREEN, YELLOW, RED, BLUE, WHITE };


// Setup a 1-Wire instances to communicate 1-Wire DS18B20 temperature sensor
OneWire oneWire(PIN_TEMPERATURE);
DallasTemperature tempSensor(&oneWire);


// Radio config
const  byte    CHANNEL =     46;  // Channel radio is using n packet array 
const  byte  WAIT_TIME =     100;  // Wait time in milliseconds. Used for non-GDO mode.  Once data comes in, it waits 100 mS for rest of data to arrive.  If using GDO, the GDO pin is set to indicate message has completed
const int16_t TX_POWER =      12;  // transmit power  (-30  -20  -15  -10  -6    0    5    7    10   11   12)
const float  FREQUENCY =  433.92;  // 433.92 MHz is library default. 868 MHz is panStamp default, other panStamp is 915. The ELECHOUSE lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ.

const uint16_t LOW_VOLT_LIMIT =      2500;  // Minimum millivolts that panStamp can operate.  If volts is below this, don't send data

// Function Prototypes
void statusButton(uint16_t battery_millivolts);
bool isWet();
uint16_t readBatteryVoltage();
byte getDeviceAddress();          // returns address of this transmitter by reading BCD rotary switch
void setVoltageCalibration(bool SaveToEEPROM);
void blinkLED(ledColor_t color); 
void printRFPacket(byte packet[]);

//============================================================================
//============================================================================
void setup()
{

  #ifdef PRINT_DEBUG
    Serial.begin(9600);
    Serial.print("Wireless Water Detector Transmitter, \rVer: ");
    Serial.println(VERSION);
    Serial.print("Transmit power: ");
    Serial.println(TX_POWER);
    Serial.print("Frequency: ");
    Serial.println(FREQUENCY);
    delay(1000);
  #endif  

  // Setup digital I/O pins
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GRN, OUTPUT);
  pinMode(PIN_LED_BLU, OUTPUT);
  pinMode(PIN_ADDR_BCD1, INPUT_PULLUP);  // Rotary BCD switch to select panStamp address
  pinMode(PIN_ADDR_BCD2, INPUT_PULLUP);
  pinMode(PIN_ADDR_BCD4, INPUT_PULLUP);
  pinMode(PIN_ADDR_BCD8, INPUT_PULLUP);
  
  pinMode(PIN_TEMPERATURE, INPUT);
  pinMode(PIN_STATUS_BUTTON,  INPUT_PULLUP);

  g_Tx_Address = getDeviceAddress();
  #ifdef PRINT_DEBUG
    Serial.print("Tx Address: ");
    Serial.println(g_Tx_Address);
  #endif
    
  // initialize panStamp radio
  ELECHOUSE_cc1101.Init();            // must be set to initialize the cc1101
  ELECHOUSE_cc1101.setMHZ(FREQUENCY);     // Here you can set your basic frequency.  
  ELECHOUSE_cc1101.setCCMode(1);      // set config for internal transmission mode.
  ELECHOUSE_cc1101.setModulation(0);  // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
  ELECHOUSE_cc1101.setSyncMode(2);    // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
  ELECHOUSE_cc1101.setPA(TX_POWER); // set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
  ELECHOUSE_cc1101.setCrc(1);         // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.
  ELECHOUSE_cc1101.setChannel(CHANNEL);  // Channel for radio, must be same as Rx


  // Start up the library for temp sensor
  tempSensor.begin();

  setVoltageCalibration(SAVE_VOLTAGE_CALIB);  // Read or write from EEPROM the voltage offset
 
  
} // end setup()


//============================================================================
// Main loop.  Read temp sensor, wet/dry status, battery voltage
// Sleeps 8 seconds, then wakes up to send data, then goes to sleep again
//============================================================================
void loop()
{
  
  static int16_t airTemp;
  uint16_t battery = readBatteryVoltage(); // Read battery voltage 
  
  tempSensor.requestTemperatures();         // Send the command to get temperatures
  airTemp = tempSensor.getTempFByIndex(0);  // Read temp sensor
  if ( airTemp > 100 || airTemp < 0)
  { airTemp = -1; }  // got invalid temp from 1wire

  // Send data only if voltage is greater than LOW_VOLT_LIMIT
  // If voltage is lower then this, panStamp doesn't work properly and send out bad data
  if ( battery >= LOW_VOLT_LIMIT )
  {
    byte TxPacket[DATA_LENGTH];
    TxPacket[0] = g_Tx_Address;         // Address of this panStamp Tx
    TxPacket[1] = 0;                    // Minutes since last Tx.  Populated in Rx sketch, it has no meaning here
    TxPacket[2] = isWet();              // Read wet-dry status
    TxPacket[3] = airTemp >> 8 & 0xff;  // High byte - shift bits 8 places, 0xff masks off the upper 8 bits
    TxPacket[4] = airTemp      & 0xff;  // Low byte, just mask off the upper 8 bits
    TxPacket[5] = battery >> 8 & 0xff;  // High byte - shift bits 8 places, 0xff masks off the upper 8 bits
    TxPacket[6] = battery      & 0xff;  // Low byte, just mask off the upper 8 bits
    TxPacket[7] = 0x00;                 // RSSI - pupulated in Rx sketch
    TxPacket[8] = 0x00;                 // Spare
    // Calculate checksum
    TxPacket[CHECKSUM_BYTE] = 0; // clear checksum
    for( byte cs = 0; cs < CHECKSUM_BYTE; cs++)
    { TxPacket[CHECKSUM_BYTE] += TxPacket[cs]; }
    
    ELECHOUSE_cc1101.SendData(TxPacket, DATA_LENGTH, WAIT_TIME); 
    // If you need to print out packet for debugging, you can't put panStamp to sleep with LowPower.powerDown - it messes up the Serial()
 
    #ifdef PRINT_DEBUG  
      printRFPacket(TxPacket);
      delay(4000);
    #endif
  
  }

  // If not using PRINT_DEBUG, put panstamp to sleep normally
  #ifndef PRINT_DEBUG
    // Put cc1101 radio into sleep mode
    ELECHOUSE_cc1101.SpiStrobe(0x36);  //Exit RX / TX, turn off frequency synthesizer and exit
    ELECHOUSE_cc1101.SpiStrobe(0x39);  //Enter power down mode when CSn goes high.
  
    // Loop several times so board is in sleep mode longer than 8 seconds
    for (int c=0; c < 2; c++)
    {  
      statusButton(battery);  // check to see if status button is being depressed
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // Enter power down state for 8 seconds with ADC and BOD module disabled. This will mess up Serial.print()
    }
   
    ELECHOUSE_cc1101.SetRx();    // wake up radio - might not need
  #endif
  
}  // end loop()


//=======================================================================
// Check to see if Status button is being pressed
//=======================================================================
void statusButton(uint16_t battery_millivolts)
{
  // When status button is pressed (LOW) turn on status LED
  // May need hold down the button up to 8 seconds because panStamp is probably asleep
  if(digitalRead(PIN_STATUS_BUTTON) == LOW)   
  {
    // LED color reflects voltage level
    if ( battery_millivolts < 2700 )
    { blinkLED(RED); }
    else if ( battery_millivolts < 2900 )
    { blinkLED(YELLOW); }
    else
    { blinkLED(GREEN); }
    delay(500);
    
    // Make LED blue if sponge is wet
    if(isWet() == true)
    { 
      blinkLED(BLUE);
      delay(500);
    }
  }
  
}

//=======================================================================
// Read the wet/dry status of sponge input
// Return true if sponge is wet
//=======================================================================
bool isWet()
{
  // Loop until two consecutive reading are the same
  bool firstreading;
  bool secondreading;
  do 
  {
    firstreading = digitalRead(PIN_SPONGE);
    delay(10);
    secondreading = digitalRead(PIN_SPONGE);
  } while (firstreading != secondreading);
  
  if (firstreading == WET)
  { return true; } 
  else
  { return false;}
  
} // end isWet()


//=======================================================================
// Read battery voltage in millivolts
// Source: https://www.gammon.com.au/adc
// Also see this forum post https://forum.arduino.cc/index.php?topic=435065.0
//=======================================================================
uint16_t readBatteryVoltage()
{

 const float InternalReferenceVoltage = 1.1; // as measured
  
  ADCSRA =  bit (ADEN);   // turn ADC on
  ADCSRA |= bit (ADPS0) |  bit (ADPS1) | bit (ADPS2);  // Prescaler of 128
  ADMUX = bit (REFS0) | bit (MUX3) | bit (MUX2) | bit (MUX1);
  
  delay (10);  // let it stabilize
  
  bitSet (ADCSRA, ADSC);  // start a conversion  
  while (bit_is_set(ADCSRA, ADSC))
    { }
  
  float volts = InternalReferenceVoltage / float (ADC + 0.5) * 1024.0; 
  uint16_t mvResult = volts * 1000;

  // Adjust voltage with calibaration value that was done with a voltmeter for this sensor and should be stored in EEPROM
  mvResult += g_volt_calibration[g_Tx_Address]; 
  
  return mvResult;
  
} // end readBatteryVoltage()

//=======================================================================
// Read BCD switch to get address for this transmitter
//=======================================================================
byte getDeviceAddress()
{

  byte txAddress = 0;
  
  // Read BCD switch and set address
  if (digitalRead(PIN_ADDR_BCD1))
  { txAddress |= 0x01; }
  if (digitalRead(PIN_ADDR_BCD2))
  { txAddress |= 0x02; }
  if (digitalRead(PIN_ADDR_BCD4))
  { txAddress |= 0x04; }
  if (digitalRead(PIN_ADDR_BCD8))
  { txAddress |= 0x08; }
//srg  return txAddress;
  
  
// srg, once BCD is connected, delete the code below
  return g_Tx_Address;
  
}  // end getDeviceAddress()


//============================================================================
// Reads or writes voltage calibration offset in EEPROM
// Voltage calibration is a mV value that represents the difference betweeen
// the voltage the code returns and the actual voltage measured with a voltmeter
//============================================================================
void setVoltageCalibration(bool SaveToEEPROM)
{
  byte volt_calib_lsb, volt_calib_msb;
  const byte CALIB_DATA_SAVED = 111;  // calibration status, 111 = calibration data has been saved, anything else it hasn't
  if( SaveToEEPROM )
  {
    // Save voltage (mV) calibration offset to EEPROM
    volt_calib_msb = g_volt_calibration[g_Tx_Address] >> 8 & 0xff;
    volt_calib_lsb = g_volt_calibration[g_Tx_Address] & 0xff;
    EEPROM.write(EEPROM_ADDR_CALIB,   CALIB_DATA_SAVED);  // So sketch can detect if calib data has been saved to EEPROM
    EEPROM.write(EEPROM_ADDR_CALIB + 1, volt_calib_lsb);
    EEPROM.write(EEPROM_ADDR_CALIB + 2, volt_calib_msb);
    Serial.print("Saved new voltage calibration offset: ");
    Serial.print(g_volt_calibration[g_Tx_Address]);
    Serial.println(" mV");
    blinkLED(WHITE); // Blink status light
    blinkLED(WHITE); 
  }
  else 
  {
    // read voltage (mV) calibration from EEPROM
    if ( EEPROM.read(EEPROM_ADDR_CALIB) == CALIB_DATA_SAVED )  // check to see if calibration has been saved to EEPROM
    { 
      // Calibration data has previously been saved to EEPROM, so go ahead and read it
      volt_calib_lsb = EEPROM.read(EEPROM_ADDR_CALIB + 1);
      volt_calib_msb = EEPROM.read(EEPROM_ADDR_CALIB + 2);
      g_volt_calibration[g_Tx_Address] = volt_calib_msb << 8;
      g_volt_calibration[g_Tx_Address] |= volt_calib_lsb;
      Serial.print("Voltage calibration is: ");
      Serial.print(g_volt_calibration[g_Tx_Address]);
      Serial.println(" mV");
      blinkLED(GREEN); // Blink status light green 
      blinkLED(GREEN);
    }
    else // there is no calibration data in EEPROM
    { 
      g_volt_calibration[g_Tx_Address] = 0; 
      Serial.println("No voltage calibration set");
      blinkLED(RED); // Blink status red 
      blinkLED(RED);
    }  
  }

  
}


//============================================================================
// Blink LED for 1/2 second.  color is passed to this function
//============================================================================
void blinkLED(ledColor_t color)
{
  byte redColor;
  byte grnColor;
  byte bluColor;
  
  // Set LED color
  switch (color)
  {
    case RED:
      redColor = 255;
      grnColor = 0;
      bluColor = 0;
      break;
    case GREEN:
      redColor = 0;
      grnColor = 255;
      bluColor = 0;
      break;
    case YELLOW:
      redColor = 255;
      grnColor = 255;
      bluColor = 0;
      break;
    case BLUE:
      redColor = 0;
      grnColor = 0;
      bluColor = 255;
      break;
    default:  // White
      redColor = 255;
      grnColor = 255;
      bluColor = 255;
      break;
  } 

  analogWrite(PIN_LED_RED, redColor);
  analogWrite(PIN_LED_GRN, grnColor);
  analogWrite(PIN_LED_BLU, bluColor);
  
  delay(500);
  
  // Turn LED off
  analogWrite(PIN_LED_RED, 0);
  analogWrite(PIN_LED_GRN, 0);
  analogWrite(PIN_LED_BLU, 0);
  delay(100);

} // end blinkLED()


//============================================================================
// print RF byte packet
//============================================================================
void printRFPacket(byte packet[])
{

  Serial.print(" [");
  for(int j = 0; j < DATA_LENGTH; j++)
  {
    Serial.print(packet[j]);
    Serial.print("  ");
  }
  Serial.print("] ");
  Serial.println();
  

} // end printSensorValues()
