/*
 This receiver listens for data coming from the wireless leak detector transmitters.  This sketch sends wireless
 data to a Raspberry Pi via I2c.
 
IDE Board to use: Arduino Pro Mini 3.3 volts
 
To Do:
maybe use UART Rx/Tx instead of I2C to communicate

 
 RF and I2C byte array structure:
 byte 0:   ID of wireless sensor sending data
 byte 1:   Seconds since last transmission of data (max 255)
 byte 2:   Wet/Dry Status. Wet = true, Dry = false
 byte 3-4: Temperature
 Byte 5-6: Battery voltage (mV)
 Byte 7:   RSSI - absolute value, it's normally a negative number.  This Rx sketch determines this
 Byte 8:   Spare
 Byte 9:   Checksum
 
 
Change Log
07/04/14  v2.00 - added checksum to panStamp and I2C data
02/24/15  v2.01 - add first floor bathroom, use stuct to hold data
03/07/15  v2.02 - Upgraded to latest panStamp API 
03/08/15  v2.03 - used array of struct WirelessWaterDetector instead of individual structures,  Deleted unused library files.
03/22/15  v2.04 - Minor print changes.  Added blinker() function prototype
03/24/15  v2.05 - Added function prototypes so it would compile in Xcode
10/27/20  v2.06 - only changed some comments 
11/02/20  v3.00 - Not using panStamp libraries anymore.  They haven't been kept up to date. Instead I'm using https://github.com/LSatan/SmartRC-CC1101-Driver-Lib
                  Increased antenna length to 1/2 wave (6.8" for 433 MHz). Changed frequence to 433, it's a lot stronger signal than 868
11/05/20   v3.10 - Changed RF and I2C packet structure.  Also, this sketch is not decoding RF then reincodeing I2C packet.  It's filling in seconds since last Tx, rssi, new checksum then passing to I2C
11/08/20   v3.11 - Changed age of last received data from minutes to seconds
02/18/21   v3.12 - Changed | to || in   if(txAge > 255 || g_isOnline[g_TxAddress_Requested] == false).  Added checksum verification
*/

#define VERSION "3.12"

#include <ELECHOUSE_CC1101_SRC_DRV.h>  //https://github.com/LSatan/SmartRC-CC1101-Driver-Lib
#include <Wire.h>

// Two LEDs are used to show panStamp and I2C communication
#define LED_RX_DATA     6  // LED flashes when it recieves wireless data from remote sensor
#define LED_I2C_DATA    7  // LED flashed when I2C packet was sent to RPi

// I2C Config
const byte ADDR_I2C =                    0x15;  // I2C address of this device
const byte DATA_LENGTH =                   10;  // Bytes in data packet
const byte CHECKSUM_BYTE =    DATA_LENGTH - 1;  // checksum is the last byte in array
volatile byte g_SentI2CDataFlag =       false;  // Used to flash LED when I2C data is sent
volatile byte g_TxAddress_Requested =       0;  // Address of remote transmitter the I2C master is requesting

// Radio config
const  byte CHANNEL   =                   46;  // Channel radio is using 
const float FREQUENCY =               433.92;  // Radio frequency 433.92 is library default.  868 is panStamp default, other panStamp is 915. The ELECHOUSE lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ.
const  byte WAIT_TIME =                  100;  // Wait time in milliseconds. Used for non-GDO mode. Once data comes in, it waits 100 mS for rest of data to arrive.  


// Packet locations
const byte TIME_BYTE = 1;
const byte RSSI_BYTE = 7;

const byte NUM_SENSORS = 8;  // number of supported sensors

// create 2 dimension array to hold packet data for each sensor
byte g_Data[NUM_SENSORS][DATA_LENGTH] = {0};


uint32_t g_lastTxTimeStamp[NUM_SENSORS] = {0};  // timestamp (millis) of last time received data fromsensor
bool g_isOnline[NUM_SENSORS] = {false};         // this is only necessary when recently booted up.  Without it offline sensors show online until 255 seconds have passed


// Function prototypes
byte calcCheckSum(byte newPacket[]);
void wireRequestEvent();
bool decodeSensorPacket(byte RxPacket[], int16_t dataLength, int16_t rssi, int16_t lqi);
void wireGetAddressEvent(int numBytes);
void printSensorValues(byte sensorID);
void blinker(byte led_pin, int count);

//============================================================================
//============================================================================
void setup()
{
  Serial.begin(9600);
  Serial.print(F("\n\rWireless Water Detector Receiver, \rVer "));
  Serial.println(VERSION);
  Serial.print("Frequency: ");
  Serial.print(FREQUENCY);
  Serial.print(" Mhz   Channel ID: ");
  Serial.println(CHANNEL);
  Serial.print("I2C Bus address: 0x");
  Serial.println(ADDR_I2C, HEX);
  

  // Flash LEDs
  pinMode(LED_RX_DATA,  OUTPUT);  
  pinMode(LED_I2C_DATA, OUTPUT);  
  blinker(LED_RX_DATA,  3);
  delay(500);
  blinker(LED_I2C_DATA, 3);

  // I2C Setup
  Wire.begin(ADDR_I2C);                    // Initiate the Wire library and join the I2C bus
  Wire.onRequest(wireRequestEvent);        // Register a function to be called when I2C master requests data from this slave device.
  Wire.onReceive(wireGetAddressEvent);     // This function is called when I2C master tells slave which data it wants.  Master sends the address of the wireless transmitter it wants

  // Setup the radio
  ELECHOUSE_cc1101.Init();                // Initialize the cc1101 
  ELECHOUSE_cc1101.setMHZ(FREQUENCY);     // Here you can set your basic frequency.  
  ELECHOUSE_cc1101.setCCMode(1);          // set config for internal transmission mode
  ELECHOUSE_cc1101.setModulation(0);      // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
  ELECHOUSE_cc1101.setSyncMode(2);        // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
  ELECHOUSE_cc1101.setCrc(1);             // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.
  ELECHOUSE_cc1101.setChannel(CHANNEL); // Set channel - this must match the Tx channel

  // Initialize 2D array with sensor ID, txAge , and checksum
  for (int i=0; i < NUM_SENSORS; i++)
  {
    g_Data[i][0] =           i;   // add sensor ID to byte 0
    g_Data[i][TIME_BYTE] = 255;   // default to offline
    g_Data[i][CHECKSUM_BYTE] = i + 255;  // add checksum to last byte.  Since the only data is ID and txAge, it's the sum of the two
  }


}  // end setup()


//============================================================================
// Get data from panStamps and put into I2C packet.
// When Master requests data from this slave, the wireRequestEvent() will
// execute and send the data to the other Arduiino
//============================================================================
void loop()
{

  byte bufferData[DATA_LENGTH] = {0};
  
  // Checks whether something has been received.
  // When something is received we give some time to receive the message in full.(time in millis)
  if (ELECHOUSE_cc1101.CheckRxFifo(WAIT_TIME))
  {
    if (ELECHOUSE_cc1101.CheckCRC())
    {
      // decode packet and put in array structure
      int len = ELECHOUSE_cc1101.ReceiveData(bufferData);   // https://github.com/LSatan/SmartRC-CC1101-Driver-Lib/blob/master/ELECHOUSE_CC1101_SRC_DRV.cpp#L1143-L1163
      int16_t  Rssi = ELECHOUSE_cc1101.getRssi();
      int16_t   lqi =  ELECHOUSE_cc1101.getLqi();
      byte txID = bufferData[0];  // trasmitter ID is in byte 0
      if( (len == DATA_LENGTH) && (txID < NUM_SENSORS) && (calcCheckSum(bufferData) ==  bufferData[CHECKSUM_BYTE]) )
      {
        memcpy(g_Data[txID], bufferData, DATA_LENGTH);  // copy new data in 2D array
        g_Data[txID][RSSI_BYTE] = abs(Rssi);
        g_Data[txID][TIME_BYTE] = 0;
        g_Data[txID][CHECKSUM_BYTE] = calcCheckSum(g_Data[txID]);
        g_lastTxTimeStamp[txID] = millis();  // store time data came in for this sensor
        g_isOnline[txID] = true;

        blinker(LED_RX_DATA, 1);  // blink LED once to indicate receipt of wireless data
        printSensorValues(txID);  // print RF data received
      }
    }
  }

  // blink LED when I2C data is sent to RPi
  if (g_SentI2CDataFlag)
  {
    blinker(LED_I2C_DATA, 1); 
    g_SentI2CDataFlag = false; // reset flag 
  }
  
} // end loop()


//============================================================================
// Calculate new checksum
//============================================================================
byte calcCheckSum(byte newPacket[])
{
  byte newChecksum = 0;
  for( byte cs = 0; cs < CHECKSUM_BYTE; cs++)
  { newChecksum += newPacket[cs]; }

  return newChecksum;
  
}  // end calcCheckSum()


//========================================================================================================================================
// function executes when master sends over the address of the
// wireless water sensor that it wants
//========================================================================================================================================
void wireGetAddressEvent(int numBytes)
{
  g_TxAddress_Requested = Wire.read();
  
} // end wireGetAddressEvent()

//========================================================================================================================================
// Function that executes whenever data is requested by RPi
// this function is registered as an event in setup()
// It will return the data for one wireless sensor.  Which sensor data to send it determined by g_TxAddress_Requested
// in wireGetAddressEvent().  RPi sends a write request and requested sensor ID is stored in g_TxAddress_Requested
//========================================================================================================================================
void wireRequestEvent()
{

  // Requsted sensor ID is out of range, return error packet with ID = 255
  if (g_TxAddress_Requested >= NUM_SENSORS) 
  {
    byte emptyPacket[] = {255,0,0,0,0,0,0,0,0,255};
    Wire.write(emptyPacket, DATA_LENGTH);
    return;   
  }
    
  // Calculate age in seconds of requested transmitter data
  uint32_t txAge = (long) (millis() - g_lastTxTimeStamp[g_TxAddress_Requested]) / 1000UL; 
  if(txAge >= 255 || g_isOnline[g_TxAddress_Requested] == false)
    g_Data[g_TxAddress_Requested][TIME_BYTE] = 255;
  else
    g_Data[g_TxAddress_Requested][TIME_BYTE] = (byte)txAge;

  // Calculate a new checksum
  g_Data[g_TxAddress_Requested][CHECKSUM_BYTE] = calcCheckSum(g_Data[g_TxAddress_Requested]);
  
  // Send data to I2C master
  Wire.write(g_Data[g_TxAddress_Requested], DATA_LENGTH);

  g_SentI2CDataFlag = true;  // Flag set when I2C data is sent out, used to flash LED
  
} // end wireRequestEvent()


//============================================================================
// Blink LED count times
//============================================================================
void blinker(byte led_pin, int count)
{
  for(int j=1; j<=count; j++)
  {
    digitalWrite(led_pin, HIGH);
    delay(50);
    digitalWrite(led_pin, LOW);
    delay(50);
  }
}  // end blinker()



//=============================================================================================================================================
//  Print data received - for debugging
//=============================================================================================================================================
void printSensorValues(byte sensorID)
{

  Serial.print("ID: ");
  Serial.print(sensorID);
  Serial.print("  [");
  for(int j = 0; j < 10; j++)
  {
    Serial.print(g_Data[sensorID][j]);
    Serial.print("  ");
  }
  Serial.print("] ");
  Serial.println();
  

} // end printSensorValues()
