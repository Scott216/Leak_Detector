# Read sensor data from panStamp via I2C
# I think default RPi i2c bus speed is too fast for Arduino/panStamp.
# Edited /boot/config.txt and added:  dtparam=i2c_arm_baudrate=10000


# Change Log
# 11/5/20 v1.00 - Initial version after moving off Arduino.  Everything seems to be working, haven't added e-ink code yet



# For pinout see pinout.txt


import smbus
import time
import datetime
import RPi.GPIO as GPIO
import private_credentials
from twilio.rest import Client  # https://pypi.org/project/twilio
from twilio.base.exceptions import TwilioRestException

i2c_address = 0x15  # 21 decimal
data_len = 10 # I2C data packet is 10 bytes

NumWirelessSensors = 4  # max is 8
NumWiredSensors    = 10

resetSensorTimeofDay = 3600 * 8  # 8 AM - number of seconds after midnight to reset all the sensor wet/dry status
doubleCheckDelay = 120  # seconds to wait after a sensor turn wet to double check it again

# wetStatus values
DRY =              0
WET_FROM_SENSOR =  1  # Status if sensor is wet, but hasn't had time for double check deley
WET_DOUBLECHECK =  2  # Status if sensor is still wet after a couple minute delay
WET_MESSAGE_SENT = 3  # Status if sensor is wet and SMS message has been sent 

# Initiliaze I2C bus
i2cbus = smbus.SMBus(1)

# Setup GPIO using board pin numbering, not BCM numbering
# Pins are configured in clsLeakSensor __init__
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)


#------------------------------------------------------------------
class clsLeakSensor:
    def __init__(self, ID, desc, isWet, timeWet, wetStatus, isWireless, IO_Pin, UpdateAge, temperature, battery, rssi, coldMsgSent):
        self.ID          = ID          # ID for wireless sensor, set on sensor with rotary swithc.  Max is 8
        self.desc        = desc        # Text location for sensor
        self.isWet      = isWet      # True if sensor detected water
        self.timeWet    = timeWet    # Timestamp of when water was first detected
        self.wetStatus   = wetStatus   # 0=dry, 1=initial wet from sensor, 2=still wet after couple min double check, 3=
        self.isWireless  = isWireless  # True if sensor is wireless
        self.IO_Pin      = IO_Pin      # Board Pin number (not GPIO Pin) for wired sensor   
        self.UpdateAge   = UpdateAge   # number of minutes since last update from wireless sensor (max 255)
        self.temperature = temperature # Temperatue for wireless sensor
        self.battery     = battery     # Battery voltage in mV for wireless sensor
        self.rssi        = rssi        # Signal strength for wireless sensor
        self.coldMsgSent = coldMsgSent # One-shot flag set to True if message was sent about cold wireless sensor
        # Setup input pins for wired sensors
        if (self.isWireless == False):
            GPIO.setup(self.IO_Pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def resetWet(self):   # Reset once a day
        self.isWet = False
        self.timeWet = 0
        self.wetStatus = DRY
        self.coldMsgSent = False
        
#------------------------------------------------------------------



nopin = 0
wired = False
Wireless = True
time1 = 0

# create array for sensors using class clsLeakSensor and fill in defaults
wireless_desc = ["Master Bath", "Guest Bath", "1st Floor Bath Wireless", "Wireless1", "Wireless2", "Wireless3", "Wireless4", "Wireless5"]
sensorInfo = []
for cc in range(NumWirelessSensors):
    sensorInfo.append( clsLeakSensor(cc, wireless_desc[cc],                      DRY, time1, DRY, Wireless, nopin, 0, 0, 0, 0, False) )
    
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 0, "1st Floor Bath Wired", DRY, time1, DRY, wired,       22, 0, 0, 0, 0, False) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 1, "Washing Machine",      DRY, time1, DRY, wired,       26, 0, 0, 0, 0, False) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 2, "Dehimidifier",         DRY, time1, DRY, wired,       32, 0, 0, 0, 0, False) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 3, "Hot Water Tank",       DRY, time1, DRY, wired,       33, 0, 0, 0, 0, False) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 4, "Boiler",               DRY, time1, DRY, wired,       35, 0, 0, 0, 0, False) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 5, "Kitchen Frig",         DRY, time1, DRY, wired,       36, 0, 0, 0, 0, False) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 6, "Dishwasher",           DRY, time1, DRY, wired,       37, 0, 0, 0, 0, False) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 7, "Kitchen Sink",         DRY, time1, DRY, wired,       38, 0, 0, 0, 0, False) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 8, "Hot Tub",              DRY, time1, DRY, wired,       40, 0, 0, 0, 0, False) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 9, "Spare 1",              DRY, time1, DRY, wired,        7, 0, 0, 0, 0, False) )



#------------------------------------------------------------------
# Get data from wireless sensors
#------------------------------------------------------------------
def getWirelessSensors():

    somethingIsWetFlag = False # Flag set to True if any sensor is wet after the double-check

    # Loop to request data from wireless sensors from panStamp receiver
    for wirelessID in range(NumWirelessSensors):
        
        I2Cpacket = i2cbus.read_i2c_block_data(i2c_address, wirelessID, data_len) # Request data from panStamp via I2C

        if (I2Cpacket[0] == wirelessID):  # panStamp returns 255 for sensorID if invalid sensor request is made
            sensorInfo[wirelessID].ID           = I2Cpacket[0]
            sensorInfo[wirelessID].UpdateAge    = I2Cpacket[1]       # minutes since data was last received for wireless sensor
            sensorInfo[wirelessID].isWet        = I2Cpacket[2]
            sensorInfo[wirelessID].temperature  = I2Cpacket[3] << 8
            sensorInfo[wirelessID].temperature |= I2Cpacket[4]
            sensorInfo[wirelessID].battery      = I2Cpacket[5] << 8
            sensorInfo[wirelessID].battery     |= I2Cpacket[6]
            sensorInfo[wirelessID].rssi         = I2Cpacket[7] * -1

        # Validate Data
        if(sensorInfo[wirelessID].temperature > 120):
            sensorInfo[wirelessID].temperature = -1;

        # If sensor is online, then run code to check for wet
        if (sensorInfo[wirelessID].UpdateAge < 120):
                
            # If sensor just turned wet, then update timeWet and wetStatus
            if ( (sensorInfo[wirelessID].isWet == True) and (sensorInfo[wirelessID].wetStatus == DRY) ):
                sensorInfo[wirelessID].timeWet = time.time()
                sensorInfo[wirelessID].wetStatus = WET_FROM_SENSOR
                
            # If sensor is wet, then after a few minutes, double check again
            # isWet would have been updated again just now by I2Cpacket[2]
            if ( (sensorInfo[wirelessID].isWet == True) and (time.time() > sensorInfo[wirelessID].timeWet + doubleCheckDelay) and (sensorInfo[wirelessID].wetStatus == WET_FROM_SENSOR) ):
                sensorInfo[wirelessID].wetStatus = WET_DOUBLECHECK
                somethingIsWetFlag = True

    return somethingIsWetFlag


#------------------------------------------------------------------
# Get data from wired sensors
# Loop through each pin, if any pin is wet, wait and double check
# need two consecutive readings to be the same
#------------------------------------------------------------------
def getWiredSensors():

    somethingIsWetFlag = False # Flag set to True if any sensor is wet
    
    for wiredID in range(NumWirelessSensors, TotalSensors):
        firstReading = False
        secondReading = True  # set this to True initally so that the while loop runs at least once
        while (firstReading != secondReading):
            firstReading = GPIO.input(sensorInfo[wiredID].IO_Pin)
            time.sleep(0.01)
            secondReading = GPIO.input(sensorInfo[wiredID].IO_Pin)
            if (firstReading != secondReading):
                time.sleep(0.01) #if reading don't match, sleep before going through loop again

        # Readings match, could be wet or dry, put value in isWet
        sensorInfo[wiredID].isWet = firstReading

        # If sensor just turned wet, then update timeWet and wetStatus
        if ( (sensorInfo[wiredID].isWet == True) and (sensorInfo[wiredID].wetStatus == DRY) ):
            sensorInfo[wiredID].timeWet = time.time()
            sensorInfo[wiredID].wetStatus = WET_FROM_SENSOR
            
        # If sensor is wet, then after a few minutes, double check again
        # isWet would have been updated again just now by I2Cpacket[2]
        if ( (sensorInfo[wiredID].isWet == True) and (time.time() > sensorInfo[wiredID].timeWet + doubleCheckDelay) and (sensorInfo[wiredID].wetStatus == WET_FROM_SENSOR) ):
            sensorInfo[wiredID].wetStatus = WET_DOUBLECHECK
            somethingIsWetFlag = True

            
    return somethingIsWetFlag



#------------------------------------------------------------------
# Print Sensor Info
#------------------------------------------------------------------
def printSensorInfo():

    for k in range(TotalSensors):
        if (sensorInfo[k].isWireless == True):
            # Wireless sensors
            print("ID:{:2.0f}\tWet:{},{}\t  Age:{:3.0f}\t Temp:{:2.0f}\tmV:{:4.0f}\t RSSI:{:3.0f}   {}".format(       \
                                              sensorInfo[k].ID, sensorInfo[k].isWet, sensorInfo[k].wetStatus, sensorInfo[k].UpdateAge,       \
                                              sensorInfo[k].temperature, sensorInfo[k].battery, sensorInfo[k].rssi, \
                                              sensorInfo[k].desc))
        else:
            #Wired sensors
            print("ID:{:2.0f}\tWet:{},{}\t  Age:{}\t Temp:{}\tmV:{}\t RSSI:{}   {}".format(   \
                                              sensorInfo[k].ID, sensorInfo[k].isWet, sensorInfo[k].wetStatus, "   ",  \
                                              "  ", "    ", "   ", sensorInfo[k].desc))
    print("-----------------------------------------------------------------------------")


#------------------------------------------------------------------
# Send SMS via Twilio
#------------------------------------------------------------------
def sendSMS(sms_msg):
##    try:
##        smsclient = Client(private_credentials.TWILIO_ACCOUNT_SID, private_credentials.TWILIO_AUTH_TOKEN)
##        message = smsclient.messages.create(body=sms_msg, from_=private_credentials.TWILIO_PHONE, to=private_credentials.TO_PHONE)
##    except TwilioRestException as smserr:
##        print("Twilio SMS failed: {}".format(smserr))
##
##    time.sleep(5)  # in case there are more than 1 message close together, don't send to quickly

    print(sms_msg)

#------------------------------------------------------------------
# Send weekly status report with status of wireless sensors
#------------------------------------------------------------------
def sendStatusReport():

    statusMsg = "" 
    for wirelessID in range (NumWirelessSensors):
        if (sensorInfo[wirelessID].UpdateAge < 255):
            statusMsg = "{}{} {}°, {:.2f}V\n".format(statusMsg, sensorInfo[wirelessID].desc, sensorInfo[wirelessID].temperature, sensorInfo[wirelessID].battery/1000 )
        else:
            statusMsg = "{}{} is offline\n".format(statusMsg, sensorInfo[wirelessID].desc)

    print(statusMsg)  # srg debug
 
#------------------------------------------------------------------
# Setup
#------------------------------------------------------------------
TotalSensors = NumWirelessSensors + NumWiredSensors

dayOfMonth = datetime.datetime.today().day  # used in new day trigger
resetSensorTimer = time.time() + (3600 * 24)  # This will be properly set at midnight

statusReportTime = time.time() + (3600 * 24 * 7)  # This will be properly set at Sunday at midnight



#------------------------------------------------------------------
# Main Program
#------------------------------------------------------------------
while True:

    wetWireleess = getWirelessSensors()  # loops through wireless sensors
    wetWired =     getWiredSensors()     # loops through wired sensors
    if(wetWireleess or wetWired):
        #Something is wet
        for k in range(TotalSensors):
            if (sensorInfo[k].wetStatus == WET_DOUBLECHECK):
                sensorInfo[k].wetStatus = WET_MESSAGE_SENT   # Set message for this sensor is not sent again
                msgWet = "{} has detectd water".format(sensorInfo[k].desc)
                sendSMS(msgWet)

    # Check wireless sensor temperature
    for k in range(NumWirelessSensors):
        if ( (sensorInfo[k].temperature < 50) and (sensorInfo[k].UpdateAge < 120) and (sensorInfo[k].coldMsgSent == False) ):
            msgCold = "{} temperature is {}° ".format(sensorInfo[k].desc, sensorInfo[k].temperature)
            sendSMS(msgCold)
            sensorInfo[k].coldMsgSent = True

    # check for a new day
    if(dayOfMonth != datetime.datetime.today().day):
        dayOfMonth = datetime.datetime.today().day # update dayOfMonth with the new day
        # set timer to clear all the wet status data later in the day
        resetSensorTimer = time.time() + resetSensorTimeofDay  # set timer to reset sensors

        #see if the new day is a Sunday
        if (datetime.datetime.today().weekday() == 6):
            statusReportTime = time.time() + (3600 * 12)  # send a status report of wireless sensors every Sunday at noon

    # Reset wet/dry status of all sensors.  This is done once a day
    if (time.time() > resetSensorTimer):
        resetSensorTimer += (3600 * 24)  # add 24 hours to timer.  It will actually get updated again at midnight to keep it accurate
        for k in range(TotalSensors):
            sensorInfo[k].resetWet()

    # check to see if it's time to send weekly update of wireless sensor status
    if(time.time() > statusReportTime):
        statusReportTime = time.time() + (3600 * 24 * 7)  # set to next sunday
        sendStatusReport()
        
        
    printSensorInfo()
    time.sleep(20)



