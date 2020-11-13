# Read sensor data from panStamp via I2C
# Code is on GitHub: https://github.com/Scott216/Leak_Detector
# I think default RPi i2c bus speed is too fast for Arduino/panStamp.
# Edited /boot/config.txt and added:  dtparam=i2c_arm_baudrate=10000

# To Do:
# Add code to reset panStamp. When you do, sleep(10) so panStamp has time to reboot and you don't get in Remote I/O error on RPi
# See if you can use Try/Exception with I2C


# Change Log
# 11/05/20  v1.00 - Initial version after moving off Arduino.  Everything seems to be working, haven't added e-ink code yet
# 11/10/20  v1.01 - Added code for e-ink display. Replaced RPi.GPIO with circuit python modules 
# 11/12/20  v1.02 - This version should be good 

# For RPi pinout see pinout.txt

import smbus  # Used for I2C
import time
import datetime
import private_credentials  # Holds Twilio info
from twilio.rest import Client  # https://pypi.org/project/twilio
from twilio.base.exceptions import TwilioRestException
from adafruit_epd.ssd1675 import Adafruit_SSD1675   #https://github.com/adafruit/Adafruit_CircuitPython_SSD1675
from adafruit_epd.epd import Adafruit_EPD    #https://github.com/adafruit/Adafruit_EPD  https://circuitpython.readthedocs.io/projects/epd/en/latest/api.html
from PIL import Image, ImageDraw, ImageFont
import digitalio   # https://circuitpython.readthedocs.io/en/5.3.x/shared-bindings/digitalio/__init__.html
import busio       # https://circuitpython.readthedocs.io/en/latest/shared-bindings/busio/
import board       # https://circuitpython.readthedocs.io/en/5.3.x/shared-bindings/board/__init__.html



i2c_address = 0x15  # 21 decimal
data_len = 10       # I2C data packet is 10 bytes

NumWirelessSensors =  4  # max is 8
NumWiredSensors    = 10

resetSensorTimeofDay = 3600 * 8  # 8:00 AM - number of seconds after midnight to reset all the sensors' wet/dry status
doubleCheckDelay =  120   # seconds to wait after a sensor turn wet to double check it again

# wetStatus values
DRY =              0
WET_FROM_SENSOR =  1  # Status if sensor is wet, but hasn't had time for double check deley
WET_DOUBLECHECK =  2  # Status if sensor is still wet after a couple minute delay
WET_MESSAGE_SENT = 3  # Status if sensor is wet and SMS message has been sent, this prevents message from going out again

# Initiliaze I2C bus
i2cbus = smbus.SMBus(1)


#------------------------------------------------------------------
class clsLeakSensor:
    def __init__(self, ID, desc, isWet, timeWet, wetStatus, isWireless, IO_Pin, UpdateAge, temperature, battery, rssi, coldMsgSent, IO):
        self.ID          = ID          # ID for wireless sensor, set on sensor with rotary swithc.  Max is 8
        self.desc        = desc        # Text location for sensor
        self.isWet       = isWet       # True if sensor detected water
        self.timeWet     = timeWet     # Timestamp of when water was first detected
        self.wetStatus   = wetStatus   # 0=dry, 1=initial wet from sensor, 2=still wet after couple min double check 3 = SMS Sent
        self.isWireless  = isWireless  # True if sensor is wireless
        self.IO_Pin      = IO_Pin      # Digital input for wired sensors
        self.UpdateAge   = UpdateAge   # number of seconds since last update from wireless sensor (max 255)
        self.temperature = temperature # Temperatue for wireless sensor
        self.battery     = battery     # Battery voltage in mV for wireless sensor
        self.rssi        = rssi        # Signal strength for wireless sensor
        self.coldMsgSent = coldMsgSent # One-shot flag set to True if message was sent about cold wireless sensor
        # Setup input pins for wired sensors
        if (self.isWireless == False):
            self.IO = digitalio.DigitalInOut(IO_Pin)
            self.IO.switch_to_input(digitalio.Pull.DOWN)

    def resetWet(self):   # Reset once a day
        self.isWet = False
        self.timeWet = 0
        self.wetStatus = DRY
        self.coldMsgSent = False
        
#------------------------------------------------------------------

lowTempSetting = 50
lowVoltSetting = 2700


nopin = 0
wired = False
Wireless = True
time1 = 0
IOThing = 0 # place holder in below array for IO_Pin object in digitalio.DigitalInOut
age = 255 # set default for UpdateAge (seconds since last received data from wireless sensor) to 255 which represents sensor is offline

# create array for sensors using class clsLeakSensor and fill in defaults
wireless_desc = ["Master Bath", "Guest Bath", "1st Fl Bath Wireless", "Wireless 4", "Wireless 5", "Wireless 6", "Wireless 7", "Wireless 8"]
sensorInfo = []
for cc in range(NumWirelessSensors):
    sensorInfo.append( clsLeakSensor(cc, wireless_desc[cc],                   DRY, time1, DRY, Wireless,  nopin, age, 0, 0, 0, False, IOThing) )

sensorInfo.append( clsLeakSensor(NumWirelessSensors + 0, "1st Fl Bath Wired", DRY, time1, DRY, wired,   board.D25, 0, 0, 0, 0, False, IOThing) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 1, "Washing Machine",   DRY, time1, DRY, wired,   board.D7,  0, 0, 0, 0, False, IOThing) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 2, "Dehimidifier",      DRY, time1, DRY, wired,   board.D12, 0, 0, 0, 0, False, IOThing) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 3, "Hot Water Tank",    DRY, time1, DRY, wired,   board.D13, 0, 0, 0, 0, False, IOThing) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 4, "Boiler",            DRY, time1, DRY, wired,   board.D19, 0, 0, 0, 0, False, IOThing) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 5, "Kitchen Frig",      DRY, time1, DRY, wired,   board.D16, 0, 0, 0, 0, False, IOThing) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 6, "Dishwasher",        DRY, time1, DRY, wired,   board.D26, 0, 0, 0, 0, False, IOThing) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 7, "Kitchen Sink",      DRY, time1, DRY, wired,   board.D20, 0, 0, 0, 0, False, IOThing) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 8, "Hot Tub",           DRY, time1, DRY, wired,   board.D21, 0, 0, 0, 0, False, IOThing) )
sensorInfo.append( clsLeakSensor(NumWirelessSensors + 9, "Wired 10",          DRY, time1, DRY, wired,   board.D4,  0, 0, 0, 0, False, IOThing) )



#------------------------------------------------------------------
# Get data from wireless sensors
# Returns List
#   0: Wet True/False
#   1: Offline True/False
#   2: Sensor ID of wireless sensor that went offline
#------------------------------------------------------------------
def getWirelessSensors():

    wirelessStatus = [False, False, 0]  # list to be returned by function

    # Loop to request wireless sensors status from panStamp
    for wirelessID in range(NumWirelessSensors):
        oldUpdateAge = sensorInfo[wirelessID].UpdateAge  # used to set flag if sensor has transitioned to offline
        
        I2Cpacket = i2cbus.read_i2c_block_data(i2c_address, wirelessID, data_len) # Request data from panStamp via I2C

        if (I2Cpacket[0] == wirelessID):  # panStamp returns 255 for sensorID if invalid sensor request is made
            sensorInfo[wirelessID].ID           = I2Cpacket[0]
            sensorInfo[wirelessID].UpdateAge    = I2Cpacket[1]       # secibds since data was last received for wireless sensor
            sensorInfo[wirelessID].isWet        = I2Cpacket[2]
            sensorInfo[wirelessID].temperature  = I2Cpacket[3] << 8
            sensorInfo[wirelessID].temperature |= I2Cpacket[4]
            sensorInfo[wirelessID].battery      = I2Cpacket[5] << 8
            sensorInfo[wirelessID].battery     |= I2Cpacket[6]
            sensorInfo[wirelessID].rssi         = I2Cpacket[7] * -1  # convert RSSI back to a negative number


        # See if sensor just went offline.  Only returns ID for one sensor.  This assumes multiple sensors will NOT go offline at the same time
        if ( (sensorInfo[wirelessID].UpdateAge == 255) and (oldUpdateAge < 255) ):
            wirelessStatus[1] = True
            wirelessStatus[2] = wirelessID

        # If offline, clear sensor info
        if (sensorInfo[wirelessID].UpdateAge == 255):
            sensorInfo[wirelessID].isWet = 0
            sensorInfo[wirelessID].temperature = 0
            sensorInfo[wirelessID].battery = 0
            sensorInfo[wirelessID].rssi = 0

        # If sensor is online, then run code to check for wet
        if (sensorInfo[wirelessID].UpdateAge < 255):
                
            # If sensor just turned wet, then update timeWet and wetStatus
            if ( (sensorInfo[wirelessID].isWet == True) and (sensorInfo[wirelessID].wetStatus == DRY) ):
                sensorInfo[wirelessID].timeWet = time.time()
                sensorInfo[wirelessID].wetStatus = WET_FROM_SENSOR
                

            # If sensor is wet, then after a few minutes, double check again
            # isWet would have been updated again just now by I2Cpacket[2]
            if ( (sensorInfo[wirelessID].isWet == True) and 
                 (time.time() > sensorInfo[wirelessID].timeWet + doubleCheckDelay) and
                 (sensorInfo[wirelessID].wetStatus == WET_FROM_SENSOR) ):
                
                sensorInfo[wirelessID].wetStatus = WET_DOUBLECHECK
                wirelessStatus[0] = True

            
            if( (sensorInfo[wirelessID].wetStatus ==   WET_DOUBLECHECK) or
                (sensorInfo[wirelessID].wetStatus ==  WET_MESSAGE_SENT) ):
                
                wirelessStatus[0] = True

    return wirelessStatus


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
            firstReading = sensorInfo[wiredID].IO.value
            time.sleep(0.01)
            secondReading = sensorInfo[wiredID].IO.value
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
        if ( (sensorInfo[wiredID].isWet == True) and
             (time.time() > sensorInfo[wiredID].timeWet + doubleCheckDelay) and
             (sensorInfo[wiredID].wetStatus == WET_FROM_SENSOR) ):

            sensorInfo[wiredID].wetStatus = WET_DOUBLECHECK
            somethingIsWetFlag = True

         # If sensor has been wet for longer then double check delay, set flag   
        if( sensorInfo[wiredID].wetStatus >= WET_DOUBLECHECK ):
            somethingIsWetFlag = True

    return somethingIsWetFlag


#------------------------------------------------------------------
# Print Sensor Info
#------------------------------------------------------------------
def printSensorInfo():

    print(time.strftime("%m/%d/%Y %I:%M:%S %p"))
    for k in range(TotalSensors):
        if (sensorInfo[k].isWireless == True):
            # Wireless sensors
            print("ID:{:2.0f}\tWet:{},{}\t  Age:{:3.0f}\t Temp:{:2.0f}\tmV:{:4.0f}\t RSSI:{:3.0f}   {}".format(       \
                                              sensorInfo[k].ID, sensorInfo[k].isWet, sensorInfo[k].wetStatus, sensorInfo[k].UpdateAge,       \
                                              sensorInfo[k].temperature, sensorInfo[k].battery, sensorInfo[k].rssi, \
                                              sensorInfo[k].desc))
        else:
            #Wired sensors
            print("ID:{:2.0f}\tWet:{:.0f},{}\t  Age:{}\t Temp:{}\tmV:{}\t RSSI:{}   {}".format(   \
                                              sensorInfo[k].ID, sensorInfo[k].isWet, sensorInfo[k].wetStatus, "   ",  \
                                              "  ", "    ", "   ", sensorInfo[k].desc))
    print("-----------------------------------------------------------------------------")


#------------------------------------------------------------------
# Send SMS via Twilio
#------------------------------------------------------------------
def sendSMS(sms_msg):
    try:
        smsclient = Client(private_credentials.TWILIO_ACCOUNT_SID, private_credentials.TWILIO_AUTH_TOKEN)
        message = smsclient.messages.create(body=sms_msg, from_=private_credentials.TWILIO_PHONE, to=private_credentials.TO_PHONE)
    except TwilioRestException as smserr:
        print("Twilio SMS failed: {}".format(smserr))

    print("SMS Message sent: {}".format(sms_msg))
    time.sleep(3)  # In case there are more than 1 message close together, don't send to quickly


#------------------------------------------------------------------
# Send weekly status report with status of wireless sensors
# ID, desc, temp and voltage
#------------------------------------------------------------------
def sendStatusReport():

    statusMsg = "" 
    for wirelessID in range (NumWirelessSensors):
        if (sensorInfo[wirelessID].UpdateAge < 255):
            statusMsg = "{}{} {}°, {:.2f}V\n".format(statusMsg, sensorInfo[wirelessID].desc, sensorInfo[wirelessID].temperature, sensorInfo[wirelessID].battery/1000 )
        else:
            statusMsg = "{}{} is offline\n".format(statusMsg, sensorInfo[wirelessID].desc)

    sendSMS(statusMsg)

 
#------------------------------------------------------------------
# e-ink display updaate
# Display can show 8 lines of text for font 16
# Arguments:
#  screen:
#     1: Everything looks good message
#     2: Pushbutton message + warnings
#     3: Display list of warning  
#------------------------------------------------------------------
def einkMessage(screen):

    image = Image.new("RGB", (display.width, display.height), color=WHITE)
    draw = ImageDraw.Draw(image)

    if(screen == 1):
        draw.text([20,50], "Everything looks good!", font=font16B, fill=BLACK)
        display.image(image)
        display.display()
        
    if(screen == 2):
        draw.text([15,17], "Push button for update >", font=font16B, fill=BLACK)
        draw.text([ 1,70], "There are sensor warnings", font=font16B, fill=BLACK)
        display.image(image)
        display.display()

    # Show all warnings
    if(screen == 3):
        numAlerts = 0
        alertMsg = []
        
        # Cycle through wireless sensors and see what needs to be displayed
        for k in range (NumWirelessSensors):
            if ( (sensorInfo[k].wetStatus > 1) and (sensorInfo[k].UpdateAge < 255) ):
                alertMsg.append("{} is wet".format(sensorInfo[k].desc))
                numAlerts += 1
            if ( (sensorInfo[k].temperature < lowTempSetting) and (sensorInfo[k].UpdateAge < 255) ):
                alertMsg.append("{} {}°F".format(sensorInfo[k].desc, sensorInfo[k].temperature))
                numAlerts += 1
            if ( (sensorInfo[k].battery < lowVoltSetting) and (sensorInfo[k].UpdateAge < 255) ):
                alertMsg.append("{} {:0.2F}V".format(sensorInfo[k].desc, sensorInfo[k].battery/1000))
                numAlerts += 1
            if ( sensorInfo[k].UpdateAge == 255 ):
                alertMsg.append("{} offline".format(sensorInfo[k].desc))
                numAlerts += 1
                
        # Cycle through wired sensors and see what needs to be displayed
        for k in range (NumWiredSensors):
            if (sensorInfo[NumWirelessSensors + k].wetStatus > 1 ):
                alertMsg.append("{} is wet".format(sensorInfo[NumWirelessSensors + k].desc))
                numAlerts += 1

        linesPerScreen = 7
        numScreens = int(round(numAlerts/linesPerScreen + 0.5, 0))  # number of screens needed to display all the messages
        msgNumCnt = 0  # counts number of messages displayed so far
        for i in range(numScreens):
            image = Image.new("RGB", (display.width, display.height), color=WHITE)
            draw = ImageDraw.Draw(image)

            for rowNum in range (linesPerScreen):
                if (msgNumCnt < numAlerts):
                    textPos = [1, 15 * rowNum]
                    draw.text(textPos, alertMsg[msgNumCnt], font=font16B, fill=BLACK)
                    msgNumCnt += 1
            if (i <= numScreens - 1):
                draw.text([1, 105], "  ----- {} of {} screens -----".format(i+1, numScreens), font=font16B, fill=BLACK)
                
            display.image(image)
            display.display()
            
            # If more then 1 screen, wait before updating to next screen
            nextScreenTimer = time.time() + 15
            while(time.time() < nextScreenTimer):
                # If either button is pressed, go to next screen right away
                if( (top_button.value == False) or (bot_button.value == False) ):
                    time.sleep(debounce)  # debounce
                    nextScreenTimer = 0


#------------------------------------------------------------------
# Setup
#------------------------------------------------------------------
TotalSensors = NumWirelessSensors + NumWiredSensors

dayOfMonth = datetime.datetime.today().day         # Used in new day trigger
resetSensorTimer = time.time() + (3600 * 24)       # This will be properly set at midnight
statusReportTimer = time.time() + (3600 * 24 * 7)  # This will be properly set at Sunday at midnight

# variables for eink display
spi =  busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
ecs =  digitalio.DigitalInOut(board.CE0)
dc =   digitalio.DigitalInOut(board.D22)
rst =  digitalio.DigitalInOut(board.D27)
busy = digitalio.DigitalInOut(board.D17)
top_button = digitalio.DigitalInOut(board.D5)   # Top button on e-ink PC board
top_button.switch_to_input()
bot_button = digitalio.DigitalInOut(board.D6)   # Bottom button on e-ink PC board
bot_button.switch_to_input()


# Initialize e-ink display
display = Adafruit_SSD1675(122, 250, spi, cs_pin=ecs, dc_pin=dc, sramcs_pin=None, rst_pin=rst, busy_pin=busy,)
display.rotation = 3  # 1 will flip 180 degrees
WHITE = (255,255,255)
BLACK = (0,0,0)

font16 =  ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16)
font16B = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)

printStatusTimer = time.time() + 5  # Prints status info 

displayOnTimer = 0  # Timer to turn on e-ink display after top button is pressed
activeMsg = False # True if any sensor has a warning Wet, cold, low battery
einkMessage(1) # Displayes default screen

#------------------------------------------------------------------
# Main Program
#------------------------------------------------------------------
while True:

    debounce = 0.1 # E-ink PCB button debounce time

    wirelessResult = getWirelessSensors()  # Loops through wireless sensors, returns list [0:wet T/F, 1:offline T/F, 2:offline TxID]
    wiredResult =    getWiredSensors()     # Loops through wired sensors, returns True if anything is wet
    
    if(wirelessResult[0] or wiredResult):
        # Something is wet, loop through sensors to see which ones.
        # Only send message if sensor has been double checked and message not already sent
        for k in range(TotalSensors):
            if (sensorInfo[k].wetStatus == WET_DOUBLECHECK):
                msgWet = "{} has detected water".format(sensorInfo[k].desc)
                sendSMS(msgWet)
                sensorInfo[k].wetStatus = WET_MESSAGE_SENT   # Set status so message isn't sent again

    # Check if sensor just went offline
    if(wirelessResult[1]):
        msgOffline = "{} just went offline".format(sensorInfo[wirelessResult[2]].desc)
        sendSMS(msgOffline)
        
    # Check wireless sensor temperatures
    coldAlert = False
    for k in range(NumWirelessSensors):
        if ( (sensorInfo[k].temperature < lowTempSetting) and (sensorInfo[k].UpdateAge < 255)):
            coldAlert = True
            if (sensorInfo[k].coldMsgSent == False):
                msgCold = "{} temperature is {}° ".format(sensorInfo[k].desc, sensorInfo[k].temperature)
                sendSMS(msgCold)
                sensorInfo[k].coldMsgSent = True

    # Check wireless sensor battery voltages
    batteryAlert = False
    for k in range(NumWirelessSensors):
        if ( (sensorInfo[k].battery < lowVoltSetting) and (sensorInfo[k].UpdateAge < 255)):
            batteryAlert = True

    # Check for a new day
    if(dayOfMonth != datetime.datetime.today().day):
        dayOfMonth = datetime.datetime.today().day # update dayOfMonth with the new day
        # Set timer to clear all the wet status data later in the day
        resetSensorTimer = time.time() + resetSensorTimeofDay  # set timer to reset sensors - 8:00 AM

        # See if the new day is a Sunday
        if (datetime.datetime.today().weekday() == 6):
            statusReportTimer = time.time() + (3600 * 12)  # send a status report of wireless sensors every Sunday at noon

    # Reset wet/dry status of all sensors.  This is done once a day
    if (time.time() > resetSensorTimer):
        resetSensorTimer += (3600 * 24)  # add 24 hours to timer.  It will actually get updated again at midnight to keep it accurate
        for k in range(TotalSensors):
            sensorInfo[k].resetWet()

    # Check to see if it's time to send weekly update of wireless sensor status
    if(time.time() > statusReportTimer):
        statusReportTimer = time.time() + (3600 * 24 * 7)  # set to next Sunday
        sendStatusReport()

    # Print sensor info
    if (time.time() > printStatusTimer):
        printSensorInfo()
        printStatusTimer = time.time() + 30

    # Transition from no warning to at least one, update display standyby screen
    if ((wirelessResult[0] or wiredResult or coldAlert or batteryAlert) and (activeMsg == False)):
        # Went from No active messages to active messages
        activeMsg = True
        einkMessage(2)

    # All warning have cleared, update display standby screen
    if (wirelessResult[0] == False and
              wiredResult == False and
                coldAlert == False and
             batteryAlert == False and
               activeMsg == True):
        # Went from No active messages to active messages
        activeMsg = False
        einkMessage(1)
 
    # Show status on display
    if( (top_button.value == False) and (activeMsg == True) ):
        displayOnTimer = time.time() + 60  # turn on message screen for 60 seconds
        time.sleep(debounce)  
        einkMessage(3)  # Display will cycle through sensors and display and warning

    # After display has been on for enough time to read messages, revert back to standby screen
    if ( (displayOnTimer > 0) and (time.time() > displayOnTimer) ):
        displayOnTimer = 0 # Set to zero so this screen is not called over and over
        if (activeMsg == True):
            einkMessage(2)  # Display reverts back to standby screen with warning
        else:
            einkMessage(1)  # Display reverts back to standby screen no warning
            
    
