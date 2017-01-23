
#define DIGITAL_INPUT_SENSOR 3  // The digital input you attached your light sensor.  (Only 2 and 3 generates interrupt!)
#define PULSE_FACTOR 1000       // Nummber of blinks per KWH of your meeter
#define SLEEP_MODE false        // Watt-value can only be reported when sleep mode is false.
#define MAX_WATT 10000          // Max watt value to report. This filetrs outliers.

//#define USE_SERIAL
#define MY_RADIO_RFM69
#define MY_RFM69_FREQUENCY   RF69_433MHZ
//#define MY_BAUD_RATE 9600
//#define MY_RF69_IRQ_PIN PA1
//#define MY_RF69_SPI_CS PA4
//#define MY_IS_RFM69HW 1
//#define MY_DEBUG 1

#include <MySensors.h>
#include <util/atomic.h>

#define POWER_CHILD  1              // Id of the sensor child
#define CONFIG_CHILD 3

#define EEPROM_LOOPTIME 0 //needs 2 bytes

double ppwh = ((double)PULSE_FACTOR)/1000; // Pulses per watt hour
bool pcReceived = false;
volatile unsigned long pulseCount = 0;
volatile unsigned long lastBlink = 0;
volatile unsigned long interval = 0;
volatile unsigned long sumInterval = 0;
volatile unsigned long runningCount = 0;
volatile bool doReset = true;
unsigned long oldPulseCount = 0;
unsigned long lastSend;
bool forceSend = true;
uint32_t loopTime;

MyMessage wattMsg(POWER_CHILD,V_WATT);
MyMessage kwhMsg(POWER_CHILD,V_KWH);
MyMessage pcMsg(POWER_CHILD,V_VAR1);
MyMessage loopTimeMsg(CONFIG_CHILD, V_VAR1);

uint16_t updateEEPROM(int address, int value)
{
   if (value < 0)
       value == 0;
   else if (value > 65535)
       value = 65535;
   saveState(address, value >> 8);
   saveState(address+1, value & 0xff);
   return value;
}

void setup()
{
    loopTime  = (loadState(EEPROM_LOOPTIME) << 8) | loadState(EEPROM_LOOPTIME + 1);
    if (loopTime == 65535 || loopTime == 0)
        loopTime = updateEEPROM(EEPROM_LOOPTIME, 20);
    loopTime = loopTime * 1000;

    // Fetch last known pulse count value from gw
    //request(CHILD_ID, V_VAR1);

    // Use the internal pullup to be able to hook up this sketch directly to an energy meter with S0 output
    // If no pullup is used, the reported usage will be too high because of the floating pin
    pinMode(DIGITAL_INPUT_SENSOR,INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(DIGITAL_INPUT_SENSOR), onPulse, RISING);
    lastSend=0;
}

void presentation()
{
    // Send the sketch version information to the gateway and Controller
    sendSketchInfo("Energy Meter", "1.0");

    // Register this device as power sensor
    present(POWER_CHILD, S_POWER);
    present(CONFIG_CHILD, S_CUSTOM);
}

void loop()
{
    unsigned long now = millis();
    // Only send values at a maximum frequency or woken up from sleep
    bool sendTime = now - lastSend > loopTime;
    if (sendTime || forceSend) {
        unsigned long localInterval;
        unsigned long localSumInterval;
        unsigned long localRunningCount;
        unsigned long localPulseCount;
        unsigned long watt;
        {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE);
            localSumInterval = sumInterval;
            localInterval = interval;
            localRunningCount = runningCount;
            localPulseCount = pulseCount;
            doReset = true;
        }
        if (localPulseCount == oldPulseCount || localRunningCount == 0) {
            watt = 0;
        } else {
            //watt = (3600000000.0 /localInterval) / ppwh;
            watt = (3600000000.0 / (localSumInterval / localRunningCount)) / ppwh;
        }

        // New watt value has been calculated
        {
            // Check that we dont get unresonable large watt value.
            // could hapen when long wraps or false interrupt triggered
            if (watt<((unsigned long)MAX_WATT)) {
                send(wattMsg.set(watt));  // Send watt value to gw
            }
            Serial.print("Watt:");
            Serial.println(watt);
        }

        // Pulse cout has changed
        {
            double kwh = ((double)localPulseCount/((double)PULSE_FACTOR));
            send(pcMsg.set(localPulseCount));  // Send pulse count value to gw
            send(kwhMsg.set(kwh, 4));  // Send kwh value to gw
            oldPulseCount = localPulseCount;
        }
        if (forceSend)
            send(loopTimeMsg.set(loopTime/1000));
        lastSend = now;
        forceSend = false;
    }
}

void receive(const MyMessage &message)
{
    if (message.sensor == POWER_CHILD && message.type==V_VAR1) {
        pulseCount = oldPulseCount = message.getLong();
        Serial.print("Received last pulse count from gw:");
        Serial.println(pulseCount);
        pcReceived = true;
    }
    if (message.sensor == CONFIG_CHILD) {
        if (message.type == V_VAR1) {
            unsigned long value = atoi(message.data);
            loopTime = (unsigned long)updateEEPROM(EEPROM_LOOPTIME, value) * 1000;
            send(loopTimeMsg.set(loopTime/1000));
        }
    }
    forceSend = true;
}

void onPulse()
{
    if (!SLEEP_MODE) {
        unsigned long newBlink = micros();
        if (doReset) {
            runningCount = 0;
            sumInterval = 0;
            doReset = false;
        }
        interval = newBlink-lastBlink;
        if (interval<10000L) { // Sometimes we get interrupt on RISING
            return;
        }
        sumInterval += interval;
        runningCount++;
        lastBlink = newBlink;
    }
    pulseCount++;
}

