//#define MY_DEBUG 1
#define MY_NODE_ID 5
#define MY_RADIO_RFM69
#define MY_RFM69_FREQUENCY   RF69_433MHZ
//#define MY_BAUD_RATE 9600
//#define MY_RF69_IRQ_PIN PA1
//#define MY_RF69_SPI_CS PA4
#define MY_IS_RFM69HW 1

#include <MySensors.h>
#include <util/atomic.h>
#include <DS1302.h>
#include <OneWire.h>


#define IR_SENSOR_PIN 3         // The digital input you attached your light sensor.  (Only 2 and 3 generates interrupt!)
#define ONE_WIRE_BUS  4
#define RTC_SCK       5
#define RTC_IO        6
#define RTC_CE        7        

#define PULSE_FACTOR 1000       // Nummber of blinks per KWH of your meeter
#define SLEEP_MODE   false      // Watt-value can only be reported when sleep mode is false.
#define MAX_WATT     15000      // Max watt value to report. This filetrs outliers.

#define POWER_CHILD  1          // Power child-sensor-id
#define TEMP_CHILD   2
#define CONFIG_CHILD 3          // Configuration child-sensor-id

#define EEPROM_LOOPTIME 0   //needs 2 bytes
#define EEPROM_HISTORY  194 //needs 62 bytes

#define RAM_DAY          0
#define RAM_PULSE_POS    1
#define RAM_POS1         2  //4 bytes
#define RAM_POS2         6  //4 bytes


extern void init_ds18b20();
extern float readTemperature();

double ppwh = ((double)PULSE_FACTOR)/1000; // Pulses per watt hour
volatile unsigned long pulseCount = 0;
volatile unsigned long lastBlink = 0;
volatile unsigned long interval = 0;
volatile unsigned long sumInterval = 0;
volatile unsigned long runningCount = 0;
volatile bool newPulse = false;
volatile bool doReset = true;
unsigned long oldPulseCount = 0;
unsigned long lastSend;
bool forceSend = true;
uint32_t loopTime;
unsigned int currentDay;


DS1302 rtc(RTC_CE, RTC_IO, RTC_SCK);
OneWire oneWire(ONE_WIRE_BUS); 

MyMessage wattMsg(POWER_CHILD,V_WATT);
MyMessage kwhMsg(POWER_CHILD,V_KWH);
MyMessage pcMsg(POWER_CHILD,V_VAR1);
MyMessage kwhHistoryMsg(POWER_CHILD,V_VAR2);

MyMessage tempMsg(TEMP_CHILD, V_TEMP);

MyMessage loopTimeMsg(CONFIG_CHILD, V_VAR1);
MyMessage rtcTimeMsg(CONFIG_CHILD, V_VAR2);

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

void read_kwh()
{
    unsigned long pulses;
    int pos = rtc.readRam(RAM_PULSE_POS);
    if (pos) {
        pulses = (rtc.readRam(RAM_POS2+0) << 24) |
                 (rtc.readRam(RAM_POS2+1) << 16) |
                 (rtc.readRam(RAM_POS2+2) <<  8) |
                 (rtc.readRam(RAM_POS2+3) <<  0);
    } else {
        pulses = (rtc.readRam(RAM_POS1+0) << 24) |
                 (rtc.readRam(RAM_POS1+1) << 16) |
                 (rtc.readRam(RAM_POS1+2) <<  8) |
                 (rtc.readRam(RAM_POS1+3) <<  0);
    }
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE);
        pulseCount = pulses;
    }
}
void write_kwh(unsigned long pulses)
{
    int pos = rtc.readRam(RAM_PULSE_POS);
    if (pos) {
        rtc.writeRam(RAM_POS1+0, (pulses >> 24) & 0xff);
        rtc.writeRam(RAM_POS1+1, (pulses >> 16) & 0xff);
        rtc.writeRam(RAM_POS1+2, (pulses >>  8) & 0xff);
        rtc.writeRam(RAM_POS1+3, (pulses >>  0) & 0xff);
        rtc.writeRam(RAM_PULSE_POS, 0);
    } else {
        rtc.writeRam(RAM_POS2+0, (pulses >> 24) & 0xff);
        rtc.writeRam(RAM_POS2+1, (pulses >> 16) & 0xff);
        rtc.writeRam(RAM_POS2+2, (pulses >>  8) & 0xff);
        rtc.writeRam(RAM_POS2+3, (pulses >>  0) & 0xff);
        rtc.writeRam(RAM_PULSE_POS, 1);
    }
}

void reset_kwh(bool save)
{
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE);
        oldPulseCount = pulseCount;
        pulseCount = 0;
    }
    Time t = rtc.time();
    uint8_t date = t.date;
    updateEEPROM(EEPROM_HISTORY + (date - 1)*2, pulseCount / 2);
    currentDay = t.day;
    oldPulseCount = 0;
    write_kwh(0);
    rtc.writeRam(RAM_DAY, currentDay);
    forceSend = true;
}
void setup()
{
    loopTime  = (loadState(EEPROM_LOOPTIME) << 8) | loadState(EEPROM_LOOPTIME + 1);
    if (loopTime == 65535 || loopTime == 0)
        loopTime = updateEEPROM(EEPROM_LOOPTIME, 20);
    loopTime = loopTime * 1000;

    // Fetch last known pulse count value from gw
    //request(CHILD_ID, V_VAR1);

    init_ds18b20();

    rtc.writeProtect(false);
    rtc.halt(false);
    currentDay = rtc.readRam(RAM_DAY);
    Time t = rtc.time();
    if (currentDay != t.day) {
        reset_kwh(true);
    } else {
        read_kwh();
    }
    // Use the internal pullup to be able to hook up this sketch directly to an energy meter with S0 output
    // If no pullup is used, the reported usage will be too high because of the floating pin
    pinMode(IR_SENSOR_PIN,INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), onPulse, RISING);
    lastSend=0;
}

void presentation()
{
    // Send the sketch version information to the gateway and Controller
    sendSketchInfo("Energy Meter2", "1.0");

    // Register this device as power sensor
    present(POWER_CHILD, S_POWER);
    present(TEMP_CHILD, S_TEMP);
    present(CONFIG_CHILD, S_CUSTOM);
}

void loop()
{
    unsigned long now = millis();
    // Only send values at a maximum frequency or woken up from sleep
    bool sendTime = now - lastSend > loopTime;
    unsigned long localPulseCount;
    if (newPulse) {
        {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE);
            localPulseCount = pulseCount;
            newPulse = false;
        }
        write_kwh(localPulseCount);
    }
    Time t = rtc.time();
    if (currentDay != t.day) {
        reset_kwh(true);
    }
    if (sendTime || forceSend) {
        unsigned long localInterval;
        unsigned long localSumInterval;
        unsigned long localRunningCount;
        unsigned long watt;
        {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE);
            localSumInterval = sumInterval;
            localInterval = interval;
            localRunningCount = runningCount;
            localPulseCount = pulseCount;
            doReset = true;
        }
        float temp = readTemperature();
        Serial.print("Temperature: ");
        Serial.println(temp);

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
            send(tempMsg.set(temp, 2));

        }
        if (forceSend)
            send(loopTimeMsg.set(loopTime/1000));
        lastSend = now;
        forceSend = false;
    }
}

void receive(const MyMessage &message)
{
    if (message.sensor == POWER_CHILD) {
        if (message.type==V_VAR1) {
            // Update/ report pulse-count
            long pc = message.getLong();
            if (pc >= 0) {
                oldPulseCount = pc;
                {
                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE);
                    pulseCount = oldPulseCount;
                }
                write_kwh(oldPulseCount);
                Serial.print("Received last pulse count from gw:");
            } else {
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE);
                pc = oldPulseCount;
            }
            send(pcMsg.set(pc));
            Serial.println(pc);
        }
        if (message.type==V_VAR2) {
            // Read historical kwH
            uint8_t date = atoi(message.data);
            if (date < 1 || date > 31)
                return;
            uint8_t offset = EEPROM_HISTORY + (date - 1) * 2;
            uint16_t pc = ((loadState(offset) << 8) | loadState(offset + 1));
            double kwh = ((double)pc) * 2/((double)PULSE_FACTOR);
            send(kwhHistoryMsg.set(kwh, 4));  // Send historical kwh value to gw
        }
    }
    if (message.sensor == CONFIG_CHILD) {
        if (message.type == V_VAR1) {
            unsigned long value = atoi(message.data);
            loopTime = (unsigned long)updateEEPROM(EEPROM_LOOPTIME, value) * 1000;
            send(loopTimeMsg.set(loopTime/1000));
        }
        if (message.type == V_VAR2) {
            int i = 0;
            int val[7];
            char* str = strtok(message.data, ",");
            while (str != 0 && i < 7) {
                Serial.println(str);
                val[i++] = atoi(str);
                str = strtok(0, ",");
            }
            if (i != 7) {
                Serial.print("Invalid time received: (");
                Serial.print(i);
                Serial.print(") ");
                Serial.println(message.data);
            } else {
                Time t(val[0]+2000, val[1], val[2], val[3], val[4], val[5], val[6]);
                rtc.time(t);
            }
            Time t = rtc.time();
            char tmp[20];
            sprintf(tmp, "%4d-%02d%-02d %02d:%02d:%02d (%d)", t.yr, t.mon, t.date, t.hr, t.min, t.sec, t.day);
            Serial.println(tmp);
            send(rtcTimeMsg.set(tmp));
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
        newPulse = true;
    }
    pulseCount++;
}

