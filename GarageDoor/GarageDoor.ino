/*  This sketch measures capacitance and temperature (from a DS18B20) and reports it out
 *  Capactance is measured as charge time to 63% of Vcc, and must be less than ~5ms)
 *  Both the raw tick count as well as a % between min and max level are reported by the sensor
 *  The controller can configure the sleep time as well as the min/max times used in the percentage calculation
 *  
 *  The sketch uses a 433MHz RFM69HW connected to a MapleMini board
 */
#define MY_RADIO_RFM69
#define MY_BAUD_RATE 9600
#define MY_RF69_IRQ_PIN PA1
#define MY_RFM69_FREQUENCY   RF69_433MHZ
#define MY_RF69_SPI_CS PA4
//#define MY_IS_RFM69HW 1
#define MY_DEBUG 1

  
//#define BUTTON_PIN   PB8
#define DOOR1_PIN    PB5  //17
#define DOOR2_PIN    PB3  //19
#define RELAY1_PIN   PB4  //18
#define RELAY2_PIN   PA15 //20
#define ONE_WIRE_BUS PA14 //21

#include <MySensors.h>
#include <SPI.h>

#include <OneWireSTM.h>
#include <DallasTemperatureSTM.h>

// Data wire is plugged into port 2 on the Arduino

#define DOOR1_CHILD  0
#define DOOR2_CHILD  1
#define TEMP_CHILD   2
#define CONFIG_CHILD 3

#define EEPROM_LOOPTIME 0 //needs 2 bytes
#define EEPROM_WAITTIME 2 //needs 2 bytes

enum {
  CLOSE = 0,
  OPEN = 1,
};
MyMessage door1Msg(DOOR1_CHILD, V_STATUS);
MyMessage door2Msg(DOOR2_CHILD, V_STATUS);
MyMessage loopTimeMsg(CONFIG_CHILD, V_VAR1);
MyMessage waitTimeMsg(CONFIG_CHILD, V_VAR2);
MyMessage tempMsg(TEMP_CHILD, V_TEMP);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);


uint32_t loopTime;
uint32_t lastChange;
uint32_t waitTime;

void wait(const uint8_t interrupt1, const uint8_t mode1,
          const uint8_t interrupt2, const uint8_t mode2,
          const uint32_t waitingMS)
{
    const uint32_t enteringMS = hwMillis();
    noInterrupts();
    _wakeUp1Interrupt  = interrupt1;
    _wakeUp2Interrupt  = interrupt2;
    attachInterrupt(interrupt1, wakeUp1, (ExtIntTriggerMode)mode1);
    attachInterrupt(interrupt2, wakeUp2, (ExtIntTriggerMode)mode2);
    interrupts();
    while (! interruptWakeUp() && hwMillis() - enteringMS < waitingMS) {
        _process();
    }
    _wokeUpByInterrupt = INVALID_INTERRUPT_NUM;
}

void presentation()
{
    sendSketchInfo("Garage Door Sensor", "1.0");
    present(DOOR1_CHILD, S_BINARY);
    present(DOOR2_CHILD, S_BINARY);
    present(TEMP_CHILD, S_TEMP);
    present(CONFIG_CHILD, S_CUSTOM);
}
int updateEEPROM(int address, int value)
{
   if (value < 0)
       value == 0;
   else if (value > 65535)
       value = 65535;
   saveState(address, value >> 8);
   saveState(address+1, value & 0xff);
   return value;
}

void closeDoor(int door)
{
    //Serial.print("Closing ");
    //Serial.println(door == RELAY1_PIN ? "Door1" : "Door2");
    digitalWrite(door, LOW);
    wait(500);
    digitalWrite(door, HIGH);
    lastChange = millis();
}

void setup()
{
    lastChange = 0;
    loopTime  = (loadState(EEPROM_LOOPTIME) << 8) | loadState(EEPROM_LOOPTIME + 1);
    if (loopTime == 65535 || loopTime == 0)
        loopTime = updateEEPROM(EEPROM_LOOPTIME, 10);
    loopTime = loopTime * 1000;
    waitTime  = (loadState(EEPROM_WAITTIME) << 8) | loadState(EEPROM_WAITTIME + 1);
    if (waitTime == 65535 || waitTime == 0)
        waitTime = updateEEPROM(EEPROM_WAITTIME, 10);
    waitTime = waitTime * 1000;
    pinMode(RELAY1_PIN, OUTPUT);
    digitalWrite(RELAY1_PIN, HIGH);
    pinMode(RELAY2_PIN, OUTPUT);
    digitalWrite(RELAY2_PIN, HIGH);
    pinMode(DOOR1_PIN, INPUT_PULLUP);
    pinMode(DOOR2_PIN, INPUT_PULLUP);
    sensors.begin();
}

void loop()
{
    static int count = 0;
    //Signal loop start
    for (int i = 0; i < 6; i++) {
      digitalWrite(PB1, i & 0x01 ? LOW : HIGH);
      wait(100);
    }
    sensors.requestTemperatures();
    float temp = sensors.getTempFByIndex(0);
    bool door1 = digitalRead(DOOR1_PIN);
    bool door2 = digitalRead(DOOR2_PIN);
    
    //Serial.print("Door1: ");
    //Serial.print(door1 ? "Open" : "Closed");
    //Serial.print(" Door2: ");
    //Serial.print(door2 ? "Open" : "Closed");
    //Serial.print(" Temp: ");
    //Serial.println(temp);
    send(door1Msg.set(door1));
    send(door2Msg.set(door2));
    send(tempMsg.set(temp, 2));
    if(count == 0) {
        send(loopTimeMsg.set(loopTime/1000));
        send(waitTimeMsg.set(waitTime/1000));
    }
    count = (count + 1) % 10;
    //wait(DOOR1_PIN, CHANGE, DOOR2_PIN, CHANGE, loopTime);
    sleep(DOOR1_PIN, CHANGE, DOOR2_PIN, CHANGE, loopTime, true);
}

void receive(const MyMessage &message)
{
    //Serial.println("Got Message");
    if (message.sensor == CONFIG_CHILD) {
        if (message.type == V_VAR1) {
            int value = atoi(message.data);
            loopTime = updateEEPROM(EEPROM_LOOPTIME, value) * 1000;
            send(loopTimeMsg.set(loopTime/1000));
        }
        if (message.type == V_VAR2) {
            int value = atoi(message.data);
            waitTime = updateEEPROM(EEPROM_WAITTIME, value) * 1000;
            send(waitTimeMsg.set(waitTime/1000));
        }
    } else if (message.sensor == DOOR1_CHILD && message.type == V_STATUS) {
        int value = atoi(message.data);
        if (value == CLOSE && digitalRead(DOOR1_PIN) == OPEN && millis() - lastChange > waitTime) {
            closeDoor(RELAY1_PIN);
            send(door1Msg.set((uint8_t)CLOSE));
        }
    } else if (message.sensor == DOOR2_CHILD && message.type == V_STATUS) {
        int value = atoi(message.data);
        if (value == CLOSE && digitalRead(DOOR2_PIN) == OPEN && millis() - lastChange > waitTime) {
            closeDoor(RELAY2_PIN);
            send(door2Msg.set((uint8_t)CLOSE));
        }
    }
}
