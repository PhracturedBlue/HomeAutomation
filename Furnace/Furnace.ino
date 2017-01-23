/*  This sketch measures capacitance and temperature (from a DS18B20) and reports it out
 *  Capactance is measured as charge time to 63% of Vcc, and must be less than ~5ms)
 *  Both the raw tick count as well as a % between min and max level are reported by the sensor
 *  The controller can configure the sleep time as well as the min/max times used in the percentage calculation
 *  
 *  The sketch uses a 433MHz RFM69HW connected to a MapleMini board
 */
#define MY_RADIO_RFM69
//#define MY_BAUD_RATE 9600
//#define MY_RF69_IRQ_PIN PA1
#define MY_RFM69_FREQUENCY   RF69_433MHZ
//#define MY_RF69_SPI_CS PA4
//#define MY_IS_RFM69HW 1
//#define MY_DEBUG 1

  
//#define BUTTON_PIN   PB8
#define HEAT_PIN     3

#include <MySensors.h>
#include <SPI.h>

#define FURNACE_CHILD  0
#define CONFIG_CHILD 3

#define EEPROM_LOOPTIME 0 //needs 2 bytes
#define EEPROM_GPH      2 //needs 2 bytes
#define EEPROM_ONDELAY  4 //needs 1 byte
#define EEPROM_OFFDELAY 5 //needs 1 byte
#define EEPROM_RUNTIME  6 //needs 3 bytes


MyMessage stateMsg(FURNACE_CHILD, V_STATUS);
MyMessage gallonsMsg(FURNACE_CHILD, V_VOLUME);
MyMessage runtimeMsg(FURNACE_CHILD, V_VAR1);
MyMessage loopTimeMsg(CONFIG_CHILD, V_VAR1);
MyMessage gphMsg(CONFIG_CHILD, V_VAR2);
MyMessage turnOnDelayMsg(CONFIG_CHILD, V_VAR3);
MyMessage turnOffDelayMsg(CONFIG_CHILD, V_VAR4);

//width: 60in
//height 44in
//depth  27in
#define TANK_SIZE 275

uint32_t loopTime;
uint32_t lastSend;
volatile uint32_t runTime;
uint32_t waitTime;
uint32_t onDelay;
uint32_t offDelay;
uint32_t startTime;
float    gph;
bool     forceSend;
bool update_runTime = false;

void presentation()
{
    sendSketchInfo("Oil Tank Sensor", "1.0");
    present(FURNACE_CHILD, S_GAS);
    present(CONFIG_CHILD, S_CUSTOM);
}

uint8_t updateEEPROM8(int32_t address, int32_t value)
{
   if (value < 0)
       value == 0;
   else if (value > 255)
       value = 255;
   saveState(address, value);
   return value;
}

uint16_t updateEEPROM16(int32_t address, int32_t value)
{
   if (value < 0)
       value == 0;
   else if (value > 65535L)
       value = 65535L;
   saveState(address, value >> 8);
   saveState(address+1, value & 0xff);
   return value;
}
uint32_t updateEEPROM24(int32_t address, int32_t value)
{
   if (value < 0)
       value == 0;
   else if (value > 16777215L)
       value = 16777215L;
   saveState(address, value >> 16);
   saveState(address+1, (value >> 8) & 0xff);
   saveState(address+2, value & 0xff);
   return value;
}

float getGallons(uint32_t seconds)
{
    return TANK_SIZE - (gph * seconds / 3600.0);
}

void stateChange()
{
    bool state = digitalRead(HEAT_PIN) ? false : true;
    uint32_t now = millis();
    if (state) {
        startTime = now;
    } else {
        uint32_t delta = (now + 500 - startTime) / 1000;
        if (delta > onDelay)
            runTime += delta + offDelay - onDelay;
            update_runTime = true;
    }
}

void setup()
{
    update_runTime = false;
    loopTime  = (loadState(EEPROM_LOOPTIME) << 8) | loadState(EEPROM_LOOPTIME + 1);
    if (loopTime == 65535L || loopTime == 0) {
        loopTime = updateEEPROM16(EEPROM_LOOPTIME, 20);
    }
    loopTime = loopTime * 1000L;
    uint16_t temp_gph = (loadState(EEPROM_GPH) << 8) | loadState(EEPROM_GPH + 1);
    if (temp_gph == 0 || temp_gph == 65535) {
        temp_gph = updateEEPROM16(EEPROM_GPH, 10000);
    }
    gph = temp_gph / 10000.0;
    onDelay = loadState(EEPROM_ONDELAY);
    if (onDelay == 255) {
        onDelay = updateEEPROM8(EEPROM_ONDELAY, 0);
    }
    offDelay = loadState(EEPROM_OFFDELAY);
    if (offDelay == 255) {
        offDelay = updateEEPROM8(EEPROM_OFFDELAY, 0);
    }
    runTime = ((uint32_t)loadState(EEPROM_RUNTIME) << 16) + ((uint32_t)loadState(EEPROM_RUNTIME + 1) << 8) | (uint32_t)loadState(EEPROM_RUNTIME + 2);
    if (runTime == 0 || runTime == 0xfffff) {
        runTime = updateEEPROM24(EEPROM_RUNTIME, 0); 
    }
    pinMode(HEAT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HEAT_PIN), stateChange, CHANGE);
    stateChange(); //Force initialization
    forceSend = true;
    lastSend = 0;
}

void loop()
{
    unsigned long now = millis();
    bool sendTime = now - lastSend > loopTime;
    if (sendTime || forceSend) {
        uint32_t local_runTime;
        uint32_t local_startTime;
        bool local_updateRunTime;
        bool local_state = digitalRead(3) ? false : true;
        {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE);
            local_runTime = runTime;
            local_startTime = startTime;
            local_updateRunTime = update_runTime;
            update_runTime = false;
        }
        if (local_updateRunTime) {
            updateEEPROM24(EEPROM_RUNTIME, local_runTime);                      
        }
        if (local_state) {
          uint32_t delta = (now + 500L - local_startTime) / 1000L;
          if (delta > onDelay) {
              local_runTime += delta - onDelay;
          }
        }
        send(gallonsMsg.set(getGallons(local_runTime), 3));
        send(runtimeMsg.set(local_runTime));
        send(stateMsg.set(local_state));
        if (forceSend) {
            send(loopTimeMsg.set(loopTime/1000L));
            send(gphMsg.set(gph, 3));
            send(turnOnDelayMsg.set(onDelay));
            send(turnOffDelayMsg.set(offDelay));
        }
        forceSend = false;
        lastSend = now;
    }
}

void receive(const MyMessage &message)
{
    mysensor_data type = message.type;
    //Serial.println("Got Message");
    int32_t value = atoi(message.data);
    if (message.sensor == FURNACE_CHILD) {
        if (type == V_VOLUME) {
            float gallons = atof(message.data);
            if (gallons > TANK_SIZE)
                gallons = TANK_SIZE;
            if (gallons > 0.0) {
                value = (uint32_t)((TANK_SIZE - gallons) * 3600 /gph);
                type = V_VAR1;
            }
        }
        // Fall through if V_VOLUME was set
        if (type == V_VAR1) {
            if (value >= 0) {
                value = updateEEPROM24(EEPROM_RUNTIME, value);
                {
                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE);
                    runTime = value;
                }
            }
            //value = ((uint32_t)loadState(EEPROM_RUNTIME) << 16) + ((uint32_t)loadState(EEPROM_RUNTIME + 1) << 8) | (uint32_t)loadState(EEPROM_RUNTIME + 2);
            send(runtimeMsg.set(value));          
            send(gallonsMsg.set(getGallons(value), 3));
        }
    } else if (message.sensor == CONFIG_CHILD) {
        if (message.type == V_VAR1) {
            if (value > 0)
                loopTime = (uint32_t)updateEEPROM16(EEPROM_LOOPTIME, value) * 1000;
            send(loopTimeMsg.set((uint32_t)(loopTime/1000)));
        } else if (message.type == V_VAR3) {
            if (value >= 0)
                onDelay = updateEEPROM8(EEPROM_ONDELAY, value);
            send(turnOnDelayMsg.set(onDelay));
        } else if (message.type == V_VAR4) {
            if (value >= 0)
                offDelay = updateEEPROM8(EEPROM_OFFDELAY, value);
            send(turnOffDelayMsg.set(offDelay));
        } else if (message.type == V_VAR2) {
            float v = atof(message.data);
            if (v > 0.0) {
                uint32_t v1 = updateEEPROM16(EEPROM_GPH, (uint32_t)(v * 10000.0));
                gph = v1 / 10000.0;
            }
            send(gphMsg.set(gph, 3));
        }
    }
}
