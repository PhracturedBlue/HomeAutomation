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
#define MY_TRANSPORT_WAIT_READY_MS 3000

  
//#define BUTTON_PIN   PB8
#define AUXHEAT_PIN  3   //WHITE
#define FAN_PIN      5   //GREEN
#define HEATPUMP_PIN 6   //YELLOW
#define COOL_PIN     7   //ORANGE
#define BUTTON_PIN   4

#include <MySensors.h>
#include <SPI.h>
#include <Bounce2.h>

#define FURNACE_CHILD  0
#define CONFIG_CHILD 3

#define EEPROM_LOOPTIME 0 //needs 2 bytes
#define EEPROM_GPH      2 //needs 2 bytes
#define EEPROM_ONDELAY  4 //needs 1 byte
#define EEPROM_OFFDELAY 5 //needs 1 byte
#define EEPROM_RUNTIME  6 //needs 3 bytes

#define BUTTON_HOLD_TIME 5000 //5sec

enum {
   AUXHEAT  = 0,
   FAN      = 1,
   HEATPUMP = 2,
   COOL     = 3,
};

//AC->DC conversion: must not see a high signal for this many msec to consider signal low
#define FURNACE_FREQ 100
#define MIN_COUNT    50

MyMessage auxHeatMsg (FURNACE_CHILD, V_STATUS);
MyMessage gallonsMsg (FURNACE_CHILD, V_VOLUME);
MyMessage runtimeMsg (FURNACE_CHILD, V_VAR1);
MyMessage fanMsg     (FURNACE_CHILD, V_VAR2);
MyMessage heatpumpMsg(FURNACE_CHILD, V_VAR3);
MyMessage coolMsg    (FURNACE_CHILD, V_VAR4);
MyMessage loopTimeMsg(CONFIG_CHILD, V_VAR1);
MyMessage gphMsg(CONFIG_CHILD, V_VAR2);
MyMessage turnOnDelayMsg(CONFIG_CHILD, V_VAR3);
MyMessage turnOffDelayMsg(CONFIG_CHILD, V_VAR4);

Bounce button = Bounce(); 

//width: 60in
//height 44in
//depth  27in
#define TANK_SIZE 275

uint8_t  furnace_state;
uint8_t  last_furnace_state;
uint32_t loopTime;
uint32_t lastSignal[4];
uint8_t  lastCount[4];
uint32_t lastSend;
uint32_t runTime;
uint32_t onDelay;
uint32_t offDelay;
uint32_t startTime;
float    gph;
bool     forceSend;
uint32_t last_but;
//bool update_runTime = false;

void updateFurnaceState();

void presentation()
{
    sendSketchInfo("Oil Tank Sensor", "1.1");
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
/*
void stateChange()
{
    bool state = digitalRead(HEAT_PIN) ? false : true;
    uint32_t now = millis();
    if (state) {
        startTime = now;
    } else {
        uint32_t delta = (now + 500L - startTime) / 1000L;
        if (delta > onDelay)
            runTime += delta + (uint32_t)offDelay - (uint32_t)onDelay;
            update_runTime = true;
    }
}
*/
void setup()
{
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
    pinMode(AUXHEAT_PIN,  INPUT);
    pinMode(FAN_PIN,      INPUT);
    pinMode(HEATPUMP_PIN, INPUT);
    pinMode(COOL_PIN,     INPUT);
    pinMode(BUTTON_PIN,   INPUT_PULLUP);
    button.attach(BUTTON_PIN);
    button.interval(5);
    furnace_state = 0;
    for(int i = 0; i < 4; i++) {
        lastSignal[i] = 0;
        lastCount[i]  = 0;
    }
    forceSend = true;
    lastSend = 0;
    startTime = millis();
    last_but = 0;
}

void loop()
{
    unsigned long now = millis();
    updateFurnaceState();
    button.update();
    bool sendTime = now - lastSend > loopTime;
    bool but = button.read() ? false : true;
    if ((furnace_state ^ last_furnace_state) & (1 << AUXHEAT)) {
        //auxHeat state changed
        if (furnace_state & (1 << AUXHEAT)) {
            startTime = now;
        } else {
            uint32_t delta = (now + 500L - startTime) / 1000L;
            if (delta > onDelay) {
                runTime += delta + (uint32_t)offDelay - (uint32_t)onDelay;
                updateEEPROM24(EEPROM_RUNTIME, runTime);                      
            }
        }
        forceSend = true;
        last_furnace_state = furnace_state;
    } else if (furnace_state ^ last_furnace_state) {
        forceSend = true;
        last_furnace_state = furnace_state;
    }
    if (but) {
        if(now - last_but > BUTTON_HOLD_TIME) {
            runTime = 0;  //reset tank to full
            updateEEPROM24(EEPROM_RUNTIME, runTime); 
            forceSend = true;
            last_but = now;
        }
    } else {
        last_but = now;
    }
    if (sendTime || forceSend) {
        uint32_t local_runTime = runTime;
        if (furnace_state & (1 << AUXHEAT)) {
          uint32_t delta = (now + 500L - startTime) / 1000L;
          if (delta > onDelay) {
              local_runTime += delta - onDelay;
          }
        }
        send(gallonsMsg.set (getGallons(local_runTime), 3));
        send(runtimeMsg.set (local_runTime));
        send(auxHeatMsg.set ((furnace_state & (1 << AUXHEAT))  ? 1 : 0));
        send(fanMsg.set     ((furnace_state & (1 << FAN))      ? 1 : 0));
        send(heatpumpMsg.set((furnace_state & (1 << HEATPUMP)) ? 1 : 0));
        send(coolMsg.set    ((furnace_state & (1 << COOL))     ? 1 : 0));
        
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
void updateSensor(uint8_t sensor, uint32_t now, uint8_t count) {
    uint8_t pin;
    switch (sensor) {
        case AUXHEAT:  pin = AUXHEAT_PIN;  break;
        case FAN:      pin = FAN_PIN;      break;
        case HEATPUMP: pin = HEATPUMP_PIN; break;
        case COOL:     pin = COOL_PIN;     break;
        default: return;
    }
    //char data[256];
    if (digitalRead(pin)) {
        furnace_state |= (1 << sensor);
        lastSignal[sensor] = now;
        lastCount[sensor]  = count;
        //sprintf(data,"+%d(%02x): %lu > %lu, %u > %u", sensor, furnace_state, now, lastSignal[sensor], (unsigned)count, (unsigned)lastCount[sensor]);
        //Serial.println(data);
    } else if (furnace_state & (1 << sensor)) {
        if ((uint8_t)(count - lastCount[sensor]) > MIN_COUNT) {
            lastCount[sensor]++; //Prevent wrap-around
            if (now - lastSignal[sensor] > FURNACE_FREQ) {
                furnace_state &= ~(1 << sensor);
            }
        }
        //sprintf(data,"-%d(%02x): %lu > %lu, %u > %u", sensor, furnace_state, now, lastSignal[sensor], (unsigned)count, (unsigned)lastCount[sensor]);
        //Serial.println(data);
    }
}
void updateFurnaceState()
{
    static uint8_t count = 0;
    uint32_t now = millis();
    count++;
    updateSensor(AUXHEAT,  now, count);
    updateSensor(FAN,      now, count);
    updateSensor(HEATPUMP, now, count);
    updateSensor(COOL,     now, count);
}

