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
#define MY_IS_RFM69HW 1
// #define MY_DEBUG 1

#define MEASURE_PIN PA8
#define ONE_WIRE_BUS PA14

#include <MySensors.h>
#include <SPI.h>

#include <OneWireSTM.h>
#include <DallasTemperatureSTM.h>

// Data wire is plugged into port 2 on the Arduino

#define WATER_CHILD 0
#define TEMP_CHILD  1
#define CONFIG_CHILD 2

#define EEPROM_LEVEL0   0 //needs 2 bytes
#define EEPROM_LEVEL100 2 //needs 2 bytes
#define EEPROM_LOOPTIME 4 //needs 2 bytes

MyMessage waterLvlMsg(WATER_CHILD, V_VOLUME);
MyMessage waterRawMsg(WATER_CHILD, V_VAR1);
MyMessage waterMaxMsg(WATER_CHILD, V_VAR2);
MyMessage waterMinMsg(WATER_CHILD, V_VAR3);
MyMessage loopTimeMsg(CONFIG_CHILD, V_VAR1);
MyMessage tempMsg(TEMP_CHILD, V_TEMP);

HardwareTimer adc_timer(3);
HardwareTimer counter(1);
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);


bool heartbeat = true;
bool doneCounting;
uint32_t level_0, level_100, loopTime;

void presentation()
{
    sendSketchInfo("Water Level Sensor", "1.0");
    present(WATER_CHILD, S_WATER);
    present(TEMP_CHILD, S_TEMP);
    present(CONFIG_CHILD, S_CUSTOM);
    heartbeat = true;
}
int updateEEPROM(int address, int value)
{
   if (value < 0)
       value == 0;
   else if (value > 65535)
       value = 65535;
   saveState(address, value >> 8);
   saveState(address+1, value & 0xff);
   heartbeat = true;
   return value;
}

void setup()
{
    level_0   = (loadState(EEPROM_LEVEL0) << 8)   | loadState(EEPROM_LEVEL0 + 1);
    level_100 = (loadState(EEPROM_LEVEL100) << 8) | loadState(EEPROM_LEVEL100 + 1);
    loopTime  = (loadState(EEPROM_LOOPTIME) << 8) | loadState(EEPROM_LOOPTIME + 1);
    if (level_100 == 0) {
        level_100 = updateEEPROM(EEPROM_LEVEL100, 65535);
    }
    if (level_0 >= level_100) {
        level_0 = updateEEPROM(EEPROM_LEVEL0, 0);
    }
    if (loopTime == 65535)
        loopTime = updateEEPROM(EEPROM_LOOPTIME, 30);
    loopTime = loopTime * 1000;

    //pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
    adc_timer.pause();
    adc_timer.setPeriod(1000000UL);
    adc_timer.refresh();

    // Uses pin PA8
    TIMER1->regs.gen->CCMR1 |= 0x0001; // Ch. 1 as TI1
    TIMER1->regs.gen->SMCR |= 0x0007; // Ext. clk mode 1
    TIMER1->regs.gen->SMCR |= 0x0050; // TI1FP1 is the trigger
    TIMER1->regs.gen->CR1 |= 0x0001; // enable counting
    counter.pause();
    counter.setMode(2, TIMER_OUTPUT_COMPARE);
    counter.setCompare(2, 1000);
    counter.setCount(0);
    counter.attachInterrupt(2, countInt);

    sensors.begin();
}

void countInt()
{
  adc_timer.pause();
  counter.pause();
  doneCounting = true;
}

void loop()
{
    static int count = 0;
    static float lastTemp = 0.0;
    static int lastWaterLevel = 0;

    if ((count % 10) == 0) {
        heartbeat = true;
    }
    count = (count + 1) % 100;
    //Signal loop start
    for (int i = 0; i < 6; i++) {
      digitalWrite(PB1, i & 0x01 ? LOW : HIGH);
      wait(100);
    }
    sensors.requestTemperatures();
    float temp = sensors.getTempFByIndex(0);

    // Read puse count to get cap value
    adc_timer.setCount(0);
    counter.setCount(0);
    doneCounting = false;
    adc_timer.resume();
    counter.resume();
    while (!doneCounting && adc_timer.getCount() < adc_timer.getOverflow()) {
    }
    adc_timer.pause();
    counter.pause();

    uint32_t delta_t = adc_timer.getCount();
    int32_t waterlevel = (10000 * (delta_t - level_0)) / level_100;
    float f_waterLevel = waterlevel / 100.0F;
    //delta_t = current raw level
    //temp = current temperature
    //level_100 = 100% value
    //level_0   = 0% value
    //f_waterlevel = water level %
    Serial.print("level: ");
    Serial.print(f_waterLevel);
    Serial.print(" raw: ");
    Serial.print(delta_t);
    Serial.print(" Temp: ");
    Serial.println(temp);
    if (heartbeat || waterlevel != lastWaterLevel) {
        send(waterLvlMsg.set(f_waterLevel, 2));
        send(waterRawMsg.set(delta_t));
        lastWaterLevel = waterlevel;
    }
    if (heartbeat || temp != lastTemp) {
        send(tempMsg.set(temp, 2));
        lastTemp = temp;
    }
    if (heartbeat) {
        send(waterMaxMsg.set(level_100));
        send(waterMinMsg.set(level_0));
        send(loopTimeMsg.set(loopTime/1000));
    }
    heartbeat = false;
    sleep(loopTime,true);
}

void receive(const MyMessage &message)
{
    if (message.sensor == WATER_CHILD) {
        if (message.type == V_VAR2) {
            int value = atoi(message.data);
            level_100 = updateEEPROM(EEPROM_LEVEL100, value);
            send(waterMaxMsg.set(level_100));
        }
        if (message.type == V_VAR3) {
            int value = atoi(message.data);
            level_0 = updateEEPROM(EEPROM_LEVEL0, value);
            send(waterMinMsg.set(level_0));
        }
    } else if (message.sensor == CONFIG_CHILD) {
        if (message.type == V_VAR1) {
            int value = atoi(message.data);
            loopTime = updateEEPROM(EEPROM_LOOPTIME, value) * 1000;
            send(loopTimeMsg.set(loopTime/1000));
        }
    }
}
