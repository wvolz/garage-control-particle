//#include <DS18B20.h>
//#include <math.h>
// include DS18B20 0.1.6 library
#include "DS18.h"

const int      MAXRETRY          = 4;
const uint32_t msSAMPLE_INTERVAL = 2500;
const uint32_t msMETRIC_PUBLISH  = 30000;
const uint32_t msDOOR_PUBLISH_INTERVAL = 300;
const int RELAY1 = D2;
const int DOOR1_UP = D4;
const int DOOR1_DOWN = D5;
const int TEMP_SENSOR = D6;
volatile bool DOOR1_UP_STATE = false;
volatile bool DOOR1_DOWN_STATE = false;
enum DoorState { door_between, door_up, door_down };
volatile DoorState DOOR1_STATE;
volatile DoorState PREVIOUS_DOOR1_STATE; // set to something != above

// D0 = unused
// D1 = unused
// D2 = relay trigger door 1
// D3 = reserved for future relay trigger
// D4 = Door 1 up
// D5 = Door 1 down
// D6 = DS18b20
// D7 = reserved for Door 2 up

// for debugging?
SerialLogHandler myLog(LOG_LEVEL_TRACE);

DS18 sensor(TEMP_SENSOR);

Timer getTempTimer(msSAMPLE_INTERVAL, getTemp); // getTemp every msSAMPLE_INTERVAL
Timer doorState(msDOOR_PUBLISH_INTERVAL, publishDoorState); // get door state every msDOOR_SAMPLE_INTERVAL
Timer publishDataTimer(msMETRIC_PUBLISH, publishData); // publishData every msMETRIC_PUBLISH

char door_stat_str[8];
char     szInfo[64];
double   celsius;
double   fahrenheit;
uint32_t msLastMetric;
uint32_t msLastSample;

void setup() {
  Time.zone(-5);
  Particle.variable("temp", fahrenheit);
  Particle.variable("doorstate", door_stat_str);
  Particle.variable("door1down", DOOR1_DOWN_STATE);
  Particle.variable("door1up", DOOR1_UP_STATE);
  getTempTimer.start();
  publishDataTimer.start();
  doorState.start();
  Serial.begin(115200);
  pinMode(RELAY1, OUTPUT);
  pinMode(DOOR1_UP, INPUT); // has external pulldown
  pinMode(DOOR1_DOWN, INPUT); // has external pulldown
  attachInterrupt(DOOR1_UP, door1_up, CHANGE);
  attachInterrupt(DOOR1_DOWN, door1_down, CHANGE);
  Particle.function("door1move", toggle_door_relay);
  Serial.println("Setup complete.");
  sprintf(door_stat_str, "unknown");
  // check door status right away
  check_door1_state(DOOR1_UP);
  check_door1_state(DOOR1_DOWN);
}

void loop() {
    //getTemp();
    Particle.process(); // seems to prevent loss of connection
}

void getTemp() {
    // Read the next available 1-Wire temperature sensor
    if (sensor.read()) {
        // Do something cool with the temperature
        Serial.printf("Temperature %.2f C %.2f F ", sensor.celsius(), sensor.fahrenheit());
        //Particle.publish("temperature", String(sensor.celsius()), PRIVATE);
        fahrenheit = sensor.fahrenheit();

        // Additional info useful while debugging
        printDebugInfo();

        // If sensor.read() didn't return true you can try again later
        // This next block helps debug what's wrong.
        // It's not needed for the sensor to work properly
    } else {
        // Once all sensors have been read you'll get searchDone() == true
        // Next time read() is called the first sensor is read again
        if (sensor.searchDone()) {
            Serial.println("No more addresses.");
            // Avoid excessive printing when no sensors are connected
            delay(250);
            // Something went wrong
        } else {
            printDebugInfo();
        }
    }
    Serial.println();
}

void printDebugInfo() {
  // If there's an electrical error on the 1-Wire bus you'll get a CRC error
  // Just ignore the temperature measurement and try again
  if (sensor.crcError()) {
    Serial.print("CRC Error ");
  }

  // Print the sensor type
  const char *type;
  switch(sensor.type()) {
    case WIRE_DS1820: type = "DS1820"; break;
    case WIRE_DS18B20: type = "DS18B20"; break;
    case WIRE_DS1822: type = "DS1822"; break;
    case WIRE_DS2438: type = "DS2438"; break;
    default: type = "UNKNOWN"; break;
  }
  Serial.print(type);

  // Print the ROM (sensor type and unique ID)
  uint8_t addr[8];
  sensor.addr(addr);
  Serial.printf(
    " ROM=%02X%02X%02X%02X%02X%02X%02X%02X",
    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]
  );

  // Print the raw sensor data
  uint8_t data[9];
  sensor.data(data);
  Serial.printf(
    " data=%02X%02X%02X%02X%02X%02X%02X%02X%02X",
    data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]
  );
}

void door1_up() {
    int val = 0;
    val = digitalRead(DOOR1_UP);
    if (val == HIGH)
        DOOR1_UP_STATE = true;
    else
        DOOR1_UP_STATE = false;
    check_door1_state(DOOR1_UP);
    Serial.println("Door up function");
}

void door1_down() {
    int val = 0;
    val = digitalRead(DOOR1_DOWN);
    if (val == HIGH)
        DOOR1_DOWN_STATE = true;
    else
        DOOR1_DOWN_STATE = false;
    check_door1_state(DOOR1_DOWN);
    Serial.println("Door down function");
}

void check_door1_state(int pin) {
    // determines state based on pins
    if (pin == DOOR1_DOWN) {
        if (DOOR1_UP_STATE == false) {
            if (DOOR1_DOWN_STATE == true) DOOR1_STATE = door_down;
            if (DOOR1_DOWN_STATE == false) DOOR1_STATE = door_between;
        }
    }
    if (pin == DOOR1_UP) {
        if (DOOR1_DOWN_STATE == false) {
            if (DOOR1_UP_STATE == true) DOOR1_STATE = door_up;
            if (DOOR1_UP_STATE == false) DOOR1_STATE = door_between;
        }
    }
    Serial.println(DOOR1_STATE);
}

int toggle_door_relay(String command) {
    // TODO handle different door #s
    // command not used
    digitalWrite(RELAY1, HIGH);
    delay(500);
    digitalWrite(RELAY1, LOW);
    return 1;
}

void publishDoorState() {
    // check door state
    if (PREVIOUS_DOOR1_STATE != DOOR1_STATE)
    {
        //String door_stat_str;
        String tmpstr;
        switch (DOOR1_STATE)
        {
            case door_between: {
                tmpstr = "between";
                break;
            }
            case door_up: {
                tmpstr = "up";
                break;
            }
            case door_down: {
                tmpstr = "down";
                break;
            }
        }
        tmpstr.toCharArray(door_stat_str, 8);
        PREVIOUS_DOOR1_STATE = DOOR1_STATE;
        Serial.print("Door ");
        Serial.println(door_stat_str);
        //Particle.publish("door1state", door_stat_str, PRIVATE);
    }
}

void publishData() {
    sprintf(szInfo, "%2.2f", fahrenheit);
    Particle.publish("dsTmp", szInfo, PRIVATE);
}
/*void publishData(){
  if(!ds18b20.crcCheck()){      //make sure the value is correct
    return;
  }
  sprintf(szInfo, "%2.2f", fahrenheit);
  Particle.publish("dsTmp", szInfo, PRIVATE);
  msLastMetric = millis();
}*/
