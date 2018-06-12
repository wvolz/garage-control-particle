// This #include statement was automatically added by the Particle IDE.
#include "ds18wv.h"

// This #include statement was automatically added by the Particle IDE.
#include <OneWire.h>
//#include "ds18wv.h" // TODO this is not resetting the CRC flag, need to fix

// enable system thread
//SYSTEM_THREAD(ENABLED);

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
volatile DoorState PREVIOUS_DOOR1_STATE = door_between; // set to something != above


//uint8_t TEMP_SENSOR_ADDR[8] = {0x28,0x87,0x31,0x52,0x00,0x00,0x00,0xE7}; // adjust for whatever is on the bus ds18b20
uint8_t TEMP_SENSOR_ADDR[8] = {0x10,0xF9,0xCB,0x21,0x00,0x08,0x00,0xC4}; // adjust for whatever is on the bus ds18s20


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

DS18WV sensor(TEMP_SENSOR);

//Timer getTempTimer(msSAMPLE_INTERVAL, getTemp); // getTemp every msSAMPLE_INTERVAL
//Timer doorState(msDOOR_PUBLISH_INTERVAL, publishDoorState); // get door state every msDOOR_SAMPLE_INTERVAL
//Timer publishDataTimer(msMETRIC_PUBLISH, publishData); // publishData every msMETRIC_PUBLISH

char door_stat_str[8];
char     szInfo[64];
double   celsius;
double   fahrenheit;
uint32_t msLastMetric;
uint32_t msLastSample;
byte mac[6];
unsigned long last_temp_time = 0;
unsigned long last_door_state_time = 0;
unsigned long last_metric_publish_time = 0;

void setup() {
  Time.zone(-7);
  Particle.variable("temp", fahrenheit);
  Particle.variable("doorstate", door_stat_str);
  Particle.variable("door1down", DOOR1_DOWN_STATE);
  Particle.variable("door1up", DOOR1_UP_STATE);
  //getTempTimer.start();
  //publishDataTimer.start();
  //doorState.start();
  Serial.begin(9600);
  // output MAC to serial port
  WiFi.macAddress(mac);
  for (int i=0; i<6; i++) {
    Serial.printf("%02x%s", mac[i], i != 5 ? ":" : "");
  }
  
  pinMode(RELAY1, OUTPUT);
  pinMode(DOOR1_UP, INPUT); // has external pulldown
  pinMode(DOOR1_DOWN, INPUT); // has external pulldown
  attachInterrupt(DOOR1_UP, door1_up, CHANGE);
  attachInterrupt(DOOR1_DOWN, door1_down, CHANGE);
  Particle.function("door1move", toggle_door_relay);
  Serial.println("Setup complete.");
  sprintf(door_stat_str, "unknown");
  // check door status right away
  Log.trace("check door state @ startup");
  door1_up();
  door1_down();
  check_door1_state(DOOR1_UP);
  check_door1_state(DOOR1_DOWN);
}

void loop() {
    char buff[64];
    int len = 64;
    unsigned long current_time_ms = millis();
    
    // double check we are connected to the cloud + wifi
    // TODO do we need this anymore?
    if (!WiFi.ready()) 
        WiFi.connect();
    if (!Particle.connected()) 
        Particle.connect();
    
    //Particle.process(); // seems to prevent loss of connection
    // remove delay for 2.5 seconds and switch to checking current time compared to last loop
    if (current_time_ms-last_temp_time >= msSAMPLE_INTERVAL) {
        getTemp();
        // now reset last_run_time
        last_temp_time = current_time_ms;
    }
	
	if (current_time_ms-last_door_state_time >= msDOOR_PUBLISH_INTERVAL) {
		publishDoorState();
		last_door_state_time = current_time_ms;
	}
	
	if (current_time_ms-last_metric_publish_time >= msMETRIC_PUBLISH) {
		publishData();
		last_metric_publish_time = current_time_ms;
	}
}

void getTemp() {
    // Read the next available 1-Wire temperature sensor
    if (sensor.read(TEMP_SENSOR_ADDR)) {
    //if (sensor.read()) {
        // Do something cool with the temperature
        Serial.printf("Temperature %.2f C %.2f F ", sensor.celsius(), sensor.fahrenheit());
        //Particle.publish("temperature", String(sensor.celsius()), PRIVATE);
        fahrenheit = sensor.fahrenheit();

        // Additional info useful while debugging
        printDebugInfo();

        // If sensor.read() didn't return true you can try again later
        // This next block helps debug what's wrong.
        // It's not needed for the sensor to work properly
        Serial.println();
    } /*else {
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
        Serial.println();
    }*/
}

void printDebugInfo() {
  // If there's an electrical error on the 1-Wire bus you'll get a CRC error
  // Just ignore the temperature measurement and try again
  if (sensor.crcError()) {
    Serial.print("CRC Error ");
    Serial.print(sensor.crcError());
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
  Serial.print("CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
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
