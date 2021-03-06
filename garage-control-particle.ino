#include <HC-SR04.h> // 1.0.1
#include <MQTT.h> // 0.4.29
#include <OneWire.h> // 2.0.3
#include "ds18wv.h" // TODO this is not resetting the CRC flag, need to fix

#define RECONNECT 15*1000
#define LEFT_LED D0
#define RIGHT_LED D1
#define LEFT_TRIGGER A0
#define LEFT_ECHO A1
#define RIGHT_TRIGGER A2
#define RIGHT_ECHO A3
#define EEPROM_ADDRESS 0
#define EEPROM_SIGNATURE 0x63
#define EEPROM_VERSION 0x02
#define DEVICE_NAME_LENGTH 60
#define MQTT_BROKER_LENGTH 60

// for MQTT
void callback(char* topic, byte* payload, unsigned int length);

// enable system thread
SYSTEM_THREAD(ENABLED);

// enable use of external antenna in 'auto' mode (switch between both)
STARTUP(WiFi.selectAntenna(ANT_AUTO));

const int      MAXRETRY          = 4;
const uint32_t msSAMPLE_INTERVAL = 2500; //2.5 seconds
const uint32_t msMETRIC_PUBLISH  = 30000; //30 seconds
const uint32_t msDOOR_PUBLISH_INTERVAL = 300; //300ms
const int RELAY1 = D2;
const int DOOR1_UP = D4;
const int DOOR1_DOWN = D5;
const int TEMP_SENSOR = D6;
volatile bool DOOR1_UP_STATE = false;
volatile bool DOOR1_DOWN_STATE = false;
enum DoorState { door_between, door_up, door_down };
volatile DoorState DOOR1_STATE;
volatile DoorState PREVIOUS_DOOR1_STATE = door_between; // set to something != above
String mqtt_id = "photon_";
volatile bool leftParkingOccupied = false;
volatile bool rightParkingOccupied = false;
struct eepromData {
    unsigned char signature;
    unsigned char version;
    char deviceName[60];
    bool mqttEnabled;
    bool rangingEnabled;
    char mqttBroker[60];
};
eepromData savedData;

// D0 = left red led
// D1 = right red led
// D2 = relay trigger door 1
// D3 = reserved for future relay trigger
// D4 = Door 1 up
// D5 = Door 1 down
// D6 = DS18b20
// D7 = reserved for Door 2 up
// A0 = Trigger left
// A1 = echo left
// A2 = Trigger right
// A3 = echo right

// for debugging?
SerialLogHandler myLog(LOG_LEVEL_TRACE);

DS18WV sensor(TEMP_SENSOR, FALSE);
/**
 * if want to use IP address,
 * byte server[] = { XXX,XXX,XXX,XXX };
 * MQTT client(server, 1883, callback);
 * want to use domain name,
 * exp) iot.eclipse.org is Eclipse Open MQTT Broker: https://iot.eclipse.org/getting-started
 * MQTT client("iot.eclipse.org", 1883, callback);
 **/
// default to localhost, select in main setup based on eeprom data
MQTT mqttclient("localhost", 1883, callback);

HC_SR04 LeftRangefinder = HC_SR04(LEFT_TRIGGER, LEFT_ECHO);
HC_SR04 RightRangefinder = HC_SR04(RIGHT_TRIGGER, RIGHT_ECHO);

char door_stat_str[8];
char     szInfo[64];
double   celsius;
double   fahrenheit;
//float   rightDistance = 0.0;
//float   leftDistance = 0.0;
char    leftDistanceStr[8];
char    rightDistanceStr[8];
int     crcerror = 0;
uint32_t msLastMetric;
uint32_t msLastSample;
byte mac[6];
unsigned long last_temp_time = 0;
unsigned long last_door_state_time = 0;
unsigned long last_metric_publish_time = 0;
unsigned long last_mqtt_reconnect_time = 0;
unsigned long lastCloudConnect;
unsigned long lastDS18NotFoundTime = 0;

// mqtt recieve message
void callback(char* topic, byte* payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;

    if (!strncmp(topic, "garage/door/set", 15))
    {
        if (!strncmp(p, "OPEN", 4))
    	{
    		if (DOOR1_STATE == door_down)
    			toggle_door_relay("");
    	}
    	else if (!strncmp(p, "CLOSE", 5))
    	{
    		if (DOOR1_STATE != door_down)
    			toggle_door_relay("");
    	}
    }
    else if (!strncmp(topic, "garage/listen/set", 15))
    {
        // tell listen to timeout after 300 seconds (5 min) for this operation
        WiFi.setListenTimeout(300);
        WiFi.listen();
        // TOOD reset listen timeout to 0?
    }
}

/*char* mac_char() {
    int MAX_MAC_STRING_LENGTH = 12;
    char deviceMac[MAX_MAC_STRING_LENGTH];
    snprintf(deviceMac, MAX_MAC_STRING_LENGTH+1 , "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return deviceMac;
}*/

void setup() {
  Time.zone(-7);
  //getTempTimer.start();
  //publishDataTimer.start();
  //doorState.start();
  LeftRangefinder.init();
  RightRangefinder.init();
  Serial.begin(9600);
  // output MAC to serial port
  // get mac from photon
  ///WiFi.macAddress(mac);
  // convert to a string
  /*String mac_addr_string;
  char tmp[1];
  for (int i=0; i<6; i++) {
    //Serial.printf("%02x%s", mac[i], i != 5 ? ":" : "");
    sprintf(tmp, "%02x%s", mac[i], i != 5 ? ":" : "");
    mac_addr_string += tmp;
  }
  Serial.print("MAC=");
  Serial.println(mac_addr_string.c_str());*/

  pinMode(LEFT_LED, OUTPUT);
  pinMode(RIGHT_LED, OUTPUT);
  pinMode(RELAY1, OUTPUT);
  pinMode(DOOR1_UP, INPUT); // has external pulldown
  pinMode(DOOR1_DOWN, INPUT); // has external pulldown
  attachInterrupt(DOOR1_UP, door1_up, CHANGE);
  attachInterrupt(DOOR1_DOWN, door1_down, CHANGE);

  sprintf(door_stat_str, "unknown");
  // check door status right away
  Log.trace("check door state @ startup");
  door1_up();
  door1_down();
  check_door1_state(DOOR1_UP);
  check_door1_state(DOOR1_DOWN);
  //sensor.setConversionTime(750);

  // don't publish data until connected to cloud
  waitUntil(Particle.connected);
  Particle.variable("tempF", fahrenheit);
  Particle.variable("tempC", celsius);
  Particle.variable("tempCRCerr", crcerror);
  Particle.variable("doorstate", door_stat_str);
  //String lout = String::format("Inch is %.2f",leftDistance);
  //Particle.variable("lDistance", String::format("Inch is %.2f",leftDistance).c_str());
  Particle.variable("lDistance", leftDistanceStr);
  Particle.variable("rDistance", rightDistanceStr);
  //Particle.variable("rDistance", String::format("=%.2f", rightDistance));
  Particle.function("door1move", toggle_door_relay);
  Particle.function("door1crush_mode", door_crusher_mode);
  Particle.variable("mqttEnabled", savedData.mqttEnabled);
  Particle.variable("rangingEnabled", savedData.rangingEnabled);
  Particle.variable("deviceName", savedData.deviceName);
  Particle.variable("mqttBroker", savedData.mqttBroker);
  Particle.function("toggleMqtt", toggle_mqtt_support);
  Particle.function("toggleRanging", toggle_ranging_support);
  Particle.function("saveEEPROM", cloud_write_eeprom_values);
  Particle.function("updateDeviceName", cloud_update_device_name);
  Particle.function("updateMqttBroker", cloud_update_mqtt_broker);

  // eeprom check
  if (!eeprom_signature_ok())
  {
      // signature check failed, reset to default values
      // NOTE this should be adapted to confirm if failure is due to
      //      no data or actual mismatch
      initialize_eeprom();
      Log.trace("eeprom signature mismatch, init eeprom to default values");

  }
  if (!eeprom_version_match())
  {
      // version mismatch
      // TODO migrate data between versions

      Log.trace("eeprom version mismatch");
      initialize_eeprom();
  }

  read_eeprom_values();
  Log.trace("Read eeprom data sig=%x, ver=%x, name=%s, mqttEn=%i, rangingEn=%i, mqttSer=%s",
                (const char*)savedData.signature,
                (const char*)savedData.version,
                (const char*)savedData.deviceName,
                (const int*)savedData.mqttEnabled,
                (const int*)savedData.rangingEnabled,
                (const char*)savedData.mqttBroker);

  // connect to the mqtt server
  if (savedData.mqttEnabled) {
    mqttclient.setBroker(savedData.mqttBroker, 1883);
    //mqttclient.connect(String(mqtt_id + mac_addr_string));
    mqttclient.connect(String(mqtt_id + savedData.deviceName));

    // mqtt publish/subscribe
    if (mqttclient.isConnected()) {
      mqttclient.publish("garage/log", String(savedData.deviceName) + " connected to MQTT broker", TRUE);
      mqttclient.subscribe("garage/door/set");
      mqttclient.subscribe("garage/listen/set");
    }
  }

  Serial.println("Setup complete.");
}

//unsigned long lastPass = 0;
//int state = 0;
unsigned long lastDistance = 0;

void loop() {
    char buff[64];
    int len = 64;

    //if (millis() > lastPass) {
    //  digitalWrite(RIGHT_LED, (state) ? HIGH : LOW);
    //  state = !state;
    //  lastPass = millis() + 5000UL;
    //}
    // 1 sec delay between readings
    if ((millis() > lastDistance) && savedData.rangingEnabled) {

        float leftDistance = LeftRangefinder.distInch();
        //float leftDistance = 0;
        String::format("%.2f", leftDistance).toCharArray(leftDistanceStr, 8);
        float rightDistance = RightRangefinder.distInch();
        String::format("%.2f", rightDistance).toCharArray(rightDistanceStr, 8);

        if (savedData.mqttEnabled) {
            mqttclient.publish("garage/sensor/distance", String::format("{\"right\": %.2f, \"left\": %.2f}", rightDistance, leftDistance));
        }
        // check to see if distance reading indicates car in garage spot 1 or 2
        if (leftDistance <= 67 && digitalRead(LEFT_LED) == LOW)
        {
            digitalWrite(LEFT_LED, HIGH);
            leftParkingOccupied = true;
        }
        if (leftDistance > 67 && digitalRead(LEFT_LED) == HIGH)
        {
            digitalWrite(LEFT_LED, LOW);
            leftParkingOccupied = false;
        }

        if (rightDistance <= 67 && digitalRead(RIGHT_LED) == LOW)
        {
            digitalWrite(RIGHT_LED, HIGH);
            rightParkingOccupied = true;
        }
        if (rightDistance > 67 && digitalRead(RIGHT_LED) == HIGH)
        {
            digitalWrite(RIGHT_LED, LOW);
            rightParkingOccupied = false;
        }

        lastDistance = millis() + 1000UL;
    }


    // double check we are connected to the cloud + wifi
    // TODO do we need this anymore?
    if (!WiFi.ready()) {
    Log.trace("reconnecting wifi in main program loop");
        WiFi.connect();
    }
    if (!Particle.connected()) {
    Log.trace("reconnecting to particle cloud in main program loop");
        Particle.connect();
    }

    // make sure mqtt maintains connection if mqtt is enabled
    if (savedData.mqttEnabled) {
        if (mqttclient.isConnected())
        {
            mqttclient.loop();
        } else {
            // disconnected from mqtt broker
            // check wifi first
            /*if (!WiFi.ready()) {
                if (millis() - lastCloudConnect > RECONNECT) {   // try not to stress the network
                    lastCloudConnect = millis();
                    WiFi.off();                 // Restart the WiFi part
                    delay(1000);
                    WiFi.on();
                    delay(500);
                    WiFi.connect();
                }
            }*/

            // try to connect if it's been 5 seconds since our last
            // connection attempt
            if (WiFi.ready()) {
                if (millis() - last_mqtt_reconnect_time > 5000) {
                    last_mqtt_reconnect_time = millis();
                    /*String mac_addr_string;
                    char tmp[1];
                    for (int i=0; i<6; i++) {
                      //Serial.printf("%02x%s", mac[i], i != 5 ? ":" : "");
                      sprintf(tmp, "%02x%s", mac[i], i != 5 ? ":" : "");
                      mac_addr_string += tmp;
                    }
                    mqttclient.connect(String(mqtt_id + mac_addr_string));*/
                    mqttclient.connect(String(mqtt_id + savedData.deviceName));
                    Log.trace("reconnecting to MQTT broker");
                    // make sure we are subscribed to set topic on broker after reconnect
                    mqttclient.subscribe("garage/door/set");
                    mqttclient.publish("garage/log","Reconnected to MQTT broker and re-subscribed to set topic", TRUE);
                }
            }
        }
    }

    //Particle.process(); // seems to prevent loss of connection
    // remove delay for 2.5 seconds and switch to checking current time compared to last loop
    if (millis()-last_temp_time >= msSAMPLE_INTERVAL) {
        getTemp();
        // now reset last_run_time
        last_temp_time = millis();
    }

    if (millis()-last_door_state_time >= msDOOR_PUBLISH_INTERVAL) {
        publishDoorState();
        last_door_state_time = millis();
    }

    if (millis()-last_metric_publish_time >= msMETRIC_PUBLISH) {
        publishData();
        last_metric_publish_time = millis();
    }
}

void getTemp() {
    // Read the next available 1-Wire temperature sensor
    //if (sensor.read(TEMP_SENSOR_ADDR)) {
    if (sensor.read()) {
        // Do something cool with the temperature
        fahrenheit = sensor.fahrenheit();
        celsius = sensor.celsius();
        Serial.printf("Temperature %.2f C %.2f F ", celsius, fahrenheit);
        //Particle.publish("temperature", String(sensor.celsius()), PRIVATE);

        // Additional info useful while debugging
        printDebugInfo();

        // If sensor.read() didn't return true you can try again later
        // This next block helps debug what's wrong.
        // It's not needed for the sensor to work properly
        Serial.println();
    /*} else {
        // CRC error?
        if (sensor.crcError()) {
            crcerror++;
            sensor.wireReset();
        }*/
    } else {
        // Once all sensors have been read you'll get searchDone() == true
        // Next time read() is called the first sensor is read again
        if (sensor.searchDone()) {
            // only print to serial if longer than 250ms to
            // avoid excess printing with no sensors connected
            if (millis()-lastDS18NotFoundTime > 250) {
                Serial.println("No more addresses.");
                lastDS18NotFoundTime = millis();
            }
        } else {
            // Something went wrong
            printDebugInfo();
        }
        Serial.println();
    }
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
  //Particle.publish("DStype", type, PRIVATE);

  // Print the ROM (sensor type and unique ID)
  uint8_t addr[8];
  sensor.addr(addr);
  /*String x;
  x.format( " ROM=%02X%02X%02X%02X%02X%02X%02X%02X",
    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]
      );
  //Particle.publish("DSaddr", x, PRIVATE);
  Serial.print(x.c_str());*/
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
  Serial.print(" CRC=");
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
    {
        DOOR1_DOWN_STATE = true
    }
    else
    {
        DOOR1_DOWN_STATE = false;
    }
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

// setup timer for door relay, turn off after 500ms, only run once
Timer doorRelayOffTimer(500, door_relay_off, true);

int toggle_door_relay(String command) {
    // TODO handle different door #s
    // command not used
    digitalWrite(RELAY1, HIGH);
    // start off timer
    doorRelayOffTimer.start();

    // TODO get rid of this delay and turn into some kind ofcallback after a timer expires?
    //delay(500);
    //digitalWrite(RELAY1, LOW);
    return 1;
}

// setup timer for crusher mode relay, turn off after 15s, only run once
Timer doorCrushModeRelayOffTimer(15000, door_relay_off, true);

int door_crusher_mode(String command) {
    // Force lower door
    digitalWrite(RELAY1, HIGH);

    doorCrushModeRelayOffTimer.start();

    return 1;
}

// Write whatever is appropriate for "off" on the door relay
void door_relay_off() {
    digitalWrite(RELAY1, LOW);
    return;
}

void publishDoorState() {
    // check door state
    //Log.trace("publishDoorState function called from timer");
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
        // change has occured so push out one time update to mqtt of door state
        // tell MQTT to retain last value
        if (savedData.mqttEnabled) {
            mqttclient.publish("garage/door/state", door_stat_str, TRUE);
        }
    }
}

void publishData() {
    //Log.trace("publishData function called from timer");
    sprintf(szInfo, "%2.2f", fahrenheit);
    Particle.publish("dsTmp", szInfo, PRIVATE);
     // mqtt publish/subscribe
    if (mqttclient.isConnected() && savedData.mqttEnabled) {
        mqttclient.publish("garage/sensor/temperature", szInfo);
        // also push out garage door state with retain flag
        mqttclient.publish("garage/door/state", door_stat_str, TRUE);
        // push spot occupied metric
        // NOTE we are already doing this once every second in the main loop
        /*if (savedData.rangingEnabled)
        {
            mqttclient.publish("garage/sensor/parking", String::format("{\"left\": %d, \"right\": %d}", leftParkingOccupied, rightParkingOccupied));
        }*/
    }
}

int initialize_eeprom() {
    // wipe eeprom contents and set to default values
    eepromData data;
    // default values below
    data.signature = EEPROM_SIGNATURE;
    data.version = EEPROM_VERSION;
    strncpy(data.deviceName, "UNKNOWN", 8);
    data.mqttEnabled = FALSE;
    data.rangingEnabled = FALSE;
    strncpy(data.mqttBroker, "127.0.0.1", 10); // added in 63, 02
    EEPROM.put(EEPROM_ADDRESS, data);
    return 1;
}

int eeprom_version_match() {
    // check to see if current eeprom version matches to saved values
    eepromData data;
    EEPROM.get(EEPROM_ADDRESS, data);
    if (data.version == EEPROM_VERSION)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int eeprom_signature_ok() {
    // validate the signature is ok in the eeprom data struct
    eepromData data;
    EEPROM.get(EEPROM_ADDRESS, data);
    if (data.signature == EEPROM_SIGNATURE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void read_eeprom_values() {
    // read data from eeprom into global
    EEPROM.get(EEPROM_ADDRESS, savedData);
    return;
}

void write_eeprom_values() {
    // save data from global to eeprom
    EEPROM.put(EEPROM_ADDRESS, savedData);
}

int toggle_mqtt_support(String arg)
{
    if (savedData.mqttEnabled)
    {
        savedData.mqttEnabled = 0;
    }
    else
    {
        savedData.mqttEnabled = 1;
    }

    return 1;
}

int toggle_ranging_support(String arg)
{
    if (savedData.rangingEnabled)
    {
        savedData.rangingEnabled = 0;
    }
    else
    {
        savedData.rangingEnabled = 1;
    }

    return 1;
}

int cloud_write_eeprom_values(String arg)
{
    write_eeprom_values();

    return 1;
}

int cloud_update_device_name(String arg)
{
    // validate length of new device name
    // note need to account for null at end of string
    int arg_length = arg.length();
    if (arg_length != 0 && arg_length > 58)
    {
        Log.trace("invalid argument to cloud update device name");
        return -1;
    }

    // looks like we are ok, copy data over
    arg.toCharArray(savedData.deviceName, DEVICE_NAME_LENGTH);

    return 1;
}

int cloud_update_mqtt_broker(String arg)
{
    int arg_length = arg.length();
    if (arg_length != 0 && arg_length > 58)
    {
        Log.trace("invalid argument to cloud update mqtt broker");
        return -1;
    }

    // looks like we are ok, copy data over
    arg.toCharArray(savedData.mqttBroker, MQTT_BROKER_LENGTH);

    // switch broker here
    mqttclient.setBroker(savedData.mqttBroker, 1883);

    return 1;
}
