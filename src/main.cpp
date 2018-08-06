/*
  Libraries :
    - ESP8266 core for Arduino :  https://github.com/esp8266/Arduino
    - ESP8266httpUpdate:          auto update esp8266 sketch
    - PubSubClient:               https://github.com/knolleary/pubsubclient

  Schematic Sonoff:
    - VCC (Sonoff) -> VCC (FTDI)
    - RX  (Sonoff) -> TX  (FTDI)
    - TX  (Sonoff) -> RX  (FTDI)
    - GND (Sonoff) -> GND (FTDI)
*/

#include <ESP8266WiFi.h>        // https://github.com/esp8266/Arduino
#include <ESP8266httpUpdate.h>  // ota update for esp8266
#include <PubSubClient.h>       // https://github.com/knolleary/pubsubclient/releases/tag/v2.6
#include "config.h"

// macros for debugging
#ifdef DEBUG
  #define         DEBUG_PRINT(x)    Serial.print(x)
  #define         DEBUG_PRINTLN(x)  Serial.println(x)
#else
  #define         DEBUG_PRINT(x)
  #define         DEBUG_PRINTLN(x)
#endif

// Board properties
#define           FW_VERSION                "esp-bell V1.3"

// MQTT
#define           MQTT_ON_PAYLOAD           "1"
#define           MQTT_OFF_PAYLOAD          "0"
#define           MQTT_ENDPOINT_SIZE        30

#define           MQTT_TOPIC_BASE           ""
#define           MQTT_TOPIC_SYSTEM_VERSION "system/version"
#define           MQTT_TOPIC_SYSTEM_UPDATE  "system/update"
#define           MQTT_TOPIC_SYSTEM_RESET   "system/reset"
#define           MQTT_TOPIC_SYSTEM_ONLINE  "system/online"

#define           MQTT_TOPIC_BELL           "bell/state"
#define           MQTT_TOPIC_DOOR_OPENER    "door/switch"
#define           MQTT_TOPIC_DOOR_REED      "door/state"

#define           DEBOUNCE_DELAY             50    // the debounce time; increase if the output flickers

char              MQTT_CLIENT_ID[7]                               = {0};
char              MQTT_ENDPOINT_SYS_VERSION[MQTT_ENDPOINT_SIZE]   = {0};
char              MQTT_ENDPOINT_SYS_UPDATE[MQTT_ENDPOINT_SIZE]    = {0};
char              MQTT_ENDPOINT_SYS_RESET[MQTT_ENDPOINT_SIZE]     = {0};
char              MQTT_ENDPOINT_SYS_ONLINE[MQTT_ENDPOINT_SIZE]    = {0};

char              MQTT_ENDPOINT_BELL[MQTT_ENDPOINT_SIZE]          = {0};
char              MQTT_ENDPOINT_DOOR_OPENER[MQTT_ENDPOINT_SIZE]   = {0};
#ifdef DOOR_REED
  char              MQTT_ENDPOINT_DOOR_REED[MQTT_ENDPOINT_SIZE]     = {0};
#endif

unsigned long     openerStart                                     = 0;

#ifdef TLS
WiFiClientSecure  wifiClient;
#else
WiFiClient        wifiClient;
#endif
PubSubClient      mqttClient(wifiClient);

uint8_t           bellState                                        = LOW; // HIGH: openend switch
uint8_t           lastBellState                                    = bellState;
unsigned long     lastBellDebounceTime                             = 0;  // the last time the output pin was toggled

#ifdef DOOR_REED
  uint8_t           doorReedState                                    = LOW; // HIGH: openend switch
  uint8_t           lastDoorReedState                                = doorReedState;
  unsigned long     lastDoorReedDebounceTime                         = 0;  // the last time the output pin was toggled
#endif

// PREDEFINED FUNCTIONS
void reconnect();
void updateFW(char* fwUrl);
void reset();
void publishData(char* topic, char* payload);
void publishState(char* topic, int state);

///////////////////////////////////////////////////////////////////////////
//   MQTT with SSL/TLS
///////////////////////////////////////////////////////////////////////////
/*
  Function called to verify the fingerprint of the MQTT server certificate
 */
#ifdef TLS
void verifyFingerprint() {
  DEBUG_PRINT(F("INFO: Connecting to "));
  DEBUG_PRINTLN(MQTT_SERVER);

  if (!wifiClient.connect(MQTT_SERVER, atoi(MQTT_PORT))) {
    DEBUG_PRINTLN(F("ERROR: Connection failed. Halting execution"));
    reset();
  }

  if (wifiClient.verify(MQTT_FINGERPRINT, MQTT_SERVER)) {
    DEBUG_PRINTLN(F("INFO: Connection secure"));
  } else {
    DEBUG_PRINTLN(F("ERROR: Connection insecure! Halting execution"));
    reset();
  }
}
#endif

///////////////////////////////////////////////////////////////////////////
//   MQTT
///////////////////////////////////////////////////////////////////////////
/*
   Function called when a MQTT message arrived
   @param topic   The topic of the MQTT message
   @param _payload The payload of the MQTT message
   @param length  The length of the payload
*/
void callback(char* topic, byte* _payload, unsigned int length) {
  // handle the MQTT topic of the received message
  _payload[length] = '\0';
  char* payload = (char*) _payload;
  if (strcmp(topic, MQTT_ENDPOINT_DOOR_OPENER)==0) {
    if (strcmp(payload, MQTT_ON_PAYLOAD)==0) {
      DEBUG_PRINTLN(F("Info: opener"));
      digitalWrite(PIN_DOOR_OPENER, HIGH);
      digitalWrite(PIN_LED, LOW); // led is inverted, so LOW => led is on
      openerStart = millis();
    }
  } else if (strcmp(topic, MQTT_ENDPOINT_SYS_RESET)==0) {
    if (strcmp(payload, MQTT_ON_PAYLOAD)==0) {
      publishState(MQTT_ENDPOINT_SYS_RESET, LOW);
      reset();
    }
  } else if (strcmp(topic, MQTT_ENDPOINT_SYS_UPDATE)==0) {
    if (length > 0) {
      updateFW(payload);
    }
  }
}

/*
  Function called to pulish payload data
*/
void publishData(char* topic, char* payload) {
  if (mqttClient.publish(topic, payload, true)) {
    DEBUG_PRINT(F("INFO: MQTT message publish succeeded. Topic: "));
    DEBUG_PRINT(topic);
    DEBUG_PRINT(F(" Payload: "));
    DEBUG_PRINTLN(payload);
  } else {
    DEBUG_PRINTLN(F("ERROR: MQTT message publish failed, either connection lost, or message too large"));
  }
}

/*
  Function called to publish a state
*/
void publishState(char* topic, int state) {
  publishData(topic, (char*) (state == 1 ? MQTT_ON_PAYLOAD : MQTT_OFF_PAYLOAD));
}

/*
  Function called to connect/reconnect to the MQTT broker
 */
void reconnect() {
  // test if the module has an IP address
  // if not, restart the module
  if (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINTLN(F("ERROR: The module isn't connected to the internet"));
    reset();
  }

  // try to connect to the MQTT broker
  while (!mqttClient.connected()) {
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS, MQTT_ENDPOINT_SYS_ONLINE, 0, 1, "0")) {
      DEBUG_PRINTLN(F("INFO: The client is successfully connected to the MQTT broker"));
    } else {
      DEBUG_PRINTLN(F("ERROR: The connection to the MQTT broker failed"));
      delay(1000);
    }
  }

  mqttClient.subscribe(MQTT_ENDPOINT_SYS_RESET);
  mqttClient.subscribe(MQTT_ENDPOINT_SYS_UPDATE);

  mqttClient.subscribe(MQTT_ENDPOINT_DOOR_OPENER);
}

///////////////////////////////////////////////////////////////////////////
//   SYSTEM
///////////////////////////////////////////////////////////////////////////
/*
 Function called to update chip firmware
 */
void updateFW(char* fwUrl) {
  DEBUG_PRINTLN(F("INFO: updating firmware ..."));
  t_httpUpdate_return ret = ESPhttpUpdate.update(fwUrl);
#ifdef DEBUG
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      DEBUG_PRINT(F("ERROR: firmware update failed: "));
      DEBUG_PRINTLN(ESPhttpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      DEBUG_PRINTLN(F("ERROR: no new firmware"));
      break;
    case HTTP_UPDATE_OK:
      DEBUG_PRINTLN(F("Info: firmware update ok"));
      break;
  }
#endif
}

/*
  Function called to reset / restart the switch
 */
void reset() {
  DEBUG_PRINTLN(F("INFO: Reset..."));
  ESP.reset();
  delay(1000);
}

/*
  Function called to connect to Wifi
 */
void connectWiFi() {
  // Connect to WiFi network
  DEBUG_PRINT(F("INFO: Connecting to "));
  DEBUG_PRINTLN(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
  DEBUG_PRINTLN(F(""));
  DEBUG_PRINTLN(F("INFO: WiFi connected"));
}

///////////////////////////////////////////////////////////////////////////
//   Setup() and loop()
///////////////////////////////////////////////////////////////////////////
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif
  DEBUG_PRINTLN(F(""));
  DEBUG_PRINTLN(F(""));
  DEBUG_PRINTLN(F("Info: booted"));

  // init the I/O
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_DOOR_OPENER, OUTPUT);
  pinMode(PIN_BELL, INPUT_PULLUP);
#ifdef DOOR_REED
  pinMode(PIN_DOOR_REED, INPUT_PULLUP);
#endif

  // switch on led and opener off
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_DOOR_OPENER, LOW);

  // connect to wifi
  connectWiFi();

  // get the Chip ID of the switch and use it as the MQTT client ID
  sprintf(MQTT_CLIENT_ID, "%06X", ESP.getChipId());
  DEBUG_PRINT(F("INFO: MQTT client ID/Hostname: "));
  DEBUG_PRINTLN(MQTT_CLIENT_ID);

  // create MQTT endpoints
  sprintf(MQTT_ENDPOINT_SYS_VERSION, "%s%06X/%s", MQTT_TOPIC_BASE, ESP.getChipId(), MQTT_TOPIC_SYSTEM_VERSION);
  sprintf(MQTT_ENDPOINT_SYS_UPDATE, "%s%06X/%s", MQTT_TOPIC_BASE, ESP.getChipId(), MQTT_TOPIC_SYSTEM_UPDATE);
  sprintf(MQTT_ENDPOINT_SYS_RESET, "%s%06X/%s", MQTT_TOPIC_BASE, ESP.getChipId(), MQTT_TOPIC_SYSTEM_RESET);
  sprintf(MQTT_ENDPOINT_SYS_ONLINE, "%s%06X/%s", MQTT_TOPIC_BASE, ESP.getChipId(), MQTT_TOPIC_SYSTEM_ONLINE);

  sprintf(MQTT_ENDPOINT_BELL, "%s%06X/%s", MQTT_TOPIC_BASE, ESP.getChipId(), MQTT_TOPIC_BELL);
  sprintf(MQTT_ENDPOINT_DOOR_OPENER, "%s%06X/%s", MQTT_TOPIC_BASE, ESP.getChipId(), MQTT_TOPIC_DOOR_OPENER);
#ifdef DOOR_REED
  sprintf(MQTT_ENDPOINT_DOOR_REED, "%s%06X/%s", MQTT_TOPIC_BASE, ESP.getChipId(), MQTT_TOPIC_DOOR_REED);
#endif

#ifdef TLS
  // check the fingerprint of io.adafruit.com's SSL cert
  verifyFingerprint();
#endif

  // configure MQTT
  mqttClient.setServer(MQTT_SERVER, atoi(MQTT_PORT));
  mqttClient.setCallback(callback);

  // connect to the MQTT broker
  reconnect();

  // publish running firmware version
  publishData(MQTT_ENDPOINT_SYS_VERSION, (char*) FW_VERSION);
  publishState(MQTT_ENDPOINT_SYS_ONLINE, 1);

  digitalWrite(PIN_LED, HIGH);
}


void loop() {
  // keep the MQTT client connected to the broker
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  yield();

int reading;

#ifdef DOOR_REED
  reading = digitalRead(PIN_DOOR_REED);
  if (reading != lastDoorReedState) {
    lastBellDebounceTime = millis();
  }
  if ((millis() - lastBellDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != doorReedState) {
      doorReedState = reading;
      if (doorReedState == HIGH) {
        DEBUG_PRINTLN(F("Info: reed"));
        publishState(MQTT_ENDPOINT_DOOR_REED, doorReedState == LOW ? HIGH : LOW);
      }
    }
  }
  lastDoorReedState = reading;
#endif

  reading = digitalRead(PIN_BELL);
  if (reading != lastBellState) {
    lastBellDebounceTime = millis();
  }
  if ((millis() - lastBellDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != bellState) {
      bellState = reading;
      if (bellState == HIGH) {
        DEBUG_PRINTLN(F("Info: bell"));
        publishState(MQTT_ENDPOINT_BELL, bellState == LOW ? HIGH : LOW);
      }
    }
  }
  lastBellState = reading;

  // reset door opener after 2 seconds
  if (openerStart != 0 && millis() - openerStart >= 1000 * 2) {
    DEBUG_PRINTLN(F("Info: opener reset"));
    digitalWrite(PIN_DOOR_OPENER, LOW);
    digitalWrite(PIN_LED, HIGH); // led is inverted, so LOW => led is on
    publishState(MQTT_ENDPOINT_DOOR_OPENER, LOW);
    openerStart = 0;
  }

  yield();
}
