// Includes
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_VL53L0X.h>

// Wifi settings
constexpr char ssid[] = "wyefye";
constexpr char password[] = "bright89";
constexpr char hostname[] = "ESP32-Car-Distance-Sensor";
unsigned long wifiLastMillis = 0;

// MQTT settings
WiFiClient wifiClient;
PubSubClient client(wifiClient);
const IPAddress server(192, 168, 0, 127);
constexpr int port = 1883;
constexpr char connected_topic[] = "esp32/car_distance_sensor/state";
constexpr char pub_topic1[] = "esp32/car_distance_sensor/sensor/status";
constexpr char sub_topic1[] = "esp32/car_distance_sensor/settings/distance_close";
constexpr char sub_topic2[] = "esp32/car_distance_sensor/settings/distance_far";
unsigned long mqttLastMillis = 0;
unsigned long sendLastMillis = 0;
unsigned long sendInterval = 3000;

// Connection variables
unsigned long reconnectInterval = 10000;

// LED setup
constexpr int sled1 = 14;
constexpr int sled2 = 26;
constexpr int cled1 = 12;
constexpr int cled2 = 27;
constexpr int cled3 = 25;
constexpr unsigned long ledOnTime = 60000; // one minute
unsigned long ledPreviousTime = 0;
boolean ledOn = true;

// Sensor
Adafruit_VL53L0X laserSensor = Adafruit_VL53L0X();
int distanceClose = 450;
int distanceFar = 700;
int state = 0; //0 = far, 1 = good, 2 = close;
int previousState = 0;

/* ------------------------------------------- */
// Connecting to Wifi
void wifiReconnect() {
    WiFi.disconnect(true);
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
    WiFi.setHostname(hostname);
    WiFi.begin(ssid, password);
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.println("Wifi connected");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    }
}

// Connecting to MQTT Server
boolean mqttReconnect() {
    if (client.connect(hostname, "mqttuser", "mqttuser")) {
        Serial.println("Connected to mqtt");
        client.subscribe(sub_topic1);
        client.subscribe(sub_topic2);
        client.publish(connected_topic, "Connected");
//        client.publish(sub_topic1, dtostrf(distanceClose, 4, 1, temp));
    }
    return client.connected();
}

// When a message is received
void callback(char* topic, byte* payload, unsigned int length) {
    char copy[length + 1];
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    int i = 0;
    for (i = 0; i < length; i++) {
//        Serial.print((char)payload[i]);
        copy[i] = (char)payload[i];
    }
    copy[++i] = '\0'; // End the c string
    Serial.println(copy);

    if (strcmp(topic, sub_topic1) == 0) distanceClose = atof(copy);
    else if (strcmp(topic, sub_topic2) == 0) distanceFar = atof(copy);
    
}

void distanceIndicator(float distance) {
    if (distance < distanceClose) {
        digitalWrite(cled1, HIGH & ledOn);
        digitalWrite(cled2, LOW);
        digitalWrite(cled3, LOW);
        state = 2;
    } else if (distance > distanceFar) {
        digitalWrite(cled3, HIGH & ledOn);
        digitalWrite(cled1, LOW);
        digitalWrite(cled2, LOW);
        state = 0;
    } else {
        digitalWrite(cled2, HIGH & ledOn);
        digitalWrite(cled1, LOW);
        digitalWrite(cled3, LOW);
        state = 1;
    }
}

void setup() {
    Serial.begin(115200);
    // Wifi setup
    WiFi.mode(WIFI_STA);
    // Mqtt setup
    client.setServer(server, port);
    client.setCallback(callback);
    // Status LEDs
    pinMode(sled1 , OUTPUT);
    pinMode(sled2 , OUTPUT);
    pinMode(cled1 , OUTPUT);
    pinMode(cled2 , OUTPUT);
    pinMode(cled3 , OUTPUT);
    // Sensor setup
    if (!laserSensor.begin()) {
      Serial.println("Failed to initialise sensor");
      while (true) {
        digitalWrite(sled1, HIGH);
        delay(500);
        digitalWrite(sled1, LOW);
        delay(500);
      }
    } 
}

void loop() {
    unsigned long currentMillis = millis();
    // Enter sensor reading code here
    VL53L0X_RangingMeasurementData_t measure;
    char measurement[6];
    laserSensor.rangingTest(&measure, false); 
    if (measure.RangeStatus != 4) {// phase failures have incorrect data
      // Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);

      // Change lights
      distanceIndicator(measure.RangeMilliMeter);

      //Measurement to string
      itoa(measure.RangeMilliMeter, measurement, 10);

    } else {
      Serial.println(" out of range ");
      strcpy(measurement, "NA");
    }

    // End code
    if ((WiFi.status() != WL_CONNECTED) && (currentMillis - wifiLastMillis > reconnectInterval)) {
        Serial.print("Connecting to wifi...");
        digitalWrite(sled1, LOW);
        wifiReconnect();
        wifiLastMillis = currentMillis;
    } else {
      digitalWrite(sled1, HIGH);
        if (!client.connected()) {
            digitalWrite(sled2, LOW);
            currentMillis = millis();
            if (currentMillis - mqttLastMillis > reconnectInterval) {
                mqttReconnect();
            }
        } else {
          digitalWrite(sled2, HIGH);
            // Enter custom code here
            currentMillis = millis();
            if (currentMillis - sendLastMillis > sendInterval) {
              if (previousState != state) {
                if (state == 1) client.publish(pub_topic1, "1");
                else if (state == 0) client.publish(pub_topic1, "0");
                else if (state == 2) client.publish(pub_topic1, "2");
                Serial.println("Sent mqtt");
                previousState = state;

                sendLastMillis = currentMillis;
              }
            }
            // End code
            client.loop();
        }
    }
//    delay(500);
}
