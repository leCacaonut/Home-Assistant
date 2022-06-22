// Include
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_VL53L0X.h>

// Wifi settings
constexpr char ssid[] = "wyefye";
constexpr char password[] = "bright89";
constexpr char hostname[] = "ESP32-Presence-Sensor";
unsigned long wifiLastMillis = 0;

// MQTT settings
WiFiClient wifiClient;
PubSubClient client(wifiClient);
const IPAddress server(192, 168, 0, 127);
constexpr int port = 1883;
constexpr char connected_topic[] = "esp32/presence_sensor/state";
constexpr char pub_topic1[] = "esp32/presence_sensor/sensor/state";
// constexpr char sub_topic1[] = "esp32/xxxxx_sensor/sensor/xxxxx";
unsigned long mqttLastMillis = 0;
unsigned long sendLastMillis = 0;
unsigned long sendInterval = 1000;

// Connection variables
unsigned long reconnectInterval = 10000;

// Sensor
Adafruit_VL53L0X laserSensor = Adafruit_VL53L0X();
boolean objectDetected = false;
boolean previousState = false;


/* ------------------------------------------- */
// Connecting to Wifi
void wifiReconnect() {
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    delay(100);
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
        // client.subscribe(sub_topic1);
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
    copy[++i] = '\0';  // End the c string
    Serial.println(copy);

    // if (strcmp(topic, sub_topic1) == 0) sendMessage = true;
}

void setup() {
    Serial.begin(115200);
    // Wifi setup
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(hostname);
    // Mqtt setup
    client.setServer(server, port);
    client.setCallback(callback);
    // Sensor
    if (!laserSensor.begin()) {
        Serial.println("Failed to initialise sensor");
    }
}

void loop() {
    unsigned long currentMillis = millis();
    // Enter sensor reading code here
    VL53L0X_RangingMeasurementData_t measure;
    laserSensor.rangingTest(&measure, false);
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 1200) {  // phase failures have incorrect data
         Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
        objectDetected = true;
    } else {
        // Serial.println(" out of range ");
        objectDetected = false;
    }

    // End code
    if ((WiFi.status() != WL_CONNECTED) && (currentMillis - wifiLastMillis > reconnectInterval)) {
        Serial.print("Connecting to wifi...");
        wifiReconnect();
        wifiLastMillis = currentMillis;
    } else if (WiFi.status() == WL_CONNECTED) {
        if (!client.connected()) {
            currentMillis = millis();
            if (currentMillis - mqttLastMillis > reconnectInterval) {
                Serial.print("Connecting to mqtt...");
                mqttReconnect();
            }
        } else {

            // Enter custom code here
            currentMillis = millis();
            if (currentMillis - sendLastMillis > sendInterval) {
                if (previousState != objectDetected) {
                    Serial.println("Sent mqtt");
                    if (objectDetected) client.publish(pub_topic1, "1");
                    else client.publish(pub_topic1, "0");
                    previousState = objectDetected;

                    sendLastMillis = currentMillis;
                }
            }
            // End code
            client.loop();
        }
    }
}
