// Includes
#include "WiFi.h"
#include <PubSubClient.h>

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
constexpr char pub_topic1[] = "esp32/presence_sensor/sensor/presence";
constexpr char pub_topic2[] = "esp32/presence_sensor/sensor/light_level";
constexpr char sub_topic1[] = "esp32/presence_sensor/sensor/get";
constexpr char sub_topic2[] = "esp32/presence_sensor/sensor/get_reading";
constexpr char sub_topic3[] = "esp32/presence_sensor/settings/threshold";
unsigned long mqttLastMillis = 0;
unsigned long sendLastMillis = 0;
unsigned long sendInterval = 1000;

// Sensor settings
constexpr int lightSensor = 34;
int lightThreshold = 2200;
bool lightBlocked = false;

// Variables
int lightReading = 0;
unsigned long reconnectInterval = 10000;
boolean sendMessage = false;

/* ------------------------------------------- */
void wifiReconnect() {
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.println("Wifi connected");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    }
}

boolean mqttReconnect() {
    if (client.connect(hostname)) {
        Serial.println("Connected to mqtt");
        client.subscribe(sub_topic1);
        client.subscribe(sub_topic2);
        client.subscribe(sub_topic3);
        client.publish(connected_topic, "Connected");
//        client.publish(sub_topic1, dtostrf(distanceClose, 4, 1, temp));
    }
    return client.connected();
}

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

    if (strcmp(topic, sub_topic1) == 0) sendMessage = true;
    else if (strcmp(topic, sub_topic2) == 0) {
      char buffer[5];
      client.publish(pub_topic2, itoa(lightReading, buffer, 10));
    }
    else if (strcmp(topic, sub_topic3) == 0) {
      lightThreshold = atoi(copy);
      if (lightThreshold < 0) lightThreshold = 0;
      else if (lightThreshold > 4095) lightThreshold = 4095;
      Serial.println(lightThreshold);
    }
}

void setup() {
    Serial.begin(115200);
    // Photoresistor
    pinMode(lightSensor, INPUT);
    // Wifi setup
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(hostname);
    // Mqtt setup
    client.setServer(server, port);
    client.setCallback(callback);
}

void loop() {
    unsigned long currentMillis = millis();
    // Enter other code here
    lightReading = analogRead(lightSensor);
//    Serial.println(lightReading);
    lightBlocked = (lightReading < lightThreshold) ? true : false;
    
    // Code end
    if ((WiFi.status() != WL_CONNECTED) && (currentMillis - wifiLastMillis > reconnectInterval)) {
        Serial.print("Connecting to wifi...");
        wifiReconnect();
        wifiLastMillis = currentMillis;

    } else {
        if (!client.connected()) {
            currentMillis = millis();

            if (currentMillis - mqttLastMillis > reconnectInterval) {
                mqttReconnect();
            }

        } else {
            // Enter transmit code here
            currentMillis = millis();
            if (sendMessage && currentMillis - sendLastMillis > sendInterval) {
                Serial.println("Sent mqtt");
                if (lightBlocked) client.publish(pub_topic1, "true");
                else client.publish(pub_topic1, "false");
                
                sendMessage = false;
                sendLastMillis = currentMillis;
            }
            // End code
            
            client.loop();
        }
    }
//    delay(500);
}
