// Include
#include <WiFi.h>
#include <PubSubClient.h>

// Wifi settings
constexpr char ssid[] = "wyefye";
constexpr char password[] = "bright89";
constexpr char hostname[] = "ESP32-xxxxx-Sensor";
unsigned long wifiLastMillis = 0;

// MQTT settings
WiFiClient wifiClient;
PubSubClient client(wifiClient);
const IPAddress server(192, 168, 0, 127);
constexpr int port = 1883;
constexpr char connected_topic[] = "esp32/xxxxx_sensor/state";
constexpr char pub_topic1[] = "esp32/xxxxx_sensor/sensor/xxxxx";
constexpr char sub_topic1[] = "esp32/xxxxx_sensor/sensor/xxxxx";
constexpr char sub_topic2[] = "esp32/xxxxx_sensor/settings/xxxxx";
unsigned long mqttLastMillis = 0;
unsigned long sendLastMillis = 0;
unsigned long sendInterval = 1000;

// Connection variables
unsigned long reconnectInterval = 10000;

// Custom variables


/* ------------------------------------------- */
// Connecting to Wifi
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

// Connecting to MQTT Server
boolean mqttReconnect() {
    if (client.connect(hostname)) {
        Serial.println("Connected to mqtt");
        client.subscribe(sub_topic1);
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

    // if (strcmp(topic, sub_topic1) == 0) sendMessage = true;
    
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
    // Enter sensor reading code here
    // lightReading = analogRead(lightSensor);
//    Serial.println(lightReading);
    // lightBlocked = (lightReading < lightThreshold) ? true : false;
    
    // End code
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
          
            // Enter custom code here
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
