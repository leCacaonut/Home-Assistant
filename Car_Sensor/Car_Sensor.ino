// Includes
#include "WiFi.h"
#include <PubSubClient.h>

// Wifi settings
constexpr char ssid[] = "wyefye";
constexpr char password[] = "bright89";
constexpr char hostname[] = "ESP32-Car-Sensor";

// MQTT settings
WiFiClient wifiClient;
PubSubClient client(wifiClient);
const IPAddress server(192, 168, 0, 127);
constexpr int port = 1883;
constexpr char connected_topic[] = "esp32/car_sensor/state";
constexpr char pub_topic[] = "esp32/car_sensor/sensor/distance";
constexpr char sub_topic1[] = "esp32/car_sensor/settings/close_distance";
constexpr char sub_topic2[] = "esp32/car_sensor/settings/far_distance";

// HC SR04 settings
constexpr float speedOfSound = 0.034;
constexpr int trigPin = 5;
constexpr int echoPin = 18;

// LED setup
constexpr int sled1 = 14;
constexpr int sled2 = 26;
constexpr int cled1 = 12;
constexpr int cled2 = 27;
constexpr int cled3 = 25;
constexpr unsigned long ledOnTime = 60000; // one minute
unsigned long ledPreviousTime = 0;
boolean ledOn = true;

// Variables
unsigned long wifiLastMillis = 0;
unsigned long mqttLastMillis = 0;
unsigned long sendLastMillis = 0;
unsigned long reconnectInterval = 10000;
unsigned long sendInterval = 3000;
float previousDistance = 0;
float distanceClose = 60.0;
float distanceFar = 75.0;
boolean sendMessage = false;
constexpr float requiredDistanceDifference = 2.5;

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
        client.publish(connected_topic, "Connected");
        char temp[4];
//        client.publish(sub_topic1, dtostrf(distanceClose, 4, 1, temp));
//        client.publish(sub_topic2, dtostrf(distanceFar, 4, 1, temp));
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

    if (strcmp(topic, sub_topic1) == 0) distanceClose = atof(copy);
    else if (strcmp(topic, sub_topic2) == 0) distanceFar = atof(copy);
    Serial.println(distanceClose);
    Serial.println(distanceFar);
}

float measureDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    float distanceCm = duration * speedOfSound / 2;
    //    Serial.print("Distance: ");
    //    Serial.println(distanceCm);
    return distanceCm;
}

void distanceIndicator(float distance) {
    if (distance < distanceClose) {
        digitalWrite(cled1, HIGH & ledOn);
        digitalWrite(cled2, LOW);
        digitalWrite(cled3, LOW);
    } else if (distance > distanceFar) {
        digitalWrite(cled3, HIGH & ledOn);
        digitalWrite(cled1, LOW);
        digitalWrite(cled2, LOW);
    } else {
        digitalWrite(cled2, HIGH & ledOn);
        digitalWrite(cled1, LOW);
        digitalWrite(cled3, LOW);
    }
}

void setup() {
    Serial.begin(115200);
    // HC SR04
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    // Status LEDs
    pinMode(sled1 , OUTPUT);
    pinMode(sled2 , OUTPUT);
    pinMode(cled1 , OUTPUT);
    pinMode(cled2 , OUTPUT);
    pinMode(cled3 , OUTPUT);
    digitalWrite(sled1, LOW);
    digitalWrite(sled2, LOW);
    digitalWrite(cled1, LOW);
    digitalWrite(cled2, LOW);
    digitalWrite(cled3, LOW);
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
    float newDistance = measureDistance();
    if (newDistance > 450 || newDistance < 4) newDistance = 0;
    distanceIndicator(newDistance);

    if (abs(previousDistance - newDistance) > requiredDistanceDifference && newDistance != 0) {
        Serial.println(newDistance);
        previousDistance = newDistance;
        sendMessage = true;
        ledPreviousTime = currentMillis;
        ledOn = true;
    } else if (currentMillis - ledPreviousTime > ledOnTime) {
        if (WiFi.status() == WL_CONNECTED && client.connected()){
            ledOn = false;
        }
    } else {
        ledOn;
    }

    // Code end
    if ((WiFi.status() != WL_CONNECTED) && (currentMillis - wifiLastMillis > reconnectInterval)) {
        Serial.print("Connecting to wifi...");
        digitalWrite(sled1, LOW);
        wifiReconnect();
        wifiLastMillis = currentMillis;

    } else {

        digitalWrite(sled1, HIGH & ledOn);
        if (!client.connected()) {
            digitalWrite(sled2, LOW);
            currentMillis = millis();

            if (currentMillis - mqttLastMillis > reconnectInterval) {
                mqttReconnect();
            }

        } else {
            // Enter transmit code here
            digitalWrite(sled2, HIGH & ledOn);
            currentMillis = millis();
            if (sendMessage && currentMillis - sendLastMillis > sendInterval) {
                Serial.println("Sent mqtt");
                char bufferd[6];
                client.publish(pub_topic, dtostrf(newDistance, 4, 2, bufferd));
                sendMessage = false;
                sendLastMillis = currentMillis;
            }
            client.loop();

            // End code
        }
    }
    delay(400);
}
