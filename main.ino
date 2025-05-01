

#include <WiFi.h>
#include <PubSubClient.h>
#include "DHTesp.h"

const int DHT_PIN = 15;

DHTesp dhtSensor;

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = {PUBLIC_EC2_IP};

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
float temp = 0;
float hum = 0;
int value = 0;

int device_id = 1; // 👉 실제 DB에 존재하는 device_id 값으로 설정할 것

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

    TempAndHumidity data = dhtSensor.getTempAndHumidity();
    float temperature = data.temperature;
    float humidity = data.humidity;

    // JSON 형식 메시지 구성
    String payload = "{";
    payload += "\"device_id\": " + String(device_id) + ",";
    payload += "\"temperature\": " + String(temperature, 2) + ",";
    payload += "\"humidity\": " + String(humidity, 2) + ",";
    payload += "\"tvoc\": " + String(0.4, 2) + ",";  // 예시값
    payload += "\"noise\": " + String(30.2, 2) + ","; // 예시값
    payload += "\"pm10\": " + String(1.0, 2) + ",";   // 예시값
    payload += "\"pm2_5\": " + String(0.5, 2);
    payload += "}";

    Serial.println("📤 MQTT 전송: ");
    Serial.println(payload);

    // 실제 토픽에 맞춰 변경
    client.publish("sensors/data", payload.c_str());
  }
}