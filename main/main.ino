#include <WiFi.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include <Wire.h>
#include <driver/i2s.h>
//#include "ens160.h"
#include "ScioSense_ENS160.h"  // ENS160 library

// ——— WiFi/MQTT 설정 —————————————————————————
const char* ssid = "Devop"; //WIFI name
const char* password = "os7240485"; // WIFI password
const char* mqtt_server = "13.125.235.68"; // ec2 server
const int mqttPort = 1883;
const char* mqttTopic = "sensors/data"; // publish topic

WiFiClient espClient;
PubSubClient client(espClient);

// ——— DHT22 설정 ————————————————————————————
const int DHT_PIN = 4;
DHTesp dht;

// ——— INMP441 (I2S 마이크) 설정 ————————————————————
#define I2S_WS   33   // Word Select (L/R)
#define I2S_SD   25   // Serial Data
#define I2S_SCK  26   // Serial Clock

const i2s_port_t I2S_PORT = I2S_NUM_0;
const int SAMPLE_COUNT = 1024;

// ——— PMS5003 설정 (Serial2 사용) ————————————
#define PMS_RX 16
#define PMS_TX 17
HardwareSerial SerialPMS(2);

// ——— ENS160 (TVOC) 설정 ——————————————————————
ScioSense_ENS160 ens160(ENS160_I2CADDR_1);

// ——— 함수 선언 ————————————————————————————
void setupWiFi();
void reconnectMQTT();
float readSoundLevel();
bool readPMS(int &pm1, int &pm2_5, int &pm10);
void readAndPublish();

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
int value = 0;

int device_id = 1; // 👉 실제 DB에 존재하는 device_id 값으로 설정할 것

// ——— setup() —————————————————————————————
void setup() {
  Serial.begin(115200);
  setupWiFi();
  client.setServer(mqtt_server, mqttPort);

  //DHT 초기화
  dht.setup(DHT_PIN, DHTesp::DHT22);

   // I2S 초기화
  i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 1024
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);

  // PMS5003 초기화
  SerialPMS.begin(9600, SERIAL_8N1, PMS_RX, PMS_TX);

  // ENS160 초기화
  Wire.begin(21, 22); //SDA=21, SCL=22
  if (!ens160.begin(&Wire)) {
    Serial.println("ENS160 init failed!");
  } else {
    ens160.setMode(ENS160_OPMODE_STD); //1초 측정 주기
  }
}

void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("✅ I2C device found at 0x");
      Serial.println(address, HEX);
    }
  }
  readAndPublish();
  delay(5000); // 5초마다 전송
}

// ——— WiFi 연결 ——————————————————————————
void setupWiFi(){
  Serial.printf("Connecting to %s ...\n", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
}

// ——— MQTT 재접속 ——————————————————————————
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("MQTT connecting…");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.printf("failed, rc=%d try again in 5s\n", client.state());
      delay(5000);
    }
  }
}

// calibration_offset: 실험실 환경(예: 94 dB 소스)에서 측정한 dBFS 값과
// 실제 SPL(94 dB)과의 차이를 보정하기 위한 값입니다.
// 처음에는 0으로 두고, 실제 소스(예: 1 kHz, 94 dB)를 틀어두고
// Serial.print(dbFS)로 확인한 뒤, offset = 94 - measured_dBFS 로 계산하세요.
// 알려진 음원(예: 94dB @1kHz)으로 보정값 설정
const float calibration_offset = 94.0;  // 실제 측정값으로 조정 필요

float readSoundLevel() {
  const int32_t samples = SAMPLE_COUNT;
  int32_t buffer[samples];
  size_t bytesRead;

  // I2S에서 PCM 데이터 읽기
  i2s_read(I2S_PORT, buffer, sizeof(buffer), &bytesRead, portMAX_DELAY);
  int count = bytesRead / sizeof(int32_t);

  // DC 제거용 평균값 계산
  double mean = 0;
  for (int i = 0; i < count; i++) {
    mean += buffer[i];
  }
  mean /= count;

  // RMS 계산
  double sumSq = 0;
  for (int i = 0; i < count; i++) {
    double centered = buffer[i] - mean;
    sumSq += centered * centered;
  }
  double rms = sqrt(sumSq / count);

  // dBFS 계산 (Full-Scale 대비)
  // buffer 값은 ±2^31 범위이므로, 정규화 시 나눌 값은 2^31(=2147483648)
  double rms_norm = rms / 2147483648.0;
  double dbFS = 20.0 * log10(rms_norm);
  
  if (rms_norm <= 0.0000000001) return 0.0;  // 🔐 방어 코드

  // 보정값을 더해 실제 SPL 근사
  float dbSPL = dbFS + calibration_offset;
  
  return dbSPL;
}

// ——— 센서 읽고 MQTT 발행 ————————————————————
void readAndPublish() {
  // DHT22
  float temperature = dht.getTemperature();
  float humidity  = dht.getHumidity();
  float noise = readSoundLevel();

  ens160.measure();
  uint16_t tvoc = ens160.getTVOC();

  // JSON 형식 메시지 구성
    String payload = "{";
    payload += "\"device_id\": " + String(device_id) + ",";
    payload += "\"temperature\": " + String(temperature, 2) + ",";
    payload += "\"humidity\": " + String(humidity, 2) + ",";
    payload += "\"tvoc\": " + String(tvoc) + ", ";  // 예시값
    payload += "\"noise\": " + String(noise) + ","; // 예시값
    payload += "\"pm10\": " + String(1.0, 2) + ",";   // 예시값
    payload += "\"pm2_5\": " + String(0.5, 2);
    payload += "}";

    Serial.println("📤 MQTT 전송: ");
    Serial.println(payload);

  // MQTT 퍼블리시
  if (client.publish(mqttTopic, payload.c_str())) {
    Serial.println("Published payload:");
    Serial.println(payload);
  } else {
    Serial.println("Publish failed");
  }
}
