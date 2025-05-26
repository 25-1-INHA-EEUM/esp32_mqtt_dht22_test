#include <WiFi.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include <Wire.h>
#include <driver/i2s.h>
#include "ScioSense_ENS160.h"  // ENS160 library
#include "PMS.h"
#include "sos-iir-filter.h"
// ——— WiFi/MQTT 설정 —————————————————————————
const char* ssid = "Devop"; //WIFI name, 
const char* password = "os7240485"; // WIFI password,
const char* mqtt_server = "13.125.235.68"; // ec2 server, 
const int mqttPort = 1883;
const char* mqttTopic = "sensors/data"; // publish topic

WiFiClient espClient;
PubSubClient client(espClient);

// ——— DHT22 설정 ————————————————————————————
const int DHT_PIN = 4;
DHTesp dht;

// ——— INMP441 (I2S 마이크) 설정 ————————————————————
#define LEQ_PERIOD        1           // second(s)
#define WEIGHTING         A_weighting // Also avaliable: 'C_weighting' or 'None' (Z_weighting)
#define LEQ_UNITS         "LAeq"      // customize based on above weighting used
#define DB_UNITS          "dBA"       // customize based on above weighting used

#define MIC_EQUALIZER     INMP441    // See below for defined IIR filters or set to 'None' to disable
#define MIC_OFFSET_DB     3.0103      // Default offset (sine-wave RMS vs. dBFS). Modify this value for linear calibration

// Customize these values from microphone datasheet
#define MIC_SENSITIVITY   -26         // dBFS value expected at MIC_REF_DB (Sensitivity value from datasheet)
#define MIC_REF_DB        94.0        // Value at which point sensitivity is specified in datasheet (dB)
#define MIC_OVERLOAD_DB   116.0       // dB - Acoustic overload point
#define MIC_NOISE_DB      33          // dB - Noise floor
#define MIC_BITS          24          // valid number of bits in I2S data
#define MIC_CONVERT(s)    (s >> (SAMPLE_BITS - MIC_BITS))
#define MIC_TIMING_SHIFT  0           // Set to one to fix MSB timing for some microphones, i.e. SPH0645LM4H-x

// Calculate reference amplitude value at compile time
constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY)/20) * ((1<<(MIC_BITS-1))-1);

#define I2S_WS            15 
#define I2S_SCK           2
#define I2S_SD            13 

// I2S peripheral to use (0 or 1)
#define I2S_PORT          I2S_NUM_0

// IIR Filters
const SOS_Coefficients INMP441_sos[] = {
  {-1.986920458344451, +0.986963226946616, +1.995178510504166, -0.995184322194091}
};
SOS_IIR_Filter INMP441(1.00197834654696, INMP441_sos);

// Weighting filters

// A-weighting IIR Filter, Fs = 48KHz 
const SOS_Coefficients A_weighting_sos[] = { // Second-Order Sections {b1, b2, -a1, -a2}
    {-2.00026996133106, +1.00027056142719, -1.060868438509278, -0.163987445885926},
    {+4.35912384203144, +3.09120265783884, +1.208419926363593, -0.273166998428332},
    {-0.70930303489759, -0.29071868393580, +1.982242159753048, -0.982298594928989}
  };
SOS_IIR_Filter A_weighting(0.169994948147430, A_weighting_sos);

// Sampling
#define SAMPLE_RATE       48000 // Hz, fixed to design of IIR filters
#define SAMPLE_BITS       32    // bits
#define SAMPLE_T          int32_t 
#define SAMPLES_SHORT     (SAMPLE_RATE / 8) // ~125ms
#define SAMPLES_LEQ       (SAMPLE_RATE * LEQ_PERIOD)
#define DMA_BANK_SIZE     (SAMPLES_SHORT / 16)
#define DMA_BANKS         32

// Data we push to 'samples_queue'
struct sum_queue_t {
  // Sum of squares of mic samples, after Equalizer filter
  float sum_sqr_SPL;
  // Sum of squares of weighted mic samples
  float sum_sqr_weighted;
};
QueueHandle_t samples_queue;

// Static buffer for block of samples
float samples[SAMPLES_SHORT] __attribute__((aligned(4)));

// I2S Microphone sampling setup 
void mic_i2s_init() {
  const i2s_config_t i2s_config = {
    mode: i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    sample_rate: SAMPLE_RATE,
    bits_per_sample: i2s_bits_per_sample_t(SAMPLE_BITS),
    channel_format: I2S_CHANNEL_FMT_ONLY_LEFT,
    communication_format: i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    intr_alloc_flags: ESP_INTR_FLAG_LEVEL1,
    dma_buf_count: DMA_BANKS,
    dma_buf_len: DMA_BANK_SIZE,
    use_apll: true,
    tx_desc_auto_clear: false,
    fixed_mclk: 0
  };
  // I2S pin mapping
  const i2s_pin_config_t pin_config = {
    bck_io_num:   I2S_SCK,  
    ws_io_num:    I2S_WS,    
    data_out_num: -1, // not used
    data_in_num:  I2S_SD   
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  #if (MIC_TIMING_SHIFT > 0) 
    REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));   
    REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);  
  #endif
  
  i2s_set_pin(I2S_PORT, &pin_config);
}

// I2S Reader Task
#define I2S_TASK_PRI   4
#define I2S_TASK_STACK 2048
//
void mic_i2s_reader_task(void* parameter) {
  mic_i2s_init();

  // Discard first block, microphone may have startup time (i.e. INMP441 up to 83ms)
  size_t bytes_read = 0;
  i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  while (true) {
    i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);

    TickType_t start_tick = xTaskGetTickCount();
    
    // Convert (including shifting) integer microphone values to floats, 
    // using the same buffer (assumed sample size is same as size of float), 
    // to save a bit of memory
    SAMPLE_T* int_samples = (SAMPLE_T*)&samples;
    for(int i=0; i<SAMPLES_SHORT; i++) samples[i] = MIC_CONVERT(int_samples[i]);

    sum_queue_t q;
    // Apply equalization and calculate Z-weighted sum of squares, 
    // writes filtered samples back to the same buffer.
    q.sum_sqr_SPL = MIC_EQUALIZER.filter(samples, samples, SAMPLES_SHORT);

    // Apply weighting and calucate weigthed sum of squares
    q.sum_sqr_weighted = WEIGHTING.filter(samples, samples, SAMPLES_SHORT);

    // Send the sums to FreeRTOS queue where main task will pick them up
    // and further calcualte decibel values (division, logarithms, etc...)
    xQueueSend(samples_queue, &q, portMAX_DELAY);
  }
}


// ——— PMS5003 설정 (Serial2 사용) ————————————
HardwareSerial SerialPMS(2);  // Serial2 사용 (GPIO16, GPIO17)
PMS pms(SerialPMS);
PMS::DATA data;

// ——— ENS160 (TVOC) 설정 ——————————————————————
ScioSense_ENS160 ens160(ENS160_I2CADDR_1); //ADDRESS : 0X53

// ——— 함수 선언 ————————————————————————————
void setupWiFi();
void reconnectMQTT();
double readSoundLevel();
void readPMS(int &pm2_5, int &pm10);
void readAndPublish();

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
int value = 0;

int location_id = 1; // 👉 실제 DB에 존재하는 device_id 값으로 설정할 것

// ——— setup() —————————————————————————————
void setup() {
  Serial.begin(115200);
  setupWiFi();
  client.setServer(mqtt_server, mqttPort);



  //DHT 초기화
  dht.setup(DHT_PIN, DHTesp::DHT22);

   // I2S 초기화
  // Create FreeRTOS queue
  samples_queue = xQueueCreate(8, sizeof(sum_queue_t));
  
  // Create the I2S reader FreeRTOS task
  // NOTE: Current version of ESP-IDF will pin the task 
  //       automatically to the first core it happens to run on
  //       (due to using the hardware FPU instructions).
  //       For manual control see: xTaskCreatePinnedToCore
  xTaskCreate(mic_i2s_reader_task, "Mic I2S Reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, NULL);
  // PMS5003 초기화
  SerialPMS.begin(9600, SERIAL_8N1, 16, 17);

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

// ——— 데시벨 읽기 ——————————————————————————
double readSoundLevel() {
  sum_queue_t q;
  uint32_t Leq_samples = 0;
  double Leq_sum_sqr = 0;
  double Leq_dB = 0;

  // Read sum of samaples, calculated by 'i2s_reader_task'
  while (xQueueReceive(samples_queue, &q,  pdMS_TO_TICKS(200))) {

    // Calculate dB values relative to MIC_REF_AMPL and adjust for microphone reference
    double short_RMS = sqrt(double(q.sum_sqr_SPL) / SAMPLES_SHORT);
    double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);

    // In case of acoustic overload or below noise floor measurement, report infinty Leq value
    if (short_SPL_dB > MIC_OVERLOAD_DB) {
      Leq_sum_sqr = INFINITY;
    } else if (isnan(short_SPL_dB) || (short_SPL_dB < MIC_NOISE_DB)) {
      Leq_sum_sqr = -INFINITY;
    }

    // Accumulate Leq sum
    Leq_sum_sqr += q.sum_sqr_weighted;
    Leq_samples += SAMPLES_SHORT;

    // When we gather enough samples, calculate new Leq value
    if (Leq_samples >= SAMPLE_RATE * LEQ_PERIOD) {
      double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
      Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);
      Leq_sum_sqr = 0;
      Leq_samples = 0;

      return Leq_dB;
    }
  }
}

// ——— PMS5003 읽기 ——————————————————————————
void readPMS(uint16_t &pm2_5, uint16_t &pm10) {
  unsigned long start = millis();
  uint16_t timeout_ms = 1000;
  while (millis() - start < timeout_ms) {
    if(pms.read(data)){
      pm2_5 = data.PM_AE_UG_2_5;
      pm10 = data.PM_AE_UG_10_0;
      return;
    }
  }
  pm2_5 = 0;
  pm10 = 0;
}

// ——— 센서 읽고 MQTT 발행 ————————————————————
void readAndPublish() {
  // DHT22
  float temperature = dht.getTemperature();
  float humidity  = dht.getHumidity();
  double noise = readSoundLevel();

  uint16_t pm25, pm10;
  readPMS(pm25, pm10);

  ens160.measure();
  uint16_t tvoc = ens160.getTVOC();

  // JSON 형식 메시지 구성
    String payload = "{";
    payload += "\"location_id\": " + String(location_id) + ",";
    payload += "\"temperature\": " + String(temperature, 2) + ",";
    payload += "\"humidity\": " + String(humidity, 2) + ",";
    payload += "\"tvoc\": " + String(tvoc) + ", ";  // 예시값
    payload += "\"noise\": " + String(noise) + ","; // 예시값
    payload += "\"pm10\": " + String(pm10) + ",";   // 예시값
    payload += "\"pm2_5\": " + String(pm25);
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
