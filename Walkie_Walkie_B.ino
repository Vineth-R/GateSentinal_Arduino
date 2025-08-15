#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <driver/i2s.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2
#endif

// ============= Configuration =============
#define WIFI_SSID       "Vineth"
#define WIFI_PSWD       "Vanitha@2003"
#define SAMPLE_RATE     16000

#define USE_I2S_MIC_INPUT

// I2S Microphone Settings
#define I2S_MIC_CHANNEL         I2S_CHANNEL_FMT_ONLY_LEFT
#define I2S_MIC_SERIAL_CLOCK    GPIO_NUM_18
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_23
#define I2S_MIC_SERIAL_DATA     GPIO_NUM_21

// I2S Speaker Settings
#define USE_I2S_SPEAKER_OUTPUT
#define I2S_SPEAKER_SERIAL_CLOCK    GPIO_NUM_5
#define I2S_SPEAKER_LEFT_RIGHT_CLOCK GPIO_NUM_25
#define I2S_SPEAKER_SERIAL_DATA     GPIO_NUM_19
#define I2S_SPEAKER_SD_PIN          GPIO_NUM_26

// Transmit button (active LOW)
#define GPIO_TRANSMIT_BUTTON 33

#define USE_ESP_NOW
#define ESP_NOW_WIFI_CHANNEL 1

#define TRANSPORT_HEADER_SIZE 4
uint8_t transport_header[TRANSPORT_HEADER_SIZE] = { 0xAA, 0xBB, 0xCC, 0xDD };
uint8_t peer_mac[6] = {0x68, 0x25, 0xDD, 0x34, 0x2C, 0x98};

// ============= I2S Configurations =============
i2s_config_t i2s_mic_Config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
  .channel_format = I2S_MIC_CHANNEL,
  .communication_format = I2S_COMM_FORMAT_STAND_I2S,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 4,
  .dma_buf_len = 64,
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0
};

i2s_pin_config_t i2s_mic_pins = {
  .mck_io_num = I2S_PIN_NO_CHANGE,
  .bck_io_num = I2S_MIC_SERIAL_CLOCK,
  .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
  .data_out_num = I2S_PIN_NO_CHANGE,
  .data_in_num = I2S_MIC_SERIAL_DATA
};

i2s_pin_config_t i2s_speaker_pins = {
  .mck_io_num = I2S_PIN_NO_CHANGE,
  .bck_io_num = I2S_SPEAKER_SERIAL_CLOCK,
  .ws_io_num = I2S_SPEAKER_LEFT_RIGHT_CLOCK,
  .data_out_num = I2S_SPEAKER_SERIAL_DATA,
  .data_in_num = I2S_PIN_NO_CHANGE
};

// ============= Class Declarations =============
class OutputBuffer {
private:
  int16_t *m_buffer;
  int m_buffer_size;
  int m_write_pos;
  int m_read_pos;
  int m_available;
public:
  OutputBuffer(int buffer_size);
  ~OutputBuffer();
  void add_sample(int16_t sample);
  void remove_samples(int16_t *samples, int count);
  void flush();
  int available();
};

class I2SSampler {
public:
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual int read(int16_t *samples, int count) = 0;
  virtual ~I2SSampler() = default;
};

class I2SMEMSSampler : public I2SSampler {
private:
  i2s_port_t m_i2s_port;
  i2s_pin_config_t m_i2s_pins;
  i2s_config_t m_i2s_config;
  int m_buffer_size;
  int32_t *m_raw_buffer;
public:
  I2SMEMSSampler(i2s_port_t i2s_port, i2s_pin_config_t &i2s_pins, 
                 i2s_config_t &i2s_config, int buffer_size);
  ~I2SMEMSSampler();
  void start() override;
  void stop() override;
  int read(int16_t *samples, int count) override;
};

class Output {
public:
  virtual void start(int sample_rate) = 0;
  virtual void stop() = 0;
  virtual void write(int16_t *samples, int count) = 0;
  virtual ~Output() = default;
};

class I2SOutput : public Output {
private:
  i2s_port_t m_i2s_port;
  i2s_pin_config_t m_i2s_pins;
  bool m_is_running;
  float m_output_gain;  // adjustable output gain
public:
  I2SOutput(i2s_port_t i2s_port, i2s_pin_config_t &i2s_pins);
  void start(int sample_rate) override;
  void stop() override;
  void write(int16_t *samples, int count) override;
  void set_output_gain(float gain);  // optional setter if needed
};

class Transport {
protected:
  OutputBuffer *m_output_buffer;
  uint8_t *m_header;
  int m_header_size;
public:
  Transport(OutputBuffer *output_buffer);
  virtual ~Transport() = default;
  virtual void begin() = 0;
  virtual void add_sample(int16_t sample) = 0;
  virtual void flush() = 0;
  void set_header(int header_size, uint8_t *header);
};

class EspNowTransport : public Transport {
private:
  int m_wifi_channel;
  int16_t *m_tx_buffer;
  int m_tx_buffer_pos;
  int m_tx_buffer_size;
  uint8_t *m_peer_mac;
public:
  EspNowTransport(OutputBuffer *output_buffer, int wifi_channel);
  ~EspNowTransport();
  void begin() override;
  void add_sample(int16_t sample) override;
  void flush() override;
  static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
  static void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status);
};

class IndicatorLed {
public:
  virtual void begin() = 0;
  virtual void set_default_color(uint32_t color) = 0;
  virtual void set_is_flashing(bool is_flashing, uint32_t color) = 0;
  virtual void update() = 0;
  virtual ~IndicatorLed() = default;
};

class GenericDevBoardIndicatorLed : public IndicatorLed {
private:
  bool m_is_flashing;
  uint32_t m_flash_color;
  uint32_t m_default_color;
  unsigned long m_last_flash_time;
public:
  GenericDevBoardIndicatorLed();
  void begin() override;
  void set_default_color(uint32_t color) override;
  void set_is_flashing(bool is_flashing, uint32_t color) override;
  void update();
};

class Application {
private:
  Output *m_output;
  I2SSampler *m_input;
  Transport *m_transport;
  IndicatorLed *m_indicator_led;
  OutputBuffer *m_output_buffer;
public:
  Application();
  void begin();
  void loop();
};

//Global Pointer for ESP-NOW Callbacks
static EspNowTransport *g_esp_now_transport = nullptr;


// OutputBuffer Implementation
OutputBuffer::OutputBuffer(int buffer_size)
    : m_buffer_size(buffer_size), m_write_pos(0), m_read_pos(0), m_available(0) {
  m_buffer = new int16_t[buffer_size];
}

OutputBuffer::~OutputBuffer() {
  delete[] m_buffer;
}

void OutputBuffer::add_sample(int16_t sample) {
  if (m_available < m_buffer_size) {
    m_buffer[m_write_pos] = sample;
    m_write_pos = (m_write_pos + 1) % m_buffer_size;
    m_available++;
  }
}

void OutputBuffer::remove_samples(int16_t *samples, int count) {
  for (int i = 0; i < count; i++) {
    if (m_available > 0) {
      samples[i] = m_buffer[m_read_pos];
      m_read_pos = (m_read_pos + 1) % m_buffer_size;
      m_available--;
    } else {
      samples[i] = 0;
    }
  }
}

void OutputBuffer::flush() {
  m_write_pos = 0;
  m_read_pos = 0;
  m_available = 0;
}

int OutputBuffer::available() {
  return m_available;
}

// I2SMEMSSampler Implementation
I2SMEMSSampler::I2SMEMSSampler(i2s_port_t i2s_port, i2s_pin_config_t &i2s_pins, 
                               i2s_config_t &i2s_config, int buffer_size)
    : m_i2s_port(i2s_port), m_i2s_pins(i2s_pins), m_i2s_config(i2s_config), m_buffer_size(buffer_size) {
  m_raw_buffer = new int32_t[buffer_size];
}

I2SMEMSSampler::~I2SMEMSSampler() {
  delete[] m_raw_buffer;
}

void I2SMEMSSampler::start() {
  i2s_driver_install(m_i2s_port, &m_i2s_config, 0, NULL);
  i2s_set_pin(m_i2s_port, &i2s_mic_pins);
  i2s_start(m_i2s_port);
}

void I2SMEMSSampler::stop() {
  i2s_stop(m_i2s_port);
  i2s_driver_uninstall(m_i2s_port);
}

int I2SMEMSSampler::read(int16_t *samples, int count) {
  size_t bytes_read = 0;
  esp_err_t result = i2s_read(m_i2s_port, m_raw_buffer, count * sizeof(int32_t), &bytes_read, portMAX_DELAY);
  if (result == ESP_OK) {
    int samples_read = bytes_read / sizeof(int32_t);
    for (int i = 0; i < samples_read; i++) {
      samples[i] = (int16_t)(m_raw_buffer[i] >> 16);
    }
    return samples_read;
  }
  return 0;
}

// I2SOutput Implementation
I2SOutput::I2SOutput(i2s_port_t i2s_port, i2s_pin_config_t &i2s_pins)
    : m_i2s_port(i2s_port), m_i2s_pins(i2s_pins), m_is_running(false), m_output_gain(0.3f) {
}

void I2SOutput::start(int sample_rate) {
  if (!m_is_running) {
    i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = sample_rate,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 4,
      .dma_buf_len = 64,
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0
    };
    i2s_driver_install(m_i2s_port, &i2s_config, 0, NULL);
    i2s_set_pin(m_i2s_port, &i2s_speaker_pins);
    i2s_start(m_i2s_port);
    m_is_running = true;
  }
}

void I2SOutput::stop() {
  if (m_is_running) {
    i2s_stop(m_i2s_port);
    i2s_driver_uninstall(m_i2s_port);
    m_is_running = false;
  }
}

void I2SOutput::write(int16_t *samples, int count) {
  if (m_is_running) {
    for (int i = 0; i < count; i++) {
      float sampleValue = samples[i] * m_output_gain;
      if (sampleValue > 32767) sampleValue = 32767;
      if (sampleValue < -32768) sampleValue = -32768;
      samples[i] = (int16_t)sampleValue;
    }
    size_t bytes_written = 0;
    i2s_write(m_i2s_port, samples, count * sizeof(int16_t), &bytes_written, portMAX_DELAY);
  }
}

void I2SOutput::set_output_gain(float gain) {
  if (gain < 0.0f) gain = 0.0f;
  if (gain > 1.0f) gain = 1.0f;
  m_output_gain = gain;
}

// Transport Implementation
Transport::Transport(OutputBuffer *output_buffer)
    : m_output_buffer(output_buffer), m_header(nullptr), m_header_size(0) {
}

void Transport::set_header(int header_size, uint8_t *header) {
  m_header_size = header_size;
  m_header = header;
}

// EspNowTransport Implementation
EspNowTransport::EspNowTransport(OutputBuffer *output_buffer, int wifi_channel)
    : Transport(output_buffer), m_wifi_channel(wifi_channel), m_tx_buffer_pos(0), m_tx_buffer_size(64) {
  m_tx_buffer = new int16_t[m_tx_buffer_size];
  m_peer_mac = peer_mac;
}

EspNowTransport::~EspNowTransport() {
  delete[] m_tx_buffer;
}

void EspNowTransport::begin() {
  g_esp_now_transport = this;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(m_wifi_channel, WIFI_SECOND_CHAN_NONE);
  Serial.print("My MAC Address: ");
  Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(on_data_recv);
  esp_now_register_send_cb(on_data_sent);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, m_peer_mac, 6);
  peerInfo.channel = m_wifi_channel;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  }
  Serial.println("ESP-NOW setup successful");
}

void EspNowTransport::add_sample(int16_t sample) {
  m_tx_buffer[m_tx_buffer_pos++] = sample;
  if (m_tx_buffer_pos >= m_tx_buffer_size) {
    flush();
  }
}

void EspNowTransport::flush() {
  if (m_tx_buffer_pos > 0) {
    int packet_size = m_header_size + (m_tx_buffer_pos * sizeof(int16_t));
    if (packet_size <= 250) {
      uint8_t packet[250];
      if (m_header_size > 0) {
        memcpy(packet, m_header, m_header_size);
      }
      memcpy(packet + m_header_size, m_tx_buffer, m_tx_buffer_pos * sizeof(int16_t));
      esp_now_send(m_peer_mac, packet, packet_size);
    }
    m_tx_buffer_pos = 0;
  }
}

void EspNowTransport::on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (g_esp_now_transport && len > TRANSPORT_HEADER_SIZE) {
    if (memcmp(data, transport_header, TRANSPORT_HEADER_SIZE) != 0) {
      return;
    }
    int audio_data_len = len - TRANSPORT_HEADER_SIZE;
    int16_t *samples = (int16_t *)(data + TRANSPORT_HEADER_SIZE);
    int sample_count = audio_data_len / sizeof(int16_t);
    for (int i = 0; i < sample_count; i++) {
      g_esp_now_transport->m_output_buffer->add_sample(samples[i]);
    }
  }
}

void EspNowTransport::on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("ESP-NOW send failed");
  }
}

// GenericDevBoardIndicatorLed Implementation
GenericDevBoardIndicatorLed::GenericDevBoardIndicatorLed()
    : m_is_flashing(false), m_flash_color(0), m_default_color(0), m_last_flash_time(0) {
}

void GenericDevBoardIndicatorLed::begin() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void GenericDevBoardIndicatorLed::set_default_color(uint32_t color) {
  m_default_color = color;
  if (!m_is_flashing) {
    digitalWrite(LED_BUILTIN, color > 0 ? HIGH : LOW);
  }
}

void GenericDevBoardIndicatorLed::set_is_flashing(bool is_flashing, uint32_t color) {
  m_is_flashing = is_flashing;
  m_flash_color = color;
  if (!is_flashing) {
    digitalWrite(LED_BUILTIN, m_default_color > 0 ? HIGH : LOW);
  }
}

void GenericDevBoardIndicatorLed::update() {
  if (m_is_flashing) {
    unsigned long now = millis();
    if (now - m_last_flash_time > 500) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      m_last_flash_time = now;
    }
  }
}

// Application Implementation
static void application_task(void *param) {
  Application *app = reinterpret_cast<Application *>(param);
  app->loop();
}

Application::Application() {
  m_output_buffer = new OutputBuffer(300 * 16);
  m_input = new I2SMEMSSampler(I2S_NUM_0, i2s_mic_pins, i2s_mic_Config, 128);
  m_output = new I2SOutput(I2S_NUM_1, i2s_speaker_pins);
  m_transport = new EspNowTransport(m_output_buffer, ESP_NOW_WIFI_CHANNEL);
  m_transport->set_header(TRANSPORT_HEADER_SIZE, transport_header);
  m_indicator_led = new GenericDevBoardIndicatorLed();
  if (I2S_SPEAKER_SD_PIN != -1) {
    pinMode(I2S_SPEAKER_SD_PIN, OUTPUT);
  }
}

void Application::begin() {
  m_indicator_led->set_default_color(0);
  m_indicator_led->set_is_flashing(true, 0xff0000);
  m_indicator_led->begin();
  Serial.print("My IDF Version is: ");
  Serial.println(esp_get_idf_version());
  Serial.print("My MAC Address is: ");
  Serial.println(WiFi.macAddress());
  m_transport->begin();
  m_indicator_led->set_default_color(0x00ff00);
  m_indicator_led->set_is_flashing(false, 0x00ff00);
  pinMode(GPIO_TRANSMIT_BUTTON, INPUT_PULLUP);
  m_output->start(SAMPLE_RATE);
  m_output_buffer->flush();
  TaskHandle_t task_handle;
  xTaskCreate(application_task, "application_task", 8192, this, 1, &task_handle);
}

void Application::loop() {
  int16_t *samples = new int16_t[128];
  if (!samples) {
    Serial.println("Memory allocation for samples failed");
    return;
  }
  while (true) {
    // Transmit mode
    if (!digitalRead(GPIO_TRANSMIT_BUTTON)) {
      Serial.println("Started transmitting");
      m_indicator_led->set_is_flashing(true, 0xff0000);
      m_output->stop();
      m_input->start();
      if (I2S_SPEAKER_SD_PIN != -1) {
        digitalWrite(I2S_SPEAKER_SD_PIN, LOW);
      }
      unsigned long start_time = millis();
      unsigned long flush_interval = 100;
      unsigned long last_flush = millis();
      while ((millis() - start_time < 1000) && (!digitalRead(GPIO_TRANSMIT_BUTTON))) {
        int samples_read = m_input->read(samples, 128);
        for (int i = 0; i < samples_read; i++) {
          m_transport->add_sample(samples[i]);
        }
        if (millis() - last_flush >= flush_interval) {
          m_transport->flush();
          last_flush = millis();
        }
      }
      m_transport->flush();
      Serial.println("Finished transmitting");
      m_indicator_led->set_is_flashing(false, 0xff0000);
      m_input->stop();
      m_output->start(SAMPLE_RATE);
    }
    // Receive mode
    Serial.println("Started receiving");
    if (I2S_SPEAKER_SD_PIN != -1) {
      digitalWrite(I2S_SPEAKER_SD_PIN, HIGH);
    }
    unsigned long start_time = millis();
    while ((millis() - start_time < 1000) && (digitalRead(GPIO_TRANSMIT_BUTTON))) {
      m_output_buffer->remove_samples(samples, 128);
      m_output->write(samples, 128);
      static_cast<GenericDevBoardIndicatorLed*>(m_indicator_led)->update();
    }
    if (I2S_SPEAKER_SD_PIN != -1) {
      digitalWrite(I2S_SPEAKER_SD_PIN, LOW);
    }
    Serial.println("Finished receiving");
  }
  delete[] samples;
}

// ============= Main Setup and Loop =============
Application *application;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 Walkie Talkie Starting...");
  application = new Application();
  application->begin();
  Serial.println("Application started");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}