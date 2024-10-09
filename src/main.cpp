#include <M5Core2.h>
// #include <driver/i2s.h>

#include "AudioTools.h"

#include "BluetoothA2DPSink.h"
#include <RingBuffer.hpp>

RingBufferInterleaved *g_ring = nullptr;

I2SStream i2s;
BluetoothA2DPSink a2dp_sink(i2s);
bool bypass = false;

#define XSTR(x) STR(x)
#define STR(x) #x

#ifdef A2DP_I2S_AUDIOTOOLS
#pragma message "A2DP_I2S_AUDIOTOOLS is set"
#else
#pragma message "A2DP_I2S_AUDIOTOOLS is not set" // here
#endif

#ifdef A2DP_LEGACY_I2S_SUPPORT
#pragma message "A2DP_LEGACY_I2S_SUPPORT is set" // here
#else
#pragma message "A2DP_LEGACY_I2S_SUPPORT is not set"
#endif

#pragma message "ESP_IDF_VERSION = " XSTR(ESP_IDF_VERSION) // 4.4.6-dirty

bool g_effect_right = false;
bool g_effect_left = false;

void audio_callback(int16_t *data, uint32_t sample_num) {
  // data is interleaved stereo with 16bit samples each
  for (int i = 0; i < sample_num; i++) {
    int16_t *left = &data[i * 2];
    int16_t *right = &data[i * 2 + 1];

    float rightf = *right;
    rightf *= 0.3;
    *right = static_cast<int16_t>(rightf);

    // if (g_effect_right) {
    //   float rightf = *right;
    //   rightf *= 0.1;
    //   *right = static_cast<int16_t>(rightf);
    // }
    // if (g_effect_left) {
    //   *left = 0;
    // }
  }

  static uint32_t processed_samples = 0;
  processed_samples += sample_num;
  if (processed_samples >= 44100) {
    processed_samples = 0;
    Serial.println("audio_callback");
  }
}

// dual button connected to Port.B
//  https://docs.m5stack.com/en/unit/dual_button
#define DUAL_BUTTON_BLUE 36 //effect right
#define DUAL_BUTTON_RED 26  //effect left

void setup() {
  M5.begin(true, false, true, true);
  pinMode(DUAL_BUTTON_BLUE, INPUT);
  pinMode(DUAL_BUTTON_RED, INPUT);
  // a2dp_sink.set_stream_reader(read_data_stream, true);
  M5.Lcd.setTextSize(2);
  M5.Lcd.print("\nAudiiSion Sound Lab.\n");
  char strtmp[100];
  sprintf(strtmp, "AudiiSion EP Ver.%s", "1.00");
  M5.Lcd.print(strtmp);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(0, 100);
  M5.Lcd.print("ON ");

  esp_log_level_set("*", ESP_LOG_INFO);

  Serial.println("setup");
  delay(1000);
  Serial.println("setup2");
  delay(100);
  g_ring = new RingBufferInterleaved();
  Serial.println("setup3");
  delay(100);
  a2dp_sink.set_raw_stream_reader_writer(audio_callback);
  Serial.println("setup4");
  delay(100);
  auto cfg = i2s.defaultConfig();

  //Internal speaker
  // cfg.pin_bck = 12;
  // cfg.pin_ws = 0;
  // cfg.pin_data = 2;
  // M5.Axp.SetSpkEnable(true);


  //External speaker
  i2s.end();
  M5.Axp.SetSpkEnable(false);
  cfg.pin_bck = 26;
  cfg.pin_ws = 25;
  cfg.pin_data = 22;
  i2s.begin(cfg);
  a2dp_sink.start("M5Blue");

  ESP_LOGI("main", "Available Heap: %zu", esp_get_free_heap_size());

  delay(1000);
}

void loop() {
  static int intCnt = 100;
  if (intCnt-- <= 1) {
    M5.update();
    Event &e = M5.Buttons.event;
    if (e & (E_TOUCH)) {
      // E_TOUCH, E_RELEASE, E_TAP, E_DBLTAP, E_PRESSING, E_PRESSED,
      // E_LONGPRESSIONG, E_LONGPRESSED
      bypass = !bypass;
      M5.Lcd.setCursor(0, 100);
      M5.Lcd.setTextSize(3);
      if (!bypass) {
        M5.Lcd.print("ON ");
      } else {
        M5.Lcd.print("OFF");
      }
      // a2dp_sink.start("M5Blue");
      delay(200);
    }
    intCnt = 100;
  }

  if (digitalRead(DUAL_BUTTON_BLUE) == LOW) {
    if (!g_effect_right){
      ESP_LOGI("main", "Effect Right");
      g_effect_right = true;
    }
  } else {
    g_effect_right = false;
  }

  if (digitalRead(DUAL_BUTTON_RED) == LOW) {
    if (!g_effect_left){
      ESP_LOGI("main", "Effect Left");
      g_effect_left = true;
      a2dp_sink.next();
    }
  } else {
    g_effect_left = false;
  }
}
