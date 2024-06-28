#include <M5Core2.h>

#include "BluetoothA2DPSink.h"
#include <M5Core2.h>
BluetoothA2DPSink a2dp_sink;
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

void audio_callback(int16_t *data, uint32_t len) { 
  //data is interleaved stereo with 16bit samples each
  for (int i = 0; i < len / 2; i++) {
    int16_t *left = &data[i * 2];
    int16_t *right = &data[i * 2 + 1];
    float rightf = *right;
    rightf *= 0.3;
    *right = static_cast<int16_t>(rightf);
  }
}

void setup() {
  M5.begin(true, true, true, true);
  a2dp_sink.set_raw_stream_reader_writer(audio_callback);
  a2dp_sink.start("AudiiSion");
  // a2dp_sink.set_stream_reader(read_data_stream, true);
  M5.Lcd.setTextSize(2);
  M5.Lcd.print("\nAudiiSion Sound Lab.\n");
  char strtmp[100];
  sprintf(strtmp, "AudiiSion EP Ver.%s", "1.00");
  M5.Lcd.print(strtmp);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(0, 100);
  M5.Lcd.print("ON ");
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
      a2dp_sink.start("AudiiSion");
      delay(200);
    }
    intCnt = 100;
  }
}
