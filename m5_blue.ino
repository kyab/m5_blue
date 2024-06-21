#include <M5Core2.h>

#include "BluetoothA2DPSink.h"
#include <M5Core2.h>
BluetoothA2DPSink a2dp_sink;
bool bypass = false;

void setup() {
  M5.begin(true, true, true, true);
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
