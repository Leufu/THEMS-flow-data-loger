#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal_I2C.h>

RTC_DS3231 rtc;

// Change to your LCD address (0x27 is most common)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA=21, SCL=22

  // Init LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Init RTC
  if (!rtc.begin()) {
    lcd.print("RTC NOT FOUND");
    while (1);
  }

  // Only set the RTC once:
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void loop() {
  DateTime now = rtc.now();

  // Format row 1: Date => YYYY-MM-DD
  char dateString[17];
  snprintf(dateString, sizeof(dateString),
           "%04d-%02d-%02d",
           now.year(), now.month(), now.day());

  // Format row 2: Time => HH:MM:SS
  char timeString[17];
  snprintf(timeString, sizeof(timeString),
           "%02d:%02d:%02d",
           now.hour(), now.minute(), now.second());

  // Print to LCD
  lcd.setCursor(0, 0);
  lcd.print(dateString);

  lcd.setCursor(0, 1);
  lcd.print(timeString);

  delay(500);
}

