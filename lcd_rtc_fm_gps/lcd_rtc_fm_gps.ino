#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>

// ================================
// RTC
// ================================
RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ================================
// SENSOR DE FLUJO
// ================================
const int flowPin = 13;
volatile unsigned long pulseCount = 0;

const float PULSES_PER_LITER = 200.0;
const unsigned long FLOW_INTERVAL = 5000;

float lastFlowLmin = 0.0;

void IRAM_ATTR pulseInterrupt() {
  pulseCount++;
}

void updateFlowRate() {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= FLOW_INTERVAL) {
    previousMillis = currentMillis;

    float liters = pulseCount / PULSES_PER_LITER;
    float flowLmin = liters * (60.0 / (FLOW_INTERVAL / 1000.0));

    pulseCount = 0;
    lastFlowLmin = flowLmin;

    Serial.print("Flujo: ");
    Serial.print(lastFlowLmin);
    Serial.println(" L/min");
  }
}

// ================================
// GPS CONFIG
// ================================
HardwareSerial GPS_Serial(2);
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

TinyGPSPlus gps;


// ================================
// SETUP
// ================================
void setup() {
  Serial.begin(115200);

  pinMode(flowPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(flowPin), pulseInterrupt, FALLING);

  Wire.begin(21, 22);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Iniciando...");

  if (!rtc.begin()) {
    lcd.clear();
    lcd.print("RTC NOT FOUND");
    while (1);
  }

  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS iniciado...");

  delay(1000);
}


// ================================
// LOOP
// ================================
void loop() {

  // ======= LEER GPS =======
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }

  bool gpsFix = gps.location.isValid() && gps.location.age() < 2000;

  // ======= FLUJO =======
  updateFlowRate();

  // ======= HORA =======
  DateTime now = rtc.now();

  char timeString[17];
  snprintf(timeString, sizeof(timeString),
           "%02d:%02d:%02d",
           now.hour(), now.minute(), now.second());

  // ==============================
  // LCD FILA 1 → Hora + Fix GPS
  // ==============================
  lcd.setCursor(0, 0);
  lcd.print(timeString);

  lcd.setCursor(9, 0);  // espacio suficiente
  if (gpsFix) {
    lcd.print("F ");   // Fix
  } else {
    lcd.print("NF");   // No Fix
  }

  // ==============================
  // LCD FILA 2 → Flujo
  // ==============================
  lcd.setCursor(0, 1);
  lcd.print("Flow ");
  lcd.print(lastFlowLmin, 2);
  lcd.print(" L/m  ");

  delay(200);
}
