#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>

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
// GPS
// ================================
HardwareSerial GPS_Serial(2);
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

TinyGPSPlus gps;

// ================================
// SD — MISMO ESTILO QUE TU EJEMPLO
// ================================
#define SD_MISO 19
#define SD_MOSI 23
#define SD_SCK  18
#define SD_CS   5

char todayFilename[20]; // "/20251117.csv"

// Funciones del ejemplo
void writeFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.print("Failed to open file for writing: ");
    Serial.println(path);
    return;
  }
  if (!file.print(message)) {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.print("Failed to open file for appending: ");
    Serial.println(path);
    return;
  }
  if (!file.print(message)) {
    Serial.println("Append failed");
  }
  file.close();
}


// Genera nombre tipo "/20251117.csv"
void generateFilename(DateTime now) {
  snprintf(todayFilename, sizeof(todayFilename),
           "/%04d%02d%02d.csv",
           now.year(), now.month(), now.day());
}


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
    lcd.print("RTC ERROR");
    while (1);
  }

  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  // SD INIT usando el estilo probado
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS)) {
    Serial.println("SD init FAIL");
    lcd.setCursor(0, 1);
    lcd.print("SD FAIL");
  } else {
    Serial.println("SD init OK");
    lcd.setCursor(0, 1);
    lcd.print("SD OK ");
  }

  delay(1000);
}


// ================================
// LOOP
// ================================
void loop() {

  // ====== GPS ======
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }

  bool gpsFix = gps.location.isValid() && gps.location.age() < 2000;

  // ====== Flujo ======
  updateFlowRate();

  // ====== RTC ======
  DateTime now = rtc.now();
  char timeString[20];
  snprintf(timeString, sizeof(timeString),
           "%02d:%02d:%02d",
           now.hour(), now.minute(), now.second());

  // LCD
  lcd.setCursor(0, 0);
  lcd.print(timeString);

  lcd.setCursor(9, 0);
  lcd.print(gpsFix ? "F " : "NF");

  lcd.setCursor(0, 1);
  lcd.print("Flow ");
  lcd.print(lastFlowLmin, 2);
  lcd.print(" L/m ");

  // =============== GUARDADO 3 Hz ===============
  static unsigned long lastSave = 0;

  if (millis() - lastSave >= 333) {
    lastSave = millis();

    // Nombre del archivo
    generateFilename(now);

    // Crear archivo si no existe
    if (!SD.exists(todayFilename)) {
      Serial.print("Creando archivo nuevo: ");
      Serial.println(todayFilename);
      writeFile(SD, todayFilename, "fecha,hora,flow,lat,lon,fix\n");
    }

    // Preparar la línea a escribir
    char line[120];
    snprintf(line, sizeof(line),
             "%04d-%02d-%02d,%s,%.3f,%.6f,%.6f,%d\n",
             now.year(), now.month(), now.day(),
             timeString,
             lastFlowLmin,
             gpsFix ? gps.location.lat() : 0.0,
             gpsFix ? gps.location.lng() : 0.0,
             gpsFix ? 1 : 0);

    // Guardar
    appendFile(SD, todayFilename, line);

    Serial.print("Guardado → ");
    Serial.println(line);
  }

  delay(5);
}

