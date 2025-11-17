#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>

// Hardware configuration
constexpr int RXD2 = 16;
constexpr int TXD2 = 17;
constexpr int GPS_BAUD = 9600;
constexpr int SD_CS_PIN = 5;           // Chip select for SD card (adjust if needed)
constexpr int SENSOR_PIN = 34;         // Placeholder analog sensor pin

constexpr uint8_t LCD_ADDRESS = 0x27;  // Change if your LCD uses a different address
constexpr uint8_t LCD_COLUMNS = 16;
constexpr uint8_t LCD_ROWS = 2;

// Logging configuration
constexpr unsigned long LOG_INTERVAL_MS = 333; // ~3 Hz
const char *LOG_FILE = "/datalog.csv";

// Struct to hold a single reading
struct DataRecord {
  DateTime timestamp;
  double latitude;
  double longitude;
  double altitudeMeters;
  double speedKmph;
  uint8_t satellites;
  float sensorValue;
};

RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
unsigned long lastLog = 0;

void setupRtc();
void setupLcd();
void setupGps();
void setupSd();
void ensureLogFile();
DataRecord captureData();
String formatDateTime(const DateTime &timestamp);
String formatCsv(const DataRecord &record);
void logRecord(const DataRecord &record);
void updateLcd(const DataRecord &record);

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA = 21, SCL = 22 on many ESP32 boards

  setupLcd();
  setupRtc();
  setupGps();
  setupSd();

  lcd.clear();
  lcd.print("Logger ready");
  Serial.println("System initialized. Beginning logging loop.");
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  const unsigned long nowMs = millis();
  if (nowMs - lastLog >= LOG_INTERVAL_MS) {
    lastLog = nowMs;
    DataRecord record = captureData();
    logRecord(record);
    updateLcd(record);
  }
}

void setupRtc() {
  if (!rtc.begin()) {
    lcd.print("RTC NOT FOUND");
    Serial.println("RTC NOT FOUND");
    while (true) {
      delay(10);
    }
  }

  // Uncomment to set the RTC once, then re-upload with the line commented out
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void setupLcd() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Starting...");
}

void setupGps() {
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("GPS serial started");
}

void setupSd() {
  lcd.clear();
  lcd.print("Mounting SD...");

  if (!SD.begin(SD_CS_PIN)) {
    lcd.clear();
    lcd.print("SD MOUNT FAIL");
    Serial.println("SD card mount failed. Check wiring/CS pin.");
    while (true) {
      delay(10);
    }
  }

  ensureLogFile();
}

void ensureLogFile() {
  if (SD.exists(LOG_FILE)) {
    return;
  }

  File file = SD.open(LOG_FILE, FILE_WRITE);
  if (!file) {
    lcd.clear();
    lcd.print("FILE CREATE ERR");
    Serial.println("Failed to create log file.");
    while (true) {
      delay(10);
    }
  }

  file.println("date,time,latitude,longitude,altitude_m,speed_kmph,satellites,sensor");
  file.close();
}

DataRecord captureData() {
  DataRecord record{};
  record.timestamp = rtc.now();

  if (gps.location.isValid()) {
    record.latitude = gps.location.lat();
    record.longitude = gps.location.lng();
  } else {
    record.latitude = NAN;
    record.longitude = NAN;
  }

  record.altitudeMeters = gps.altitude.isValid() ? gps.altitude.meters() : NAN;
  record.speedKmph = gps.speed.isValid() ? gps.speed.kmph() : NAN;
  record.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;

  // Placeholder sensor read; replace with your actual sensor logic
  record.sensorValue = analogRead(SENSOR_PIN);

  return record;
}

String formatDateTime(const DateTime &timestamp) {
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d,%02d:%02d:%02d",
           timestamp.year(), timestamp.month(), timestamp.day(),
           timestamp.hour(), timestamp.minute(), timestamp.second());
  return String(buffer);
}

String formatCsv(const DataRecord &record) {
  String dateTime = formatDateTime(record.timestamp);

  char buffer[128];
  snprintf(buffer, sizeof(buffer), ",%0.6f,%0.6f,%0.2f,%0.2f,%u,%0.2f",
           record.latitude, record.longitude, record.altitudeMeters,
           record.speedKmph, record.satellites, record.sensorValue);

  return dateTime + String(buffer);
}

void logRecord(const DataRecord &record) {
  File file = SD.open(LOG_FILE, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open log file for appending.");
    lcd.clear();
    lcd.print("LOG WRITE ERR");
    return;
  }

  String line = formatCsv(record);
  file.println(line);
  file.close();
  Serial.println(line);
}

void updateLcd(const DataRecord &record) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(formatDateTime(record.timestamp));

  lcd.setCursor(0, 1);
  if (gps.location.isValid()) {
    lcd.print(record.latitude, 4);
    lcd.print(",");
    lcd.print(record.longitude, 4);
  } else {
    lcd.print("GPS acquiring");
  }
}
