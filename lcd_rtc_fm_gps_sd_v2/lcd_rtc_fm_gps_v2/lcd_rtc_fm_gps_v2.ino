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
volatile unsigned long pulseCount = 0; // Pulsos desde la última medición

// Constantes de calibración del sensor
const float ML_PER_PULSE = 5.0; // Mililitros por pulso (ajusta según tu sensor)
const float PULSES_PER_LITER = 200.0; // Pulsos por litro
const unsigned long FLOW_INTERVAL = 1000; // Periodo de muestreo en ms (1 segundo)

float lastFlowLmin = 0.0;
float lastFlowLhour = 0.0;

// Variables de Acumulación
// Cambiamos el nombre para ser más explícitos y la inicializamos globalmente
float totalLitersConsumed = 0.0; 


// ===== INTERRUPCIÓN =====
void IRAM_ATTR pulseInterrupt() {
  pulseCount++;
}

// ================================
// GPS (sin cambios)
// ================================
HardwareSerial GPS_Serial(2);
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600
#define LCD_UPDATE 5000
#define SD_HZ       1000

TinyGPSPlus gps;

// ================================
// SD (sin cambios)
// ================================
#define SD_MISO 19
#define SD_MOSI 23
#define SD_SCK  18
#define SD_CS   5

char todayFilename[20];

void writeFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_WRITE);
  if (!file) return;
  file.print(message);
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) return;
  file.print(message);
  file.close();
}

void generateFilename(DateTime now) {
  snprintf(todayFilename, sizeof(todayFilename),
           "/%04d%02d%02d.csv",
           now.year(), now.month(), now.day());
}

// ================================
// FREE RTOS LCD TASK
// ================================
TaskHandle_t hTaskLCD;

int lcdPage = 0;
char lcdTimeString[20] = "";
bool lcdGpsFix = false;
float lcdFlow = 0.0;
float lcdFlowHour = 0.0;
double lcdLat = 0.0;
double lcdLon = 0.0;
float lcdSpeed = 0.0;
float lcdTotalLiters = 0.0; // Nuevo para el total

// ================================
// TASK LCD (Actualizada)
// ================================
void TaskLCD(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(LCD_UPDATE));

  for (;;) {
    lcd.clear();

    if (lcdPage == 0) {
      lcd.setCursor(0, 0);
      lcd.print(lcdTimeString);

      lcd.setCursor(9, 0);
      lcd.print(lcdGpsFix ? "F" : "NF");

      lcd.setCursor(0, 1);
      lcd.print("Flow ");
      lcd.print(lcdFlow, 2);
      lcd.print(" L/m");

    } else if (lcdPage == 1) {
      lcd.setCursor(0, 0);
      lcd.print("Lat:");
      lcd.print(lcdLat, 4);

      lcd.setCursor(0, 1);
      lcd.print("Lon:");
      lcd.print(lcdLon, 4);

    } else if (lcdPage == 2) {
      lcd.setCursor(0, 0);
      lcd.print("Flow ");
      lcd.print(lcdFlowHour, 1);
      lcd.print(" L/h");

      lcd.setCursor(0, 1);
      lcd.print("Speed ");
      lcd.print(lcdSpeed, 1);
      lcd.print(" km/h");

    } else if (lcdPage == 3) {
      
      lcd.setCursor(0, 0);
      lcd.print("Consumido:"); // Etiqueta más clara
      lcd.print(lcdTotalLiters, 2); // Usa el total
      lcd.print(" L"); // Corregido a Litros (L)

      lcd.setCursor(0, 1);
      lcd.print("Flow L/m: ");
      lcd.print(lcdFlow, 2);
    }

    lcdPage++;
    if (lcdPage > 3) lcdPage = 0;

    vTaskDelay(pdMS_TO_TICKS(LCD_UPDATE));
  }
}

// ================================
// SETUP (sin cambios)
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

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  SD.begin(SD_CS);

  lcd.setCursor(0, 1);
  lcd.print("SD OK");

  xTaskCreatePinnedToCore(
    TaskLCD,
    "LCD_TASK",
    2048 * 2,
    NULL,
    1,
    &hTaskLCD,
    1
  );
}

// ================================
// LOOP (Actualizada)
// ================================
void loop() {

  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }

  bool gpsFix = gps.location.isValid() && gps.location.age() < 2000;

  // ===== Flujo =====
  static unsigned long prevFlowMillis = 0;
  unsigned long nowMs = millis();

  if (nowMs - prevFlowMillis >= FLOW_INTERVAL) {
    prevFlowMillis = nowMs;

    // 1. CALCULAR LITROS CONSUMIDOS EN ESTE INTERVALO (5 segundos)
    // Usamos pulseCount que es la cantidad de pulsos acumulados en FLOW_INTERVAL
    float litersInInterval = (float)pulseCount / PULSES_PER_LITER;

    // 2. ACUMULAR EL TOTAL
    totalLitersConsumed += litersInInterval; // Acumula el consumo total

    // 3. CALCULAR LA TASA DE FLUJO INSTANTÁNEA (L/min y L/h)
    // lastFlowLmin = (Litros en intervalo) * (Tasa de Intervalos por minuto)
    lastFlowLmin = litersInInterval * (60000.0 / FLOW_INTERVAL);
    lastFlowLhour = lastFlowLmin * 60.0;

    // 4. RESETEAR EL CONTADOR DE PULSOS
    // Esto es crucial para que la siguiente medición (después de 5s) sea limpia.
    pulseCount = 0; 
    
    // Debug en Serial
    Serial.printf("Pulsos en %i s: %i | Consumo Intervalo: %.3f L | Flujo: %.2f L/min | Acumulado Total: %.3f L\n", 
                  (int)(FLOW_INTERVAL/1000), (int)pulseCount, litersInInterval, lastFlowLmin, totalLitersConsumed);

  }
  // El código continua ejecutándose aquí sin esperar los 5 segundos.

  // ===== RTC (sin cambios) =====
  DateTime now = rtc.now();
  char timeString[20];
  snprintf(timeString, sizeof(timeString),
           "%02d:%02d:%02d",
           now.hour(), now.minute(), now.second());

  // ===== Pasar info a LCD (Actualizada) =====
  strcpy(lcdTimeString, timeString);
  lcdGpsFix = gpsFix;
  lcdFlow = lastFlowLmin;
  lcdFlowHour = lastFlowLhour;
  lcdLat = gpsFix ? gps.location.lat() : 0.0;
  lcdLon = gpsFix ? gps.location.lng() : 0.0;
  lcdSpeed = gpsFix ? gps.speed.kmph() : 0.0;
  lcdTotalLiters = totalLitersConsumed; // Pasa el total al LCD

  // ===== Guardado SD (Actualizada) =====
  static unsigned long lastSave = 0;

  if (millis() - lastSave >= SD_HZ) {
    lastSave = millis();

    generateFilename(now);

    if (!SD.exists(todayFilename)) {
      writeFile(SD, todayFilename,
                 "fecha,hora,flow_Lmin,lat,lon,fix,vel_kmh,litros_acumulados\n"); // Encabezado más claro
    }

    char line[200];
    snprintf(line, sizeof(line),
             "%04d-%02d-%02d,%s,%.3f,%.6f,%.6f,%d,%.2f,%.3f\n",
             now.year(), now.month(), now.day(),
             timeString,
             lastFlowLmin, // Flujo instantáneo
             gpsFix ? gps.location.lat() : 0.0,
             gpsFix ? gps.location.lng() : 0.0,
             gpsFix ? 1 : 0,
             lcdSpeed,
             totalLitersConsumed); // ¡Guarda el acumulado!

    appendFile(SD, todayFilename, line);

    Serial.print("Guardado → ");
    Serial.println(line);
  }

  delay(5);
}