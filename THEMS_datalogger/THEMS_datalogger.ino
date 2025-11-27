#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>

// ================================
// CONFIGURACIÓN GENERAL
// ================================
#define LCD_REFRESH_MS      1000  // Refresco datos: 1 seg
#define LCD_PAGE_CHANGE_SEC 5     // Cambio página: 5 seg
#define SD_HZ               1000  // Guardado SD: 1 seg

// ================================
// OBJETOS
// ================================
RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(0x27, 16, 2);
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);

// ================================
// SENSOR DE FLUJO
// ================================
const int flowPin = 13;
volatile unsigned long pulseCount = 0; 

// --- CALIBRACIÓN ---
const float PULSES_PER_LITER = 200.0; // AJUSTAR SEGÚN SENSOR
const unsigned long FLOW_INTERVAL = 1000; 

float lastFlowLmin = 0.0;
float lastFlowLhour = 0.0;
float totalLitersConsumed = 0.0; 

void IRAM_ATTR pulseInterrupt() {
  pulseCount++;
}

// ================================
// GPS PINOUT
// ================================
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

// ================================
// SD PINOUT
// ================================
#define SD_MISO 19
#define SD_MOSI 23
#define SD_SCK  18
#define SD_CS   5

char todayFilename[32]; // Aumentado ligeramente por seguridad

// Funciones SD Helper
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
  // Formato: /20231127.csv
  snprintf(todayFilename, sizeof(todayFilename),
           "/%04d%02d%02d.csv",
           now.year(), now.month(), now.day());
}

// ================================
// VARIABLES VISUALIZACIÓN
// ================================
TaskHandle_t hTaskLCD;
int lcdPage = 0;
int lcdSecCounter = 0; 

char lcdTimeString[20] = "";
bool lcdGpsFix = false;
float lcdFlow = 0.0;
float lcdFlowHour = 0.0;
float lcdSpeedKnots = 0.0;
float lcdTotalLiters = 0.0;

// ================================
// TAREA LCD (PANTALLA)
// ================================
void TaskLCD(void *pvParameters) {
  for (;;) {
    lcd.clear();

    // PAGINA 0: FLUJO
    if (lcdPage == 0) {
      lcd.setCursor(0, 0); lcd.print("L/m: "); lcd.print(lcdFlow, 1); 
      lcd.setCursor(0, 1); lcd.print("L/h: "); lcd.print(lcdFlowHour, 0); 
    } 
    // PAGINA 1: NAV (Hora + Vel)
    else if (lcdPage == 1) {
      lcd.setCursor(0, 0); lcd.print(lcdTimeString);
      lcd.setCursor(13, 0); lcd.print(lcdGpsFix ? "FIX" : "NO");
      
      lcd.setCursor(0, 1); lcd.print("Vel: "); 
      lcd.print(lcdSpeedKnots, 1); lcd.print(" Nud");
    } 
    // PAGINA 2: TOTAL
    else if (lcdPage == 2) {
      lcd.setCursor(0, 0); lcd.print(lcdTimeString);
      lcd.setCursor(13, 0); lcd.print(lcdGpsFix ? "FIX" : "NO");

      lcd.setCursor(0, 1); lcd.print("Tot: "); 
      lcd.print(lcdTotalLiters, 1); lcd.print(" L");
    }

    lcdSecCounter++; 
    if (lcdSecCounter >= LCD_PAGE_CHANGE_SEC) {
      lcdSecCounter = 0;
      lcdPage++;
      if (lcdPage > 2) lcdPage = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(LCD_REFRESH_MS));
  }
}

// ================================
// SETUP
// ================================
void setup() {
  Serial.begin(115200);

  pinMode(flowPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(flowPin), pulseInterrupt, FALLING);

  Wire.begin(21, 22);
  lcd.init(); lcd.backlight();
  lcd.setCursor(0,0); lcd.print("Iniciando...");

  if (!rtc.begin()) { lcd.clear(); lcd.print("RTC ERROR"); while (1); }

  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if(!SD.begin(SD_CS)){
     lcd.setCursor(0,1); lcd.print("SD ERROR"); delay(2000);
  }

  // Lanzar tarea LCD en Núcleo 1
  xTaskCreatePinnedToCore(TaskLCD, "LCD_TASK", 4096, NULL, 1, &hTaskLCD, 1);
}

// ================================
// LOOP PRINCIPAL
// ================================
void loop() {
  // 1. Leer GPS
  while (GPS_Serial.available() > 0) gps.encode(GPS_Serial.read());
  bool gpsFix = gps.location.isValid(); // Simplificado

  // 2. Calcular Flujo
  static unsigned long prevFlowMillis = 0;
  if (millis() - prevFlowMillis >= FLOW_INTERVAL) {
    prevFlowMillis = millis();
    
    float litersInInterval = (float)pulseCount / PULSES_PER_LITER;
    totalLitersConsumed += litersInInterval; 

    lastFlowLmin = litersInInterval * (60000.0 / FLOW_INTERVAL);
    lastFlowLhour = lastFlowLmin * 60.0;
    pulseCount = 0; 
  }

  // 3. Preparar Datos Globales
  DateTime now = rtc.now();
  char timeString[20];
  snprintf(timeString, sizeof(timeString), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());

  strcpy(lcdTimeString, timeString);
  lcdGpsFix = gpsFix;
  lcdFlow = lastFlowLmin;
  lcdFlowHour = lastFlowLhour;
  lcdSpeedKnots = gpsFix ? gps.speed.knots() : 0.0;
  lcdTotalLiters = totalLitersConsumed;

  // 4. Guardar en SD (CON UNIDADES EN EL ENCABEZADO)
  static unsigned long lastSave = 0;
  if (millis() - lastSave >= SD_HZ) {
    lastSave = millis();
    generateFilename(now);

    // --- ENCABEZADO DEL CSV (Aquí definimos las unidades) ---
    if (!SD.exists(todayFilename)) {
      // Explicación de las columnas:
      // Fecha, Hora, Flujo(L/min), Flujo(L/hora), Lat(grados), Lon(grados), Fix(0/1), Vel(Nudos), Total(Litros)
      writeFile(SD, todayFilename, "Fecha,Hora,Flujo_Lmin,Flujo_Lh,Lat_deg,Lon_deg,Fix,Vel_Knots,Total_Litros\n"); 
    }

    // --- TRAMA DE DATOS ---
    char line[256];
    snprintf(line, sizeof(line),
             "%04d-%02d-%02d,%s,%.2f,%.1f,%.6f,%.6f,%d,%.2f,%.2f\n",
             now.year(), now.month(), now.day(),
             timeString,
             lastFlowLmin,        // Flujo_Lmin
             lastFlowLhour,       // Flujo_Lh
             gpsFix ? gps.location.lat() : 0.0,
             gpsFix ? gps.location.lng() : 0.0,
             gpsFix ? 1 : 0,
             lcdSpeedKnots,       // Vel_Knots
             totalLitersConsumed  // Total_Litros
             );

    appendFile(SD, todayFilename, line);
    // Serial.print("SD Log: "); Serial.print(line); // Descomentar para debug
  }
  delay(1);
}