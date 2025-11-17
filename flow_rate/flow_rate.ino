// ===== CONFIGURACIÓN DEL SENSOR DE FLUJO =====
const int flowPin = 13;
volatile unsigned long pulseCount = 0;

// Pulsos por litro del flujómetro (ajusta si tu modelo usa otro valor)
const float PULSES_PER_LITER = 200.0;

// Intervalo de medición en milisegundos
const unsigned long FLOW_INTERVAL = 5000;  // 5 segundos


// ===== RUTINA DE INTERRUPCIÓN =====
void IRAM_ATTR pulseInterrupt() {
  pulseCount++;
}


// ===== FUNCIÓN PARA OBTENER EL CAUDAL EN L/h =====
float readFlowRate() {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= FLOW_INTERVAL) {

    previousMillis = currentMillis;

    // Convertir pulsos en litros
    float liters = pulseCount / PULSES_PER_LITER;

    // Convertir a litros por hora
    float flowRate = liters * (60000.0 / FLOW_INTERVAL) * 60.0;

    // Reiniciar contador
    pulseCount = 0;

    return flowRate;
  }

  return -1;  // Todavía no hay nuevo dato
}


// ===== SETUP =====
void setup() {
  Serial.begin(115200);

  pinMode(flowPin, INPUT);

  // Interrupción por flanco de bajada (pulso del sensor)
  attachInterrupt(digitalPinToInterrupt(flowPin), pulseInterrupt, FALLING);

  Serial.println("Sensor de flujo listo...");
}


// ===== LOOP =====
void loop() {
  float flow = readFlowRate();

  if (flow >= 0) {  // Solo imprimir cuando hay nuevo cálculo
    Serial.print("Flujo: ");
    Serial.print(flow);
    Serial.println(" L/h");
  }
}
