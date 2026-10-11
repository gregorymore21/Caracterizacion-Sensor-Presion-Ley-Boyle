#include <Arduino.h>

// Definición de pines para la conexión
#define HX710_DOUT_PIN 19
#define HX710_SCK_PIN  21

// Parámetros y constantes de configuración
const double ATMOSPHERIC_P_KPA = 101.3;  // Presión atmosférica en kPa
const double INITIAL_VOL_ML    = 1.00;   // Volumen inicial en mL
const double DEAD_VOL_ML       = 0.76;   // Volumen muerto en mL
const double MAX_SAFE_P_KPA    = 35.0;   // Límite de seguridad
const double SENSOR_LIMIT_KPA  = 40.0;   // Límite máximo del sensor

// Configuración del promediado de muestras
const int SAMPLE_COUNT = 20;

// Prototipos de funciones
bool readSensorRaw(long &rawData);
bool getAverageAndStdDev(double &meanVal, double &stdDevVal);
double computeReferencePressure(double vj);
double computeMinInjectedVolume();
void displayConfiguration();
void processZeroCalibration();
void processIncomingCommand(String rawInput);

void setup() {
  Serial.begin(115200);
  
  pinMode(HX710_SCK_PIN, OUTPUT);
  digitalWrite(HX710_SCK_PIN, LOW);
  pinMode(HX710_DOUT_PIN, INPUT);
  
  delay(500);
  displayConfiguration();
}

void loop() {
  if (Serial.available() > 0) {
    String inputBuffer = Serial.readStringUntil('\n');
    inputBuffer.trim();
    
    if (inputBuffer.length() == 0) return;

    if (inputBuffer.equalsIgnoreCase("z")) {
      processZeroCalibration();
    } else {
      processIncomingCommand(inputBuffer);
    }
  }
}

// Lectura de datos del módulo HX710B
bool readSensorRaw(long &rawData) {
  uint32_t startTime = millis();
  
  // Esperar a que DOUT baje de nivel (dato listo)
  while (digitalRead(HX710_DOUT_PIN) == HIGH) {
    if (millis() - startTime > 1000) {
      return false; // Tiempo de espera agotado
    }
    delay(1);
  }

  uint32_t bitBuffer = 0;
  portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
  
  portENTER_CRITICAL(&mutex);
  for (int b = 0; b < 24; b++) {
    digitalWrite(HX710_SCK_PIN, HIGH);
    delayMicroseconds(2);
    bitBuffer = (bitBuffer << 1) | (uint32_t)digitalRead(HX710_DOUT_PIN);
    digitalWrite(HX710_SCK_PIN, LOW);
    delayMicroseconds(2);
  }
  
  // Pulso extra (25) para configurar lectura de 10 Hz diferencial
  digitalWrite(HX710_SCK_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(HX710_SCK_PIN, LOW);
  portEXIT_CRITICAL(&mutex);

  // Ajuste de signo para entero de 24 bits
  if (bitBuffer & 0x800000UL) {
    bitBuffer |= 0xFF000000UL;
  }
  
  rawData = (long)(int32_t)bitBuffer;
  return true;
}

// Cálculo de media y desviación estándar para N lecturas
bool getAverageAndStdDev(double &meanVal, double &stdDevVal) {
  static long samples[SAMPLE_COUNT];
  double sum = 0.0;
  
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    if (!readSensorRaw(samples[i])) return false;
    sum += samples[i];
  }
  
  meanVal = sum / SAMPLE_COUNT;
  
  double sumSquaredDiff = 0.0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    double diff = samples[i] - meanVal;
    sumSquaredDiff += diff * diff;
  }
  
  stdDevVal = sqrt(sumSquaredDiff / (SAMPLE_COUNT - 1));
  return true;
}

// Ecuación de la Ley de Boyle
double computeReferencePressure(double vj) {
  return ATMOSPHERIC_P_KPA * (INITIAL_VOL_ML - vj) / (vj + DEAD_VOL_ML);
}

// Cálculo del volumen mínimo inyectable
double computeMinInjectedVolume() {
  double ratio = MAX_SAFE_P_KPA / ATMOSPHERIC_P_KPA;
  return (INITIAL_VOL_ML - ratio * DEAD_VOL_ML) / (1.0 + ratio);
}

// Muestra parámetros de inicialización por puerto serie
void displayConfiguration() {
  Serial.println("# ---- Configuración de Parámetros ----");
  Serial.printf("# Patm = %.3f kPa | V0 = %.2f mL | Vm = %.3f mL | N = %d\n",
                ATMOSPHERIC_P_KPA, INITIAL_VOL_ML, DEAD_VOL_ML, SAMPLE_COUNT);
  Serial.printf("# Límite seguro Vj,min = %.3f mL (Pmax = %.0f kPa)\n", computeMinInjectedVolume(), MAX_SAFE_P_KPA);
  Serial.println("# Opciones disponibles:");
  Serial.println("#   z                -> Calibración / nivel de cero");
  Serial.println("#   ciclo,sentido,Vj -> Formato de prueba (Ej: 1,C,0.94)");
  Serial.println("# Estructura CSV: ciclo,sentido,Vj,Pref,media,desv,N");
}

// Ajuste del valor cero
void processZeroCalibration() {
  double avg, dev;
  if (!getAverageAndStdDev(avg, dev)) {
    Serial.println("# ERROR: Sin respuesta del módulo HX710B");
    return;
  }
  Serial.printf("# CERO MEDIDO: Promedio = %.2f | Desv = %.2f | N = %d\n", avg, dev, SAMPLE_COUNT);
  Serial.printf("0,Z,%.2f,%.3f,%.2f,%.2f,%d\n", INITIAL_VOL_ML, 0.0, avg, dev, SAMPLE_COUNT);
}

// Procesamiento y validación de comandos ingresados
void processIncomingCommand(String rawInput) {
  rawInput.replace(';', ',');
  rawInput.replace(' ', ',');
  
  int sep1 = rawInput.indexOf(',');
  int sep2 = rawInput.indexOf(',', sep1 + 1);
  
  if (sep1 < 0 || sep2 < 0) {
    Serial.println("# ERROR: Formato incorrecto. Utilice: ciclo,sentido,Vj");
    return;
  }

  int cycle = rawInput.substring(0, sep1).toInt();
  String direction = rawInput.substring(sep1 + 1, sep2);
  double injectedVol = rawInput.substring(sep2 + 1).toDouble();
  
  direction.toUpperCase();

  if (cycle <= 0) {
    Serial.println("# ERROR: Número de ciclo no válido");
    return;
  }
  if (direction != "C" && direction != "D") {
    Serial.println("# ERROR: Sentido debe ser 'C' (Compresión) o 'D' (Descompresión)");
    return;
  }
  if (injectedVol > INITIAL_VOL_ML || injectedVol <= 0) {
    Serial.println("# ERROR: Valor de Vj fuera de rango permitido");
    return;
  }

  double pRef = computeReferencePressure(injectedVol);
  if (pRef > SENSOR_LIMIT_KPA) {
    Serial.printf("# ALERTA: Pref de %.2f kPa supera el límite del sensor (%.0f kPa)\n", pRef, SENSOR_LIMIT_KPA);
    return;
  }

  double avg, dev;
  if (!getAverageAndStdDev(avg, dev)) {
    Serial.println("# ERROR: Fallo de lectura del sensor");
    return;
  }

  // Imprimir resultado en formato CSV
  Serial.printf("%d,%s,%.2f,%.3f,%.2f,%.2f,%d\n", cycle, direction.c_str(), injectedVol, pRef, avg, dev, SAMPLE_COUNT);
}
