// main.cpp
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h> //BLE2904 solo indica formato (int/float/etc.) y no habilita notificaciones.
#include <string.h>
#include <FastLED.h>
#include <Keypad.h>

/*
  Integración BLE + motor PWM + patrones (TRI/SQR/SIN/CONST/STOP) + NeoPixel + Keypad
  - Mantiene nombre BLE y UUIDs del programa que compartiste
  - Serial y BLE aceptan los mismos comandos (texto, no bloqueante)
  - Keypad preservado (puede usarse para modos físicos si deseas)
  - Motor en un solo sentido: usamos 2 canales ledc, pero siempre aplicamos PWM en uno y 0 en el otro
  - Notificaciones por BLE con valor actual (cada X ms)
*/

// ============ Hardware / BLE config (mantener los mismos UUIDs solicitados) ============
#define DATA_PIN 48
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// BLE identifiers
#define SERVICE_UUID                "c8ab9930-e233-402d-97a8-a9cfc8db12e4"
#define LED_CHARACTERISTIC_UUID     "d6ff81ff-2030-4f30-a7be-41f2fbc03a9b" // write (control LED / botones)
#define SENSOR_CHARACTERISTIC_UUID  "03bea938-ba38-48e1-928c-5a300a294fdd" // notify (estado)

// BLE globals (like en tu código original)
BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
unsigned long t0 = 0;

// ============ Motor / PWM config ============
#define MOT_A1_PIN 4   // pin física A1
#define MOT_A2_PIN 5   // pin física A2

// Usamos un canal PWM "convención": CH_DIR = 0 (lo dejamos 0), CH_PWM = 1 -> valor pwm
#define CH_DIR 0
#define CH_PWM 1
#define PWM_FREQ 20000
#define PWM_RESOLUTION 8  // 0..255

// estado del motor
volatile int current_pwm_value = 0; // 0..255 actual aplicado (si CONST o modos), para notificar
// modo actual: 0=STOP, 1=TRI(sierra), 2=SQR, 3=SIN, 4=CONST
volatile int modo = 0;

// ============ Keypad config (se deja operativo) ============
const byte ROWS = 1;
const byte COLS = 4;
// Map lógico: si tu keypad físico es distinto, ajusta el makeKeymap correspondientemente
char teclas[ROWS][COLS] = {{'1','2','3','4'}};
byte rowPins[ROWS] = {47};
byte colPins[COLS] = {15, 16, 17, 18};
Keypad customKeypad = Keypad(makeKeymap(teclas), rowPins, colPins, ROWS, COLS);

// ============ Parámetros de modos (valores por defecto) ============
int tri_amp = 180;                 // amplitud triángulo (0..255)
unsigned long tri_ramp_ms = 1000;  // tiempo subida 0->A en ms (sierra: sube y cae instant.)

int sq_amp = 180;                 // amplitud cuadrada
unsigned long sq_high_ms = 500;   // tiempo en alto
unsigned long sq_low_ms  = 500;   // tiempo en bajo (si no se envía, igual a high)

int sin_amp = 180;                // amplitud seno
float sin_freq_hz = 0.25f;        // frecuencia seno (Hz)

int const_duty = 0;               // valor para CONST (0..255)

// Tiempo para notificación periódica
unsigned long lastNotifyMs = 0;
const unsigned long notifyIntervalMs = 50; //ms

// Serial buffer (no bloqueante)
#define SERIAL_BUF_LEN 128
char serialBuf[SERIAL_BUF_LEN];
size_t serialBufPos = 0;

// BLE read/write helper forward
void processCommandLine(const char *line);

// ============ Utils PWM / motor (un solo sentido) ============

// Convendión: dejamos CH_DIR en 0 y CH_PWM en pwm. Si quieres invertir hardware, cambia aquí.
void set_motor_pwm_unidirectional(int pwm) {
  // asegurar rango
  int p = constrain(pwm, 0, (1 << PWM_RESOLUTION) - 1);
  ledcWrite(CH_DIR, 0);
  ledcWrite(CH_PWM, p);
  current_pwm_value = p;
}

// compute saw (sierra): sube lineal 0->amp en ramp_ms, luego cae a 0 y repite
int compute_saw_pwm(int amplitude, unsigned long ramp_ms, unsigned long t_ms) {
  if (ramp_ms == 0) return amplitude;
  unsigned long pos = t_ms % ramp_ms;      // 0 .. ramp_ms-1
  float frac = (float)pos / (float)ramp_ms; // 0..<1
  int pwm = (int)roundf(amplitude * frac);
  return constrain(pwm, 0, 255);
}

// compute square: 0 or amplitude
int compute_square_pwm(int amplitude, unsigned long high_ms, unsigned long low_ms, unsigned long t_ms) {
  unsigned long period = high_ms + low_ms;
  if (period == 0) return 0;
  unsigned long pos = t_ms % period;
  return (pos < high_ms) ? amplitude : 0;
}

// compute sine unidirectional 0..A
int compute_sine_pwm(int amplitude, float freq_hz, unsigned long t_ms) {
  float t_s = t_ms / 1000.0f;
  float v = sinf(2.0f * PI * freq_hz * t_s); // -1..1
  float scaled = 0.5f * (1.0f + v);          // 0..1
  int pwm = (int)roundf(amplitude * scaled);
  return constrain(pwm, 0, 255);
}

// ============ BLE callbacks (característica de control) ============

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pLedCharacteristic) {
    std::string val = pLedCharacteristic->getValue();
    if (val.length() == 0) return;
    // Interpretemos la carga completa como texto (comando)
    // Puede venir ASCII: e.g. "TRI 200 1000"
    String s = String(val.c_str());
    s.trim();
    if (s.length() == 0) return;
    processCommandLine(s.c_str());
  }
};

// ============ Serial input (no bloqueante) ============
void serialPoll() {
  // leer todos los bytes disponibles y acumular hasta '\n'
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue; // ignorar CR
    if (c == '\n') {
      // fin de línea: procesar
      serialBuf[serialBufPos] = '\0';
      processCommandLine(serialBuf);
      serialBufPos = 0;
      serialBuf[0] = '\0';
    } else {
      if (serialBufPos < SERIAL_BUF_LEN - 1) {
        serialBuf[serialBufPos++] = c;
      } else {
        // overflow: reset
        serialBufPos = 0;
        serialBuf[0] = '\0';
      }
    }
  }
}

// ============ Procesamiento de línea de comando (Serial o BLE) ============
// Comandos válidos (case-insensitive):
//   TRI <amp> <ramp_ms>
//   SQR <amp> <high_ms> [low_ms]
//   SIN <amp> <freq_hz>
//   CONST <duty>
//   STOP
//   STATUS
void processCommandLine(const char *line) {
  String s = String(line);
  s.trim();
  if (s.length() == 0) return;

  // convert to uppercase command token and args
  int sp = s.indexOf(' ');
  String cmd = (sp == -1) ? s : s.substring(0, sp);
  cmd.toUpperCase();
  String args = (sp == -1) ? "" : s.substring(sp + 1);
  args.trim();

  if (cmd == "TRI") {
    int sp2 = args.indexOf(' ');
    if (sp2 == -1) {
      Serial.println("TRI <amp> <ramp_ms>");
      return;
    }
    int a = args.substring(0, sp2).toInt();
    unsigned long r = (unsigned long)args.substring(sp2 + 1).toInt();
    tri_amp = constrain(a, 0, 255);
    tri_ramp_ms = (r == 0) ? 1 : r;
    modo = 1;
    t0 = millis();
    Serial.printf("Modo TRI set: amp=%d ramp_ms=%lu\n", tri_amp, tri_ramp_ms);
  }
  else if (cmd == "SQR") {
    int sp1 = args.indexOf(' ');
    if (sp1 == -1) {
      Serial.println("SQR <amp> <high_ms> [low_ms]");
      return;
    }
    int a = args.substring(0, sp1).toInt();
    String rest = args.substring(sp1 + 1);
    rest.trim();
    int sp2 = rest.indexOf(' ');
    unsigned long h, l;
    if (sp2 == -1) {
      h = (unsigned long)rest.toInt();
      l = h;
    } else {
      h = (unsigned long)rest.substring(0, sp2).toInt();
      l = (unsigned long)rest.substring(sp2 + 1).toInt();
    }
    sq_amp = constrain(a, 0, 255);
    sq_high_ms = (h == 0) ? 1 : h;
    sq_low_ms  = (l == 0) ? 1 : l;
    modo = 2;
    t0 = millis();
    Serial.printf("Modo SQR set: amp=%d high=%lums low=%lums\n", sq_amp, sq_high_ms, sq_low_ms);
  }
  else if (cmd == "SIN") {
    int sp1 = args.indexOf(' ');
    if (sp1 == -1) {
      Serial.println("SIN <amp> <freq_hz>");
      return;
    }
    int a = args.substring(0, sp1).toInt();
    float f = args.substring(sp1 + 1).toFloat();
    sin_amp = constrain(a, 0, 255);
    sin_freq_hz = (f < 0.0f) ? 0.0f : f;
    modo = 3;
    t0 = millis();
    Serial.printf("Modo SIN set: amp=%d freq=%.3fHz\n", sin_amp, sin_freq_hz);
  }
  else if (cmd == "CONST") {
    // CONST <duty>
    if (args.length() == 0) {
      Serial.println("CONST <duty>");
      return;
    }
    int d = args.toInt();
    const_duty = constrain(d, 0, 255);
    modo = 4; // modo constante
    set_motor_pwm_unidirectional(const_duty);
    Serial.printf("Modo CONST set: duty=%d\n", const_duty);
  }
  else if (cmd == "STOP") {
    modo = 0;
    set_motor_pwm_unidirectional(0);
    Serial.println("STOP: motor apagado");
  }
  else if (cmd == "STATUS") {
    // enviar por Serial y por BLE (notify)
    Serial.println("--- STATUS ---");
    Serial.printf("modo=%d (0=STOP,1=TRI,2=SQR,3=SIN,4=CONST)\n", modo);
    Serial.printf("TRI: amp=%d ramp_ms=%lu\n", tri_amp, tri_ramp_ms);
    Serial.printf("SQR: amp=%d high=%lums low=%lums\n", sq_amp, sq_high_ms, sq_low_ms);
    Serial.printf("SIN: amp=%d freq=%.3fHz\n", sin_amp, sin_freq_hz);
    Serial.printf("CONST: duty=%d\n", const_duty);
    Serial.println("--------------");

    if (pSensorCharacteristic && deviceConnected) {
      String payload = String("STATUS MODE=") + String(modo) +
                       String(" PWM=") + String(current_pwm_value);
      pSensorCharacteristic->setValue(payload.c_str());
      pSensorCharacteristic->notify();
    }
  }
  else {
    Serial.printf("Comando desconocido: %s\n", cmd.c_str());
    Serial.println("Comandos válidos: TRI, SQR, SIN, CONST, STOP, STATUS");
  }
}

// ============ LED (NeoPixel) control helper ============
// pequeño helper para indicar modo por LED
void indicateModeLED(int modoLocal) {
  FastLED.setBrightness(5); // para controlar la intensidad del led en cada modo
  switch (modoLocal) {
    case 0: leds[0] = CRGB::Black; break;
    case 1: leds[0] = CRGB::Red;   break; // TRI
    case 2: leds[0] = CRGB::Green; break; // SQR
    case 3: leds[0] = CRGB::Blue;  break; // SIN
    case 4: leds[0] = CRGB::White; break; // CONST
    default: leds[0] = CRGB::Black; break;
  }
  FastLED.show();
}

// ============ BLE & setup ============

class MyCallbacksBLE : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void setup() {
  Serial.begin(115200);
  delay(10);

  // NeoPixel (FastLED) init
  FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);
  leds[0] = CRGB::Black;
  FastLED.show();

  // PWM channels
  ledcSetup(CH_DIR, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_PWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOT_A1_PIN, CH_DIR);
  ledcAttachPin(MOT_A2_PIN, CH_PWM);
  set_motor_pwm_unidirectional(0);

  // Keypad no necesita init extra (ya creado)
  // Inicializar BLE (reutiliza tus UUIDs/nombre)
  BLEDevice::init("ESP32S3MOTVIB"); //no usar minúsculas
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyCallbacksBLE());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Characteristic para notificar estado
  pSensorCharacteristic = pService->createCharacteristic(
    SENSOR_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_INDICATE
  );
  pSensorCharacteristic->setValue("0");

  // Characteristic para recibir comandos (write)
  pLedCharacteristic = pService->createCharacteristic(
    LED_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();

  Serial.println("BLE Motor controller started. Waiting client...");
  Serial.println("Commands: TRI <amp> <ramp_ms> | SQR <amp> <high_ms> [low_ms] | SIN <amp> <freq_hz> | CONST <duty> | STOP | STATUS");

  // inicial nota
  lastNotifyMs = millis();
}

// ============ main loop (no bloqueante) ============
void loop() {
  // 1) procesar Serial sin bloquear
  serialPoll();

  // 2) procesar keypad (si quieres añadir comportamiento, aquí)
  char tecla = customKeypad.getKey();
  if (tecla) {
    Serial.print("Keypad: ");
    Serial.println(tecla);
    // Mantener keypad funcional: en este diseño no cambia modos, solo da info.
    // Si quieres que las teclas activen modos físicos, añadelas aquí.
    // Ejemplo rápido: tecla '4' apaga
    if (tecla == '4') {
      modo = 0;
      set_motor_pwm_unidirectional(0);
      indicateModeLED(modo);
    }
  }

  // 3) actualizar estado BLE connection -> re-advertise si necesario
  if (!deviceConnected && oldDeviceConnected) {
    // desconectado, retomamos advertising
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // 4) calcular tiempo relativo desde inicio del modo
  unsigned long t_ms = millis() - t0;

  // 5) ejecutar modo activo (no bloqueante)
  int pwm_out = 0;
  switch (modo) {
    case 0: // STOP
      pwm_out = 0;
      break;
    case 1: // TRI (sierra)
      pwm_out = compute_saw_pwm(tri_amp, tri_ramp_ms, t_ms);
      break;
    case 2: // SQR
      pwm_out = compute_square_pwm(sq_amp, sq_high_ms, sq_low_ms, t_ms);
      break;
    case 3: // SIN
      pwm_out = compute_sine_pwm(sin_amp, sin_freq_hz, t_ms);
      break;
    case 4: // CONST (manual):
      pwm_out = const_duty;
      break;
    default:
      pwm_out = 0;
      break;
  }

  // Aplicar PWM calculado
  set_motor_pwm_unidirectional(pwm_out);
  // indicar el modo por NeoPixel
  indicateModeLED(modo);

  // 6) notificar por BLE periódicamente (no más frecuente que notifyIntervalMs)
  if (deviceConnected && (millis() - lastNotifyMs > notifyIntervalMs)) {
    // enviar un payload de estado simple
    String payload = String("MODE=") + String(modo) + String(" PWM=") + String(current_pwm_value);
    pSensorCharacteristic->setValue(payload.c_str());
    pSensorCharacteristic->notify();
    lastNotifyMs = millis();
  }

  // cede tiempo, loop muy rápido (sin delays largos)
  yield();
}
