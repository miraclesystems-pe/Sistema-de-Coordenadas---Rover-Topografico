// ===== ROVER: avance continuo + giro libre (sin sensores, el rumbo lo envías tú) + pitch con filtro complementario =====
#define USAR_ENCODERS 1
#define USAR_MPU      1
#define USAR_VL53     1
#define USAR_WIFI     1

#include <Wire.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Adafruit_VL53L0X.h>
#include <LittleFS.h>

const char* ssid = "ROVER_WIFI";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ======================================================
//  AJUSTES: aquí se afina el comportamiento del carrito
// ======================================================
// --- Avance/retroceso continuo (mientras se mantiene presionado) ---
const unsigned long LATIDO_MS     = 400;    // seguridad: si no llega señal del botón en este tiempo, se detiene solo
                                            // (la página envía el comando cada 150 ms mientras mantienes presionado)

// --- Giro libre: gira mientras mantienes presionado I o D (a la izquierda / a la derecha) ---
// El giro NO usa sensores: solo mueve los motores. El rumbo (en grados) lo escribes tú en la página
// y lo envías con "ENVIAR POSICIÓN"; desde ese momento el mapa dibuja en ese ángulo.
const int   PWM_ARRANQUE_GIRO     = 200;     // PWM con el que empieza el giro (sube gradualmente hasta VEL_GIRO)
const unsigned long RAMPA_GIRO_MS = 120;    // tiempo de la subida gradual; sube si aún se reinicia, baja si arranca flojo

const int VEL_AVANCE = 120;                 // PWM 0-255 al avanzar/retroceder
const int VEL_GIRO   = 170;                 // PWM 0-255 al girar
const unsigned long FRENO_MS = 40;          // freno activo al terminar

const int SIGNO_AVANCE = 1;                 // pon -1 si X disminuye cuando avanzas

// ---- Pines ----
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7
#define ENCODER_DERECHO 42
#define ENCODER_IZQUIERDO 41
#define I2C_SDA 9
#define I2C_SCL 10

const float DIAMETRO_RUEDA = 5.0;
const int   RANURAS_ENCODER = 20;
const float PI_VALOR = 3.14159265;

// ---- Calibración de distancia ----
// FACTOR_CALIB = distancia real / distancia que marcaba el carro.
// Para reajustar: FACTOR_CALIB_nuevo = FACTOR_CALIB_actual * (real / medido)
const float FACTOR_CALIB = 1.325;
const float CM_POR_PULSO = (PI_VALOR * DIAMETRO_RUEDA) / RANURAS_ENCODER * FACTOR_CALIB;

// ---- Pitch (inclinación) ----
const float Z0_CM          = 5.0;
const float PITCH_SIGN     = 1.0;           // signo del ACELERÓMETRO (interno del filtro). No lo cambies solo: debe ser coherente con GYRO_SIGN
const float GYRO_SIGN      = -1.0;          // signo del GIROSCOPIO (interno del filtro). PITCH_SIGN y GYRO_SIGN se cambian JUNTOS o ninguno

// *** SIGNO DE SALIDA ***
// Invierte SOLO el pitch final que se usa para Z, pendiente, tendencia y pantalla (no toca el filtro).
//  - Si al SUBIR el carrito la pendiente sale NEGATIVA y dibuja bajada  -> déjalo en -1.0
//  - Si ya sale correcto (subir = positivo)                              -> ponlo en 1.0
const float SALIDA_SIGN    = -1.0;

const float PITCH_MUERTO   = 1.5;           // grados que se ignoran (cabeceo del chasis / ruido). Sube a 2.0-2.5 si aún se acumula
const float K_ACEL_MOV     = 0.01;          // corrección del acelerómetro en movimiento (lenta = confía en el giroscopio)
const float K_ACEL_QUIETO  = 0.10;          // corrección del acelerómetro con el carrito detenido (rápida)
const float TOL_1G         = 0.05;          // solo usa el acelerómetro si |a| está dentro de 1 g ± este valor
const int   MUESTRAS_NIVEL = 100;           // lecturas para promediar el nivel de referencia

// ---- Anti-"bolea": suavizado del pitch que se usa para calcular Z ----
// Al arrancar y al frenar el chasis cabecea y el pitch da un pico falso que dibujaba una bolea en el perfil.
const unsigned long PITCH_TRANSITORIO_MS = 300;  // tras arrancar/frenar/girar se congela el pitch de pendiente (sube a 500 si aún hay bolea)
const float PITCH_FILTRO    = 0.30;         // suavizado por cada 50 ms (menor = más suave pero más lento; 0.05-0.3)
const float PITCH_MAX_DEG_S = 45.0;         // la pendiente no puede cambiar más rápido que esto (°/s); baja a 15 si aún salta

// ---- Re-sincronización del pitch tras un giro (arregla el giro en U en una pendiente) ----
// Durante el giro no se lee el MPU, así que el pitch queda "viejo". Al terminar el giro se vuelve a tomar
// el pitch directo del acelerómetro y se salta pitchPend a ese valor.
const unsigned long REINICIO_PITCH_MS = 150;     // espera tras el giro antes de re-sincronizar (el chasis ya debe estar quieto)

Adafruit_VL53L0X vl53 = Adafruit_VL53L0X();
bool mpuDisponible = false, vl53Disponible = false;

float pitchFilt = 0, pitchOffset = 0, pitchUtil = 0;
float pitchPend = 0;                         // pitch suavizado y sin transitorios: es el que se usa para Z, pendiente y pantalla
unsigned long tTransitorio = 0;              // último arranque/frenada/giro
bool  pitchInicializado = false;
bool  reinicioPitch = false;                 // pendiente de re-sincronizar el pitch tras un giro
unsigned long ultimoPitchUs = 0;
float distanciaSuperficie = -1.0;
float gyroBiasY = 0.0;                       // °/s en reposo (eje Y, para el pitch)

volatile long pulsosDerecha = 0, pulsosIzquierda = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

float x = 0.0, y = 0.0;
float z = Z0_CM, zMax = Z0_CM, zMin = Z0_CM;
float subidaTotal = 0.0, distanciaTotal = 0.0;
float heading = 0.0;                         // rumbo en grados (0-360), como brújula: derecha = 90, 180, 270 · izquierda = 270, 180, 90

long pulsosAnterioresDerecha = 0, pulsosAnterioresIzquierda = 0;
int dirDerecha = 1, dirIzquierda = 1;

// ---- Estado del movimiento ----
bool moviendo = false;
bool girando  = false;
bool continuo = false;                       // avance/retroceso mientras se mantiene presionado
volatile bool frenarPedido = false;          // la página soltó el botón
int  sentidoContinuo = 0;                    // +1 adelante, -1 atrás
int  sentidoMov = 1;                         // último sentido de marcha: +1 adelante, -1 atrás
unsigned long ultimoLatido = 0;              // última señal recibida del botón
int  giroDireccion = 0;                      // +1 izquierda, -1 derecha (giro en curso)
unsigned long tiempoInicioMov = 0;
int   pwmGiroActual = 0;

unsigned long tiempoCalculo = 0, tiempoTelemetria = 0, tiempoSensores = 0, tiempoDebug = 0;

#define PWM_FREC 1000
#define PWM_RES  8

// ---- Utilidades ----
void configurarSentidos(int d, int i) { portENTER_CRITICAL(&mux); dirDerecha = d; dirIzquierda = i; portEXIT_CRITICAL(&mux); }

void leerPulsos(long &d, long &i) {
  portENTER_CRITICAL(&mux); d = pulsosDerecha; i = pulsosIzquierda; portEXIT_CRITICAL(&mux);
}

void sincronizarBase() {                     // olvida los pulsos acumulados (no cuentan como avance)
  long d, i; leerPulsos(d, i);
  pulsosAnterioresDerecha = d; pulsosAnterioresIzquierda = i;
}

// ---- Motores (PWM) ----
void pwm(int a, int b, int c, int d) {
  ledcWrite(IN1, a); ledcWrite(IN2, b); ledcWrite(IN3, c); ledcWrite(IN4, d);
}
void detener() { pwm(0, 0, 0, 0); moviendo = false; girando = false; continuo = false; frenarPedido = false; }

// ---- Giroscopio Y (solo para el pitch / inclinación) ----
void calibrarGyroY() {                       // bias del giroscopio Y (usado para el pitch)
  float suma = 0; int n = 0;
  for (int k = 0; k < 200; k++) {
    Wire.beginTransmission(0x68); Wire.write(0x45);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)0x68, (uint8_t)2) == 2) {
      uint8_t hi = Wire.read();
      uint8_t lo = Wire.read();
      int16_t raw = (int16_t)((hi << 8) | lo);
      suma += raw / 131.0; n++;
    }
    delay(5);
  }
  if (n > 20) gyroBiasY = suma / n;
}

// ---- Rumbo enviado desde la página ----
// El rumbo se guarda tal cual lo envías (sin conversiones): 90 = derecha, 270 = izquierda.
void fijarRumbo(float rumbo) {
  heading = fmodf(rumbo, 360.0f);
  if (heading < 0) heading += 360.0f;
  Serial.printf("Posicion enviada: %.1f grados | rumbo = %.1f\n", rumbo, heading);
}

// ---- Avance / retroceso continuo ----
void iniciarContinuo(int sentido) {          // +1 adelante, -1 atrás
  tTransitorio = millis();                   // el arranque cabecea: congela el pitch un momento
  moviendo = true; girando = false; continuo = true; frenarPedido = false;
  sentidoContinuo = sentido;
  sentidoMov = sentido;                      // recuerda el sentido de marcha para la pendiente
  ultimoLatido = millis();
  configurarSentidos(sentido * SIGNO_AVANCE, sentido * SIGNO_AVANCE);
  if (sentido > 0) pwm(VEL_AVANCE, 0, VEL_AVANCE, 0);
  else             pwm(0, VEL_AVANCE, 0, VEL_AVANCE);
}

// ---- Giro libre (solo motores) ----
void aplicarGiro(int dir, int p) {           // dir: +1 izquierda, -1 derecha; p: PWM
  p = constrain(p, 0, 255);
  if (dir > 0) { configurarSentidos(1, -1); pwm(p, 0, 0, p); }
  else         { configurarSentidos(-1, 1); pwm(0, p, p, 0); }
  pwmGiroActual = p;
}

void iniciarGiro(int dir) {                  // +1 izquierda, -1 derecha (sigue hasta soltar el botón)
  moviendo = true; girando = true; continuo = false; frenarPedido = false; giroDireccion = dir;
  ultimoLatido = millis(); tiempoInicioMov = ultimoLatido;
  aplicarGiro(dir, PWM_ARRANQUE_GIRO);       // arranca suave; el loop sube la potencia gradualmente
}

void finalizarGiro() {                       // se soltó el botón: frena y para
  pwm(255, 255, 255, 255); delay(FRENO_MS);  // freno activo
  pwm(0, 0, 0, 0);
  delay(60);
  girando = false; moviendo = false; frenarPedido = false;
  tTransitorio = millis();
  pitchInicializado = false;                 // la próxima lectura del MPU toma el pitch del acelerómetro (el del giro quedó viejo)
  reinicioPitch = true;                      // y pitchPend salta a ese valor cuando el chasis se asiente
  sincronizarBase();                         // los pulsos del giro no cuentan como avance
}

// ---- Encoders y sensores ----
void IRAM_ATTR contarDerecha()   { portENTER_CRITICAL_ISR(&mux); pulsosDerecha   += dirDerecha;   portEXIT_CRITICAL_ISR(&mux); }
void IRAM_ATTR contarIzquierda() { portENTER_CRITICAL_ISR(&mux); pulsosIzquierda += dirIzquierda; portEXIT_CRITICAL_ISR(&mux); }

// Pitch con filtro complementario:
//  - El giroscopio (eje Y) da el cambio rápido de inclinación y no se confunde con aceleraciones.
//  - El acelerómetro corrige la deriva lenta, pero SOLO cuando mide ~1 g (sin aceleración extra del carrito).
void leerMPU6050() {
  if (!mpuDisponible) return;
  Wire.beginTransmission(0x68); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom((uint8_t)0x68, (uint8_t)14) != 14) return;
  uint8_t b[14];
  for (int k = 0; k < 14; k++) b[k] = Wire.read();

  // b[0..5] acelerómetro, b[6..7] temperatura, b[8..13] giroscopio X, Y, Z
  float ax_g = (int16_t)((b[0] << 8) | b[1]) / 8192.0;     // rango ±4 g
  float ay_g = (int16_t)((b[2] << 8) | b[3]) / 8192.0;
  float az_g = (int16_t)((b[4] << 8) | b[5]) / 8192.0;
  float gy   = ((int16_t)((b[10] << 8) | b[11]) / 131.0 - gyroBiasY) * GYRO_SIGN;    // °/s

  unsigned long us = micros();
  float dt = (us - ultimoPitchUs) / 1e6;
  ultimoPitchUs = us;
  if (dt <= 0 || dt > 0.2) dt = 0.02;        // tras un giro o una pausa larga

  float pitchAcel = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI_VALOR * PITCH_SIGN;
  float mag = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

  if (!pitchInicializado) { pitchFilt = pitchAcel; pitchInicializado = true; }

  pitchFilt += gy * dt;                                    // 1) predicción con el giroscopio
  if (fabs(mag - 1.0) < TOL_1G) {                          // 2) corrección solo si no hay aceleración extra
    float k = moviendo ? K_ACEL_MOV : K_ACEL_QUIETO;
    pitchFilt += k * (pitchAcel - pitchFilt);
  }

  // Zona muerta "suave": no hay salto al cruzar el umbral
  float p = pitchFilt - pitchOffset;
  if (fabs(p) < PITCH_MUERTO) p = 0;
  else p -= (p > 0 ? 1 : -1) * PITCH_MUERTO;
  pitchUtil = p * SALIDA_SIGN;               // signo de salida: aquí se corrige "subida dibujada como bajada"
}

void leerVL53L0X() {
  if (!vl53Disponible) return;
  VL53L0X_RangingMeasurementData_t m;
  vl53.rangingTest(&m, false);
  distanciaSuperficie = (m.RangeStatus != 4 && m.RangeMilliMeter < 8190) ? m.RangeMilliMeter / 10.0 : -1.0;
}

void actualizarPitchPend() {
  static unsigned long ultimo = 0;
  unsigned long ahora = millis();
  float dt = (ahora - ultimo) / 1000.0;
  ultimo = ahora;
  if (dt <= 0 || dt > 0.5) dt = 0.05;

  // Tras un giro: salta al pitch real (sin límite de rapidez) en cuanto el chasis se asienta
  if (reinicioPitch && !moviendo && ahora - tTransitorio >= REINICIO_PITCH_MS) {
    pitchPend = pitchUtil;
    reinicioPitch = false;
    return;
  }

  if (ahora - tTransitorio < PITCH_TRANSITORIO_MS) return;   // en arranque/frenada se mantiene el último valor bueno
  if (!moviendo) { pitchPend = pitchUtil; return; }          // quieto: sigue la inclinación al instante (Z no cambia, no hace falta suavizar)
  float paso = PITCH_FILTRO * (pitchUtil - pitchPend);       // suavizado exponencial
  float lim = PITCH_MAX_DEG_S * dt;                          // y límite de rapidez de cambio
  pitchPend += constrain(paso, -lim, lim);
}

void calcularCoordenadas() {
  actualizarPitchPend();
  long actualD, actualI; leerPulsos(actualD, actualI);
  long nuevosD = actualD - pulsosAnterioresDerecha;
  long nuevosI = actualI - pulsosAnterioresIzquierda;
  pulsosAnterioresDerecha = actualD; pulsosAnterioresIzquierda = actualI;

  if (girando) return;                       // al girar no se mueve el mapa

  float dMedia = ((nuevosD + nuevosI) / 2.0) * CM_POR_PULSO;
  float pRad = pitchPend * PI_VALOR / 180.0;
  float dHoriz = dMedia * cos(pRad);
  float dVert  = dMedia * sin(pRad);

  distanciaTotal += fabs(dMedia);
  float hRad = heading * PI_VALOR / 180.0;   // 0° = eje +X; 90° = hacia Y negativo (derecha); 270° = hacia Y positivo (izquierda)
  x += dHoriz * cos(hRad);
  y -= dHoriz * sin(hRad);

  z += dVert;
  if (dVert > 0) subidaTotal += dVert;
  if (z > zMax) zMax = z;
  if (z < zMin) zMin = z;
}

void enviarTelemetria() {
  if (ws.count() == 0) return;
  long pD, pI; leerPulsos(pD, pI);

  StaticJsonDocument<512> doc;
  doc["x"] = x; doc["y"] = y; doc["z"] = z;
  doc["heading"] = heading;

  doc["pitch"] = pitchPend;                  // inclinación suavizada (la misma que se usa para Z)
  // Pendiente en el sentido de marcha: + sube, - baja
  doc["pendientePct"] = tan(pitchPend * PI_VALOR / 180.0) * 100.0 * sentidoMov;
  doc["tendencia"] = (fabs(pitchPend) < 0.1) ? 0 : ((pitchPend * sentidoMov > 0) ? 1 : -1);  // 1 subiendo, -1 bajando, 0 plano
  doc["zMax"] = zMax; doc["zMin"] = zMin;
  doc["subidaTotal"] = subidaTotal;
  doc["pulsosDerecha"] = pD; doc["pulsosIzquierda"] = pI;
  doc["distanciaTotal"] = distanciaTotal;
  doc["distanciaSuperficie"] = distanciaSuperficie;
  doc["modoGiro"] = moviendo;
  doc["moviendo"] = moviendo;
  doc["girando"] = girando;

  String json;
  serializeJson(doc, json);
  ws.textAll(json);
}

void reiniciarMapa() {
  portENTER_CRITICAL(&mux);
  pulsosDerecha = 0; pulsosIzquierda = 0;
  pulsosAnterioresDerecha = 0; pulsosAnterioresIzquierda = 0;
  portEXIT_CRITICAL(&mux);
  x = 0; y = 0; heading = 0; distanciaTotal = 0;
  z = Z0_CM; zMax = Z0_CM; zMin = Z0_CM; subidaTotal = 0;
  pitchOffset = pitchFilt;                   // el nivel actual pasa a ser el "plano" de referencia
  pitchPend = 0;
  reinicioPitch = false;
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
               void *arg, uint8_t *data, size_t len) {
  if (type != WS_EVT_DATA) return;
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)) return;
  if (len == 0) return;
  char c = (char)data[0];

  // 'S' = soltaste el botón (o parada de emergencia): siempre se atiende
  if (c == 'S') {
    if (continuo || girando) frenarPedido = true;   // el loop frena con suavidad
    else                     detener();
    return;
  }

  // 'A' / 'R' (avanzar/retroceder) e 'I' / 'D' (girar izq/der) llegan una vez al presionar
  // y se repiten cada ~150 ms mientras se mantiene presionado
  if (c == 'A' || c == 'R' || c == 'I' || c == 'D') {
    bool esGiro = (c == 'I' || c == 'D');
    int sentido = (c == 'A' || c == 'I') ? 1 : -1;
    if (!esGiro && continuo && sentidoContinuo == sentido) { ultimoLatido = millis(); return; }
    if (esGiro && girando && giroDireccion == sentido && !frenarPedido) { ultimoLatido = millis(); return; }
    if (moviendo) return;                    // si está en otra maniobra, ignora
    if (esGiro) iniciarGiro(sentido);
    else        iniciarContinuo(sentido);
    return;
  }

  if (moviendo) return;                      // mientras se mueve, ignora lo demás

  if (c == 'C') reiniciarMapa();
  else if (c == 'G' && len > 1) {            // ENVIAR POSICIÓN: "G45.5" = el rover está a 45.5° (rumbo de pantalla)
    char buf[24]; size_t n = min(len - 1, sizeof(buf) - 1);
    memcpy(buf, data + 1, n); buf[n] = 0;
    fijarRumbo(atof(buf));
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n[1] inicio");

  ledcAttach(IN1, PWM_FREC, PWM_RES);
  ledcAttach(IN2, PWM_FREC, PWM_RES);
  ledcAttach(IN3, PWM_FREC, PWM_RES);
  ledcAttach(IN4, PWM_FREC, PWM_RES);
  detener();
  Serial.println("[2] motores OK (PWM)");

#if USAR_ENCODERS
  pinMode(ENCODER_DERECHO, INPUT_PULLUP); pinMode(ENCODER_IZQUIERDO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DERECHO), contarDerecha, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_IZQUIERDO), contarIzquierda, RISING);
  Serial.println("[3] encoders OK");
  Serial.printf("    CM_POR_PULSO = %.4f (FACTOR_CALIB = %.4f)\n", CM_POR_PULSO, FACTOR_CALIB);
#endif

  Wire.begin(I2C_SDA, I2C_SCL, 400000);      // 400 kHz: la lectura de 14 bytes es más rápida
  delay(100);

#if USAR_MPU
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();   // despertar
  delay(50);
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0) {
    mpuDisponible = true;
    Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x08); Wire.endTransmission(); // acel ±4 g
    Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(); // gyro ±250 °/s
    Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(); // filtro pasa-bajos ~44 Hz (quita vibración de motores)
    delay(50);

    Serial.println("[4] MPU6050 OK - calibrando giroscopio (no muevas el carrito)...");
    calibrarGyroY();                         // eje Y (pitch)
    Serial.printf("    bias gyro Y = %.2f grados/s\n", gyroBiasY);

    // Nivel de referencia = promedio de varias lecturas (carrito quieto sobre superficie plana)
    ultimoPitchUs = micros();
    float s = 0;
    for (int k = 0; k < MUESTRAS_NIVEL; k++) {
      leerMPU6050(); s += pitchFilt; delay(10);
    }
    pitchOffset = s / MUESTRAS_NIVEL;
    Serial.printf("    nivel de referencia (pitch) = %.2f grados\n", pitchOffset);
    Serial.printf("    signos: PITCH_SIGN=%.0f GYRO_SIGN=%.0f SALIDA_SIGN=%.0f\n", PITCH_SIGN, GYRO_SIGN, SALIDA_SIGN);
  } else {
    Serial.println("[4] MPU6050 NO responde (sin pitch)");
  }
#endif

#if USAR_VL53
  if (vl53.begin(0x29, false, &Wire)) { vl53Disponible = true; Serial.println("[5] VL53L0X OK"); }
  else Serial.println("[5] VL53L0X NO responde");
#endif

#if USAR_WIFI
  WiFi.softAP(ssid, password);
  Serial.print("[6] WiFi AP listo, IP: "); Serial.println(WiFi.softAPIP());
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  if (!LittleFS.begin(true)) Serial.println("[7] ERROR LittleFS");
  else Serial.println("[7] LittleFS OK");

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.begin();
  Serial.println("[8] servidor web iniciado");
#endif

  Serial.println("[9] setup terminado");
}

void loop() {
#if USAR_WIFI
  ws.cleanupClients();
#endif
  unsigned long ahora = millis();

  // --- Avance continuo: sigue hasta que se suelte el botón (o se pierda la señal) ---
  if (continuo) {
    if (frenarPedido || ahora - ultimoLatido > LATIDO_MS) {
      pwm(255, 255, 255, 255); delay(FRENO_MS); pwm(0, 0, 0, 0);   // freno activo y paro
      delay(60);                                                    // deja que termine de rodar
      continuo = false; moviendo = false; frenarPedido = false;
      tTransitorio = millis();                                      // la frenada cabecea: no uses ese pitch
      calcularCoordenadas();                                        // suma lo recorrido hasta parar (antes se descartaba)
    }
  }

  // --- Giro libre: gira mientras se mantiene presionado (sin sensores) ---
  if (girando) {
    unsigned long t = ahora - tiempoInicioMov;
    int p = VEL_GIRO;                        // subida gradual: de PWM_ARRANQUE_GIRO a VEL_GIRO durante RAMPA_GIRO_MS
    if (t < RAMPA_GIRO_MS) p = PWM_ARRANQUE_GIRO + (int)((long)(VEL_GIRO - PWM_ARRANQUE_GIRO) * (long)t / (long)RAMPA_GIRO_MS);
    if (p != pwmGiroActual) aplicarGiro(giroDireccion, p);
    if (frenarPedido || ahora - ultimoLatido > LATIDO_MS) finalizarGiro();   // soltaste el botón o se perdió la señal
  }

  if (!girando && ahora - tiempoSensores >= 20) {
    tiempoSensores = ahora;
    leerMPU6050();
    if (!moviendo) leerVL53L0X();
  }
  if (ahora - tiempoCalculo >= 50) { tiempoCalculo = ahora; calcularCoordenadas(); }

#if USAR_WIFI
  if (ahora - tiempoTelemetria >= 100) { tiempoTelemetria = ahora; enviarTelemetria(); }
#endif

  if (ahora - tiempoDebug >= 1000) {
    tiempoDebug = ahora;
    // pitch = pitchUtil (con SALIDA_SIGN) | filt = interno del filtro | pend = el que se usa para Z y se manda a la página
    Serial.printf("%s | sent=%+d | pitch=%.2f (filt=%.2f) pend=%.2f | z=%.2f | x=%.1f y=%.1f | dist=%.1f | rumbo=%.1f\n",
                  moviendo ? (girando ? "Girando" : (continuo ? "Continuo" : "Mov")) : "Detenido",
                  sentidoMov, pitchUtil, pitchFilt - pitchOffset, pitchPend, z, x, y, distanciaTotal, heading);
  }
  delay(2);
}
