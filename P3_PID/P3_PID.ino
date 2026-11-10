#include <limits.h>
#include <numbers>

// NOTA: sustituye "apellido" por tu primer apellido en los nombres
// P3_PID_apellido() y P3_actua_apellido() (p. ej. P3_PID_garcia).

// ---------------- Encoder ----------------
// A y B son las señales del encoder; si A adelanta a B gira en sentido horario
const int channelPinA = 26;
const int channelPinB = 27;

volatile int ISRCounter = 0;      // posición en cuentas (x4)
volatile uint8_t encoderState = 0;

static const int8_t QUAD_TABLE[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

const uint PPR_encoder = 385;
const float conv = (2 * std::numbers::pi) / (4 * PPR_encoder);   // cuentas -> rad

// ---------------- PWM ----------------
const int pwm1 = 32;
const int pwm2 = 33;
const int freq = 1000;                        // periodo PWM = 1 ms
const uint8_t resolution = 10;
const int PWM_MAX = (1 << resolution) - 1;    // 1023


const uint32_t T_CONTROL_US = 10000;          // periodo de control en µs
const float Ts = T_CONTROL_US / 1000000.0f;   // periodo en segundos

hw_timer_t *timerControl = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool nuevoDato = false;              // la ISR avisa al loop para imprimir

// ---------------- PID ----------------
float Kp = 0.3f;
float Ki = 0.1f;    // Ki = Kp / Ti
float Kd = 0.0f;    // Kd = Kp * Td

volatile int referencia = 0;   // en cuentas
volatile int pos = 0;
volatile int error_act = 0;
volatile int m = 0;            // actuación
int pos_prev = 0;
volatile float integral = 0;
int cuenta_print = 0;

// ---------------- Motor ----------------
void giro_horario(int duty_cycle) {
  ledcWrite(pwm2, 0);
  ledcWrite(pwm1, duty_cycle);
}

void giro_antihorario(int duty_cycle) {
  ledcWrite(pwm1, 0);
  ledcWrite(pwm2, duty_cycle);
}

// ---------------- ISR encoder ----------------
void IRAM_ATTR handleEncoderISR() {
  uint8_t a = digitalRead(channelPinA);
  uint8_t b = digitalRead(channelPinB);

  uint8_t currentState = (a << 1) | b;
  uint8_t index = (encoderState << 2) | currentState;

  ISRCounter += QUAD_TABLE[index];
  encoderState = currentState;
}

// ---------------- Función 1: PID de posición ----------------
// Calcula la actuación m a partir de la referencia y la posición medida.
void P3_PID_apellido() {
  pos = ISRCounter;
  error_act = referencia - pos;

  // Integral con anti-windup (se limita a la saturación del PWM)
  integral += Ki * error_act * Ts;
  integral = constrain(integral, (float)-PWM_MAX, (float)PWM_MAX);

  // Derivada sobre la medida (evita el "golpe" al cambiar la referencia)
  float deriv = -(float)(pos - pos_prev) / Ts;
  pos_prev = pos;

  float u = Kp * error_act + integral + Kd * deriv;
  m = (int)constrain(u, (float)-PWM_MAX, (float)PWM_MAX);
}

// ---------------- Función 2: actuación sobre los PWM ----------------
// Traduce la actuación m (con signo) en los duty cycles de los dos PWM.
void P3_actua_apellido() {
  if (m >= 0) {
    giro_horario(m);
  } else {
    giro_antihorario(-m);                 // el duty siempre positivo
  }
}

// ---------------- ISR del Timer (periodo constante) ----------------
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  P3_PID_apellido();
  P3_actua_apellido();
  portEXIT_CRITICAL_ISR(&timerMux);

  nuevoDato = true;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  pinMode(channelPinA, INPUT_PULLUP);
  pinMode(channelPinB, INPUT_PULLUP);

  // PWM antes de empezar a controlar
  ledcAttach(pwm1, freq, resolution);
  ledcAttach(pwm2, freq, resolution);
  giro_horario(0);

  // Estado inicial AB antes de activar interrupciones
  encoderState = (digitalRead(channelPinA) << 1) | digitalRead(channelPinB);
  attachInterrupt(digitalPinToInterrupt(channelPinA), handleEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelPinB), handleEncoderISR, CHANGE);

  pos_prev = ISRCounter;

  // Timer a 1 MHz (1 tick = 1 µs), alarma periódica cada T_CONTROL_US
  timerControl = timerBegin(1000000);
  timerAttachInterrupt(timerControl, &onTimer);
  timerAlarm(timerControl, T_CONTROL_US, true, 0);

  Serial.println("Introduce la referencia en cuentas (ej: 3000)");
}

void loop() {
  // Nueva referencia por serie
  if (Serial.available() > 0) {
    String dato = Serial.readStringUntil('\n');
    int nuevaRef = dato.toInt();

    portENTER_CRITICAL(&timerMux);        // evita que la ISR vea valores a medias
    referencia = nuevaRef;
    integral = 0;                         // reinicia la integral al cambiar de referencia
    portEXIT_CRITICAL(&timerMux);
  }

  // Impresión fuera de la ISR (cada 2 periodos de control = 20 ms)
  if (nuevoDato) {
    nuevoDato = false;
    if (++cuenta_print >= 2) {
      cuenta_print = 0;

      int r, p, e, u;
      portENTER_CRITICAL(&timerMux);
      r = referencia; p = pos; e = error_act; u = m;
      portEXIT_CRITICAL(&timerMux);

      Serial.print("Ref: ");     Serial.print(r);
      Serial.print(" pos: ");    Serial.print(p);
      Serial.print(" error: ");  Serial.print(e);
      Serial.print(" m: ");      Serial.println(u);
    }
  }
}
