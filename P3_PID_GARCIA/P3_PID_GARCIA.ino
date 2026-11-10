#include <limits.h>
#include <numbers>


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
const int freq = 1000;
const uint8_t resolution = 10;
const int PWM_MAX = (1 << resolution) - 1;   // 1023

// ---------------- PID ----------------
const unsigned long t_window = 20;           // periodo de control en ms
const float Ts = t_window / 1000.0f;         // periodo en segundos

float Kp = 0.3f;
float Ki = 0.1f;    // Ki = Kp / Ti
float Kd = 0.1f;    // Kd = Kp * Td

int32_t referencia = 0;   // en cuentas
int32_t pos = 0;
int32_t pos_prev = 0;
int32_t error_act = 0;
float integral = 0;             
volatile bool nuevaActuacion = false;
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

//---------------- Timer ---------------- 
hw_timer_t *timer =NULL;


// ---------------- PID ----------------
void IRAM_ATTR timeInterrupt() {

  nuevaActuacion = true;
}

int32_t PID_Posicion()
{
  int32_t m;
  pos = ISRCounter;                       // lectura de 32 bits, atómica en ESP32
  error_act = referencia - pos;

  // Derivada sobre la medida (evita el "golpe" al cambiar la referencia)
  float deriv = -(float)(pos - pos_prev) / Ts;
  pos_prev = pos;
  // Integral con anti-windup (se limita a la saturación del PWM)
  float u = Kp * error_act + integral + Kd * deriv;

  bool saturado = (u > PWM_MAX && error_act > 0) || (u < -PWM_MAX && error_act < 0);
  if (!saturado) {
    integral += Ki * error_act * Ts;
    integral = constrain(integral, (float)-PWM_MAX, (float)PWM_MAX);
  }

  u = Kp * error_act + integral + Kd * deriv;
  m = (int)constrain(u, (float)-PWM_MAX, (float)PWM_MAX);

  return m;
}
void P3_actua_GARCIA( int32_t m ) {
  if (m >= 0) {
    giro_horario(m);
  } else {
    giro_antihorario(-m);                 // el duty siempre positivo
  }
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

  //Timer calculation
  timer = timerBegin(1000000);
  timerAttachInterrupt( timer, &timeInterrupt);
  timerAlarm( timer, 20000, true, 0);
  Serial.println("Introduce la referencia en cuentas (ej: 3000)");
}

void loop() {
  // Nueva referencia por serie
  if (Serial.available() > 0) {
    String dato = Serial.readStringUntil('\n');
    referencia = dato.toInt();
    integral = 0;                         // reinicia la integral al cambiar de referencia
  }

  // Lazo de control a periodo fijo (fuera de la ISR)
  if ( nuevaActuacion == true)
  {
    nuevaActuacion = false;
    int32_t m = PID_Posicion();
    P3_actua_GARCIA( m );
    Serial.print("Ref: ");     Serial.print(referencia);
    Serial.print(" pos: ");    Serial.print(pos);
    Serial.print(" error: ");  Serial.print(error_act);
    Serial.print(" m: ");      Serial.println(m);
  }
  
  
}