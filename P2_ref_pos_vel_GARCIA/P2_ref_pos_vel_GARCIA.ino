#include <limits.h>
#include <numbers>
//Variables para parte encoder
// Pines seguros para interrupciones 4, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33

//A y b son los pines de las señales del encoder, si A adelanta a B gira horario
const int channelPinA = 26;
const int channelPinB = 27;

//Volatile se necesita poner siempre que haya algo externo como una interrupción que cambie la variable
//en vez de la sección de código en la que aparece
volatile int ISRCounter = 0;
int counter = 0;
const int32_t maxSteps = INT_MAX;
const int32_t minSteps = INT_MIN;
bool IsCW = true;


//Variables sección pwm
const int pwm1 = 32;
const int pwm2 = 33;
const int freq = 1000;
const uint8_t resolution = 10;
const int resol_num = (1 << resolution) - 1;



//Variables para número de cuentas y el estado del encoder
const uint PPR_encoder = 385; //Pulses per round of the encoder
unsigned long t_actual = 0;
unsigned long t_prev = 0;
float vel = 0;
const int t_window = 20;

//Variables used in P3
float pos_rad = 0;
float vel_rads = 0;
const float conv = (2*std::numbers::pi)/(4*PPR_encoder);

volatile uint8_t encoderState = 0;  // last 2-bit AB state

static const int8_t QUAD_TABLE[16] = {
  0, -1, +1, 0,
  +1, 0, 0, -1,
  -1, 0, 0, +1,
  0, +1, -1, 0
};

void IRAM_ATTR handleEncoderISR(){
  uint8_t a = digitalRead(channelPinA);
  uint8_t b = digitalRead(channelPinB);

  
  uint8_t currentState = ( a << 1) | b; //In this line a is being displaced in order to compare the other b bit 
  //and get a full 4 state number, as the output of digitalRead can only be 0 or 1 the displacement is properly done

  //Here is comparing the currentState and the encoder state which represents the transistion index that must be used
  uint8_t index = (encoderState << 2) | currentState;

  if (ISRCounter + 1 < maxSteps && ISRCounter - 1 > minSteps  )
  {
    ISRCounter += QUAD_TABLE[index];
  }
  else
  {
    ISRCounter = 0;
  }
  
  encoderState = currentState;
}
//Giro de motor funciones
void giro_horario(int duty_cycle) {

  ledcWrite(pwm2, 0);
  ledcWrite(pwm1, duty_cycle);
}

void giro_antihorario(int duty_cycle) {

  ledcWrite(pwm1, 0);
  ledcWrite(pwm2, duty_cycle);
}
void setup() {
  // put your setup code here, to run once:
  pinMode(channelPinA, INPUT_PULLUP);
  pinMode(channelPinB, INPUT_PULLUP);
  delay(3000); // Wait for serial monitor to connect
  Serial.begin(115200);
  Serial.setTimeout(20);
  
  // Initialize starting AB state before enabling interrupts
  uint8_t a = digitalRead(channelPinA);
  uint8_t b = digitalRead(channelPinB);
  encoderState = (a << 1) | b;

  // Attach interrupt on both channels  
  attachInterrupt(digitalPinToInterrupt(channelPinA), handleEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelPinB), handleEncoderISR, CHANGE);

  // Timer interrupt for the actuator calculation
  ledcAttach(pwm1, freq, resolution);
  ledcAttach(pwm2, freq, resolution);

  Serial.println("Introduce un valor entre -1023 y 1023");
}

void loop() {
  
  /* --Control del motor
  if (Serial.available() > 0) {
    String dato = Serial.readStringUntil('\n');
    referencia = dato.toInt();


  }*/
  
  // ---- Lectura de posición, velocidad y envío cada 0.2 ms (sin delay) ----
  t_actual = millis();
  if (t_actual - t_prev >= t_window) {
    noInterrupts();
     pos = ISRCounter;  // copia atómica de la posición
    interrupts();
    
    pos_rad = pos * conv;
    vel = float( 1000 * ( pos - counter ) )/( t_actual - t_prev );  // cuentas/s, resolucion por 4
    vel_rads = vel * conv;
    counter = pos;
    t_prev = t_actual;


    Serial.print(" Pos: ");
    Serial.print(counter);
    Serial.print(" Pos(rad): ");
    Serial.print(pos);
    Serial.print(" pos(rad/s): ");
    Serial.print(pos_rad);
    Serial.print(" vel: ");
    Serial.print(error_act);
    Serial.print(" vel(rad/s): ");
    Serial.print(vel_rads);

 
  }
}
