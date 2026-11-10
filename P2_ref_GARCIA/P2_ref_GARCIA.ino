const int pwm1 = 32;
const int pwm2 = 33;

const int freq = 1000;
const uint resolution = 10;
const uint resol_num = (1 << resolution) - 1 ;

int duty_cycle = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  ledcAttach(pwm1, freq, resolution);
  ledcAttach(pwm2, freq, resolution);

  Serial.println("Introduce un valor entre -1000 y 1000");
}

void loop() {
  // put your main code here, to run repeatedly
  if (Serial.available() > 0) {
     String dato = Serial.readStringUntil('\n');

     duty_cycle = constrain(dato.toInt(), -1023, 1023);  
     Serial.print("PWM recibido: ");
     Serial.println(duty_cycle);
  }


  if (duty_cycle >= 0) {

    giro_horario(duty_cycle);

  } 
  else {
    giro_antihorario(-duty_cycle);
  }


}

void giro_horario(int duty_cycle) {

  ledcWrite(pwm2, 0);
  ledcWrite(pwm1, duty_cycle);
  
}

void giro_antihorario(int duty_cycle) {


  ledcWrite(pwm1, 0);
  ledcWrite(pwm2, duty_cycle );
}

/*int duty_cycle_to_value( int duty_cycle){

  return (duty_cycle * resol_num)/100;
}*/
