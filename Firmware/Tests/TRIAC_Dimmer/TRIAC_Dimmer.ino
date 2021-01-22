
int PWM_Heater_Pin = 5;
int Zero_Crossing_Pin = 2;
int Zero_Crossing_Interrupt = 0;

volatile boolean zero_cross = false;

int Heater_Power = 0; //0-255 (duty cycle)

void setup() {
  
  Serial.begin(9600);
  Serial.println("--- START ---");

  pinMode(PWM_Heater_Pin, OUTPUT);
  
  attachInterrupt(Zero_Crossing_Interrupt, ISR_GPIO, RISING);
}

void loop() {

  if(zero_cross) {
    zero_cross = false;
    analogWrite(PWM_Heater_Pin, Heater_Power);    
  }
  
}

void ISR_GPIO() {
  zero_cross = true;
}
