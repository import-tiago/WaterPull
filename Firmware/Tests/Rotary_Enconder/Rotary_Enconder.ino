#include <RotaryEncoder.h>
#include <TimerOne.h>

const int pinoCLK = 8;
const int pinoDT = 9;
const int pinoSW = 4;

RotaryEncoder encoder(pinoDT, pinoCLK);

static int pos = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("START");
  
  pinMode (pinoCLK, INPUT_PULLUP);
  pinMode (pinoDT, INPUT_PULLUP);
  pinMode (pinoSW, INPUT_PULLUP);

  Timer1.initialize(10000);
  Timer1.attachInterrupt(IST_TIMER_0);
}

void loop() {
}

void Read_Encoder() {
  int valor = digitalRead(pinoSW);
  if (valor != 1)
  {
    Serial.println("Pressed");
    while (digitalRead(pinoSW) == 0)
      delay(3);
  }
  
  encoder.tick();
  int newPos = encoder.getPosition();  
  
  if (pos != newPos) {
    Serial.print(newPos);
    Serial.println();
    pos = newPos;
  }
}

void IST_TIMER_0(void) {
  Read_Encoder();
}
