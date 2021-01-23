#define Buzzer_Pin 3

void setup() {
  pinMode(Buzzer_Pin, OUTPUT);

}

void loop() {
  digitalWrite(Buzzer_Pin, HIGH);
  delay(500);
  digitalWrite(Buzzer_Pin, LOW);
  delay(500);
}
