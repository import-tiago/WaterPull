// Native Libraries
#include <Arduino.h>

// ANSI C Libraries
#include <stdlib.h>

// Application Libraries
#include <RotaryEncoder.h>
#include <TimerOne.h>
#include "Adafruit_Sensor.h"
#include <DHT.h>
#include <LiquidCrystal_I2C.h>

// Hardware Definitions
#define Temperature_Sensor_Pin      7
#define Rotary_Enconder_Clock_Pin   8
#define Rotary_Enconder_Data_Pin    9
#define Rotary_Enconder_Switch_Pin  4

// Global Objects
RotaryEncoder RotaryEnconder(Rotary_Enconder_Data_Pin, Rotary_Enconder_Clock_Pin);
DHT AmbientSensor(Temperature_Sensor_Pin, DHT22);
LiquidCrystal_I2C LCD(0x27, 16, 2); //I2C address = 0x27

// Global Variables
int Current_Rotary_Position = 0;
float Current_Humidity = 0;
float Current_Temperature = 0;

// Finite State Machine States
typedef enum {
    STARTING = 0,
    TEMPERATURE_ADJUSTMENT,
    TIME_ADJUSTMENT,
    SHOW_CURRENT_VALUES
} StatesFSM;

StatesFSM stateFSM = STARTING;

void Read_Rotary_Enconder() {
  
  if (digitalRead(Rotary_Enconder_Switch_Pin) != 1)
  {
    Serial.println("Pressed");
    while (digitalRead(Rotary_Enconder_Switch_Pin) == 0)
      delay(3);
  }
  
  RotaryEnconder.tick();
  int Last_Rotary_Position = RotaryEnconder.getPosition();  
  
  if (Current_Rotary_Position != Last_Rotary_Position) {
    Serial.print(Last_Rotary_Position);
    Serial.println();
    Current_Rotary_Position = Last_Rotary_Position;
  }
}

void Read_Ambient_Sensor() {
    Current_Humidity = AmbientSensor.readHumidity();  
    Current_Temperature = AmbientSensor.readTemperature();
}

void IST_TIMER_0(void) {
  Read_Rotary_Enconder();
  Read_Ambient_Sensor();
}


void setup() {
  
  Serial.begin(9600);
  Serial.println("START");
  
  pinMode (Rotary_Enconder_Clock_Pin, INPUT_PULLUP);
  pinMode (Rotary_Enconder_Data_Pin, INPUT_PULLUP);
  pinMode (Rotary_Enconder_Switch_Pin, INPUT_PULLUP);

  Timer1.initialize(10000);
  Timer1.attachInterrupt(IST_TIMER_0);

  LCD.init();
  LCD.setBacklight(HIGH);

  AmbientSensor.begin();
}

void loop() {

      switch(stateFSM) {

       case STARTING:          
          LCD.setCursor(0, 0);
          LCD.print("   Water Pull");
          LCD.setCursor(0, 1);
          LCD.print(" Filament Dryer ");
          delay(100);
          LCD.clear();
          
          stateFSM = SHOW_CURRENT_VALUES;
          break;      
        
        case SHOW_CURRENT_VALUES:
          
          LCD.setCursor(0, 0);
          LCD.print(Current_Temperature, 1); LCD.print((char)223); LCD.print("C"); 
          LCD.setCursor(0, 1);          
          LCD.print(Current_Humidity, 1); LCD.print("%");
          break;
        
        case TEMPERATURE_ADJUSTMENT: break;
        
        case TIME_ADJUSTMENT: break;
        
      }
}

