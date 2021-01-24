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
#define Temperature_Sensor_Pin 7

#define Rotary_Enconder_Clock_Pin 8
#define Rotary_Enconder_Data_Pin 9
#define Rotary_Enconder_Switch_Pin 4
#define Buzzer_Pin 3
#define Heater_Pin 5
#define Cooler_Pin 6


// Application Constants
#define STARTING_MESSAGE_TIME 250
#define TEXT_BLINK_TIME 	  300
#define ARROW_DOWN 			  0
#define ARROW_RIGHT 		  1

#define SOUND_ALARM_NUMBER_BEEP 	  3
#define SOUND_ALARM_PERIOD_BEEP 	  400



// Global Objects
RotaryEncoder RotaryEnconder(Rotary_Enconder_Data_Pin, Rotary_Enconder_Clock_Pin);
DHT AmbientSensor(Temperature_Sensor_Pin, DHT22);
LiquidCrystal_I2C LCD(0x27, 16, 2); //I2C address = 0x27

// Global Variables
int Last_Rotary_Position = 0;
float Current_Humidity = 0;
float Current_Temperature = 0;
unsigned int Current_Cure_Time = 0;

float Setpoint_Temperature = 50;
unsigned int Setpoint_Cure_Time = 180;

unsigned long Current_Millis = 0;
unsigned long Previous_Millis = 0;
unsigned long ISR_Tick = 0;

volatile int Current_Rotary_Position = 0;


bool Toggle = false;

bool increment = false;


byte button;

// LCD
byte DownArrow[] = {
	B00100,
	B00100,
	B00100,
	B00100,
	B00100,
	B11111,
	B01110,
	B00100};

byte RightArrow[] = {
	B00000,
	B10000,
	B11000,
	B11100,
	B11110,
	B11100,
	B11000,
	B10000};

// Finite State Machine States
typedef enum
{
	STARTING = 0,
	TEMPERATURE_ADJUSTMENT,
	CURE_TIME_ADJUSTMENT,
	HEATING,
	WAITING_START,
	FINISH_CURE
} StatesFSM;

StatesFSM stateFSM = STARTING;

// Functions Prototypes
void Change_Machine_State_to(StatesFSM New_State);
void Trigger_Buzzer_Alarm();

void Read_Rotary_Enconder()
{
	if (digitalRead(Rotary_Enconder_Switch_Pin) == 0)	
	{
		
		while (digitalRead(Rotary_Enconder_Switch_Pin) == 0)
			delay(3);

		Serial.println("Pressed");

		if(stateFSM == TEMPERATURE_ADJUSTMENT) {			
			Change_Machine_State_to(CURE_TIME_ADJUSTMENT);			
		}

		else if(stateFSM == CURE_TIME_ADJUSTMENT) {			
		 	Change_Machine_State_to(WAITING_START);
		}

		else if(stateFSM == WAITING_START) {	
			Current_Cure_Time = Setpoint_Cure_Time;					
		 	Change_Machine_State_to(HEATING);
		}

		else
			Serial.println("F");			
	}

	RotaryEnconder.tick();
	Current_Rotary_Position = RotaryEnconder.getPosition();

	if (Current_Rotary_Position != Last_Rotary_Position)
	{

		//if ( (Current_Rotary_Position == 1) && (stateFSM != TEMPERATURE_ADJUSTMENT) &&  (stateFSM != CURE_TIME_ADJUSTMENT) )
		if ( (stateFSM != TEMPERATURE_ADJUSTMENT) &&  (stateFSM != CURE_TIME_ADJUSTMENT) )
			Change_Machine_State_to(TEMPERATURE_ADJUSTMENT);
		
		//else if (Current_Rotary_Position == 0 && (stateFSM != TEMPERATURE_ADJUSTMENT) &&  (stateFSM != CURE_TIME_ADJUSTMENT) )
		else if ( (stateFSM != TEMPERATURE_ADJUSTMENT) &&  (stateFSM != CURE_TIME_ADJUSTMENT) )
			Change_Machine_State_to(CURE_TIME_ADJUSTMENT);
		
		else if (stateFSM == TEMPERATURE_ADJUSTMENT) {		
			
			Current_Rotary_Position > Last_Rotary_Position ? increment = true : increment = false;
					
			if(increment)
				Setpoint_Temperature++;
			else
			{
				if(Setpoint_Temperature > 30)
					Setpoint_Temperature--;
			}
		}

		else if (stateFSM == CURE_TIME_ADJUSTMENT) {
			Current_Rotary_Position > Last_Rotary_Position ? increment = true : increment = false;
					
			if(increment)
				Setpoint_Cure_Time += 10;
			else
			{
				if(Setpoint_Cure_Time >= 10)
				Setpoint_Cure_Time -= 10;
			}
			//Current_Rotary_Position = 0;
		}

		Serial.print(Current_Rotary_Position);
		Serial.println();
		Last_Rotary_Position = Current_Rotary_Position;
	}
}

void Read_Ambient_Sensor()
{
	Current_Humidity = AmbientSensor.readHumidity();
	Current_Temperature = AmbientSensor.readTemperature();
}

void IST_TIMER_0(void)
{
	Read_Rotary_Enconder();
	Read_Ambient_Sensor();

	if(stateFSM == HEATING) {
		ISR_Tick++;
		
		if(ISR_Tick >= 60000) {

			ISR_Tick = 0;
			Current_Cure_Time--;

			if(round(Current_Cure_Time) <= 0) {								
				Change_Machine_State_to(FINISH_CURE);
			}
		}
	}	
}

void Trigger_Buzzer_Alarm()
{
	int beep = SOUND_ALARM_NUMBER_BEEP + 1;
		
	do {
		if (millis() - Previous_Millis > SOUND_ALARM_PERIOD_BEEP)
		{
			Previous_Millis = millis();
			
			if(Toggle)
			{
				Toggle = false;
				digitalWrite(Buzzer_Pin, HIGH);				
			}
			else
			{
				Toggle = true;
				digitalWrite(Buzzer_Pin, LOW);
				beep--;
			}
		}
	}while(beep > 0);
}

void Change_Machine_State_to(StatesFSM New_State)
{

	switch (New_State)
	{

	case STARTING:
		stateFSM = STARTING;
		break;

	case HEATING:
		stateFSM = HEATING;
		break;

	case TEMPERATURE_ADJUSTMENT:
		stateFSM = TEMPERATURE_ADJUSTMENT;
		break;

	case CURE_TIME_ADJUSTMENT:
		stateFSM = CURE_TIME_ADJUSTMENT;
		break;

	case WAITING_START:
		stateFSM = WAITING_START;
		break;

	case FINISH_CURE:
		stateFSM = FINISH_CURE;
		break;		
	}
}

void Show_Current_Temperature()
{

	LCD.setCursor(0, 0);
	LCD.print(Current_Temperature, 1);
	LCD.print((char)223);
	LCD.print("C");
}

void Show_Temperature_Setpont()
{
	LCD.setCursor(12, 0);	
	LCD.print(Setpoint_Temperature, 0);
	LCD.print((char)223);
	LCD.print("C");
}

void Show_Current_Cure_Time()
{	
	int hours = Current_Cure_Time / 60;
	int mins = round((((Current_Cure_Time / 60.0)) - hours) * 60);
	
	char array_buf[10];
	if(mins < 10)
		sprintf(array_buf, "%dh0%d", hours, mins);
	else	
		sprintf(array_buf, "%dh%d", hours, mins);

	LCD.setCursor(0, 1);
	LCD.print(array_buf);
	LCD.print(" ");
}

void Show_Cure_Time_Setpont()
{
	int hours = Setpoint_Cure_Time / 60;
	int mins = round((((Setpoint_Cure_Time / 60.0)) - hours) * 60);

	if(hours < 0 || mins < 0){
		hours = 0;
		mins = 1;
	}
	
	char array_buf[10];
	if(mins < 10)
		sprintf(array_buf, "%dh0%d", hours, mins);
	else	
		sprintf(array_buf, "%dh%d", hours, mins);	
	
	LCD.setCursor(12, 1);
	LCD.print(array_buf);
}

void Show_Start_Button() {	
	LCD.setCursor(1, 1);
	LCD.print("START");	
}

void Hide_Start_Button() {	
	LCD.setCursor(1, 1);
	LCD.print("     ");	
}

void Run_SFM()
{
	switch (stateFSM)
	{
		case STARTING: {
			LCD.setCursor(3, 0);
			LCD.print("Water Pull");
			LCD.setCursor(1, 1);
			LCD.print("Filament Dryer");
			delay(STARTING_MESSAGE_TIME);
			LCD.clear();

			Change_Machine_State_to(WAITING_START);
			break;
		}

		case WAITING_START: {
			
			LCD.setCursor(11, 0); LCD.write(ARROW_RIGHT);
			LCD.setCursor(11, 1); LCD.write(ARROW_RIGHT);

			if (millis() - Previous_Millis > TEXT_BLINK_TIME)
			{
				Previous_Millis = millis();
				
				if(Toggle)
				{
					Toggle = false;					
					Show_Current_Temperature();					
					Show_Temperature_Setpont();

					LCD.setCursor(0, 1); LCD.write(ARROW_RIGHT);
					Show_Start_Button();

					Show_Cure_Time_Setpont();
				}
				else
				{
					Toggle = true;			
					Show_Current_Temperature();					
					Show_Temperature_Setpont();

					LCD.setCursor(0, 1); LCD.print(" ");
					Show_Start_Button();

					Show_Cure_Time_Setpont();
				}
			}
			break;
		}

		case HEATING: {

			if (millis() - Previous_Millis > TEXT_BLINK_TIME)
			{
				Previous_Millis = millis();
				
				if(Toggle)
				{
					Toggle = false;		
					Show_Current_Temperature();
					LCD.setCursor(11, 0); LCD.write(ARROW_RIGHT);
					Show_Temperature_Setpont();
					LCD.setCursor(5, 1); LCD.write(ARROW_DOWN);
					Show_Current_Cure_Time();
					LCD.setCursor(11, 1); LCD.write(ARROW_RIGHT);
					Show_Cure_Time_Setpont();
				}
				else
				{
					Toggle = true;
					Show_Current_Temperature();
					LCD.setCursor(11, 0); LCD.write(ARROW_RIGHT);
					Show_Temperature_Setpont();
					LCD.setCursor(5, 1); LCD.print(" ");
					Show_Current_Cure_Time();
					LCD.setCursor(11, 1); LCD.write(ARROW_RIGHT);
					Show_Cure_Time_Setpont();
				}
			}			
			break;
		}
		
		case TEMPERATURE_ADJUSTMENT: {			

			if (millis() - Previous_Millis > TEXT_BLINK_TIME)
			{
				Previous_Millis = millis();
				
				if(Toggle)
				{
					Toggle = false;
					LCD.setCursor(0, 1); LCD.print(" ");
					Show_Current_Temperature();
					LCD.setCursor(11, 0); LCD.write(ARROW_RIGHT);
					Show_Temperature_Setpont();
					Hide_Start_Button();
					Show_Cure_Time_Setpont();
				}
				else
				{
					Toggle = true;
					Show_Current_Temperature();
					LCD.setCursor(11, 0); LCD.print(" ");
					Show_Temperature_Setpont();
					Hide_Start_Button();
					Show_Cure_Time_Setpont();
				}
			}

			break;
		}

		case CURE_TIME_ADJUSTMENT: {		

			if (millis() - Previous_Millis > TEXT_BLINK_TIME)
			{
				Previous_Millis = millis();
				
				if(Toggle)
				{
					Toggle = false;		
					Show_Current_Temperature();
					LCD.setCursor(11, 1); LCD.write(ARROW_RIGHT);
					Show_Temperature_Setpont();
					Hide_Start_Button();
					Show_Cure_Time_Setpont();
				}
				else
				{
					Toggle = true;			
					Show_Current_Temperature();
					LCD.setCursor(11, 1); LCD.print(" ");
					Show_Temperature_Setpont();
					Hide_Start_Button();
					Show_Cure_Time_Setpont();
				}
			}

		break;
		}
	
		case FINISH_CURE: {
			Change_Machine_State_to(WAITING_START);
			Trigger_Buzzer_Alarm();
			break;
		}
	}
}

void setup()
{
	Serial.begin(9600);
	Serial.println("START");

	pinMode(Rotary_Enconder_Clock_Pin, INPUT_PULLUP);
	pinMode(Rotary_Enconder_Data_Pin, INPUT_PULLUP);
	pinMode(Rotary_Enconder_Switch_Pin, INPUT_PULLUP);

	pinMode(Buzzer_Pin, OUTPUT);
	digitalWrite(Buzzer_Pin, LOW);	
	
	pinMode(Cooler_Pin, OUTPUT);
	digitalWrite(Cooler_Pin, LOW);

	pinMode(Heater_Pin, OUTPUT);
	digitalWrite(Heater_Pin, HIGH);

	

	Timer1.initialize(1000); // 1ms
	Timer1.attachInterrupt(IST_TIMER_0);

	LCD.init();
	LCD.setBacklight(HIGH);

	LCD.createChar(0, DownArrow);
	LCD.createChar(1, RightArrow);

	AmbientSensor.begin();
}


void loop()
{
	Run_SFM();	
}
