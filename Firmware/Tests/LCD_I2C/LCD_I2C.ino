#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); //I2C address = 0x27

void setup()
{
  lcd.init();
  lcd.setBacklight(HIGH);
}

void loop()
{  
  lcd.setCursor(0, 0);
  lcd.print("Hello World!");
  lcd.setCursor(0, 1);
  lcd.print("Basic Test");
  delay(100);  
}
