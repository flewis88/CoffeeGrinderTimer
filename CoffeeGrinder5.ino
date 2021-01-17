/*
Coffee Grinder timer

- Rotary encoder rotation adjusts time duration.
- Pushing encoder powers the relay for set duration, and stores changed duration to EEPROM
- Holding Calibration button allows rotary encoder to adjust the dose weight for the given duration, changing the time/weight ratio
- Releasing the Calibration button stores dose weight to EEPROM
*/


//load libraries
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

//Define variables 

//LCD I2C
#define I2C_ADDR          0x27        //Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN      3
#define En_pin             2
#define Rw_pin             1
#define Rs_pin             0
#define D4_pin             4
#define D5_pin             5
#define D6_pin             6
#define D7_pin             7

//RELAY
#define RELAYSIGNAL        6

// Rotary Encoder Inputs
#define CLK 2
#define DT 3
#define SW 4

//CALIBRATION BUTTON
#define CAL 5

//Initialise the LCD
LiquidCrystal_I2C      lcd(I2C_ADDR, En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

//EEPROM
int timeAddr = 0;
int timeEERead = 0;
int weightAddr = 1;
int weightEERead = 0;
int byteMin = 0;
int byteMax = 255;

//Dose variables
unsigned int grindTimeINT = EEPROM.read(timeAddr);
unsigned int grindWeightINT = EEPROM.read(weightAddr);
float grindTime = float(grindTimeINT)/10; //1 byte max allows max 25.5 second grind time
float grindWeight = float(grindWeightINT)*0.2; //1 byte max allows max 51 gram dose
float grindCoeff = float(grindWeightINT)/float(grindTimeINT);


//Rotary Encoder variables
int counter = 0;
int currentStateCLK;
int lastStateCLK;
unsigned long lastButtonPress = 0;

//Calibration Variables
bool calMode = 0;
unsigned long lastCalButtonPress = 0;
int lastCalBtnState = HIGH;

//Screen Update
long previousMillis = 0;
unsigned int interval = 500;


void setup()
 {
    //Define the LCD as 16 column by 2 rows 
    Serial.begin(9600);
    lcd.begin (16,2);

    Serial.print("Grind Time INT: ");
    Serial.println(grindTimeINT);
    Serial.print("Grind Weight INT: ");
    Serial.println(grindWeightINT);
    Serial.print("Grind Coeff: ");
    Serial.println(grindCoeff);
        
    //Switch on the backlight
    lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
    lcd.setBacklight(HIGH);

    //Configure relay pin, and set to LOW
    pinMode(RELAYSIGNAL, OUTPUT);
    digitalWrite(RELAYSIGNAL,LOW);
    
    lcd.setCursor(3, 0);
    lcd.print("GRINDY BOI");
    screenUpdate();
    
    // Set encoder & pushbutton pins as inputs with internal pull-up
    pinMode(CLK,INPUT_PULLUP);
    pinMode(DT,INPUT_PULLUP);
    pinMode(SW, INPUT_PULLUP);
    pinMode(CAL, INPUT_PULLUP);

    // Read the initial state of CLK
    lastStateCLK = digitalRead(CLK);

 }
 
 
void loop()
{ 
  unsigned long currentMillis = millis();

  //Checks if Calibration Mode is active
  calCheck();

  //Checks Encoder
  encoderCheck();

  //Periodic screen refresh, at defined interval
  if (currentMillis - previousMillis > interval)
  { 
    previousMillis = currentMillis;
  }  

}

void grindRun()
{
  //Compare EEPROM value to variable, and write new value to EEPROM if changed
  timeEERead = EEPROM.read(timeAddr);
  //Serial.print("Grind Time INT: ");
  //Serial.println(grindTimeINT);
  //Serial.print("EEPROM: ");
  //Serial.println(timeEERead);
  if (timeEERead != grindTimeINT)
  {
    EEPROM.write(timeAddr, grindTimeINT);
    Serial.print("Grind Time changed. New time stored to EEPROM: ");
    Serial.println(grindTimeINT);
  }
  
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("GRINDING"); 
  lcd.setCursor(3, 1);
  lcd.print("UP A STORM");

  previousMillis = millis();
  unsigned long startMillis = millis();
  //unsigned long currentMillis = millis();
  //Turn on Relay
  digitalWrite(RELAYSIGNAL,HIGH);

  //Relay powered for defined duration
  while (millis() - startMillis < (grindTimeINT*100)) {
    // Loop til grind time passed.
  } 
   
  //Turn off Relay
  digitalWrite(RELAYSIGNAL,LOW);

  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("GRIND COMPLETE");
  delay(1000);
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("GRINDY BOI");
  screenUpdate();
}

void encoderCheck() 
{  
  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK ){
  //if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (calMode==0){
      if (digitalRead(DT) != currentStateCLK) {
        //Serial.println("Grind Time Change");
        if (grindTimeINT != byteMin)
        {
          grindTimeINT--;
	      }
      }       
      else {
        if (grindTimeINT != byteMax)
        {
          grindTimeINT++;      
        }
      }
      
      grindTime=float(grindTimeINT)/10;
      /*
      Serial.print("Time changed to ");
      Serial.println(grindTime);
      */
      grindWeightINT = grindTimeINT * grindCoeff; 
      grindWeight = float(grindWeightINT)*0.2;
      /*
      Serial.print("Grind Weight Updated to ");
      Serial.println(grindWeight); 
      */
      screenUpdate();    
    }
    else if (calMode == 1){
      if (digitalRead(DT) != currentStateCLK) {
        //Serial.println("Calibration Change");
        if (grindWeightINT != byteMin)
        {
          grindWeightINT--;
        }
      } 
      else {
        if (grindWeightINT != byteMax)
        {
          grindWeightINT++; 
        }
      }
      grindWeight = float(grindWeightINT)*0.2;
      /* 
      Serial.print("Weight changed to ");
      Serial.println(grindWeight);
      Serial.println("Grind Coeff Updated");
      */
      grindCoeff = float(grindWeightINT)/float(grindTimeINT);
      screenUpdate();
              
    }

   /*
   Serial.print("Grind Time Display: ");
   Serial.println(grindTime,1); 
   Serial.print("Grind Weight Display: ");
   Serial.println(grindWeight);
   */
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;

  // Read the button state
  int btnState = digitalRead(SW);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    //debounce
    if (millis() - lastButtonPress > 50) {
      Serial.println("Grinding");
      grindRun();
    }

    // Remember last button press event
    lastButtonPress = millis();
  }

  // Put in a slight delay to help debounce the reading
  delay(1);
}

void calCheck()
{
  // Read the button state
  int calBtnState = digitalRead(CAL);

  //Save grind weight if calibration button has been released
  if (calBtnState == HIGH && lastCalBtnState == LOW) {
    Serial.println("Saving Calibration");
    EEPROM.write(weightAddr, grindWeightINT);
    Serial.println("Grind Weight changed. New weight stored to EEPROM");
    //unnecessary update to grindCoeff?
    //grindCoeff = float(grindWeightINT)/float(grindTimeINT);
    //Serial.print("Grind Coeff changed: ");
    //Serial.println(grindCoeff);
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("GRINDY BOI");
    screenUpdate();
    }

  //Calibration mode when button pressed
  if (calBtnState == LOW) {
    //debounce
    if (millis() - lastCalButtonPress > 50) {
      Serial.println("Calibration Mode");
      calMode = 1;
      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("CALIBRATE");
      screenUpdate();
    }

    // Remember last button press event
    lastCalButtonPress = millis();
  }
  else {
    calMode = 0;  
  }
  // Put in a slight delay to help debounce the reading
  delay(1);
  lastCalBtnState = calBtnState;
}

void screenUpdate(){
    
    lcd.setCursor(0, 1);
    lcd.print("                "); 
    lcd.setCursor(0, 1);
    lcd.print(grindTime,1); 
    lcd.setCursor(4, 1);
    lcd.print("sec"); 
    lcd.setCursor(11, 1);
    //lcd.print("    ");
    lcd.setCursor(10, 1);
    lcd.print(grindWeight,1); 
    lcd.setCursor(14, 1);
    lcd.print("g"); 
}
