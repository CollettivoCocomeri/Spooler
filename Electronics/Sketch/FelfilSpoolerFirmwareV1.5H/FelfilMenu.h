#ifndef _FELFILMENU_h
#define _FELFILMENU_h

#include <ClickEncoder.h>
#include <TimerOne.h>
#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>
#include <WString.h>
#include <EEPROM.h>

//TTL- Width Sensor
#define NUMTEMPS 16  //length of lookup table
#define smooth 10  //exponential smoothing factor, higher is smoother
#define sensIn A7

#define N_ITEMS 9
#define LOGO_ARRAY_LEN 8

#define MAX_SELECTED_MODE 3
#define MIN_SELECTED_MODE 0

#include "FelfilController.h"

class FelfilMenu
{
  private:
    //PID
    int StepperPosition = 0;  // To store Stepper Motor Position
    int lastStepperPosition = 0;  // To store Stepper Motor Position
    int StepsToTake = 100;    // Controls the speed of the Stepper per Rotary click
    
    int mm = 0;

    float Total = 0;

    // ENCODER
    int16_t last, value;
  
  public:
    unsigned long LcdpreviousMillis;
    int Lcdinterval;
    unsigned long LcdpreviousMillis2;
    int Lcdinterval2;

    // ADC input, diameter output
    float lut3[NUMTEMPS][2] = {
      {-10, 0},
      {0, 0},
      {5, 0},
      {30, 0.5},
      {96, 1.17},
      {115, 1.46},
      {127, 1.60},
      {140, 1.75 },
      {160, 1.99 },
      {200, 2.48},
      {240, 3.00},
      {248, 3.10},
      {256, 3.20},
      {280, 3.50},
      {345, 4.00},
      {360, 4.15}
  };
  
    float sensbuf = lut3[0][0]; //smoothed raw value

    FelfilMenu();
    
    int menuItem = 1;
    int frame = 1;
    int page = 1;
    int lastMenuItem = 1;
    boolean up = false;
    boolean down = false;
    boolean middle = false;

//    menuState[0].nome = "   Set up:"; menuState[0].value = 0;
//    menuState[1].nome = " Diameter:"; menuState[1].value = 0;
//    menuState[2].nome = "     Mode:"; menuState[2].value = 0;
//    menuState[3].nome = "  PullSpd:"; menuState[3].value = 0;
//    menuState[4].nome = "   Offset:"; menuState[4].value = intOffset;
//    menuState[5].nome = "  TravSpd:"; menuState[5].value = travelspd;
//    menuState[6].nome = " SpoolSpd:"; menuState[6].value = spoolRPM;
//    menuState[7].nome = "   FanSpd:"; menuState[7].value = fanspd;
//    menuState[8].nome = "    Stats:"; menuState[8].value = Total;

    String menuState[N_ITEMS] = {
      "   Set up:", // 1 [0]
      " Diameter:", // 2 [1]
      "     Mode:", // 3 [2]
      "  PullSpd:", // 4 [3]
      "   Offset:", // 5 [4]
      "  TravSpd:", // 6 [5]
      " SpoolSpd:", // 7 [6]
      "   FanSpd:", // 8 [7]
      "    Stats:"  // 9 [8]
    };   

    String mode[4] = {"Soft", "Medium", "Hard", "Manual"};
    
    int selectedMode = 0;
    int getSelectedMode();
    void moveDownSelectedMode();
    void moveUpSelectedMode();
    void moveUpStepperPosition();
    void moveDownStepperPosition(int position);
        
    int TravelBegin = 0;

    float Pullinterval = 0;
    
    //EEPROM
    int const adressDiam = 0;
    int const adressOffset = 19;
    
    //Fan
    int pwmPin = 11;
    int fanspd = 255;

    int newpositionEnd = 0;
    int newposition = 0;

    void setNewpositionEnd(int value);

    const int DistrpinDir = 3;
    const int DistrpinStep = 6;

    void blinkDistr(int end_position);
  
    //Sensor Reding
    float sensorValue = 0;  // variable to store the value coming from the sensor
    float measure = 0 ;
    float targettino = 0;
    float sensorMin = 0;

    int diameter;
    float intDiameter = diameter*0.01;
    void moveDownDiameter();
    void moveUpDiameter();
    void moveUpOffset();
    void moveDownOffset();
    void moveUpFanspd();
    void moveDownFanspd();
    void moveUpSpoolspd();
    void moveDownSpoolspd();
    void moveUpTravelspd();
    void moveDownTravelspd();

    void setIntdiameter(float value);
    
    //Enable
    int enablePin;
    int enableState;
    int preenableState;

    void setEnableStates(int enableState, int preenableState);
    void setEnablePinConst(int value);

    void setPullinterval(float value);
    
    void setStepperPosition(float value);
    float getStepperPosition();
    
    float travel;
    float travelspd = 4;
    const float pullspd = 12;
    int spoolspd = 12;
    float spoolRPM = 25;
    float extspd[2];
    int offset = 0;
    float intOffset;
 
    void setExtspd(float vet[]);
    void setIntOffset(float value);
    void setSpoolRPM(float value);
    int getOffset();    
    
    void setMiddle(boolean value);
    
    void drawHome(LiquidCrystal_I2C &lcd);
    void drawMenu(LiquidCrystal_I2C &lcd);
    void updateMenu(LiquidCrystal_I2C &lcd);
    void displayIntMenuPage(LiquidCrystal_I2C &lcd, String menuItem, int position, int value);
    void displayStringMenuPage(LiquidCrystal_I2C &lcd, String value);
    void displayString2MenuPage(LiquidCrystal_I2C &lcd, String value);
    void displayIntStringMenuPage(LiquidCrystal_I2C &lcd, String item, int position, boolean selected, String value);
    void displayMenuItem(LiquidCrystal_I2C &lcd, String item, int position, boolean selected, int value);

    void setMenuItem(int value);
    int getMenuItem();
    void setPage(int value);
    int getPage();

    float getMeasure();

    void updateAfterMenuUpdate(FelfilController &controller);
    void updateAfterVarUpdate(FelfilController &controller);

    void updateDiamEEPROM();
    void updateOffsetEEPROM();
    
    void readRotaryEncoder(ClickEncoder &encoder);
    
    void calibrateSensor();
    void fans();
    void Sensor();
    void setSensorValue(float value);
    void setSensorMin(float value);
    void setupEEPROM();

    int getDiameter();
    float getIntOffset();

    float lookup(float inval, float lut[][2]);
    
    void setupDiameterAndOffsetFromEEPROM(FelfilController &controller);
};
#endif
