#include <ClickEncoder.h>
#include <TimerOne.h>
#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>
#include <WString.h>
#include <EEPROM.h>

#include "FelfilMenu.h"
#include "FelfilController.h"

byte DiameterLogo[LOGO_ARRAY_LEN] = {0x1F, 0x1F, 0x13, 0x15, 0x15, 0x15, 0x13, 0x1F};
byte SpdLogo[LOGO_ARRAY_LEN] = {0x1F, 0x1F, 0x11, 0x17, 0x11, 0x1D, 0x11, 0x1F};
byte mmLogo[LOGO_ARRAY_LEN] = {0x00, 0x00, 0x00, 0x00, 0x0A, 0x15, 0x15, 0x15};
byte MetrLogo1[LOGO_ARRAY_LEN] = {0x11, 0x1B, 0x15, 0x10, 0x11, 0x12, 0x04, 0x08};
byte MetrLogo2[LOGO_ARRAY_LEN] = {0x02, 0x04, 0x08, 0x10, 0x0A, 0x15, 0x15, 0x15};
byte ExLogo[LOGO_ARRAY_LEN] = {0x1F, 0x1F, 0x11, 0x15, 0x12, 0x15, 0x11, 0x1F};
byte XtLogo[LOGO_ARRAY_LEN] = {0x1F, 0x1F, 0x10, 0x15, 0x0D, 0x15, 0x15, 0x1F};

FelfilMenu *menu;
FelfilController *controller;
ClickEncoder *encoder;
LiquidCrystal_I2C *lcd;

//Enable
const int enablePin = 8;
int enableState = 1;
int preenableState = 0;

//Stepper
float intOffset = 0;

boolean middle = false;

int StepperPosition = 0;  // To store Stepper Motor Position
int R = 0;

int selectedMode = 0;
int menuItem = 1;
int page = 1;

float measure = 0;

// MAIN //
void setup() {
  //Controller
  controller = new FelfilController();
  controller->initializePID();
  controller->setEnableStates(enableState, preenableState);
  controller->setEnablePinConst(enablePin);

  //Menu
  menu = new FelfilMenu();
  int selectedMode = menu->getSelectedMode();
  menu->setEnableStates(enableState, preenableState);
  menu->setEnablePinConst(enablePin);
  StepperPosition = menu->getStepperPosition();
  menu->setExtspd(controller->getExtspd());
  menu->setNewpositionEnd(controller->getNewpositionEnd());
  menu->setSpoolRPM(controller->getSpoolRPM());

  //LCD
  lcd = new LiquidCrystal_I2C(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
  lcd->begin(16, 2);
  lcd->setCursor(0, 0);
  lcd->print("Felfil Spooler");
  lcd->setCursor(0, 1);
  lcd->print("Calibrating 1.5H");
  lcd->createChar(0, DiameterLogo);
  lcd->createChar(1, SpdLogo);
  lcd->createChar(2, mmLogo);
  lcd->createChar(3, MetrLogo1);
  lcd->createChar(4, MetrLogo2);
  lcd->createChar(5, ExLogo);
  lcd->createChar(6, XtLogo);

  //Encoder
  encoder = new ClickEncoder(A2, A1, A3);
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);

  //Sensor
//  menu->calibrateSensor();
  float sensorMin = 0;
  float sensorValue = 0;
    pinMode(sensIn, INPUT);
  // calibrate during the first second
  while(millis() < 1000)
  {
    sensorValue = analogRead(sensIn);
    // record the minimum sensor value
    if(sensorValue > sensorMin)
      sensorMin = sensorValue;
  }
  menu->setSensorValue(sensorValue);
  menu->setSensorMin(sensorMin);
  controller->turnPinsOff();

  controller->resetDistr();

  // EEPROM
  menu->setupEEPROM();
  menu->setupDiameterAndOffsetFromEEPROM(*controller);
  intOffset = menu->getIntOffset();
 
  //LCD clear
  delay(500);
  lcd->clear();
}

void loop() {
  // LCD //
  menu->updateMenu(*lcd);
  menuItem = menu->getMenuItem();
  page = menu->getPage();
  measure = menu->getMeasure();
  selectedMode = menu->getSelectedMode();
  StepperPosition = menu->getStepperPosition();
  menu->updateAfterMenuUpdate(*controller);

  // ENCODER //
  menu->readRotaryEncoder(*encoder);
  ClickEncoder::Button button = encoder->getButton();

  if(button != ClickEncoder::Open) {
    switch(button) {
      case ClickEncoder::Clicked:
        middle = true;
        menu->setMiddle(middle);
        lcd->clear();
        break;
    }
  }
  if(button != ClickEncoder::Open) {
    switch(button) {
      case ClickEncoder::Held:
        lcd->clear();
        lcd->setCursor(0, 0);
        lcd->print("Resetting...");
        controller->resetDistr();
        StepperPosition = 0;
        menu->setStepperPosition(StepperPosition);
        menuItem = 1;
        menu->setMenuItem(menuItem);
        controller->setMenuItem(menuItem);
        page = 2;
        menu->setPage(page);
        lcd->clear();
        break;
    }
  }
  if(button != ClickEncoder::Open) {
    switch(button) {
      case ClickEncoder::DoubleClicked:
        if(selectedMode < 3) {
          if(page == 2 && menuItem == 8) {
            R = 0;
          }
        } else if(selectedMode == 3) {
          if(page == 2 && menuItem == 9) {
            R = 0;
          }
        }
        break;
    }
  }

  controller->setR(R);
  controller->Var();
  menu->updateAfterVarUpdate(*controller);

  // Fans
  menu->fans();

  if(measure-intOffset <= 0.10 && menuItem != 1)
  {
    enableState = 0;
    preenableState = enableState;
  }
  else
  {
    enableState = 1;
    preenableState = enableState;
    controller->Brain();
    R = controller->getR();
    menu->setPullinterval(controller->getPullinterval());
  }  //    myPID.SetMode(AUTOMATIC);

  if(enableState == 0)
    digitalWrite(enablePin, HIGH);
  else
    digitalWrite(enablePin, LOW);

  controller->setEnableStates(enableState, preenableState);
  menu->setEnableStates(enableState, preenableState);

}
// END MAIN //

// encoder
void timerIsr() {
  encoder->service();
}
