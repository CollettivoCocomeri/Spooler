#include <ClickEncoder.h>
#include <TimerOne.h>
#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>
#include <WString.h>
#include <EEPROM.h>
// LCD
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
unsigned long     LcdpreviousMillis = 0;
int               Lcdinterval = 1000;
unsigned long     LcdpreviousMillis2 = 0;
int               Lcdinterval2 = 100;

byte DiameterLogo[] = {
  0x1F,
  0x1F,
  0x13,
  0x15,
  0x15,
  0x15,
  0x13,
  0x1F
};

byte SpdLogo[] = {
  0x1F,
  0x1F,
  0x11,
  0x17,
  0x11,
  0x1D,
  0x11,
  0x1F
};
byte mmLogo[] = {
  0x00,
  0x00,
  0x00,
  0x00,
  0x0A,
  0x15,
  0x15,
  0x15
};

byte MetrLogo1[] = {
  0x11,
  0x1B,
  0x15,
  0x10,
  0x11,
  0x12,
  0x04,
  0x08
};
byte MetrLogo2[] = {
  0x02,
  0x04,
  0x08,
  0x10,
  0x0A,
  0x15,
  0x15,
  0x15
};
byte ExLogo[] = {
  0x1F,
  0x1F,
  0x11,
  0x15,
  0x12,
  0x15,
  0x11,
  0x1F
};
byte XtLogo[] = {
  0x1F,
  0x1F,
  0x10,
  0x15,
  0x0D,
  0x15,
  0x15,
  0x1F
};
// MENU
int menuItem = 1;
int frame = 1;
int page = 1;
int lastMenuItem = 1;

String menuItem1 = "   Set up:";
String menuItem2 = " Diameter:";
String menuItem3 = "     Mode:";
String menuItem4 = "  PullSpd:";
String menuItem5 = "   Travel:";
String menuItem6 = "  TravSpd:";
String menuItem7 = " SpoolSpd:";
String menuItem8 = "   FanSpd:";
String menuItem9 = "    Stats:";


String mode[4] = { "Soft", "Medium", "Hard", "Manual" };

int selectedMode = 0;

int diameter = 175;
float intdiameter = 0;
int travel = 80;
int travelspd = 0;
float pullspd = 12;
int spoolspd = 12;
int fanspd = 255;
int kilograms = 0;
float extspd = 0;
float extspd2 = 0;
boolean up = false;
boolean down = false;
boolean middle = false;
// ENCODER
ClickEncoder *encoder;
int16_t last, value;

//TTL - Width Sensor
#define NUMTEMPS 16  //length of lookup table
#define smooth 10  //exponential smoothing factor, higher is smoother
#define sensIn A7

// ADC input, diameter output
static float lut3[NUMTEMPS][2] = {
  { -10, 0},
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

//Sensor Reding
float sensorValue = 0;  // variable to store the value coming from the sensor
float measure = 0 ;
float targettino = 0;
float sensorMin = 0;
char Result [0];

//PID
double Setpoint, Input, Output;
//Define the aggressive and conservative Tuning Parameters
double SoftKp = 6.9 ,   SoftKi = 0.23,   SoftKd = 5.175;
double MediumKp = 10.8 , MediumKi = 0.45, MediumKd = 6.48;
double HardKp = 15.48,      HardKi = 0.62,   HardKd = 9.675 ;
double Hard2Kp = 0.1548,      Hard2Ki = 0.0062,   Hard2Kd = 0.9675 ;
PID myPID(&Input, &Output, &Setpoint, SoftKp, SoftKi, SoftKd, DIRECT);

float lastOutput = 0;

//Stepper 1 - Puller - Pin Definition
const int         PullpinDir = 2;
const int         PullpinStep = 5;
int               PullpinStepState = HIGH;
//Stepper 1 - Puller - Time
unsigned long     Pulltime;
unsigned long     PullpreviousMillis = 0;
float             Pullinterval = 0 ;
//Stepper 1 - round counter
int               r = 0;
int               R = 0;
float             Total = 0;
// start speed
float             stsp = 0;
float             stspMin ;
float             stspMax ;

//Stepper 2 - Distribution - Pin Definitio
const int         DistrpinDir = 3;
const int         DistrpinStep = 6;
int               DistrpinStepState = HIGH;
int               DistrpinDirState = LOW;
int               DistributionSteps = 15600;
//Stepper 2 - Distribution - Speed Time
unsigned long     Distrtime;
unsigned long     DistrpreviousMillis = 0;
float             Distrinterval = 0;
//Stepper 2 - Distribution - Direction Time
unsigned long     DirDistrtime;
unsigned long     DirDistrpreviousMillis = 0;
float             DirDistrinterval = 50000;
// Stepper 2 - Distribution - Radious counter
float             rad = 0;
// Stepper 2 reset
int               numStepMotore = 7800;
int               Stepmm = 24;
//const int numStepMotore = 200;
//Stepper 3 - Spool - Pin Definition
const int         SpoolpinDir = 4;
const int         SpoolpinStep = 7;
int               SpoolpinStepState = HIGH;
//Stepper 3 - Spool - Time
unsigned long     Spooltime;
unsigned long     SpoolpreviousMillis = 0;
float             Spoolinterval = 0;
//Enable
const int enablePin = 8;
//Millis
extern unsigned long timer0_millis;

//EEPROM
int adressDiam = 0;

//Fan

int pwmPin = 11;

int y = 0;
// MAIN //
void setup() {
  //LCD
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Felfil Spooler");
  lcd.setCursor(0, 1);
  lcd.print("Calibrating....");
  lcd.createChar(1, SpdLogo);
  lcd.createChar(0, DiameterLogo);
  lcd.createChar(2, mmLogo);
  lcd.createChar(3, MetrLogo1);
  lcd.createChar(4, MetrLogo2);
  lcd.createChar(5, ExLogo);
  lcd.createChar(6, XtLogo);
  //Encoder
  encoder = new ClickEncoder(A2, A1, A3);
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
  last = encoder->getValue();
  if ( page >= 2 ) {
    encoder->setAccelerationEnabled(true);
  } else if ( page >= 2 && menuItem == 1) {
    encoder->setAccelerationEnabled(true);
  } else  encoder->setAccelerationEnabled(false);

  //Sensor
  pinMode( sensIn, INPUT);
  // calibrate during the first second
  while (millis() < 1000) {
    sensorValue = analogRead(sensIn);
    // record the minimum sensor value
    if (sensorValue > sensorMin) {
      sensorMin = sensorValue;
    }
  }
  //Stepper 1 - Puller
  pinMode(PullpinStep, OUTPUT);
  pinMode(PullpinDir, OUTPUT);
  //Stepper 2 - Distribution
  pinMode(DistrpinStep, OUTPUT);
  pinMode(DistrpinDir, OUTPUT);
  //Stepper 3 - Spool
  pinMode(SpoolpinStep, OUTPUT);
  pinMode(SpoolpinDir, OUTPUT);
  //Stepper enable
  pinMode ( enablePin, OUTPUT);
  //ResetDistr
  resetDistr ();
  // EEPROM Diameter
  diameter = ((EEPROM.read(adressDiam) * 256) + EEPROM.read(adressDiam + 1));
  //LCD clear
  lcd.clear();
  // Fan
  //pinMode(pwmPin, OUTPUT);
}
void loop() {



  // LCD //
  unsigned long currentMillis = millis();
  if (currentMillis - LcdpreviousMillis2 >= Lcdinterval2) {
    // save the last time you blinked the LED
    LcdpreviousMillis2 = currentMillis;
    drawMenu();
  }
  // ENCODER //
  readRotaryEncoder();
  ClickEncoder::Button b = encoder->getButton();
  if (b != ClickEncoder::Open) {
    switch (b) {
      case ClickEncoder::Clicked:
        middle = true;
        lcd.clear();
        break;
    }
  }
  if (b != ClickEncoder::Open) {
    switch (b) {
      case ClickEncoder::Held:
        lcd.clear();
        digitalWrite(enablePin, HIGH);
        menuItem = 1;
        page = 2;
        break;

    }
  }
  // Sensor
  // Sensor();
  // Steppers
  //  Pull();
  //  Distr();
  //  Spool();
  // PID
  Brain();
  // Variabilies
  Var();
  // Fans
  fans();
}
// MAIN //

// MENU //
void drawHome() {
  //MenuHome
  if (Input <= 0.1) {
    lcd.setCursor (0, 0);
    lcd.print (" Spooler ready! ");
  } else {
    lcd.setCursor (0 , 0 );
    lcd.print(char(0));
    unsigned long currentMillis = millis();
    if (currentMillis - LcdpreviousMillis >= Lcdinterval) {
      // save the last time you blinked the LED
      LcdpreviousMillis = currentMillis;
      lcd.setCursor ( 1, 0);
      lcd.print abs(measure);
      lcd.setCursor (9, 0);
      lcd.print (extspd, 2);
      lcd.setCursor (13, 0);
      lcd.print (" ");
    }
    lcd.setCursor(5, 0);
    lcd.print(char(2));
    lcd.setCursor(6, 0);
    lcd.print(char(2));
    //
    lcd.setCursor(7, 0);
    lcd.print(" ");
    lcd.setCursor(8, 0);
    lcd.print(char(1));
    lcd.setCursor (13, 0);
    lcd.print(" ");
    lcd.setCursor (14, 0);
    lcd.print(char(3));
    lcd.setCursor (15, 0);
    lcd.print(char(4));
  }
  //  lcd.setCursor (0, 0);
  //  lcd.print (DistributionSteps);
  //  lcd.setCursor (6, 0);
  //  lcd.print(y);
}

void drawMenu() {

  if (menuItem == 2 && page == 1) {
    up = false;
  }
  if (up && page == 1 ) {

    up = false;
    lastMenuItem = menuItem;
    menuItem--;
    if (menuItem == 0)
    {
      menuItem = 1;
    }
  }

  if (down && page == 1) //We have turned the Rotary Encoder Clockwise
  {
    down = false;
    lastMenuItem = menuItem;
    menuItem++;
    if (menuItem == 10 && selectedMode == 3)
    {
      menuItem--;
    }
    else if (menuItem == 9 && selectedMode != 3)
    {
      menuItem--;
    }
  }
  if (page == 1 && menuItem == 1) {
    page = 2;
  }

  if (middle && menuItem == 1) //Middle Button is Pressed
  {
    middle = false;

    if (page == 1 ) {

      page = 2;
    } else if (page == 2)
    {
      page = 3;
    } else if (page == 3)
    {
      page = 4;
    } else if (page == 4) {

      if (selectedMode <= 2) {
        menuItem = 2;
        page = 1;
      } else (page = 5);
    }
    else if (page == 5) {
      menuItem = 2;
      page = 1;
    }

  }

  if (middle && menuItem >= 2) //Middle Button is Pressed
  {
    middle = false;

    //    if (page == 1 && menuItem == 1) {
    //
    //      page = 2;
    //
    //    }


    if (page == 1 ) {

      page = 2;


    } else if (page == 2)
    {
      page = 1;
    }
  }
  ////////

  if (page == 1 && menuItem >= 2 && selectedMode == 3)
  {
    if (menuItem == 2 )
    {
      drawHome();
      displayMenuItem(menuItem2, 1, true, intdiameter);
    }
    else if (menuItem == 3)
    {
      drawHome();
      displayIntStringMenuPage(menuItem3, 1, true, mode[selectedMode]);
    }
    else if (menuItem == 4 ) {

      drawHome();
      //displayMenuItem(menuItem4, 1, true, (extspd,2));
      lcd.setCursor (0, 1);
      lcd.print(">  PullSpd:");
      lcd.setCursor(12, 1);
      lcd.print(extspd2, 2);
    }
    else if (menuItem == 5 )
    {
      drawHome();
      displayMenuItem(menuItem5, 1, true, travel);
    } else if (menuItem == 6 )
    {
      drawHome();
      //displayMenuItem(menuItem6, 1, true, travelspd);
      lcd.setCursor (0, 1);
      lcd.print (">  TravSpd:");
      if (  travelspd > 0 && travelspd < 10  )
      {
        lcd.setCursor(12, 1);
        lcd.print ("   ");
        lcd.setCursor(15, 1);
        lcd.print (travelspd);
      } else if ( travelspd > 9 && travelspd < 100 )
      {
        lcd.setCursor(12, 1);
        lcd.print ("  ");
        lcd.setCursor(14, 1);
        lcd.print (travelspd);
      } else if (travelspd > 99 && travelspd < 999)
      {
        lcd.setCursor(12, 1);
        lcd.print (" ");
        lcd.setCursor(13, 1);
        lcd.print (travelspd);
      } else if (travelspd == 0)
      {
        lcd.setCursor(12, 1);
        lcd.print ("Auto");
      }
    } else if (menuItem == 7 )
    {
      drawHome();
      displayMenuItem(menuItem7, 1, true, spoolspd);
    } else if (menuItem == 8 )
    {
      drawHome();
      displayMenuItem(menuItem8, 1, true, fanspd);
    }
    else if (menuItem == 9 )
    {
      drawHome();
      displayMenuItem(menuItem9, 1, true, kilograms);
    }
  }
  else if (page == 1 && menuItem >= 2 && selectedMode != 3) {
    if (menuItem == 2 )
    {
      drawHome();
      displayMenuItem(menuItem2, 1, true, intdiameter);
    }
    else if (menuItem == 3)
    {
      drawHome();
      displayIntStringMenuPage(menuItem3, 1, true, mode[selectedMode]);
    }
    else if (menuItem == 4 ) {

      drawHome();
      displayMenuItem(menuItem5, 1, true, travel);
    }
    else if (menuItem == 5 )
    {
      drawHome();
      // displayMenuItem(menuItem6, 1, true, travelspd);
      lcd.setCursor (0, 1);
      lcd.print (">  TravSpd:");
      if (  travelspd > 0 && travelspd < 10  )
      {
        lcd.setCursor(12, 1);
        lcd.print ("   ");
        lcd.setCursor(15, 1);
        lcd.print (travelspd);
      } else if ( travelspd > 9 && travelspd < 100 )
      {
        lcd.setCursor(12, 1);
        lcd.print ("  ");
        lcd.setCursor(14, 1);
        lcd.print (travelspd);
      } else if (travelspd > 99 && travelspd < 999)
      {
        lcd.setCursor(12, 1);
        lcd.print (" ");
        lcd.setCursor(13, 1);
        lcd.print (travelspd);
      } else if (travelspd == 0)
      {
        lcd.setCursor(12, 1);
        lcd.print ("Auto");
      }
    }
    else if (menuItem == 6 )
    {
      drawHome();
      displayMenuItem(menuItem7, 1, true, spoolspd);
    }
    else if (menuItem == 7 )
    {
      drawHome();
      displayMenuItem(menuItem8, 1, true, fanspd);
    }
    else if (menuItem == 8 )
    {
      drawHome();
      displayMenuItem(menuItem9, 1, true, kilograms);
    }
  }


  //

  //
  if (selectedMode == 3) {
    if (page == 2 && menuItem == 1)
    {
      digitalWrite (enablePin, HIGH);
      displayStringMenuPage( mode[selectedMode]);
      if (up) {
        up = false;
        selectedMode--;

        if (selectedMode <= 0)
        {
          selectedMode = 0;
        }
      } else if (down) {
        down = false;
        selectedMode++;

        if (selectedMode >= 3)
        {
          selectedMode = 3;
        }

      }
    }
    if (page == 3 && menuItem == 1)
    {
      digitalWrite (enablePin, HIGH);
      lcd.setCursor (0, 0);
      lcd.print ("Set Diameter:");
      lcd.setCursor (0, 1);
      lcd.print (intdiameter, 2);

      if (up) {
        up = false;
        diameter--;
        EEPROM.update(adressDiam, highByte(diameter));
        EEPROM.update(adressDiam + 1, lowByte(diameter));
      } else if (down) {
        down = false;
        diameter++;
        EEPROM.update(adressDiam, highByte(diameter));
        EEPROM.update(adressDiam + 1, lowByte(diameter));
      }

    }
    if (page == 4 && menuItem == 1)
    {
      lcd.setCursor (0, 0);
      lcd.print ("Set Travel:");
      lcd.setCursor (0, 1);
      lcd.print (travel);

      if (up) {
        up = false;
        travel--;

        //        UpDistr();

      } else if (down) {
        down = false;
        travel++;
        //        DownDistr();
      }

    }
    if (page == 5 && menuItem == 1)
    {
      digitalWrite (enablePin, HIGH);
      lcd.setCursor (0, 0);
      lcd.print ("Set PullSpeed:");
      lcd.setCursor (0, 1);
//      extspd2 = 60 / ((pullspd * 400) / 1000) * 0.0942;
//      extspd2= 1413/(100*pullspd);
      
      lcd.print (extspd2, 2);
      if (up) {
        up = false;
        extspd2 = extspd2 + 0.1;
      } else if (down) {
        down = false;
        extspd2 = extspd2 - 0.1;
      }

      pullspd = 1413/(100*extspd2);
      

      
      Pull();
    }
    if (page == 2 && menuItem == 2 )
    {
      drawHome();
      displayIntMenuPage(menuItem2, 1, intdiameter);
      if (up) {
        up = false;
        diameter--;
        EEPROM.update(adressDiam, highByte(diameter));
        EEPROM.update(adressDiam + 1, lowByte(diameter));
      } else if (down) {
        down = false;
        diameter++;
        EEPROM.update(adressDiam, highByte(diameter));
        EEPROM.update(adressDiam + 1, lowByte(diameter));
      }
    }
    else if (page == 2 && menuItem == 3)
    {
      drawHome();
      displayString2MenuPage( mode[selectedMode]);
      if (up) {
        up = false;
        selectedMode--;

        if (selectedMode <= 0)
        {
          selectedMode = 0;
        }
      } else if (down) {
        down = false;
        selectedMode++;

        if (selectedMode >= 3)
        {
          selectedMode = 3;
        }

      }
    }
    else if (page == 2 && menuItem == 4)
    {
      drawHome();
      lcd.setCursor(0, 1);
      lcd.print("Set");
      lcd.setCursor(3, 1);
      lcd.print("Pullspd");
      lcd.setCursor(11, 1);
      lcd.print ("  ");
      lcd.setCursor(12, 1);
      extspd2 = 60 / ((pullspd * 400) / 1000) * 0.0942;
      lcd.print (extspd2, 3);
      if (up) {
        up = false;
        pullspd = pullspd + 0.1;
      } else if (down) {
        down = false;

        pullspd = pullspd - 0.1;
      }
    }
    else if (page == 2 && menuItem == 5)
    {
      drawHome();
      displayIntMenuPage(menuItem5, 1, travel);
      if (up) {
        up = false;
        travel--;
      } else if (down) {
        down = false;
        travel++;
      }
    }
    else if (page == 2 && menuItem == 6)
    {
      drawHome();
      // displayIntMenuPage(menuItem6, 1, travelspd);
      lcd.setCursor (0, 1);
      lcd.print ("Set TravSpd ");
      if (  travelspd > 0 && travelspd < 10  )
      {
        lcd.setCursor(12, 1);
        lcd.print ("   ");
        lcd.setCursor(15, 1);
        lcd.print (travelspd);
      } else if ( travelspd > 9 && travelspd < 100 )
      {
        lcd.setCursor(12, 1);
        lcd.print ("  ");
        lcd.setCursor(14, 1);
        lcd.print (travelspd);
      } else if (travelspd > 99 && travelspd < 999)
      {
        lcd.setCursor(12, 1);
        lcd.print (" ");
        lcd.setCursor(13, 1);
        lcd.print (travelspd);
      } else if (travelspd == 0)
      {
        lcd.setCursor(12, 1);
        lcd.print ("Auto");
      }
      if (up) {
        up = false;
        travelspd--;
      } else if (down) {
        down = false;
        travelspd++;
      }
    } else if (page == 2 && menuItem == 7)
    {
      drawHome();
      displayIntMenuPage(menuItem7, 1, spoolspd);
      if (up) {
        up = false;
        spoolspd--;
      } else if (down) {
        down = false;
        spoolspd++;
      }
    }
    else if (page == 2 && menuItem == 8)
    {
      drawHome();
      displayIntMenuPage(menuItem8, 1, fanspd);
      if (up) {
        up = false;
        fanspd--;
      } else if (down) {
        down = false;
        fanspd++;
      }
    }
    else if (page == 2 && menuItem == 9)
    {
      drawHome();
      lcd.setCursor ( 0, 1);
      lcd.print ("Total meter:");
      lcd.setCursor (13, 1);
      lcd.print (Total);
    }
  } else if (selectedMode != 3) {
    if (page == 2 && menuItem == 1)
    {
      digitalWrite (enablePin, HIGH);
      displayStringMenuPage( mode[selectedMode]);
      if (up) {
        up = false;
        selectedMode--;

        if (selectedMode <= 0)
        {
          selectedMode = 0;
        }
      } else if (down) {
        down = false;
        selectedMode++;

        if (selectedMode >= 3)
        {
          selectedMode = 3;
        }

      }
    }
    if (page == 3 && menuItem == 1)
    {
      digitalWrite (enablePin, HIGH);
      lcd.setCursor (0, 0);
      lcd.print ("Set Diameter:");
      lcd.setCursor (0, 1);
      lcd.print (intdiameter, 2);
      if (up) {
        up = false;
        diameter--;
        EEPROM.update(adressDiam, highByte(diameter));
        EEPROM.update(adressDiam + 1, lowByte(diameter));
      } else if (down) {
        down = false;
        diameter++;
        EEPROM.update(adressDiam, highByte(diameter));
        EEPROM.update(adressDiam + 1, lowByte(diameter));
      }

      //displayIntMenuPage(menuItem1, 1, diameter);

    }
    if (page == 4 && menuItem == 1)
    {
      lcd.setCursor (0, 0);
      lcd.print ("Set Travel:");
      lcd.setCursor (0, 1);
      lcd.print (travel);
      digitalWrite (enablePin, LOW);
      //      EncoderMoveStepper();
      if (up) {
        up = false;
        travel--;
      } else if (down) {
        down = false;
        travel++;
      }
    }
    if (page == 5 && menuItem == 1)
    {
      digitalWrite (enablePin, HIGH);
      lcd.setCursor (0, 0);
      lcd.print ("Set PullSpeed:");
      lcd.setCursor (0, 1);
      lcd.print (pullspd);
    }
    if (page == 2 && menuItem == 2 )
    {
      drawHome();
      lcd.setCursor(0, 1);
      lcd.print("Set");
      lcd.setCursor(3, 1);
      lcd.print("Diameter");
      lcd.setCursor(11, 1);
      lcd.print ("  ");
      lcd.setCursor(12, 1);
      lcd.print (intdiameter, 2);
      //displayIntMenuPage(menuItem2, 1, intdiameter);
      if (up) {
        up = false;
        diameter--;
        EEPROM.update(adressDiam, highByte(diameter));
        EEPROM.update(adressDiam + 1, lowByte(diameter));
      } else if (down) {
        down = false;
        diameter++;
        EEPROM.update(adressDiam, highByte(diameter));
        EEPROM.update(adressDiam + 1, lowByte(diameter));
      }
    }
    else if (page == 2 && menuItem == 3)
    {
      drawHome();
      displayString2MenuPage( mode[selectedMode]);
      if (up) {
        up = false;
        selectedMode--;

        if (selectedMode <= 0)
        {
          selectedMode = 0;
        }
      } else if (down) {
        down = false;
        selectedMode++;
        if (selectedMode >= 3)
        {
          selectedMode = 3;
        }
      }
    }
    else if (page == 2 && menuItem == 4)
    {
      drawHome();
      displayIntMenuPage(menuItem5, 1, travel);
      if (up) {
        up = false;
        travel--;
      } else if (down) {
        down = false;
        travel++;
      }
    }
    else if (page == 2 && menuItem == 5)
    {
      drawHome();
      //displayIntMenuPage(menuItem6, 1, travelspd);
      lcd.setCursor (0, 1);
      lcd.print ("Set TravSpd ");
      if (  travelspd > 0 && travelspd < 10  )
      {
        lcd.setCursor(12, 1);
        lcd.print ("   ");
        lcd.setCursor(15, 1);
        lcd.print (travelspd);
      } else if ( travelspd > 9 && travelspd < 100 )
      {
        lcd.setCursor(12, 1);
        lcd.print ("  ");
        lcd.setCursor(14, 1);
        lcd.print (travelspd);
      } else if (travelspd > 99 && travelspd < 999)
      {
        lcd.setCursor(12, 1);
        lcd.print (" ");
        lcd.setCursor(13, 1);
        lcd.print (travelspd);
      } else if (travelspd == 0)
      {
        lcd.setCursor(12, 1);
        lcd.print ("Auto");
      }
      if (up) {
        up = false;
        travelspd--;
      } else if (down) {
        down = false;
        travelspd++;
      }
    }
    else if (page == 2 && menuItem == 6)
    {
      drawHome();
      displayIntMenuPage(menuItem7, 1, spoolspd);
      if (up) {
        up = false;
        spoolspd--;
      } else if (down) {
        down = false;
        spoolspd++;
      }
    } else if (page == 2 && menuItem == 7)
    {
      drawHome();
      displayIntMenuPage(menuItem8, 1, fanspd);
      if (up) {
        up = false;
        fanspd--;
      } else if (down) {
        down = false;
        fanspd++;
      }
    }
    else if (page == 2 && menuItem == 8)
    {
      drawHome();
      // STATS
      lcd.setCursor ( 0, 1);
      lcd.print ("Total meter:");
      lcd.setCursor (13, 1);
      lcd.print (Total);
    }

  }

  if (menuItem > 1) {
    Sensor();
  }
}

void displayIntMenuPage(String menuItem, int position, int value) {

  lcd.setCursor(0, position);
  lcd.print("Set");
  lcd.setCursor(3, position);
  lcd.print(menuItem);

  if ( value == 0 && menuItem != 2) {
    lcd.setCursor (12, position);
    lcd.print ("   ");
    lcd.setCursor(15, position);
  }
  else if (  value > 0 && value < 10 && menuItem != 2 )
  {
    lcd.setCursor(12, position);
    lcd.print ("   ");
    lcd.setCursor(15, position);
  } else if ( value > 9 && value < 100 && menuItem != 2)
  {
    lcd.setCursor(12, position);
    lcd.print ("  ");
    lcd.setCursor(14, position);

  } else if (value > 99 && value < 999 && menuItem != 2)
  {
    lcd.setCursor(12, position);
    lcd.print (" ");
    lcd.setCursor(13, position);
  } else if (value > 999 && value < 9999 && menuItem != 2)
  {
    lcd.setCursor(12, position);
  }
  lcd.print (value);



}

void displayStringMenuPage(String value) {
  lcd.setCursor(0, 0);
  lcd.print("Set Mode:");
  lcd.setCursor(0, 1);
  lcd.print(">");
  if (selectedMode == 0) {
    lcd.setCursor (1, 1);
    lcd.print ("    ");
    lcd.setCursor(5, 1);
  } else if (selectedMode == 1) {
    lcd.setCursor (1, 1);
    lcd.print (" ");
    lcd.setCursor(3, 1);
  } else if (selectedMode == 2) {
    lcd.setCursor (1, 1);
    lcd.print ("    ");
    lcd.setCursor(5, 1);
  } else if (selectedMode == 3) {
    lcd.setCursor (1, 1);
    lcd.print (" ");
    lcd.setCursor(3, 1);
  }

  lcd.print(value);
}
void displayString2MenuPage(String value) {
  lcd.setCursor(0, 1);
  lcd.print("Set");
  lcd.setCursor(3, 1);
  lcd.print(" Mode");

  if (selectedMode == 0) {
    lcd.setCursor (8, 1);
    lcd.print ("    ");
    lcd.setCursor(12, 1);
  } else if (selectedMode == 1) {
    lcd.setCursor (8, 1);
    lcd.print (" ");
    lcd.setCursor(10, 1);
  } else if (selectedMode == 2) {
    lcd.setCursor (8, 1);
    lcd.print ("    ");
    lcd.setCursor(12, 1);
  } else if (selectedMode == 3) {
    lcd.setCursor (8, 1);
    lcd.print (" ");
    lcd.setCursor(10, 1);
  }
  lcd.print(value);
}
void displayIntStringMenuPage(String item, int position, boolean selected, String value) {
  lcd.setCursor(0, position);
  lcd.print (">");
  lcd.setCursor(1, position);
  lcd.print( item);

  lcd.setCursor(12, position);


  lcd.print (value);
}

void displayMenuItem(String item, int position, boolean selected, int value) {

  lcd.setCursor(0, position);
  lcd.print (">");
  lcd.setCursor(1, position);
  lcd.print( item);

  if ( value == 0 && menuItem != 2) {
    lcd.setCursor (12, position);
    lcd.print ("   ");
    lcd.setCursor(15, position);
  }
  else if (  value > 0 && value < 10 && menuItem != 2 )
  {
    lcd.setCursor(12, position);
    lcd.print ("   ");
    lcd.setCursor(15, position);
  } else if ( value > 9 && value < 100 && menuItem != 2)
  {
    lcd.setCursor(12, position);
    lcd.print ("  ");
    lcd.setCursor(14, position);

  } else if (value > 99 && value < 999 && menuItem != 2)
  {
    lcd.setCursor(12, position);
    lcd.print (" ");
    lcd.setCursor(13, position);
  } else if (value > 999 && value < 9999 && menuItem != 2)
  {
    lcd.setCursor(12, position);
  }
  lcd.print (value);

  if ( menuItem == 2 ) {
    lcd.setCursor(11, position);
    lcd.print ("  ");
    lcd.setCursor(12, position);
    lcd.print (intdiameter, 2);
  }

}
// MENU  //

//  ENCODER //
void readRotaryEncoder() {

  value += encoder->getValue();

  if (value > last) {
    last = value ;
    down = true;
    //lcd.clear();
  } else   if (value < last) {
    last = value;
    up = true;
    //lcd.clear();
  }
}

void timerIsr() {
  encoder->service();
}
//  ENCODER //

// SENSOR //
void Sensor() {
  sensorValue = analogRead(sensIn);
  sensbuf += (sensorValue - sensbuf) - sensorMin; //smoothing
  measure = abs (lookup(sensbuf, lut3));
}

float lookup(float inval, float lut[][2]) {
  float out;
  byte i;
  for (i = 1; i < NUMTEMPS; i++)
  {
    if (lut[i][0] > inval)
    {
      return lut[i - 1][1] + (inval - lut[i - 1][0]) * (lut[i][1] - lut[i - 1][1]) / (lut[i][0] - lut[i - 1][0]);
    }
  }
}
// SENSOR //

// VARIABILIES //
void Var() {
  intdiameter = diameter * 0.01;
  Spoolinterval = spoolspd;
  if (travelspd == 0) {
    Distrinterval = Pullinterval * 20;
  } else if (travelspd != 0) {
    Distrinterval = travelspd;
  }
  if (travel <= 0 ) {
    travel = 0;
  }
  if (travel >= 80 ) {
    travel = 80;
  }

  DistributionSteps = map (travel, 0, 80, 0, 15600);

  extspd = 60 / ((Pullinterval * 400) / 1000) * 0.0942;



  //travel = numStepMotore * (spoolwidth * 0.01);
  //DirDistrinterval = Distrinterval * (travel * 41.5);
  Total = R * 0.0942;
}
// VARIABILIES //

// PID //
void Brain () {
  //initialize the variables we're linked to
  Setpoint = diameter * 0.01;
  //turn the PID on

  //Strarting speed pot
  stsp = pullspd;

  if (pullspd == 0 && selectedMode == 3) {

    digitalWrite(enablePin, HIGH);

  }


  //PID
  Input = measure ;
//  myPID.SetSampleTime (1000);
  double gap = abs(Setpoint - Input); //distance away from setpoint
  if (menuItem == 1 ) {
    digitalWrite(enablePin, HIGH);
    myPID.SetTunings(SoftKp, SoftKi, SoftKd);
    myPID.SetMode(AUTOMATIC);
    myPID.Compute();
    myPID.SetOutputLimits(0, 1);
    myPID.SetMode(MANUAL);
    myPID.SetMode(AUTOMATIC);
    myPID.Compute();
    myPID.SetOutputLimits(-1, 0);
  }
  if (Input <= 0.10 )
  {
    //myPID.SetMode(MANUAL);
    digitalWrite(enablePin, HIGH);
  }
  else
  { digitalWrite(enablePin, LOW);
    //    myPID.SetMode(AUTOMATIC);

    if (selectedMode == 0) {
      myPID.SetOutputLimits(6, 120);
      if ( Setpoint >= 2.4){
       myPID.SetOutputLimits(12, 100);  
      }
      myPID.SetTunings(SoftKp, SoftKi, SoftKd);
      myPID.SetMode(AUTOMATIC);
      myPID.Compute();
      Pullinterval = Output;
      Pull();
      Distr();
      Spool();
    } else if (selectedMode == 1) {
      myPID.SetOutputLimits(6, 120);
            if ( Setpoint >= 2.4){
       myPID.SetOutputLimits(12, 100);  
      }
      myPID.SetTunings(MediumKp, MediumKi, MediumKd);
      //      if( gap < 0.1){ myPID.SetTunings(SoftKp, SoftKi, SoftKd);}
      myPID.Compute();
      myPID.SetMode(AUTOMATIC);
      Pullinterval = Output;
      Pull();
      Distr();
      Spool();
    } else if (selectedMode == 2 ) {
      myPID.SetOutputLimits(6, 120);
           if ( Setpoint >= 2.4){
       myPID.SetOutputLimits(12, 100);  
      }
      myPID.SetTunings(HardKp, HardKi, HardKd);
            if ( gap < 0.06) {
              myPID.SetTunings(Hard2Kp, Hard2Ki, Hard2Kd);
            }

      myPID.Compute();
      myPID.SetMode(AUTOMATIC);
      lastOutput = Output;
      Pullinterval = Output;
      Pull();
      Distr();
      Spool();
    } else if (selectedMode == 3) {
      myPID.SetMode(MANUAL);
      Pullinterval = pullspd;
      Pull();
      Distr();
      Spool();
    }
  }
}
// PID //

// STEPPERS //
// Puller
void Pull() {
  unsigned long currentMillis = millis();
  //Stepper 1 - Puller
  digitalWrite(PullpinDir, HIGH);
  if (currentMillis - PullpreviousMillis >= Pullinterval) {
    // save the last time you blinked the LED
    PullpreviousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa:
    if (PullpinStepState == LOW)
      PullpinStepState = HIGH , r++;
    else
      PullpinStepState = LOW;
    // set the LED with the ledState of the variable:
    digitalWrite(PullpinStep, PullpinStepState);
  }
  if (r == 400) {
    r = 0;
    R++;
  }

}
// Spooler
void Spool() {
  unsigned long currentMillis = millis();
  //Stepper 3 - Spool
  digitalWrite(SpoolpinDir, LOW);
  if (currentMillis - SpoolpreviousMillis >= Spoolinterval) {
    // save the last time you blinked the LED
    SpoolpreviousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa:
    if (SpoolpinStepState == LOW)
      SpoolpinStepState = HIGH;
    else
      SpoolpinStepState = LOW;
    // set the LED with the ledState of the variable:
    digitalWrite(SpoolpinStep, SpoolpinStepState);
  }
}
//Distribution
void Distr() {
  unsigned long currentMillis = millis();
  //Stepper 2 - Distribution
  if (currentMillis - DistrpreviousMillis >= Distrinterval) {
    // save the last time you blinked the LED
    DistrpreviousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa:
    if (DistrpinStepState == LOW) {
      DistrpinStepState = HIGH;
      y++;
    }  else
      DistrpinStepState = LOW;
    // set the LED with the ledState of the variable:
    digitalWrite(DistrpinStep, DistrpinStepState);
  }
  //Stepper 2 - Distribution Direction

  //travel = map (DistributionSteps, 0, 80, 0 , 3900);

  if ( y >= 0 && y <= DistributionSteps / 2) {
    DistrpinDirState = HIGH;
  } else if (  y >= DistributionSteps / 2 && y <= DistributionSteps - 1) {
    DistrpinDirState = LOW;
  }
  if (y >= DistributionSteps ) {
    y = 0;
  }

  digitalWrite(DistrpinDir, DistrpinDirState);
}

void resetDistr () {
  //definiamo la direzione del motore
  digitalWrite(DistrpinDir, LOW);

  //esegue un giro completo in un senso
  for (int x = 0; x < numStepMotore; x++) {
    digitalWrite(DistrpinStep, HIGH);
    delay(1);
    digitalWrite(DistrpinStep, LOW);
    delay(1);
  }
  //  digitalWrite(DistrpinDir, HIGH);
  //  for (int x = 0; x < numStepMotore; x++) {
  //    digitalWrite(DistrpinStep, HIGH);
  //    delay(1);
  //    digitalWrite(DistrpinStep, LOW);
  //    delay(1);
  //  }
}

// STEPPERS //

// FANS //
void fans() {
  if (fanspd <= 0) {
    fanspd = 0;
  }
  if (fanspd >= 255) {
    fanspd = 255;
  }
  analogWrite ( pwmPin, fanspd);
}
