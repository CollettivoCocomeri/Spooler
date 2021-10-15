#ifndef _FELFILCONTROLLER_h
#define _FELFILCONTROLLER_h

#include <ClickEncoder.h>
#include <TimerOne.h>
#include <PID_v1.h>
#include <WString.h>
#include <EEPROM.h>
    
typedef struct {
  double kp;
  double ki;
  double kd;
} s_parametri;
    
class FelfilController
{
  private:
    //PID
    PID* pid;
    double Input, Output, Setpoint;

    //Define Tuning Parameters
    s_parametri softK;
    s_parametri mediumK;
    s_parametri hardK;

    float lastOutput = 0;

    //Stepper 1- Puller- Pin Definition
    const int PullpinDir = 2;
    const int PullpinStep = 5;
    int PullpinStepState = HIGH;
    
    //Stepper 1- Puller-Time
    unsigned long Pulltime;
    unsigned long PullpreviousMicros = 0;
    
    //Stepper 1- round counter
    int r = 0;
    int R = 0;
    
    //Stepper 2- Distribution- Pin Definitio
    const int DistrpinDir = 3;
    const int DistrpinStep = 6;
    int DistrpinStepState = HIGH;
    int DistrpinDirState = LOW;
    int DistributionSteps = 0;
    
    //Stepper 2- Distribution- Speed Time
    unsigned long Distrtime;
    unsigned long DistrpreviousMillis = 0;
    float Distrinterval = 0;
    
    //Stepper 2- Distribution- Direction Time
    unsigned long DirDistrtime;
    unsigned long DirDistrpreviousMillis = 0;
    float DirDistrinterval = 50000;
    
    // Stepper 2- Distribution- Radious counter
    float rad = 0;
   
    // Stepper 2 reset
    int numStepMotore = 7800;
    int steppini = 0;
    
    //Stepper 3- Spool- Pin Definition
    const int SpoolpinDir = 4;
    const int SpoolpinStep = 7;
    int SpoolpinStepState = HIGH;
    
    //Stepper 3- Spool- Time
    unsigned long Spooltime;
    unsigned long SpoolpreviousMillis = 0;
    float Spoolinterval = 0;
    
    //Enable
    int enablePin;
    int enableState;
    int preenableState;
    
    //fan
    int y = 0;

  public:
    FelfilController();

    //Millis
    float travel = 0;
    float travelspd = 4;
    const float pullspd = 12;
    int spoolspd = 12;
    float spoolRPM = 0 ;
    int kilograms = 0;
    float extspd[2] = {0, 0};
    int offset = 0;
    float intOffset = 0;

    float* getExtspd();
    float getSpoolRPM();
    void setOffset(int value);
    
    //fan
    int newpositionEnd = 0;
    int newposition = 0;

    int getNewpositionEnd();

    float Total = 0;

    //enable
    void setEnableStates(int enableState, int preenableState);
    void setEnablePinConst(int value);
    
    //sensor
    float measure = 0;
    int diameter;
    float intDiameter;

    float getIntdiameter();

    void setTunings(s_parametri K);
    
    //menu
    int menuItem;
    int selectedMode;

    void setMenuItem(int value);
    
    float getIntOffset();
    
    float Pullinterval = 0;
    
    void initializePID();

    void Brain();
    
    void Var();
    
    void turnPinsOff();
    
    void Pull();
    void ManualPull();

    void Spool();

    void Distr();
    void resetDistr();
    
    float getPullinterval();
        
    void setR(int value);
    int getR();
};
#endif
