#include "FelfilController.h"

FelfilController::FelfilController()
{

}

float FelfilController::getIntOffset()
{
  return intOffset;
}

float FelfilController::getPullinterval()
{
  return this->Pullinterval;
}

void FelfilController::setEnableStates(int enableState, int preenableState)
{
  this->enableState = enableState;
  this->preenableState = preenableState;
}

void FelfilController::setEnablePinConst(int value)
{
  this->enablePin = value;
}

void FelfilController::setR(int value)
{
  this->R = value;
}

int FelfilController::getR()
{
  return this->R;
}

float* FelfilController::getExtspd()
{
  return this->extspd;
}

float FelfilController::getIntdiameter()
{
  return this->intDiameter;
}

int FelfilController::getNewpositionEnd()
{
  return this->newpositionEnd;
}

void FelfilController::setMenuItem(int value)
{
  this->menuItem = value;
}

float FelfilController::getSpoolRPM()
{
  return this->spoolRPM;
}

void FelfilController::setOffset(int value)
{
  this->offset = value;
}

void FelfilController::setTunings(s_parametri K)
{
  this->pid->SetTunings(K.kp, K.ki, K.kd);
}

// Variables
void FelfilController::Var() {
  if(extspd[0] < 0) 
    extspd[0] = 0;

  intDiameter = diameter * 0.01;
  if(spoolspd <= 2) 
    spoolspd = 2;
  else if(spoolspd >= 30) 
    spoolspd = 30;

  Spoolinterval = spoolspd;
  spoolRPM = 300 / spoolspd;

  Distrinterval = 160 / travelspd;

  if(travelspd < 0) 
    travelspd = 0;
  if(travelspd > 160) 
    travelspd = 160;
  
  intOffset = offset * 0.01;
  if(offset <-25) 
    offset =-25;
  if(offset > 25)
    offset = 25;

  newpositionEnd = 7900-newposition;
  
  if(selectedMode == 3)
    if(Pullinterval == 0)
      Pullinterval = 9000;
      
  extspd[0] = 60/((Pullinterval*400)/1000)*0.062*1000;  // 0.062 = diametro puller | 400 = numero passi per un giro completo
  extspd[1] = extspd[0] *1000;
  DistributionSteps = 2*travel ;
  Total = R*0.194;
}

void FelfilController::initializePID()
{
  softK.kp = 6.9;
  softK.ki = 0.23;
  softK.kd = 5.175;
  
  mediumK.kp = 10.8;
  mediumK.ki = 0.45;
  mediumK.kd = 6.48;

  hardK.kp = 15.48;
  hardK.ki = 0.62;
  hardK.kd = 9.675;
  
  pid = new PID(&Input, &Output, &Setpoint, softK.kp, softK.ki, softK.kd, DIRECT);
  pid->SetMode(AUTOMATIC);

  if(Pullinterval == 0)
    Pullinterval = 9000;
}

void FelfilController::turnPinsOff()
{
  //Stepper 1- Puller
  pinMode(PullpinStep, OUTPUT);
  pinMode(PullpinDir, OUTPUT);
  //Stepper 2- Distribution
  pinMode(DistrpinStep, OUTPUT);
  pinMode(DistrpinDir, OUTPUT);
  //Stepper 3- Spool
  pinMode(SpoolpinStep, OUTPUT);
  pinMode(SpoolpinDir, OUTPUT);
  //Stepper enable
  pinMode(enablePin, OUTPUT);
}

// PID //
void FelfilController::Brain() {
  //initialize the variables we're linked to
  Setpoint = diameter * 0.01;
  //turn the PID on
  float lastOutput;
  //Strarting speed pot
  //PID
  Input = measure ;
  
  double gap = abs(Setpoint-Input);
  if(menuItem != 1)
  {
    if(selectedMode == 3) 
    {
      pid->SetMode(MANUAL);
      //    Pullinterval = extspd;
      
//      if(Pullinterval == 0)
//        Pullinterval = 9000;
        
      ManualPull();
    }
    else
    {        
      if(Setpoint < 2.4)
        pid->SetOutputLimits(6, 120);
      else
        pid->SetOutputLimits(12, 100);
      
      if(selectedMode == 0 || (selectedMode == 2 && gap < 0.06)) 
        setTunings(softK);
      else if(selectedMode == 1) 
        setTunings(mediumK);
      else if(selectedMode == 2)
        setTunings(hardK);

      pid->SetMode(AUTOMATIC);
      pid->Compute();
      Pullinterval = Output*1000;
      Pull();
    }
    Distr();
    Spool();
  }
}

// STEPPERS //
// Puller
void FelfilController::Pull() {
  unsigned long currentMillis = micros();
  //Stepper 1- Puller
  digitalWrite(PullpinDir, HIGH);
  if(currentMillis- PullpreviousMicros >= Pullinterval) {
    // save the last time you blinked the LED
    PullpreviousMicros = currentMillis;
    // if the LED is off turn it on and vice-versa:
    if(PullpinStepState == LOW)
      PullpinStepState = HIGH , r++;
    else
      PullpinStepState = LOW;
    // set the LED with the ledState of the variable:
    digitalWrite(PullpinStep, PullpinStepState);
  }
  if(r == 400) {
    r = 0;
    R++;
  }
}
void FelfilController::ManualPull(){
  unsigned long currentMicros = micros();
  //Stepper 1- Puller
  digitalWrite(PullpinDir, HIGH);
  if(currentMicros - PullpreviousMicros >= Pullinterval) {
    // save the last time you blinked the LED
    PullpreviousMicros = currentMicros;
    // if the LED is off turn it on and vice-versa:
    if(PullpinStepState == LOW)
      PullpinStepState = HIGH , r++;
    else
      PullpinStepState = LOW;
    // set the LED with the ledState of the variable:
    digitalWrite(PullpinStep, PullpinStepState);
  }
  if(r == 400) {
    r = 0;
    R++;
  }
}
// Spooler
void FelfilController::Spool() {
  unsigned long currentMillis = millis();
  //Stepper 3 - Spool
  digitalWrite(SpoolpinDir, LOW);
  if(currentMillis-SpoolpreviousMillis >= Spoolinterval) {
    // save the last time you blinked the LED
    SpoolpreviousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa:
    if(SpoolpinStepState == LOW)
      SpoolpinStepState = HIGH;
    else
      SpoolpinStepState = LOW;
    // set the LED with the ledState of the variable:
    digitalWrite(SpoolpinStep, SpoolpinStepState);
  }
}

//Distribution
void FelfilController::Distr() {
  unsigned long currentMillis = millis();
  //Stepper 2- Distribution
  if(currentMillis-DistrpreviousMillis >= Distrinterval) {
    // save the last time you blinked the LED
    DistrpreviousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa:
    if(DistrpinStepState == LOW) {
      DistrpinStepState = HIGH;
      y++;
    }  else
      DistrpinStepState = LOW;
    // set the LED with the ledState of the variable:
    digitalWrite(DistrpinStep, DistrpinStepState);
  }
  //Stepper 2- Distribution Direction
  if(y >= 0 && y <= DistributionSteps/2) {
    DistrpinDirState = LOW;
  } else if( y >= DistributionSteps/2 && y <= DistributionSteps-1) {
    DistrpinDirState = HIGH;
  }
  if(y >= DistributionSteps) {
    y = 0;
  }
  digitalWrite(DistrpinDir, DistrpinDirState);
}
void FelfilController::resetDistr() {
  //definiamo la direzione del motore

  digitalWrite(DistrpinDir, LOW);

  //esegue un giro completo in un senso
  for(int x = 0; x < numStepMotore; x++) { //x < numStepMotore
    digitalWrite(enablePin, LOW);
    digitalWrite(DistrpinStep, HIGH);
    delay(1);
    digitalWrite(DistrpinStep, LOW);
    delay(1);
  }
  digitalWrite(DistrpinDir, HIGH);
}
