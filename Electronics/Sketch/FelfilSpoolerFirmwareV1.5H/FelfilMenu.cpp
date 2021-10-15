#include "FelfilMenu.h"

FelfilMenu::FelfilMenu(){
  LcdpreviousMillis = 0;
  Lcdinterval = 1000;
  LcdpreviousMillis2 = 0;
  Lcdinterval2 = 100;

  down = false;
}

void FelfilMenu::setPullinterval(float value){
  this->Pullinterval = value;
}

int FelfilMenu::getDiameter(){
  return this->diameter;
}

void FelfilMenu::setIntdiameter(float value){
  this->intDiameter = value;
}

void FelfilMenu::setStepperPosition(float value){
  this->StepperPosition = value;
}

float FelfilMenu::getStepperPosition(){
  return this->StepperPosition;
}

int FelfilMenu::getOffset(){
  return this->offset;
}

void FelfilMenu::setEnableStates(int enableState, int preenableState){
  this->enableState = enableState;
  this->preenableState = preenableState;
}

void FelfilMenu::setEnablePinConst(int value){
  this->enablePin = value;
}

void FelfilMenu::setExtspd(float vet[]){
  this->extspd[0] = vet[0];
  this->extspd[1] = vet[1];
}

void FelfilMenu::setIntOffset(float value){
  this->intOffset = value;
}

float FelfilMenu::getIntOffset(){
  return this->intOffset;
}

void FelfilMenu::setNewpositionEnd(int value){
  this->newpositionEnd = value;
}

void FelfilMenu::updateAfterMenuUpdate(FelfilController &controller){
  controller.diameter = this->diameter;
  controller.newposition = this->newposition;
  controller.menuItem = this->menuItem;
  controller.measure = this->measure;
  controller.selectedMode = this->selectedMode;
  controller.Pullinterval = this->Pullinterval;
  controller.spoolspd = this->spoolspd;
  controller.travel = this->travel;
  controller.travelspd = this->travelspd;
  controller.offset = this->offset;
}

void FelfilMenu::updateAfterVarUpdate(FelfilController &controller){
  this->extspd[0] = controller.extspd[0];
  this->extspd[1] = controller.extspd[1];
  this->intDiameter = controller.intDiameter;
  this->intOffset = controller.intOffset;
  this->offset = controller.offset;
  this->newpositionEnd = controller.newpositionEnd;
  this->spoolRPM = controller.spoolRPM;
  this->spoolspd = controller.spoolspd;
  this->Total = controller.Total;
  this->travelspd = controller.travelspd;
}

void FelfilMenu::updateDiamEEPROM() {
  EEPROM.update(adressDiam, highByte(diameter));
  EEPROM.update(adressDiam+1, lowByte(diameter));
}

void FelfilMenu::updateOffsetEEPROM() {
  EEPROM.update(adressOffset , offset);
}

void FelfilMenu::setupEEPROM() {
  diameter =((EEPROM.read(adressDiam) * 256) + EEPROM.read(adressDiam+1));
  intDiameter = diameter*0.01;
  offset = EEPROM.read(adressOffset);
  intOffset = offset*0.01;
}

void FelfilMenu::setupDiameterAndOffsetFromEEPROM(FelfilController &controller){
  this->intDiameter = this->diameter*0.01;

  controller.offset = this->offset;
  controller.intOffset = this->intOffset;
  
  controller.diameter = this->diameter;
  controller.intDiameter = this->intDiameter;
}

int FelfilMenu::getSelectedMode(){
  return this->selectedMode;
}

void FelfilMenu::setMiddle(boolean value){
  this->middle = value;
}

void FelfilMenu::setMenuItem(int value){
  this->menuItem = value;
}

int FelfilMenu::getMenuItem(){
  return this->menuItem;
}

void FelfilMenu::setPage(int value){
  this->page = value;
}

int FelfilMenu::getPage(){
  return this->page;
}

float FelfilMenu::getMeasure(){
  return this->measure;
}

void FelfilMenu::setSpoolRPM(float value){
  this->spoolRPM = value;
}

// MENU //
void FelfilMenu::drawHome(LiquidCrystal_I2C &lcd) {
  //MenuHome
  if(enableState == 0)
  {
    lcd.setCursor(0, 0);
    lcd.print(" Spooler ready! ");
  } else {
    lcd.setCursor(0 , 0);
    lcd.print(char(0));
    unsigned long currentMillis = millis();
    if(currentMillis- LcdpreviousMillis >= Lcdinterval)
    {
      // save the last time you blinked the LED
      LcdpreviousMillis = currentMillis;
      lcd.setCursor(1, 0);
      lcd.print abs(measure);
      lcd.setCursor(9, 0);
      if(selectedMode != 3)
      {
        lcd.print(extspd[0], 2);
      } else lcd.print(extspd[1], 2);

      lcd.setCursor(13, 0);
      lcd.print(" ");
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
    lcd.setCursor(13, 0);
    lcd.print(" ");
    lcd.setCursor(14, 0);
    lcd.print(char(3));
    lcd.setCursor(15, 0);
    lcd.print(char(4));
  }
}

void FelfilMenu::blinkDistr(int end_position){
  for(int i = 0; i < end_position ; i++) //end_position
  {
    digitalWrite(DistrpinStep, HIGH);
    delay(1);
    digitalWrite(DistrpinStep, LOW);
    delay(1);
  }
}

void FelfilMenu::moveDownSelectedMode(){
  down = false;
  if(selectedMode < MAX_SELECTED_MODE)
    selectedMode++;
}

void FelfilMenu::moveUpSelectedMode(){ 
  up = false;
  if(selectedMode > MIN_SELECTED_MODE)
    selectedMode--;
}

void FelfilMenu::moveDownDiameter(){
  down = false;
  this->diameter++;
}

void FelfilMenu::moveUpDiameter(){
  up = false;
  this->diameter--;
}

void FelfilMenu::moveUpStepperPosition() {
  up = false;
  digitalWrite(DistrpinDir, LOW);
  if(StepperPosition > 0)
    StepperPosition -= StepsToTake;
  else
    StepperPosition = 0;
  
  if(StepperPosition != lastStepperPosition)
    blinkDistr(StepsToTake);
}

void FelfilMenu::moveDownStepperPosition(int position) {
    down = false;
    digitalWrite(DistrpinDir, HIGH);
    if(StepperPosition < position)
      StepperPosition = StepperPosition+ StepsToTake;
    else
      StepperPosition = position;

    if(StepperPosition != lastStepperPosition)
      blinkDistr(StepsToTake);
}

void FelfilMenu::moveUpOffset(){
  up = false;
  offset--;
}

void FelfilMenu::moveDownOffset(){
  down = false;
  offset++;
}

void FelfilMenu::moveUpFanspd(){
    up = false;
    fanspd--;
}

void FelfilMenu::moveDownFanspd(){
    down = false;
    fanspd++;
}

void FelfilMenu::moveUpSpoolspd(){
    up = false;
    spoolspd++;
}

void FelfilMenu::moveDownSpoolspd(){
    down = false;
    spoolspd--;
}

void FelfilMenu::moveUpTravelspd(){
    up = false;
    travelspd--;
}

void FelfilMenu::moveDownTravelspd(){
    down = false;
    travelspd++;
}

void FelfilMenu::drawMenu(LiquidCrystal_I2C &lcd) {
  // MENU UI Start
  if(menuItem == 2 && page == 1)
    up = false;
  if(up && page == 1)
  {
    up = false;
    lastMenuItem = menuItem;
    menuItem--;
    if(menuItem == 0)
      menuItem = 1;
  }
  if(down && page == 1) //We have turned the Rotary Encoder Clockwise
  {
    down = false;
    lastMenuItem = menuItem;
    menuItem++;
    if(menuItem == 10 && selectedMode == 3)
      menuItem --;
    else if(menuItem == 9 && selectedMode != 3)
      menuItem --;
  }

  // SETUP MENU Begin____________________________________________________________________
  if(page == 1 && menuItem == 1) {
    page = 2;
  }
  if(middle && menuItem == 1) //Middle Button is Pressed- SETUP MENU
  {
    middle = false;

    if(page >= 1 && page <= 3)
      page++;
    else if(page == 4)
    {
      TravelBegin = StepperPosition;
      lcd.setCursor(0, 0);
      lcd.print("Wait while I'm");
      lcd.setCursor(0, 1);
      lcd.print("moving stepper");
      digitalWrite(DistrpinDir, HIGH);

      blinkDistr(newpositionEnd);

      StepperPosition = newpositionEnd;
      lcd.clear();
      page = 5;
    }
    else if(page == 5)
    {
      travel = StepperPosition;
      if(selectedMode < 3)
      {
        menuItem = 2;
        page = 1;
      }else(page = 6);
    }
    else if(page == 6)
    {
      menuItem = 2;
      page = 1;
    }
  }
  // SETUP MENU End_______________________________________________________________________

  if(middle && menuItem >= 2) //Middle Button is Pressed- USING MENU
  {
    middle = false;
    if(page == 1)
      page = 2;
    else if(page == 2)
      page = 1;
  }
  // MENU UI END
  // Menu using structure begin
  if(page == 1 && menuItem >= 2 && selectedMode == 3)
  {
    int value;

    drawHome(lcd);

    if(menuItem == 2)
    {
      displayMenuItem(lcd, menuState[menuItem - 1], 1, true, intDiameter);
    }
    else if(menuItem == 3)
    {
      displayIntStringMenuPage(lcd, menuState[menuItem - 1], 1, true, mode[selectedMode]);
    }
    else if(menuItem == 4)
    {
      lcd.setCursor(0, 1);
      lcd.print(">  PullSpd:");
      lcd.setCursor(12, 1);
      lcd.print(extspd[1], 2);
    }
    else if(menuItem == 5)
    {
      displayMenuItem(lcd, menuState[menuItem - 1], 1, true, intOffset);
    }
    else if(menuItem == 6)
    {
      lcd.setCursor(0, 1);
      lcd.print(">  TravSpd: ");
      lcd.setCursor(12, 1);

      if( travelspd > 0 && travelspd < 10 )
      {
        lcd.print("   ");
        lcd.setCursor(15, 1);
        lcd.print(travelspd);
      }
      else if(travelspd > 9 && travelspd < 100)
      {
        lcd.print("  ");
        lcd.setCursor(14, 1);
        lcd.print(travelspd);
      }
      else if(travelspd > 99 && travelspd < 999)
      {
        lcd.print(" ");
        lcd.setCursor(13, 1);
        lcd.print(travelspd);
      }
      else if(travelspd == 0)
        lcd.print("Auto");
    }
    else if(menuItem == 7)
    {
      displayMenuItem(lcd, menuState[menuItem - 1], 1, true, spoolRPM);
    }
    else if(menuItem == 8)
    {
      displayMenuItem(lcd, menuState[menuItem - 1], 1, true, fanspd);
    }
    else if(menuItem == 9)
    {
      displayMenuItem(lcd, menuState[menuItem - 1], 1, true, Total);
    }
  }
  else if(page == 1 && menuItem >= 2 && selectedMode != 3)
  {
    drawHome(lcd);

    if(menuItem == 2)
    {
      displayMenuItem(lcd, menuState[menuItem - 1], 1, true, intDiameter);
    }
    else if(menuItem == 3)
    {
      displayIntStringMenuPage(lcd, menuState[menuItem - 1], 1, true, mode[selectedMode]);
    }
    else if(menuItem == 4)
    {
      displayMenuItem(lcd, menuState[menuItem], 1, true, intOffset);
    }
    else if(menuItem == 5)
    {
      lcd.setCursor(0, 1);
      lcd.print(">  TravSpd: ");
      lcd.setCursor(12, 1);

      if( travelspd > 0 && travelspd < 10 )
      {
        lcd.print("   ");
        lcd.setCursor(15, 1);
        lcd.print(travelspd);
      }
      else if(travelspd > 9 && travelspd < 100)
      {
        lcd.print("  ");
        lcd.setCursor(14, 1);
        lcd.print(travelspd);
      }
      else if(travelspd > 99 && travelspd < 999)
      {
        lcd.print(" ");
        lcd.setCursor(13, 1);
        lcd.print(travelspd);
      }
      else if(travelspd == 0)
        lcd.print("Auto");
    }
    else if(menuItem == 6)
    {
      displayMenuItem(lcd, menuState[menuItem], 1, true, spoolRPM);
    }
    else if(menuItem == 7)
    {
      displayMenuItem(lcd, menuState[menuItem], 1, true, fanspd);
    }
    else if(menuItem == 8)
    {
      displayMenuItem(lcd, menuState[menuItem], 1, true, Total);
    }
  }
  // Menu using structure end
  if(selectedMode == 3)
  {
    //Manual Mode Setting Begin___________________________________________________________
    if(page == 2 && menuItem == 1)
    {
      displayStringMenuPage(lcd, mode[selectedMode]);
      if(up)
        moveUpSelectedMode();
      else if(down)
        moveDownSelectedMode();
    }
    if(page == 3 && menuItem == 1)
    {
      lcd.setCursor(0, 0);
      lcd.print("Set Diameter:");
      lcd.setCursor(0, 1);
      lcd.print(intDiameter, 2);

      if(diameter < 1)
      {
        diameter = 175;
        updateDiamEEPROM();
      }
      
      if(up)
        moveUpDiameter();
      else if(down)
        moveDownDiameter();

      updateDiamEEPROM();
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(page == 4 && menuItem == 1)
    {
      lcd.setCursor(0, 0);
      lcd.print("Set Spool Begin:");
      lcd.setCursor(0, 1);
      lcd.print(mm);

      if(mm <= 9) {
        lcd.setCursor(1, 1);
        lcd.print(" ");
      }

      if(up)
        moveUpStepperPosition();
      else if(down)
        moveDownStepperPosition(7900);

      newposition = StepperPosition;
    }
    //////////////////////////////////////////////////////////
    if(page == 5 && menuItem == 1)
    {
      lcd.setCursor(0, 0);
      lcd.print("Set Spool End:");
      lcd.setCursor(0, 1);
      lcd.print(mm);

      if(mm <= 9) {
        lcd.setCursor(1, 1);
        lcd.print(" ");
      }

      if(up)
        moveUpStepperPosition();
      else if(down)
        moveDownStepperPosition(newpositionEnd);
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(page == 6 && menuItem == 1)
    {
      lcd.setCursor(0, 0);
      lcd.print("Set PullSpeed:");
      lcd.setCursor(0, 1);
      lcd.print(extspd[1], 2);

      if(Pullinterval == 0)
        Pullinterval = 9000;

      if(up)
      {
        up = false;
        if(Pullinterval < 90000)
          Pullinterval = Pullinterval+100;
        else
          Pullinterval = Pullinterval;
      }
      else if(down)
      {
        down = false;
        if(Pullinterval > 1000)
          Pullinterval = Pullinterval-100;
        else
          Pullinterval = Pullinterval;
      }
    }
    //Manual Mode Setting End_______________________________________________________________

    if(page == 2 && menuItem == 2)
    {
      drawHome(lcd);
      displayIntMenuPage(lcd, menuState[menuItem - 1], 1, intDiameter);
      
      if(up)
        moveUpDiameter();
      else if(down)
        moveDownDiameter();

      updateDiamEEPROM();
    }
    else if(page == 2 && menuItem == 3)
    {
      drawHome(lcd);
      displayString2MenuPage(lcd, mode[selectedMode]);
      if(up)
        moveUpSelectedMode();
      else if(down)
        moveDownSelectedMode();
    }
    else if(page == 2 && menuItem == 4)
    {
      drawHome(lcd);

      lcd.setCursor(0, 1);
      lcd.print("SetPullspd");
 //     lcd.setCursor(3, 1);
 //     lcd.print("Pullspd");
      lcd.setCursor(11, 1);
      lcd.print("  ");
      lcd.setCursor(12, 1);
      lcd.print(extspd[1], 2);

      if(Pullinterval == 0)
        Pullinterval = 9000;

      if(up)
      {
        up = false;
        if(Pullinterval < 90000)
          Pullinterval = Pullinterval+ 100;
        else
          Pullinterval = Pullinterval;
      }
      else if(down)
      {
        down = false;
        if(Pullinterval > 1000)
          Pullinterval = Pullinterval- 100;
        else
          Pullinterval = Pullinterval;
      }
    }
    else if(page == 2 && menuItem == 5)
    {
      drawHome(lcd);

      lcd.setCursor(0, 1);
      lcd.print("Set");
      lcd.setCursor(3, 1);
      lcd.print("Offset");
      lcd.setCursor(11, 1);

      if(intOffset >= 0)
      {
        lcd.print("  ");
        lcd.setCursor(12, 1);
      }
      lcd.print(intOffset, 2);

      if(up)
        moveUpOffset();
      else if(down)
        moveDownOffset();
  
      updateOffsetEEPROM();
    }
    else if(page == 2 && menuItem == 6)
    {
      drawHome(lcd);
      lcd.setCursor(0, 1);
      lcd.print("Set TravSpd ");
      lcd.setCursor(12, 1);
      if( travelspd > 0 && travelspd < 10 )
      {
        lcd.print("   ");
        lcd.setCursor(15, 1);
      }
      else if(travelspd > 9 && travelspd < 100)
      {
        lcd.print("  ");
        lcd.setCursor(14, 1);
      }
      else if(travelspd > 99 && travelspd < 999)
      {
        lcd.print(" ");
        lcd.setCursor(13, 1);
      }
      else if(travelspd == 0)
      {
        lcd.print("Auto");
      }

      lcd.print(travelspd);

      if(up)
        moveUpTravelspd();
      else if(down)
        moveDownTravelspd();
    }
    else if(page == 2 && menuItem == 7)
    {
      drawHome(lcd);
      displayIntMenuPage(lcd, menuState[menuItem - 1], 1, spoolRPM);

      if(up)
        moveUpSpoolspd();
      else if(down)
        moveDownSpoolspd();
    }
    else if(page == 2 && menuItem == 8)
    {
      drawHome(lcd);
      displayIntMenuPage(lcd, menuState[menuItem - 1], 1, fanspd);
      if(up) 
        moveUpFanspd();
      else if(down)
        moveDownFanspd();
    }
    else if(page == 2 && menuItem == 9)
    {
      drawHome(lcd);
      lcd.setCursor(0, 1);
      lcd.print("Total meter:");
      lcd.setCursor(13, 1);
      lcd.print(Total);
    }
  }
  else if(selectedMode != 3) {
    //Preset Mode Setting Begin_____________________________________________________________
    if(page == 2 && menuItem == 1)
    {
      displayStringMenuPage(lcd, mode[selectedMode]);
      if(up)
        moveUpSelectedMode();
      else if(down)
        moveDownSelectedMode();
    }
    if(page == 3 && menuItem == 1)
    {
      lcd.setCursor(0, 0);
      lcd.print("Set Diameter:");
      lcd.setCursor(0, 1);
      lcd.print(intDiameter, 2);

      if(diameter < 1)
      {
        diameter = 175;
        updateDiamEEPROM();
      }
      
      if(up)
        moveUpDiameter();
      else if(down)
        moveDownDiameter();

      updateDiamEEPROM();
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(page == 4 && menuItem == 1)
    {
      lcd.setCursor(0, 0);
      lcd.print("Set Spool Begin:");
      if(mm <= 9)
      {
        lcd.setCursor(0, 1);
        lcd.print(mm);
        lcd.setCursor(1, 1);
        lcd.print(" ");
      }
      else if(mm >= 10)
      {
        lcd.setCursor(0, 1);
        lcd.print(mm);
      }
     
      if(up)
        moveUpStepperPosition();
      else if(down)
        moveDownStepperPosition(7900);
      
      newposition = StepperPosition;
    }
    //////////////////////////////////////////////////////////////////////////////////////
    if(page == 5 && menuItem == 1)
    {
      lcd.setCursor(0, 0);
      lcd.print("Set Spool End:");
      if(mm <= 9) {
        lcd.setCursor(0, 1);
        lcd.print(mm);
        lcd.setCursor(1, 1);
        lcd.print(" ");
      }
      else if(mm >= 10)
      {
        lcd.setCursor(0, 1);
        lcd.print(mm);
      }

      if(up)
        moveUpStepperPosition();
      else if(down)
        moveDownStepperPosition(newpositionEnd);
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(page == 6 && menuItem == 1)
    {
      lcd.setCursor(0, 0);
      lcd.print("Set PullSpeed:");
      lcd.setCursor(0, 1);
      lcd.print(extspd[1]);
    }
    //Preset Mode Setting End________________________________________________________________

    if(page == 2 && menuItem == 2)
    {
      drawHome(lcd);
      lcd.setCursor(0, 1);
      lcd.print("Set");
      lcd.setCursor(3, 1);
      lcd.print("Diameter");
      lcd.setCursor(11, 1);
      lcd.print("  ");
      lcd.setCursor(12, 1);
      lcd.print(intDiameter, 2);
      if(up)
        moveUpDiameter();
      else if(down)
        moveDownDiameter();

      updateDiamEEPROM();
    }
    else if(page == 2 && menuItem == 3)
    {
      drawHome(lcd);
      displayString2MenuPage(lcd, mode[selectedMode]);
   
      if(up)
        moveUpSelectedMode();
      else if(down)
        moveDownSelectedMode();
    }
    else if(page == 2 && menuItem == 4)
    {
      drawHome(lcd);

      lcd.setCursor(0, 1);
      lcd.print("Set");
      lcd.setCursor(3, 1);
      lcd.print("Offset");
      lcd.setCursor(11, 1);

      if(intOffset >= 0)
      {
        lcd.print("  ");
        lcd.setCursor(12, 1);
      }
      lcd.print(intOffset, 2);

      if(up)
        moveUpOffset();
      else if(down)
        moveDownOffset();
        
      EEPROM.update(adressOffset, offset);
    }
    else if(page == 2 && menuItem == 5)
    {
      drawHome(lcd);

      lcd.setCursor(0, 1);
      lcd.print("Set TravSpd ");
      lcd.setCursor(12, 1);

      if( travelspd >= 0 && travelspd < 10 )
      {
        lcd.print("   ");
        lcd.setCursor(15, 1);
      }
      else if(travelspd > 9 && travelspd < 100)
      {
        lcd.print("  ");
        lcd.setCursor(14, 1);
      }
      else if(travelspd > 99 && travelspd < 999)
      {
        lcd.print(" ");
        lcd.setCursor(13, 1);
      }

      lcd.print(travelspd);

      if(up)
        moveUpTravelspd();
      else if(down)
        moveDownTravelspd();
    }
    else if(page == 2 && menuItem == 6)
    {
      drawHome(lcd);
      displayIntMenuPage(lcd, menuState[menuItem], 1, spoolRPM);
      
      if(up)
        moveUpSpoolspd();
      else if(down)
        moveDownSpoolspd();
    }
    else if(page == 2 && menuItem == 7)
    {
      drawHome(lcd);
      displayIntMenuPage(lcd, menuState[menuItem], 1, fanspd);
  
      if(up)
        moveUpFanspd();
      else if(down)
        moveDownFanspd();
    }
    else if(page == 2 && menuItem == 8)
    {
      drawHome(lcd);
      // STATS
      lcd.setCursor(0, 1);
      lcd.print("Total meter:");
      lcd.setCursor(13, 1);
      lcd.print(Total);
    }
  }

  if(menuItem > 1)
    Sensor();
}

void FelfilMenu::updateMenu(LiquidCrystal_I2C &lcd) {
  unsigned long currentMillis = millis();
  if(currentMillis - LcdpreviousMillis2 >= Lcdinterval2) {
    // save the last time you blinked the LED
    LcdpreviousMillis2 = currentMillis;
    drawMenu(lcd);
  }
}

void FelfilMenu::displayIntMenuPage(LiquidCrystal_I2C &lcd, String menuItem, int position, int value) {

  lcd.setCursor(0, position);
  lcd.print("Set");
  lcd.setCursor(3, position);
  lcd.print(menuItem);
  lcd.setCursor(12, position);
  
  if(value == 0 && menuItem != 2)
  {
    lcd.print("   ");
    lcd.setCursor(15, position);
  }
  else if( value > 0 && value < 10 && menuItem != 2)
  {
    lcd.print("   ");
    lcd.setCursor(15, position);
  }
  else if(value > 9 && value < 100 && menuItem != 2)
  {
    lcd.print("  ");
    lcd.setCursor(14, position);
  }
  else if(value > 99 && value < 999 && menuItem != 2)
  {
    lcd.print(" ");
    lcd.setCursor(13, position);
  }
  lcd.print(value);
}

void FelfilMenu::displayStringMenuPage(LiquidCrystal_I2C &lcd, String value) {
  lcd.setCursor(0, 0);
  lcd.print("Set Mode:");
  lcd.setCursor(0, 1);
  lcd.print(">");
  lcd.setCursor(1, 1);

  if(selectedMode == 0)
  {
    lcd.print("    ");
    lcd.setCursor(5, 1);
  }
  else if(selectedMode == 1)
  {
    lcd.print(" ");
    lcd.setCursor(3, 1);
  }
  else if(selectedMode == 2)
  {
    lcd.print("    ");
    lcd.setCursor(5, 1);
  }
  else if(selectedMode == 3)
  {
    lcd.print(" ");
    lcd.setCursor(3, 1);
  }
  
  lcd.print(value);
}

void FelfilMenu::displayString2MenuPage(LiquidCrystal_I2C &lcd, String value) {
  lcd.setCursor(0, 1);
  lcd.print("Set");
  lcd.setCursor(3, 1);
  lcd.print(" Mode");
  lcd.setCursor(8, 1);

  if(selectedMode == 0)
  {
    lcd.print("    ");
    lcd.setCursor(12, 1);
  }
  else if(selectedMode == 1)
  {
    lcd.print(" ");
    lcd.setCursor(10, 1);
  }
  else if(selectedMode == 2)
  {
    lcd.print("    ");
    lcd.setCursor(12, 1);
  }
  else if(selectedMode == 3)
  {
    lcd.print(" ");
    lcd.setCursor(10, 1);
  }
  
  lcd.print(value);
}

void FelfilMenu::displayIntStringMenuPage(LiquidCrystal_I2C &lcd, String item, int position, boolean selected, String value) {
  lcd.setCursor(0, position);
  lcd.print(">");
  lcd.setCursor(1, position);
  lcd.print(item);
  lcd.setCursor(11, position);
  lcd.print(" ");
  lcd.setCursor(12, position);
  
  lcd.print(value);
}

void FelfilMenu::displayMenuItem(LiquidCrystal_I2C &lcd, String item, int position, boolean selected, int value) {
  lcd.setCursor(0, position);
  lcd.print(">");
  lcd.setCursor(1, position);
  lcd.print(item);
  lcd.setCursor(11, position);
  
  if(value == 0 && menuItem != 2 && menuItem != 4)
  {
    lcd.print("     ");
    lcd.setCursor(15, position);
  }
  else if( value > 0 && value < 10 && menuItem != 2 && menuItem != 4)
  {
    lcd.print("     ");
    lcd.setCursor(15, position);
  }
  else if(value > 9 && value < 100 && menuItem != 2 && menuItem != 4)
  {
    lcd.print("    ");
    lcd.setCursor(14, position);
  }
  else if(value > 99 && value < 999 && menuItem != 2 && menuItem != 4)
  {
    lcd.print("   ");
    lcd.setCursor(13, position);
  }
  else if(value > 999 && value < 9999 && menuItem != 2 && menuItem != 4)
  {
    lcd.print("  ");
    lcd.setCursor(12, position);
  }
  lcd.print(value);

  if(menuItem == 2)
  {
    lcd.setCursor(11, position);
    lcd.print("  ");
    lcd.setCursor(12, position);
    lcd.print(intDiameter, 2);
  }

  if(menuItem == 4)
  {
    lcd.setCursor(11, position);
    
    if(intOffset >= 0)
    {
      lcd.print("+");
      lcd.setCursor(12, position);
    }
    lcd.print(intOffset, 2);
  }
}
// MENU  //

//  ENCODER //
void FelfilMenu::readRotaryEncoder(ClickEncoder &encoder) {
  value+= encoder.getValue();
  if(value > last) {
    last = value ;
    down = true;
  } else   if(value < last) {
    last = value;
    up = true;
  }
  lastStepperPosition = StepperPosition;
  mm = StepperPosition / StepsToTake;
}

// END ENCODER //

// FANS //
void FelfilMenu::fans() {
  if(fanspd <= 0) {
    fanspd = 0;
  }
  if(fanspd >= 255) {
    fanspd = 255;
  }
  analogWrite(pwmPin, fanspd);
}
// END FANS //

// SENSOR //
void FelfilMenu::setSensorValue(float value)
{
  this->sensorValue = value;
}

void FelfilMenu::setSensorMin(float value)
{
  this->sensorMin = value;
}

void FelfilMenu::Sensor() {
  sensorValue = analogRead(sensIn);
//  Serial.println(sensorValue);
  sensbuf += (sensorValue-sensbuf) - sensorMin; //smoothing
  measure = abs(lookup(sensbuf, lut3)) + intOffset;
//  Serial.print("measure: ");Serial.println(measure);
}

float FelfilMenu::lookup(float inval, float lut[][2]) {
  float out;
  byte i;

  for(i = 1; i < NUMTEMPS; i++)
    if(lut[i][0] > inval)
      return lut[i-1][1]+(inval-lut[i-1][0])*(lut[i][1]-lut[i-1][1])/(lut[i][0]-lut[i-1][0]);
}

void FelfilMenu::calibrateSensor()
{
  pinMode(sensIn, INPUT);
  // calibrate during the first second
  while(millis() < 1000)
  {
    sensorValue = analogRead(sensIn);
    // record the minimum sensor value
    if(sensorValue > sensorMin)
      sensorMin = sensorValue;
  }
}

// END SENSOR //
