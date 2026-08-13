// V1.2
// August 12 2026
// Code refactoring by Onze

#include <FlashAsEEPROM_SAMD.h>
#include <avr/pgmspace.h>
#include "antilog.h"
#include <Arduino.h>


// all the constants are now set here as defines
#define WAVES_NUMBER 5
#define SQUARE 1
#define TRIANGLE 2
#define RAMP_DOWN 3
#define RAMP_UP 4
#define RANDOM 5 

// used for filtering the pot and cv values
#define FREQ 0
#define POT 1

// button handling stuff
#define BUTTON_STATE_UP 0
#define BUTTON_DEBOUNCE_CHECK 1
#define BUTTON_STATE_PRESSED 2
#define BUTTON_STATE_LONG_PRESS 3

// times are in milisecs 
#define BUTTON_PRESS_DEBOUNCE_THRESH 80
#define BUTTON_PRESS_MID_TIME 1000
#define BUTTON_PRESS_LONG_TIME 3000

// sync handling stuff
#define SYNC_THESH_VALUE 370
#define SYNC_START_STATE 0
#define SYNC_FIRST_RISING_EDGE 1
#define SYNC_FIRST_FALLING_EDGE 2
#define SYNC_NEXT_RISING_EDGE 3

// how it's working generally
#define CLOCK_MODE_SELF 0
#define CLOCK_MODE_SYNC 1

// switching between different modes
#define INTERFACE_MODES_NUMBER 2
#define INTERFACE_MODE_CLASSIC 0
#define INTERFACE_MODE_MANUAL 1

// phasor value for 1 hz given the 48 MHz PWM value / 9 bits
// i'm unsure about how this was calculated to be honest
#define HZPHASOR 91183


long unsigned int accumulator1 = 0;
long unsigned int accumulator2 = 0;
long unsigned int accumulator3 = 0;
long unsigned int accumulator4 = 0;
long unsigned int phasor1;
long unsigned int phasor2;
long unsigned int phasor3;
long unsigned int phasor4;

char randNum[4];

FlashStorage(div_storage, int);
FlashStorage(wave_storage, int);
FlashStorage(init_storage, char);

////////////////////////////////////////////////////////////////////////////////////
//       DIVIDE DOWN ARRAYS                                                       //
//       Add more if you want!                                                    //
////////////////////////////////////////////////////////////////////////////////////

// classic mode definitions

#define DIVSIZE 3 // if you add more divide down arrays, increase this number from 3 (the default number of arrays) to how many arrays you have total  
char divs[DIVSIZE][4]={ {1,3,7,11},
                        {1,2,4,8},
                        {1,4,8,16} }; // You could add more divide down arrays here 

char waveSelects[4] = { SQUARE, SQUARE, SQUARE, SQUARE };


////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

//char buttonState = 0;
//unsigned long int buttonPressTime = 0;
//int waveSelect = 1;
int divSelect = 0;
unsigned long lastSettingsSave = 0;

bool ClockMode = CLOCK_MODE_SELF; // 0 = POT and 1 = SYNC 
float sweepValue;
long unsigned int Time1 = 0;
long unsigned int Time2 = 0; 
long unsigned int Periud = 0; // Period (arduino didn't allow use of word "period")
float syncFrequency = 0; 

void timerIsr();
void setupTimers();
void TCC0_Handler();



// +++++++++++++++++++++++++++++++++++ SETUP ++++++++++++++++++++++++++++++++++++++++
void setup()
{
  pinMode(1, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(13, OUTPUT); // using pin 13 to check interupt on timer 1
  pinMode(6, INPUT_PULLUP); // pin 7 pushbutton to select waveform
  pinMode(A10, INPUT); //used for analogReading sync (cv2) 
  readSettings();
  //Serial.begin(9600);
  setupTimers(); //  **this may not be the right location
  randomSeed(analogRead(A8));
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++ MAIN LOOP +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void loop()
{
  static char interfaceMode = INTERFACE_MODE_CLASSIC;
  static char selectedWave = SQUARE;
  
  static int syncPopOutCounter = 0;
  //bool lastButtonState = 0;
  //static bool buttonState = BUTTON_STATE_UP;

  static char buttonState = 0;
  static unsigned long int buttonPressTime = 0;

  static int syncState = SYNC_START_STATE; // used to find find the leading edge to calculate period (static so it isn't updated to zero each loop)
  static int syncCounter = 0;
  int SyncRead; 

////////////////////////////////////////////////////////////////////////////////////
//       "EEPROM" Save                                                            //
//       Store the settings after 60 seconds                                      //
////////////////////////////////////////////////////////////////////////////////////
  if ((millis() - (lastSettingsSave)) > (60000))
  {
    saveSettings();
    lastSettingsSave = millis();
  }
  
////////////////////////////////////////////////////////////////////////////////////
//        Sync mode                                                               //
//       sync the 1:1 output with an incoming signal                              //
////////////////////////////////////////////////////////////////////////////////////
  SyncRead = analogRead(A10); // CV2 pin A10 on XIAO 

  if ((SyncRead < SYNC_THESH_VALUE) && (syncState == SYNC_START_STATE))
  {
    //if analog read CV2 is less than 370 : RISING EDGE
    Time1 = micros(); // Capture the time 
    syncState = SYNC_FIRST_RISING_EDGE;  // set syncState to 1 
    //This only enters on the first pulse received,
    //following pulses alternate between syncstate 2 and 3
  }

  else if ((SyncRead > SYNC_THESH_VALUE) && (syncState == SYNC_FIRST_RISING_EDGE))
  {
    //FALLING
    syncState = SYNC_FIRST_FALLING_EDGE; 
  }

  else if ((SyncRead < SYNC_THESH_VALUE) && (syncState == SYNC_FIRST_FALLING_EDGE))
  {
    //NEXT RISING EDGE - compute the frequency
    syncState = SYNC_NEXT_RISING_EDGE;
    Time2 = micros();
    ClockMode = CLOCK_MODE_SYNC;
    Periud = (Time2 - Time1); // Find the period by subtracting Time2 from Time1 
    // convert the period into a frequency
    syncFrequency = 1000000 / Periud; // NEW FREQUENCY HERE 
    
    /*
    // this next section is probably useless because I believe that it's computed in the next part.
    // (i believe that in C, N%1==0 every time)
    // for now, I'm leaving it there, but commenting out
    if(divs[divSelect-1][0] == 1)
    {
      //if the first divider is 1, clear acc 1 every rising edge to keep synced
      accumulator1=0;
    }
    if(divs[divSelect-1][1] == 1)
    {
      //if the second divider is 1, clear acc 2
      accumulator2=0;
    }
    if(divs[divSelect-1][2] == 1)
    {
      //if the third divider is 1, clear acc 3
      accumulator3=0;
    }
    if(divs[divSelect-1][3] == 1)
    {
      //if the fourth divider is 1, clear acc 4
      accumulator4=0;
    }*/
    
    // frequency has been calculated, prepare for the next pulse
    Time1 = Time2;
    
    
    if (syncCounter == 0)  //first time calculation, clear all accs so waveform starts at 0 and begins synced
    {
      accumulator1=0;
      accumulator2=0;  
      accumulator3=0;
      accumulator4=0;
    }

    // i have the impression that this accumulator reset is necessary only because in the phasor calculation,
    // we don't have the guarantee that phasor values are multiples
    if (syncCounter % divs[divSelect][0] == 0)
      // div 1
      accumulator1 = 0;

    if (syncCounter % divs[divSelect][1] == 0)
      // div 2
      accumulator2 = 0;

    if (syncCounter % divs[divSelect][2] == 0)
      // div 3
      accumulator3 = 0;

    if (syncCounter % divs[divSelect][3] == 0)
      // div 4
      accumulator4 = 0;
  

    //if((syncCounter%divs[divSelect-1][0] == 0) && (syncCounter%divs[divSelect-1][1] == 0) && (syncCounter%divs[divSelect-1][2] == 0) && (syncCounter%divs[divSelect-1][3] == 0)){
    if ( (accumulator1==0) && (accumulator2==0) && (accumulator3==0) && (accumulator4==0) )
      // avoid accumulating values into syncCounter and getting an overflow (I suppose)
      syncCounter = 0;

    syncCounter++;

  }
  
  else if ((SyncRead > SYNC_THESH_VALUE) && (syncState == SYNC_NEXT_RISING_EDGE))
  {
    // already in syncing mode, prepare for the next pulse
    syncState = SYNC_FIRST_FALLING_EDGE; 
  }

  ////////////////////////////////////////////////////////////
  //                   END OF SYNC STUFF                    //
  ////////////////////////////////////////////////////////////




  ////////////////////////////////////////////////////////////
  //                 BUTTON HANDLING STUFF                  //
  ////////////////////////////////////////////////////////////

  if (buttonState==BUTTON_STATE_UP && digitalRead(6)==LOW )
  {
    //button has just been pressed and hasn't been previously pressed                           
    buttonState = BUTTON_DEBOUNCE_CHECK;   //real button press?
    buttonPressTime = millis();
  }

  else if (buttonState == BUTTON_DEBOUNCE_CHECK )
  {
    if ((millis() - buttonPressTime) > BUTTON_PRESS_DEBOUNCE_THRESH)
    {
      if (digitalRead(6) == LOW)
        buttonState = BUTTON_STATE_PRESSED;
      else
        buttonState = BUTTON_STATE_UP;
    }
  }

  else if (buttonState == BUTTON_STATE_PRESSED)
  {
    if (digitalRead(6) == HIGH)
    {
      // button has been released
      buttonState = BUTTON_STATE_UP;

      // checking the time since the last press
      if ((millis() - buttonPressTime) < BUTTON_PRESS_MID_TIME)
      {
        //short press
        if (interfaceMode == INTERFACE_MODE_CLASSIC)
        {
          // if in classic mode: change all the waves altogether to match the first one
          char newWaveSelect = (waveSelects[0] % WAVES_NUMBER) + 1;  // wave shapes are numbered starting from 1, the typical (var + 1) % number_vals works when starting with 0

          // apply to all the waves
          waveSelects[0] = newWaveSelect;
          waveSelects[1] = newWaveSelect;
          waveSelects[2] = newWaveSelect;
          waveSelects[3] = newWaveSelect;
        }
        else if (interfaceMode == INTERFACE_MODE_MANUAL)
        {
          // only change the selected wave 
          waveSelects[selectedWave] = (waveSelects[selectedWave] % WAVES_NUMBER) + 1;
        }
        //short press to change waveform
        //char waveSelect = (waveSelects[0] % WAVES_NUMBER) + 1;  // wave shapes are numbered starting from 1, the typical (var + 1) % number_vals works when starting with 0
                                                       // the logical calculation would be waveSelec
           //waveSelect++;  //move to the next waveform
           //   if(waveSelect>4){ //if we're trying to select more than 4 waveforms, cycle back to the 1st
           //      waveSelect=1;
           //   }
      }
      else if ((millis() - buttonPressTime) <= BUTTON_PRESS_LONG_TIME)  //medium time press
      {
        // medium time press and release to change divisions
        if (interfaceMode == INTERFACE_MODE_CLASSIC)
        {
          accumulator1=0; 
          accumulator2=0; 
          accumulator3=0; 
          accumulator4=0;
          phasor1 = 0;
          phasor2 = 0;
          phasor3 = 0;
          phasor4 = 0;
          
          // pausing to show that something has happened
          delay(800);
          
          divSelect = (divSelect + 1) % DIVSIZE;
        }
        else if (interfaceMode == INTERFACE_MODE_MANUAL)
        {
          // just change the selected wave
          selectedWave = (selectedWave + 1) % 4;

          // now what we need to do is add the manual mode to the pot handling to change the divisions or stuff like that
        }

      }
      else if ((millis() - buttonPressTime) > BUTTON_PRESS_LONG_TIME)
      {
        // long time press to change interface mode
        buttonState = BUTTON_STATE_LONG_PRESS;
        buttonPressTime = millis();

        // change the interface mode
        interfaceMode = (interfaceMode + 1) % INTERFACE_MODES_NUMBER;

        if (interfaceMode == INTERFACE_MODE_MANUAL)
          // we just switched to the manual mode, selectedWave has to be the first one
          selectedWave = 0;
        
        //divSelect++;
        //if (divSelect>DIVSIZE) {
        //  divSelect=1;
        ///}

      }
    }
  }
  else if (buttonState == BUTTON_STATE_LONG_PRESS)
  {
    if ((millis() - buttonPressTime) > BUTTON_PRESS_LONG_TIME)
    {
      // long press, cycling through
      // divSelect = (divSelect + 1) % DIVSIZE;
      interfaceMode = (interfaceMode + 1) % INTERFACE_MODES_NUMBER;

      // reinitializing the time for the next cycle
      buttonPressTime = millis();
    }
    else if (digitalRead(6) == HIGH)
    {
      buttonState = BUTTON_STATE_UP;
    }
  }

  ////////////////////////////////////////////////////////////
  //              END OF BUTTON HANDLING STUFF              //
  ////////////////////////////////////////////////////////////
  
  



  ////////////////////////////////////////////////////////////
  //           READ THE POT AND FREQ CV-IN VALUES           //
  ////////////////////////////////////////////////////////////
   
  float tempphasor;
  int cv1Value; // to store value of cv1 (Frequency)
  static int potValue; // to store the value of the potentiometer
  static int oldpotValue; 

  // read the potentiometer value
  // the value is smoothed with filterPut (accumulates the readings) and filterGet (computes a mean)
  filterPut(POT, analogRead(A0));
  potValue = filterGet(POT);
  

  //This section pops us out of sync mode
  syncPopOutCounter++;
  if (syncPopOutCounter > 100) //do this only every 100 samples
  {
    // actually what this seems to do is verify if the pot value is quickly changed, rather than changed at all
    // this is not necessarily an issue
    if ((potValue - oldpotValue)> 20 || (oldpotValue - potValue) > 20)
    {
      ClockMode = CLOCK_MODE_SELF; 
    }

    oldpotValue = potValue;
    syncPopOutCounter = 0;
  }

  // read the freq cv value
  // the value is smoothed with filterPut (accumulates the readings) and filterGet (computes a mean)
  filterPut(FREQ, analogRead(A4));
  cv1Value = filterGet(FREQ);
  
  // at this stage, a -12V CV corresponds to +3.3V (1023 as an analog read) on the XIAO (Because of the inverting op-amp)
  // we center the value around 0
  
  cv1Value = 1023 - cv1Value; // so we want to invert it (making -12V correspond to 0V on the XIAO)
  cv1Value = cv1Value - 565;  //at this point cv1Value contains between -512 and +511 (this line used to center values around zero)
  





  ////////////////////////////////////////////////////////////
  //              UPDATE THE PHASER INCREMENTS              //
  ////////////////////////////////////////////////////////////

  if (ClockMode == CLOCK_MODE_SYNC)
  {
    //better way to update sweepvalue with sync frequency
    sweepValue = syncFrequency;
  }
  else
  {
    int totalcv = potValue;

    // the device is self-clocked
    if ( (ClockMode == CLOCK_MODE_SELF) && ((cv1Value > 20) || (cv1Value < -20)) )
    {
      // Don't want the frequency cv input to be considered if it is close to 0 (+/- 20)
      // (actually, is it that relevant???)
      totalcv += cv1Value;
      if (totalcv < 0)
        totalcv = 0;
      else if (totalcv > 1023)
        totalcv = 1023;
    }
      
    // read the frequency value in the antilog.h table.
    // hzcurve is a lut table, so +totalcv provides the position in the table
    sweepValue = pgm_read_float_near(hzcurve + totalcv);
  }

  // tempphasor is the fastest increment according to the general clock
  tempphasor = sweepValue * HZPHASOR;

  // update the individual values with their corresponding dividers
  phasor1 = (unsigned long int) (tempphasor / divs[divSelect][0]);
  phasor2 = (unsigned long int) (tempphasor / divs[divSelect][1]); // dividing down for the slower outputs 
  phasor3 = (unsigned long int) (tempphasor / divs[divSelect][2]);
  phasor4 = (unsigned long int) (tempphasor / divs[divSelect][3]);
}






// +++++++++++++++++++++++++++++++++++++++++ FUNCTION DEFINITIONS ++++++++++++++++++++++++++++++++++

// previous_acc is used to store the last known acc value
// it's useful to know when we start a new cycle, for the random steps generation
unsigned long int previous_acc[4];

unsigned int generator(unsigned long int acc, char waveshape, char channel)
{
  //December 12 - Adjusted for 9 bit
  //unsigned char shifted_acc = acc>>24;
  
  // unsigned long int is using 32 bits
  // so we keep the 9 outmost bits and base our calculation on those
  
  unsigned int shifted_acc = acc>>23;
  #define HALFPOINT 255
  #define MAXPOINT 511

  switch(waveshape)
  {
    case SQUARE:
      if (shifted_acc > HALFPOINT)
      {
        return MAXPOINT;
      }
      else
      {
        return 0;
      }

    case RAMP_DOWN:
      return shifted_acc;

    case RAMP_UP:
      return MAXPOINT - shifted_acc;

    case TRIANGLE:
      if (shifted_acc <= HALFPOINT)
      {
        return (shifted_acc<<1);
      }
      else //greater than HALFPOINT
      {
        return ((MAXPOINT - shifted_acc)<<1);      
      }
    
    case RANDOM:
      if (shifted_acc < previous_acc[channel])
      // Do this so we only update when we cycle around the accumulator
      {
        randNum[channel] = random(MAXPOINT);      
      }

      previous_acc[channel] = shifted_acc;  
      return randNum[channel];

    default:
      return 0;
  }
  
  return 0;
}



void setupTimers() // used to set up fast PWM on pins 1,9,2,3
{
 
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(2) |          //// Divide the 48MHz clock source by divisor N=1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            //// Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          //// Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  //enable our 4 pins to be PWM outputs
  PORT->Group[g_APinDescription[1].ulPort].PINCFG[g_APinDescription[1].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[9].ulPort].PINCFG[g_APinDescription[9].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[3].ulPort].PINCFG[g_APinDescription[3].ulPin].bit.PMUXEN = 1;

  //assign the 4 outputs to the PWM registers on PMUX
  PORT->Group[g_APinDescription[1].ulPort].PMUX[g_APinDescription[1].ulPin >> 1].reg = PORT_PMUX_PMUXE_E;  // D3 is on PA11 = odd      
  PORT->Group[g_APinDescription[9].ulPort].PMUX[g_APinDescription[9].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E; // D11 is on PA08 = even 
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;  // D3 is on PA11 = odd
  PORT->Group[g_APinDescription[3].ulPort].PMUX[g_APinDescription[3].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F; // D11 is on PA08 = even

 

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK4 //0 only works for interrup?
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization


  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |           // Reverse the output polarity on all TCC0 outputs
                    TCC_WAVE_WAVEGEN_DSBOTH |
                    TCC_WAVE_WAVEGEN_NFRQ;     // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation: Freq = 48Mhz/(2*N*PER)
  REG_TCC0_PER = 0x1FF;                           // Set the FreqTcc and period of the PWM on TCC1
  while (TCC0->SYNCBUSY.bit.PER);                // Wait for synchronization
 
  REG_TCC0_CC1 = 10;                             // TCC1 CC1 - on D3  50% pin 9
  while (TCC0->SYNCBUSY.bit.CC1);                   // Wait for synchronization
  REG_TCC0_CC0 = 50;                             // TCC1 CC0 - on D11 50% pin 1
  while (TCC0->SYNCBUSY.bit.CC0);                   // Wait for synchronization
  REG_TCC0_CC2 = 200;                             // TCC1 CC1 - on D3  50% pin 2
  while (TCC0->SYNCBUSY.bit.CC2);                   // Wait for synchronization
  REG_TCC0_CC3 = 254;                             // TCC1 CC0 - on D11 50% pin 3
  while (TCC0->SYNCBUSY.bit.CC3);                   // Wait for synchronization
 
  // Divide the GCLOCK signal by 1 giving  in this case 48MHz (20.83ns) TCC1 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1 ****************************************************************************
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
  
  TCC0->INTENSET.reg = 0;
  TCC0->INTENSET.bit.CNT = 1;  //*****************************************************
  TCC0->INTENSET.bit.MC0 = 0;

  NVIC_EnableIRQ(TCC0_IRQn);
  TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE ;
}


void TCC0_Handler() 
{
  if (TCC0->INTFLAG.bit.CNT == 1)
  {
    //*************************************************
    accumulator1 = accumulator1 + phasor1;
    accumulator2 = accumulator2 + phasor2;
    accumulator3 = accumulator3 + phasor3;
    accumulator4 = accumulator4 + phasor4;
    delayMicroseconds(6);

    REG_TCC0_CC0 = generator(accumulator1, waveSelects[0], 0); // pin 9 //#4
    REG_TCC0_CC1 = generator(accumulator4, waveSelects[1], 3); // pin 2 //#1
    REG_TCC0_CC2 = generator(accumulator2, waveSelects[2], 1); // pin 1 //#2  
    REG_TCC0_CC3 = generator(accumulator3, waveSelects[3], 2); // pin 3 //#3

    TCC0->INTFLAG.bit.CNT = 1;
    //*******************************************************
  }
}


void readSettings (void)
{
  char x;
  char c = 'S';
  int y = 1;
  int z = SQUARE;

  init_storage.read(x);

  if (x == 'S')  //S means eeprom has been initialized.
  {
    wave_storage.read(z);
    waveSelects[0] = z;
    waveSelects[1] = z;
    waveSelects[2] = z;
    waveSelects[3] = z;
    //wave_storage.read(waveSelects[1]);
    //wave_storage.read(waveSelects[2]);
    //wave_storage.read(waveSelects[3]);
    div_storage.read(divSelect);
  }
  else
  {
    //we initialize, no 'S' found
    init_storage.write(c);  //use variables because this library hates constants
    wave_storage.write(z);
    div_storage.write(y);
    divSelect = 0;
    //waveSelect = 1;
    waveSelects[0] = z;
    waveSelects[1] = z;
    waveSelects[2] = z;
    waveSelects[3] = z;
  }
}


void saveSettings (void)
{
  int x;

  // only store the value if it's different from the previous stored value
  wave_storage.read(x);
  if (x != waveSelects[0])
  {
    x = waveSelects[0];
    wave_storage.write(x);  // <-- save the waveSelect 
  }

  div_storage.read(x);
  if (x != divSelect)
    div_storage.write(divSelect);  // <-- save the divSelect  
}


#define NUMREADINGS 50
unsigned int pot[NUMREADINGS];
unsigned int freq[NUMREADINGS];

void filterPut (char input, unsigned int newreading)
{
  static unsigned char potptr = 0;
  static unsigned char freqptr = 0;

  if (input == POT)
  {
    pot[potptr] = newreading;
    potptr++;
    if (potptr >= NUMREADINGS)
      potptr=0;
  }

  else if (input == FREQ)
  {
    freq[freqptr] = newreading;
    freqptr++;
    if (freqptr >= NUMREADINGS)
      freqptr = 0;
  }
  
}

unsigned int filterGet (bool input)
{
  unsigned long int x;
  float z;
  unsigned char y;

  x = 0;
  if (input == POT)
  {
    for (y=0;y<NUMREADINGS;y++)
    {
      x = x + pot[y];
    }
  }
  else if (input == FREQ)
  {
    for (y=0;y<NUMREADINGS;y++)
    {
      x = x + freq[y];
    }
  }

  z = x;
  z = z / NUMREADINGS;
  z = z + 0.5;
  return (unsigned int)z;
}
