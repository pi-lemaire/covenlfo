// V1.2
// August 12 2026
// Code refactoring by Onze

#include <FlashAsEEPROM_SAMD.h>
#include <avr/pgmspace.h>
//#include "antilog.h"
#include "coven_luts.h"
#include "coven_defs.h"
#include <Arduino.h>
#include <array>



// phasor value for 1 hz given the 48 MHz PWM value / 9 bits
constexpr uint32_t HZPHASOR = 91625;
// original value : #define HZPHASOR 91183

// the calculation is the following:
// baseclock is 48 Mhz
// TCC0 interrupts happen every 2x512 cycle of this master-clock
// so the frequency of TCC0 is 48000000/1024 = 46875 Hz
// we want phasor to reach the long unsigned int overflow after one increment every TCC0 interrupt
// long unsigned int max is 2^32-1 = 4294967295. It starts from 0 so it's actually 4294967296 values
// so HZPhasor is 4294967296 / 46875 = 91625.97
// we round to the lower value because it's probably better to have the clock lagging
// back slightly behind the sync pulse when in sync



// first 3 presets are identical to the original CovenLFO code (int dividers, base tempo identical to sync, triangle waves)
// last one displays the added features:
// for each LFO, we can set
//   - the divider (can be a float), relative to the master clock (see below)
//   - a flag (true|false) idicating whether we want the wave to be in sync with the master clock (see below). default=sync
//     setting this flag to true means that the lfo divider sync is automatically truncated to an int (setting 0.7 value may result in s)
//   - the wave used for this particular output (default = Triangle)
//   - the master clock multiplier (end of the line, after the last } character) (default = 1)
// to handle better syncs, we have added a master clock, which is calculated relative to the sync signal
// this means that setting the master clock to x4 results in the main tempo being quarter notes relative to the sync signal
// setting a high values also pushes the lfos further into audio territory


CovenSettings defaultPresets[] = {
  CovenSettings({{ {1}, {4}, {12}, {20} }}, 4),
  CovenSettings({{ {1}, {4}, {8}, {16} }}, 4),
  CovenSettings({{ {1}, {4}, {16}, {64} }}, 4),
  CovenSettings({{ {1, true, CovenWaves::Square}, {3, true, CovenWaves::RampDown}, {12.5f, false, CovenWaves::Random}, {19.2f, false, CovenWaves::Triangle} }}, 4.0f)
};

CovenPresets<sizeof(defaultPresets) / sizeof(defaultPresets[0])> Presets(defaultPresets);      // N est déduit automatiquement (=3)






// Class to handle and store the accumulators and phasers,
// that MUST be updated with the best possible accuracy
// This mechanism now also handles the sync
// data is shared between the main loop and the ISR
// the main loop sets the settings, the ISR doesn't need to access the other elements anymore

class AccsAndPhasors {
  public:
    uint8_t increment()
    {
      uint8_t ret = 0x00;

      bool resetAcc0 = false;

      uint32_t prevAcc0 = acc0;
      acc0 += phasor0;
      if (acc0<prevAcc0)
      {
        // we just overflowed, the divided clock is reset
        resetAcc0 = true;
        ++count0;
      }

      
      for (uint8_t i=0; i<4; ++i)
      {
        uint32_t prevAcc = accumulator[i];
        accumulator[i] += phasor[i];

        if (doReset[i] && resetAcc0)
        {
          // the master clock has been reset
          ++count[i];
        }
        if (accumulator[i]<prevAcc)
          // this also handles the case of unsync oscillators
          ret |= 0x01 << i;
      }
      
      return ret;
    }

    uint32_t getAccValue(uint8_t ind, bool mainAcc=false) const
    {
      uint32_t ret;
      if (mainAcc)
        ret = acc0;
      else
        ret = accumulator[ind];
      return ret;
    }

    void setPhasor(uint8_t ind, uint32_t value, bool mainAcc=false)
    {
      //noInterrupts();
      if (mainAcc)
      {
        phasor0 = value;
        //interrupts();
        return;
      }
        
      // set the value of the corresponding phaser
      phasor[ind] = value;
      //interrupts();
    }

    void setResetInfo(uint8_t ind, uint16_t div, bool do_reset)
    {
      // noInterrupts();
      resetEvery[ind] = div;
      doReset[ind] = do_reset;
      // interrupts();
    }

    void reSync(uint8_t ind, bool mainAcc=false)
    {
      if (mainAcc)
      {
        acc0 = 0;
        count0 = 0;
        return;
      }

      // simply reset the accumulator and the counter
      accumulator[ind] = 0;
      count[ind] = 0;
    }

    void reSyncAll()
    {
      acc0 = 0;
      count0 = 0;
      for (uint8_t i=0; i<4; ++i)
      {
        accumulator[i] = 0;
        count[i] = 0;
      }
    }

  private:
    // i've added count0 to be used as some sort of a master clock resetter.
    // for now, it's not used, but resets based on the lcd may be useful because there's currently
    // no way of resyncing the clocks if they ever desynchronized other than changing the div
    // No waveform is going to be calculated on this one
    volatile uint32_t acc0 = 0;
    volatile uint32_t phasor0 = 91625;
    volatile uint16_t count0 = 0;
    volatile uint32_t accumulator[4] = {0, 0, 0, 0}; // volatile for the accumulators is probably not useful
    volatile uint32_t phasor[4] = {91625, 91625/2, 91625/4, 91625/8};  
    volatile uint16_t count[4] = {0, 0, 0, 0};
    volatile uint16_t resetEvery[4] = {1, 2, 4, 8};
    volatile bool doReset[4] = {true, true, true, true};  // may be replaced by flags
};

AccsAndPhasors accumulators;






// the flash storage handler will need to be updated at some point
FlashStorage(div_storage, int);
FlashStorage(wave_storage, int);
FlashStorage(init_storage, char);


////////////////////////////////////////////////////////////////////////////////////
//       DIVIDE DOWN ARRAYS                                                       //
//       Add more if you want!                                                    //
////////////////////////////////////////////////////////////////////////////////////

// classic mode definitions


/*
constexpr uint8_t DIVSIZE = 3;

char divs[DIVSIZE][4]={ {1,3,7,11},
                        {1,2,4,8},
                        {1,4,8,16} }; // You could add more divide down arrays here 


CovenWaves waveSelects[4] = { CovenWaves::Square, CovenWaves::Square, CovenWaves::Square, CovenWaves::Square };
*/

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

//char buttonState = 0;
//unsigned long int buttonPressTime = 0;
//int waveSelect = 1;
int divSelect = 0;

float sweepValue;
volatile float syncFrequency = 1.; 


// draw 8 random values on 9 bits
CovenDice<8> Dice((0x01<<9)-1);


// not sure those ar really useful
void timerIsr();
void setupTimers();
void TCC0_Handler();
// void TCC1_Handler();



// +++++++++++++++++++++++++++++++++++ SETUP ++++++++++++++++++++++++++++++++++++++++
void setup()
{
  // for debugging purposes - comment out when not necessary
  /*
  Serial.begin(115200);
  while (!Serial) {}
  */
  if (!Presets.load())
    Presets.save();


  // Serial.print("div preset[0][0] BEFORE modification: ");
  // Serial.println(Presets.curr()[0].divider);

  // // Modifie une valeur bien identifiable
  // Presets.curr()[0].divider = 42.0f;
  // Presets.save();

  // Serial.print("div preset[0][0] AFTER save: ");
  // Serial.println(Presets.curr()[0].divider);

  // Serial.println("---> Reset la carte maintenant et regarde si 42.00 persiste <---");
  // if (!Presets.load())
  //   Presets.save();


  //EEPROM.begin();
  pinMode(1, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(13, OUTPUT); // using pin 13 to check interupt on timer 1
  pinMode(6, INPUT_PULLUP); // pin 7 pushbutton to select waveform
  //pinMode(A10, INPUT); //used for analogReading sync (cv2) 
  pinMode(10, INPUT); //used for analogReading sync (cv2) 
  // readSettings();
  randomSeed(analogRead(A8));

  //Serial.begin(9600);
  setupTimers(); //  **this may not be the right location
  //randomSeed(analogRead(A8));
}




////////////////////////////////////////////////////////////
//                 BUTTON HANDLING STUFF                  //
////////////////////////////////////////////////////////////


ButtonState getButtonState(void)
{
  //static char buttonState = BUTTON_STATE_UP;
  static ButtonState knownState = ButtonState::Released;
  static unsigned long int buttonPressTime = 0;

  if ( (knownState==ButtonState::Released) && (digitalRead(6)==LOW) )
  {
    // button has just been pressed and hasn't been previously pressed
    // we activate a flag (sort of) allowing us to state that we're in debounce check mode
    knownState = ButtonState::Debounce_Check;
    buttonPressTime = millis();
    return ButtonState::Undefined;
  }
  else if (knownState == ButtonState::Debounce_Check)
  {
    // we're in check mode, verify if enough time has elapsed
    if ((millis() - buttonPressTime) > ButtonPressTimeDebounce)
    {
      // more time than the debounce check has passed
      // is the button still pressed?
      if (digitalRead(6) == LOW)
        // yes: ok this is not a bounce - next we will look at the time
        // if there's a new click within a short interval, it's a dbl click
        // otherwise it's a classic click, or a medium / long press
        knownState = ButtonState::Dbl_Click_Check_LOW;
      else
        // no: this was not a real press
        knownState = ButtonState::Released;
        return ButtonState::Undefined;
    }
    // no else : we don't want to change the state if nothing was recorded
  }

  // now check the Double Click
  if (knownState==ButtonState::Dbl_Click_Check_LOW)
  {
    if ((millis() - buttonPressTime) > ButtonPressDblClickInterv)
    {
      // we didn't have a release within the expected time, we're in normal press territory
      knownState == ButtonState::Pressed;
    }
    else if (digitalRead(6) == HIGH)
    {
      // release within the dbl click interval time, we're still in possible DblClick Territory but we still need a push
      knownState==ButtonState::Dbl_Click_Check_HIGH;
    }
  }

  if (knownState==ButtonState::Dbl_Click_Check_HIGH)
  {
    if ((millis() - buttonPressTime) > ButtonPressDblClickInterv)
    {
      // we didn't have a new click within the time interval, we're in normal click territory
      knownState == ButtonState::Pressed;
    }
    else if (digitalRead(6) == LOW)
    {
      // new click within the expected time, this is a double click
      knownState = ButtonState::Double_Click;
      
      // record the click time for a debounce check
      buttonPressTime = millis();

      return ButtonState::Double_Click;
    }
  }

  if (knownState==ButtonState::Double_Click)
  {
    // just wait for a new high status to resume to the original state
    if ((digitalRead(6) == HIGH) && ((millis() - buttonPressTime) > ButtonPressTimeDebounce))
    {
      knownState = ButtonState::Released;
      return ButtonState::Undefined;
    }
  }

  if ((knownState == ButtonState::Pressed) || (knownState == ButtonState::Long_Press))
  {
    // 3 options:
    // 1. release before mid time : short press
    // 2. release after mid time but before long time : medium press
    // 3. exceeded long time : long press, even without a release

    unsigned long int currentTime = millis();

    if (digitalRead(6) == HIGH)
    {
      bool previously_not_long = (knownState == ButtonState::Pressed);

      // button has been released, store it into the status straight away
      knownState = ButtonState::Released;

      // checking the time since the last press
      // ShortPress and MediumPress are only going to happen if it wasn't a long press already
      if (previously_not_long && ((currentTime - buttonPressTime) < ButtonPressTimeMid))
        //short press
        return ButtonState::Simple_Click;
      else if (previously_not_long && ((currentTime - buttonPressTime) <= ButtonPressTimeLong))
        // medium press
        return ButtonState::Medium_Press;
      else if ((currentTime - buttonPressTime) > ButtonPressTimeLong)
        // avoid the unlikely event when the actual release happens before the check
        buttonPressTime = currentTime;
        return ButtonState::Long_Press;
    }
    else if ((currentTime - buttonPressTime) > ButtonPressTimeLong)
    {
      // this is a long press. We cycle through, but we also need to change the status
      buttonPressTime = currentTime;
      knownState = ButtonState::Long_Press;
      return ButtonState::Long_Press;
    }
  }

  // return undefined: 
  return ButtonState::Undefined;
}







///////////////////////////////////////////////////////////
//        Sync related stuff                             //
//       sync the 1:1 output with an incoming signal     //
///////////////////////////////////////////////////////////


// what threshold we use to compare to when using analogread()
constexpr uint16_t SYNC_THESH_VALUE = 370;

// number of sync verifications before deciding that it's possible that we leave the sync mode
constexpr uint8_t SYNC_DUMMY_CHECKS = 100;

// max allowed time in microseconds before we stop tracking the pulses and interprete the next
// pulse as a new train of triggers (basically, the minimum frequency allowed)
constexpr uint32_t SYNC_MAX_TIME = 4 * 1000000;

// TODO: implement this



// now the general sync handling is performed through an object
// the advantage of doing this is that it can be called from either the main loop or an IRC
// and that some variables stay hidden. Within the loop, we just call the ReadTrigger function
// and we read the active / inactivate status, and also we know when a pulse just happened
// getting out of sync mode is also handled through the class, by providing the pot value

class SyncHandler {
public:
  bool readTriggerCV()
  {
    // read the sync trigger input, and finds out the sync state mode
    // this method must to be called either into the main loop,
    // either in an ISR for better timing, but cannot be read by both concurrently

    // returns True if it just received a trigger and entered sync mode

    // store the timing between 2 pulses
    static long unsigned int Time1 = micros();

    // read the input
    bool SyncUpState = (analogRead(A10) < SYNC_THESH_VALUE); // CV2 pin A10 on XIAO
    // analogRead is rather slow, it's better to replace it with a digitalRead
    // since it's much faster and thus relevant for synchronization
    // bool SyncUpState = (digitalRead(10) == LOW); // CV2 pin A10 on XIAO
    
    // all the following is to detect state change
    if (SyncUpState && (syncState==SyncState::Unsync))
    {
      // if we've received a gate/trigger, and we haven't had any before
      Time1 = micros(); // Capture the time 
      syncState = SyncState::FirstRisingEdge;

      return false;
    }
    else if (!SyncUpState && (syncState==SyncState::FirstRisingEdge))
    {
      //FALLING
      syncState = SyncState::FirstFallingEdge;

      return false;
    }
    else if (SyncUpState && (syncState==SyncState::FirstFallingEdge))
    {
      if (!active)
      {
        // we've just changed state, record it
        checksCounter = 0;
        syncCounter = 0;
      }

      // we're in sync mode now
      active = true;

      // calculate the period
      uint32_t Time2 = micros();
      period = Time2 - Time1;
      Time1 = Time2;
      // TODO: check what is susceptible of happening when an overflow happens, and find a strategy to counter that
      
      // frequency has been calculated, prepare for the next pulse
      //NEXT RISING EDGE - compute the frequency
      syncState = SyncState::NextRisingEdge;
      
      // we've just received a pulse, return this information
      return true;
    }
    
    else if (!SyncUpState && (syncState == SyncState::NextRisingEdge))
    {
      // already in syncing mode, prepare for the next pulse
      syncState = SyncState::FirstFallingEdge;

      return false;
    }

    // none of the above: no state changed, return false and don't change anything, be yourself, forever
    return false;
  }

  void checkSyncMode(uint16_t potValue)
  {
    if (!active)
    {
      // not supposed to happen
      return;
    }
    else if (checksCounter==0)
    {
      // first check, store the potValue
      knownPotValue = potValue;
      checksCounter++;
    }
    else if (checksCounter<SYNC_DUMMY_CHECKS)
      checksCounter++;
    else
    {
      if ((potValue - knownPotValue)> 20 || (knownPotValue - potValue) > 20)
      {
        active = false;
        syncState = SyncState::Unsync;
      }
    }
  }

  bool isActive() const
  {
    return active;
  }

  uint32_t getPeriod() const
  {
    // avoid corrupted returns since period is a multi-byte value
    // susceptible of being shared and modify between main loop and ISR
    // noInterrupts();
    uint32_t p = period;
    // interrupts();
    return p;
  }

  uint8_t getSyncCounter() const
  {
    return syncCounter;
  }

  void resetSyncCounter()
  {
    syncCounter = 0;
  }

  void incSyncCounter()
  {
    ++syncCounter;
  }

private:
    volatile bool active = false;
    volatile SyncState syncState = SyncState::Unsync;
    volatile uint32_t period = 0;
    volatile uint8_t checksCounter = 0;
    uint16_t knownPotValue = 0; // not volatile because it only needs to be read to and from the main loop
    uint8_t syncCounter = 0;
};

// global object used to handle the sync
// for now it's handled in the main loop, so it may be static into the main loop
// but the main goal is to handle it into the TCC1 loop for better accuracy
SyncHandler sync; 




volatile uint32_t _debug_total_time_spent_in_TCC0 = 0;
volatile uint32_t _debug_max_time_spent_in_TCC0 = 0;
volatile uint32_t _debug_number_of_calls_to_TCC0 = 0;


uint16_t min_pot_value = 0x01<<10;
uint16_t max_pot_value = 0;

uint16_t min_cv_value = 0x01<<10;
uint16_t max_cv_value = 0;




// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++ MAIN LOOP +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void loop()
{
  static int loop_cycles = 0;

  // default values for the interface:
  static CovenAlgo interfaceMode = CovenAlgo::Classic;
  static int selectedWave = 0;

  // set a clock timer to save the settings regularly
  static unsigned long lastSettingsSave = millis();
  static uint32_t time_since_last_debug_output = millis();

  // the template value <5> is used to smoothe the read values on 2^5=32 values
  // if you want more smoothing, use 6 or 7 (the max). Conversely, less is possible.
  // original CovenLFO version was 50 values, so between 5 and 6
  //static CVInHandler<6> PotReader(A0);
  static CVInHandler_DualQuantile PotReader(A0);
  static CVInHandler<5> FreqReader(A4);



  
  // evaluate the sync status
  bool gotAPulse = sync.readTriggerCV();

  if (gotAPulse)
  {
    // we've just got an actual pulse from the sync input
    // we probably have to update the frequency

    // convert the period time (in microseconds) in Hz
    // note: we probably don't have to go into hertz territory here
    syncFrequency = 1000000 / static_cast<float>(sync.getPeriod()); // NEW FREQUENCY HERE, IN HERTZ

    // also reset the phasers when needed
    uint8_t syncCount = sync.getSyncCounter();

    if (syncCount == 0)  //first time sync, clear all accs so waveform starts at 0 and begins synced
    {
      accumulators.reSyncAll();
    }
    // probably perform it rather in the sync itself systematically, but for now it's in the main loop
    sync.incSyncCounter();
  }

  
  ////////////////////////////////////////////////////////
  //       "EEPROM" Save                                //
  //       Store the settings after 60 seconds          //
  ////////////////////////////////////////////////////////
  if ((millis() - (lastSettingsSave)) > (5000))
  {
    Presets.save();
    // saveSettings();
    lastSettingsSave = millis();
  }
  
  


  ////////////////////////////////////////////////////////////
  //                 BUTTON HANDLING STUFF                  //
  ////////////////////////////////////////////////////////////

  // get the type of button action with this function
  ButtonState currentButtonState = getButtonState();

  // act accordingly
  if (currentButtonState == ButtonState::Simple_Click)
  {
    // circulate through the waves
    if (interfaceMode == CovenAlgo::Classic)
    {
      // if in classic mode: change all the waves altogether to match the first one

      // we have to copy the value because the ++ operator is not compatible with volatile values
      CovenWaves currW = Presets.curr()[0].wave;
      ++currW;

      for (uint8_t i=0; i<4; ++i)
      {
        // stupid copy is to handle the volatile thing
        // wonder if it's actually useful to declare this as volatile???
        //CovenWaves currW = Presets.curr()[i].wave;
        Presets.curr()[i].wave = currW;
      }
    }
    else if ((interfaceMode == CovenAlgo::Manual_Div) || (interfaceMode==CovenAlgo::Manual_Float))
    {
      // probably a good idea to add a class method for incrementing a specific wave instead of this
      //CovenWaves currW = Presets.curr()[selectedWave].wave;
      ++Presets.curr()[selectedWave].wave; // = ++currW;
    }
  }
  else if (currentButtonState == ButtonState::Medium_Press)
  {
    // medium time press and release to change divisions
    if (interfaceMode == CovenAlgo::Classic)
    {      
      // pausing to show that something has happened
      delay(400);
      // TODO : replace this delay by some additional led display

      Presets.next();

      // update the divider and sync infos for the accumulators
      noInterrupts();
      // forbidding the interrupts prevents a cycle with divs coming from different settings from happening
      for (uint8_t i=0; i<4; ++i)
      {
        accumulators.setResetInfo(i, static_cast<uint16_t>(Presets.curr()[i].divider * Presets.curr().div), Presets.curr()[i].synced);
      }
      accumulators.reSyncAll();
      interrupts();
        
    }
    else if ((interfaceMode==CovenAlgo::Manual_Div) || (interfaceMode==CovenAlgo::Manual_Float))
    {
      // just change the selected output
      selectedWave = (selectedWave + 1) % 4;

      // now what we need to do is add the manual mode to the pot handling to change the divisions or stuff like that
    }
  }
  else if (currentButtonState == ButtonState::Long_Press)
  {
    // change the interface mode
    ++interfaceMode; // = (interfaceMode + 1) % ALGOS_Number;

    if ((interfaceMode==CovenAlgo::Manual_Div) || (interfaceMode==CovenAlgo::Manual_Float))
      // we just switched to the manual mode, selectedWave has to be the first one
      selectedWave = 0;
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
  // the value is smoothed with PotReader
  potValue = PotReader.readCV(); //filterGet(POT);

  // if (PotReader.isLocked())
  //   Serial.println("pot value locked");


  // DEBUG STUFF
  if (potValue < min_pot_value)
    min_pot_value = potValue;
  if (potValue > max_pot_value)
    max_pot_value = potValue;
  // END OF DEBUG STUFF


  if (sync.isActive())
    sync.checkSyncMode(potValue);



  ////////////////////////////////////////////////////////////
  //              UPDATE THE PHASER INCREMENTS              //
  ////////////////////////////////////////////////////////////

  if (sync.isActive())
  {
    //better way to update sweepvalue with sync frequency
    sweepValue = syncFrequency;
  }
  else
  {
    float potFreq = pgm_read_float_near(hzcurve + potValue);

    

    // int totalcv = potValue;

    // read the freq cv value (useless when in sync mode)
    // the value is smoothed with filterPut (accumulates the readings) and filterGet (computes a mean)
    cv1Value = FreqReader.readCV(); //filterGet(FREQ);

    // DEBUG STUFF
    if (cv1Value < min_cv_value)
      min_cv_value = cv1Value;
    if (cv1Value > max_cv_value)
      max_cv_value = cv1Value;
    // END OF DEBUG STUFF
    
    // at this stage, a -12V CV corresponds to +3.3V (1023 as an analog read) on the XIAO (Because of the inverting op-amp)
    // we center the value around 0
    
    cv1Value = 1023 - cv1Value; // so we want to invert it (making -12V correspond to 0V on the XIAO)
    //cv1Value = cv1Value - 512;  //at this point cv1Value contains between -512 and +511 (this line used to center values around zero)
    //cv1Value = cv1Value - 565;  //at this point cv1Value contains between -512 and +511 (this line used to center values around zero)
    
    // float cv_multiplier = pgm_read_float_near(hzcurve + potValue);

    constexpr uint8_t cv_scaler_bit_depth = 2;
      // 1 bit of scaling is cv can affect main frequency from approx /2 to *2
    
    // cv value is encoded on 10 bits

    // retrieve the floating part on 10 - cv_scaler_bit_depth bits
    uint16_t cv_float_part_idx = cv1Value & ((0x01 << (10 - cv_scaler_bit_depth)) - 1);

    // adjust so that we are effectively using the 9 bits of the LUT
    cv_float_part_idx = cv_float_part_idx << (9 - (10 - cv_scaler_bit_depth));

    // the integer part is this:
    uint16_t cv_int_part = cv1Value >> (10 - cv_scaler_bit_depth);

    // calculation will be 2^cv_int_part * lut(cv_float_part)

    float cv_frac = pgm_read_float_near(pow2curve + cv_float_part_idx);

    float cv_multiplier = ldexpf(cv_frac, (int) cv_int_part - cv_scaler_bit_depth);

    sweepValue = cv_multiplier * potFreq;
    //cv1Value = 0;

    /*
    // the device is self-clocked
    if ( (cv1Value > 20) || (cv1Value < -20) ) 
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
    */
  }

  // testing the value of HZPHASOR, we force the main clock to be 1 Hz
  //sweepValue = 1;

  // tempphasor is the fastest increment according to the general clock
  tempphasor = sweepValue * HZPHASOR * Presets.curr().div;

  // update the base clock
  accumulators.setPhasor(5, (unsigned long int) tempphasor, true);

  // update the individual phasers.
  for (uint8_t i=0; i<4; ++i)
    accumulators.setPhasor(i, (unsigned long int) (tempphasor / Presets.curr()[i].divider));


  // draw a random value
  Dice.draw();


  static bool first_display = true;

  
  /*
  // DEBUGGING DISPLAY STUFF
  if (millis()-1000 > time_since_last_debug_output)
  {
    uint32_t record_time_since_last_debug_output = time_since_last_debug_output;
    time_since_last_debug_output = millis();

    record_time_since_last_debug_output = time_since_last_debug_output - record_time_since_last_debug_output;

    noInterrupts();
    // just gathering the data in copy to break the interrupt cycles as shortly as possible
    uint32_t total_time_spent_in_TCC0_cpy = _debug_total_time_spent_in_TCC0;
    _debug_total_time_spent_in_TCC0 = 0;
    uint32_t max_time_spent_in_TCC0_cpy = _debug_max_time_spent_in_TCC0;
    _debug_max_time_spent_in_TCC0 = 0;
    uint32_t calls_to_TCC0_cpy = _debug_number_of_calls_to_TCC0;
    _debug_number_of_calls_to_TCC0 = 0;
    interrupts();
    */
  if (false)
  {
    if (first_display)
    {
      //Serial.println("interv,loops,TCC0_spent,TCC0_max,TCC0_calls,min_pot,max_pot,min_cv,max_cv");
      Serial.println("time_ms,read,filtered,result,locked");
      first_display = false;
    }

    /*
    Serial.print(PotReader.lastReadingTime());
    Serial.print(",");
    Serial.print(PotReader.lastReadValue());
    Serial.print(",");
    Serial.print(PotReader.lastFilteredValue());
    Serial.print(",");
    Serial.print(potValue);
    Serial.print(",");
    Serial.println(PotReader.isLocked());
    */

    /*
    Serial.print(record_time_since_last_debug_output);
    Serial.print(",");
    Serial.print(loop_cycles);
    Serial.print(",");
    Serial.print(total_time_spent_in_TCC0_cpy);
    Serial.print(",");
    Serial.print(max_time_spent_in_TCC0_cpy);
    Serial.print(",");
    Serial.print(calls_to_TCC0_cpy);
    Serial.print(",");
    Serial.print(min_pot_value);
    Serial.print(",");
    Serial.print(max_pot_value);
    Serial.print(",");
    Serial.print(min_cv_value);
    Serial.print(",");
    Serial.println(max_cv_value);

    min_pot_value = 0x01<<10;
    max_pot_value = 0;

    min_cv_value = 0x01<<10;
    max_cv_value = 0;

    loop_cycles = 0;
    */
  }
  // END OF DEBUGGING DISPLAY STUFF
  

  ++loop_cycles;
}






// +++++++++++++++++++++++++++++++++++++++++ FUNCTION DEFINITIONS ++++++++++++++++++++++++++++++++++

// generate a waveform based on its accumulator value
// acc is the accumulator value
// waveshape is the waveshape
// channel is the channel, between 0 and 3
// and justCycled indicates whether this wave just cycled (acc<prevAcc)
// it's computed externally in the accumulators object

uint16_t generator(uint32_t acc, CovenWaves waveshape, uint8_t channel, bool justCycled)
{
  // to know when we're at the middle of the waveform  
  constexpr uint16_t HALFPOINT = 255;
  constexpr uint16_t MAXPOINT = 511;

  // store the random values for cycles
  static uint16_t randNum[4];
  
  // unsigned long int is using 32 bits
  // so we keep the 9 outmost bits and base our calculation on those
  uint16_t shifted_acc; // = static_cast<uint16_t>(acc>>23);

  switch(waveshape)
  {
    case CovenWaves::Square:
      //if (shifted_acc > HALFPOINT)
      if (acc & 0x01<<31) // the leftmost bit is 1
      {
        return MAXPOINT;
      }
      else
      {
        return 0;
      }

    case CovenWaves::RampDown:
      // return shifted_acc;
      return static_cast<uint16_t>(acc>>23);

    case CovenWaves::RampUp:
      // return MAXPOINT - shifted_acc;
      return MAXPOINT - static_cast<uint16_t>(acc>>23);

    case CovenWaves::Triangle:
      // keep the precision here
      shifted_acc = static_cast<uint16_t>(acc>>22);
      //if (shifted_acc <= HALFPOINT)
      if (shifted_acc <= MAXPOINT)
      {
        //return (shifted_acc<<1);
        return shifted_acc;
      }
      else //greater than HALFPOINT
      {
        return shifted_acc ^ (0x3FF); // XOR with 1023 is equivalent to 1023-shifted_acc
        //return ((MAXPOINT + 1 - shifted_acc) << 1); 
        // +1 because if shifted_acc==MAXPOINT, we shift 0 and end up with an undesired value
      }

    case CovenWaves::Sine:
      shifted_acc = static_cast<uint16_t>(acc>>22);
      return pgm_read_word_near(sin_curve + shifted_acc);
    
    case CovenWaves::Random:
      if (justCycled)
      // Do this so we only update when we cycle around the accumulator
      {
        // this is probably quite wrong regarding the timing needed to compute a random() in an interrupt
        // TODO : change this behaviour by generating random numbers in the main loop and just retrieving the value here
        // randNum may probably be store as static????
        randNum[channel] = Dice.get_value(); //random(MAXPOINT);      
      }

      //previous_acc[channel] = shifted_acc;  
      return randNum[channel];

    default:
      return 0;
  }
  
  return 0;
}


/*
// Variables globales pour le clignotement gere dans TCC1_Handler
volatile bool led_D5_State = false;
volatile uint32_t blink_D5_Counter = 0;

volatile bool led_D8_State = false;
volatile uint32_t blink_D8_Counter = 0;

// Nombre d'overflows de TCC1 pour atteindre 0.5s
// Frequence TCC1 = Fgclk / (2 * N * PER) ~ 94000 Hz (voir calcul plus bas, PER=0xFF donc 8 bits)
// => Pour 0.5s : ~94000 * 0.5 ~ 47000 overflows (a ajuster si vous changez PRESCALER ou PER)
#define BLINK_D5_OVERFLOW_COUNT 47000

// D8 est en simple GPIO on/off, clignotement gere par le meme handler TCC1 (comme reference
// de temps), mais avec son propre compteur et sa propre periode -> totalement independant de D5.
#define BLINK_D8_OVERFLOW_COUNT 47000   // Meme periode que D5 par defaut ; changez cette valeur pour desynchroniser

// Rapport cyclique (8 bits, 0-255) pour D5 uniquement
#define LED_D5_DUTY 180   // ~70%
*/

void setupTimers() // used to set up fast PWM on pins 1,9,2,3 (TCC0) + D5 (TCC1, PWM) + D8 (GPIO on/off)
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

  
  /*
  // --- AJOUT : configuration de D5 (PA09) en PWM via TCC1 ---
  // D5 = PA09 -> fonction F = TCC1/WO[3], duplique depuis CC1 (confirme fonctionnel)
  PORT->Group[g_APinDescription[5].ulPort].PINCFG[g_APinDescription[5].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[5].ulPort].PMUX[g_APinDescription[5].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F; // D5 = PA09 impair -> TCC1/WO[3]
  // --- FIN AJOUT D5 ---

  // --- AJOUT : configuration de D8 (PA07) en simple GPIO de sortie (pas de PWM, pas de PMUX) ---
  // On NE touche PAS au PINCFG.PMUXEN ici : en le laissant a 0 (valeur par defaut), la broche
  // reste sous controle direct du PORT (GPIO classique), ce qui est exactement ce qu'il faut
  // pour un pilotage on/off simple, independant de tout timer.
  PORT->Group[g_APinDescription[7].ulPort].DIRSET.reg = (1 << (g_APinDescription[7].ulPin % 32)); // D8 en sortie
  PORT->Group[g_APinDescription[7].ulPort].OUTCLR.reg = (1 << (g_APinDescription[7].ulPin % 32)); // D8 eteinte au demarrage
  // --- FIN AJOUT D8 ---
  */
  

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK4 //0 only works for interrup?
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  // NOTE : TCC0 et TCC1 partagent le meme bus de clock generique (ID_TCC0_TCC1),
  // donc la ligne ci-dessus alimente deja TCC1 avec la meme horloge que TCC0 (GCLK0, 48MHz).
  // C'est ici (GCLK_CLKCTRL_GEN_GCLKx) qu'il faudrait changer la source si on voulait
  // une frequence differente entre TCC0 et TCC1 - mais comme ils sont sur le meme ID
  // TCC0_TCC1, il faudrait en realite les decoupler autrement (ce n'est pas fait ici
  // puisqu'on garde volontairement la meme horloge pour les deux).


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
  //TCC0->INTENSET.bit.OVF = 1;  //*****************************************************
  TCC0->INTENSET.bit.MC0 = 0;

  NVIC_EnableIRQ(TCC0_IRQn);
  NVIC_SetPriority(TCC0_IRQn, 0);   // High priority (0 = highest)
  TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE ;

  /*
  // below is the code to enable the second interrupt TCC1
  // ================= AJOUT : configuration TCC1 (D5 uniquement, 8 bits) =================

  // Meme principe que TCC0 : mode dual slope PWM
  REG_TCC1_WAVE |= TCC_WAVE_POL(0x3) |            // Inverse la polarite sur les sorties de TCC1
                    TCC_WAVE_WAVEGEN_DSBOTH;      // Dual slope PWM sur TCC1
  while (TCC1->SYNCBUSY.bit.WAVE);                // Attente de synchronisation

  // Resolution 8 bits -> PER = 0xFF = 255 (256 pas)
  REG_TCC1_PER = 0xFF;                            // TOP = 255 => resolution 8 bits
  while (TCC1->SYNCBUSY.bit.PER);                 // Attente de synchronisation

  REG_TCC1_CC1 = LED_D5_DUTY;                     // Rapport cyclique de D5 (WO[3], duplique depuis CC1)
  while (TCC1->SYNCBUSY.bit.CC1);                 // Attente de synchronisation

  // Meme prescaler que TCC0 (DIV1) : la clock source est identique, mais PER etant plus petit
  // (8 bits au lieu de 9), la frequence de decoupage de TCC1 sera plus elevee que celle de TCC0.
  // NOTE : c'est ICI (TCC_CTRLA_PRESCALER_DIVx) qu'on peut ralentir TCC1 sans toucher a sa
  // resolution 8 bits, par exemple en passant a TCC_CTRLA_PRESCALER_DIV2, DIV4, DIV8, etc.
  REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |
                    TCC_CTRLA_ENABLE;
  while (TCC1->SYNCBUSY.bit.ENABLE);              // Attente de synchronisation

  // Interruption sur overflow (OVF) du compteur TCC1, utilisee pour le clignotement
  // de D5 (PWM) ET de D8 (GPIO), chacune avec son propre compteur -> independantes.
  TCC1->INTENSET.reg = 0;
  TCC1->INTENSET.bit.OVF = 1;

  NVIC_EnableIRQ(TCC1_IRQn);
  NVIC_SetPriority(TCC1_IRQn, 1);   // Priorité haute (0 = la plus prioritaire)
  TCC1->CTRLA.reg |= TCC_CTRLA_ENABLE;
  // ================= FIN AJOUT TCC1 =================
  */
}


void TCC0_Handler() 
{
  if (TCC0->INTFLAG.bit.CNT)
  {
    // just for debugging
    uint32_t tcc0_start_time = micros();

    // increment the accumulators and get the reset flags
    uint8_t rst_flags = accumulators.increment();

    // calculate the waveforms and allocate them to the right outputs
    REG_TCC0_CC0 = generator(accumulators.getAccValue(0), Presets.curr()[0].wave, 0, (rst_flags & 0b00000001));
    REG_TCC0_CC2 = generator(accumulators.getAccValue(1), Presets.curr()[1].wave, 1, (rst_flags & 0b00000010));
    REG_TCC0_CC3 = generator(accumulators.getAccValue(2), Presets.curr()[2].wave, 2, (rst_flags & 0b00000100));
    REG_TCC0_CC1 = generator(accumulators.getAccValue(3), Presets.curr()[3].wave, 3, (rst_flags & 0b00001000));

    // DEBUG STUFF
    uint32_t time_spent = micros() - tcc0_start_time;

    _debug_total_time_spent_in_TCC0 += time_spent;
    if (time_spent>_debug_max_time_spent_in_TCC0)
      _debug_max_time_spent_in_TCC0 = time_spent;
    ++_debug_number_of_calls_to_TCC0;
    // END OF DEBUG STUFF

    // reset the interruption flag
    TCC0->INTFLAG.bit.CNT = 1;
    //*******************************************************
  }
}



/*

// Handler d'interruption de TCC1 : appele a chaque overflow du compteur.
// Sert de base de temps commune pour clignoter D5 (PWM, via CC1) et D8 (GPIO on/off),
// mais avec deux compteurs et deux periodes separes -> reglages independants.
void TCC1_Handler()
{
  // On verifie que l'interruption vient bien de l'overflow (OVF)
  if (TCC1->INTFLAG.bit.OVF)
  {
    TCC1->INTFLAG.bit.OVF = 1;   // Acquittement du flag (obligatoire, sinon l'IRQ reste active en boucle)

    // --- Clignotement D5 (PWM 8 bits) ---
    blink_D5_Counter++;
    if (blink_D5_Counter >= BLINK_D5_OVERFLOW_COUNT)
    {
      blink_D5_Counter = 0;
      led_D5_State = !led_D5_State;
      REG_TCC1_CC1 = led_D5_State ? LED_D5_DUTY : 0;
      // Pas besoin d'attendre SYNCBUSY ici : l'ecriture de CCx dans un handler
      // d'interruption sur overflow est mise a jour au prochain cycle automatiquement
      // via le registre de "double buffering" du TCC.
    }

    // --- Clignotement D8 (GPIO on/off, totalement independant de D5) ---
    blink_D8_Counter++;
    if (blink_D8_Counter >= BLINK_D8_OVERFLOW_COUNT)
    {
      blink_D8_Counter = 0;
      led_D8_State = !led_D8_State;

      if (led_D8_State)
        PORT->Group[g_APinDescription[7].ulPort].OUTSET.reg = (1 << (g_APinDescription[7].ulPin % 32)); // D7 allumee
      else
        PORT->Group[g_APinDescription[7].ulPort].OUTCLR.reg = (1 << (g_APinDescription[7].ulPin % 32)); // D7 eteinte
    }
  }
}
*/


/*
void readSettings (void)
{
  char x;
  char c = 'S';
  int y = 1;
  int z = static_cast<int>(CovenWaves::Square);

  init_storage.read(x);

  if (x == 'S')  //S means eeprom has been initialized.
  {
    wave_storage.read(z);
    for (size_t i=0; i<4; i++)
      Presets.curr()[i].wave = static_cast<CovenWaves>(z);
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
    
    for (size_t i=0; i<4; i++)
      Presets.curr()[i].wave = static_cast<CovenWaves>(z);

    for (uint8_t i=0; i<4; ++i)
    {
      accumulators.setResetInfo(i, static_cast<uint16_t>(Presets.curr()[i].divider * Presets.curr().div), Presets.curr()[i].synced);
    }

  }
}


void saveSettings (void)
{
  int x;

  // only store the value if it's different from the previous stored value
  wave_storage.read(x);
  if (x != static_cast<int>(Presets.curr()[0].wave))
  {
    x = static_cast<int>(Presets.curr()[0].wave);
    wave_storage.write(x);  // <-- save the waveSelect 
  }

  div_storage.read(x);
  if (x != divSelect)
    div_storage.write(divSelect);  // <-- save the divSelect  
}
*/
