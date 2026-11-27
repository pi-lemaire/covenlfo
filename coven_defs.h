// ButtonState.h
#ifndef COVEN_DEFS_H
#define COVEN_DEFS_H



#include <stdint.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <FlashAsEEPROM_SAMD.h>
#include <array>



// available waveforms.
// Count is not one of them, but it is used to handle more easily the cycling through different waveforms
// always leave it at the end

enum class CovenWaves: uint8_t {
    Square,
    Triangle,
    RampDown,
    RampUp,
    Sine,
    Random,
    Trigger,
    Count
};



// timings in milliseconds to distinguish between different types of presses
constexpr uint32_t ButtonPressTimeDebounce = 30;
constexpr uint32_t ButtonPressDblClickInterv = 300;
constexpr uint32_t ButtonPressTimeMid = 600;
constexpr uint32_t ButtonPressTimeLong = 2000;


// hopefully easier to understand and cleaner way to handle the button state
enum class ButtonState : uint8_t {
    Undefined,
    Simple_Click,
    Double_Click,
    Medium_Press,
    Long_Press,
    Released,
    Debounce_Check,
    Dbl_Click_Check_LOW,
    Dbl_Click_Check_HIGH,
    Pressed
};


enum class CovenInterfaceMode: uint8_t {
    Main,
    LFO_Edit,
    Div_Edit,
    Main_Edit
};



// always leave Count at the end. It's only used to cycle simply through the algos
enum class CovenAlgo: uint8_t {
    Classic,
    Manual_Div,
    Manual_Float,
    Count
};


// this bit of code allow us to simply cycle through Algos by simply doing ++ind; where ind is of CovenAlgo type
inline CovenAlgo& operator++(CovenAlgo& a)
{
    a = static_cast<CovenAlgo>(
        (static_cast<uint8_t>(a) + 1) %
         static_cast<uint8_t>(CovenAlgo::Count)
    );
    return a;
}



// allows us to cycle simply between the waves by simply doing ++ind; where ind is of CovenWave type
inline CovenWaves& operator++(CovenWaves& w)
{
    w = static_cast<CovenWaves>(
        (static_cast<uint8_t>(w) + 1) %
         static_cast<uint8_t>(CovenWaves::Count)
    );
    return w;
}


// sync handling stuff
// #define COVEN_SYNC_THESH_VALUE 370

enum class SyncState: uint8_t {
    Unsync = 0,
    FirstRisingEdge = 1,
    FirstFallingEdge = 2,
    NextRisingEdge = 3
};



// Struct to store a single LFO settings. We switch to floats, that
// will be rounded (truncated actually) when synced is true
struct SingleLFOSettings {
  float divider;
  bool synced = true;
  //volatile CovenWaves wave = CovenWaves::Triangle;
  CovenWaves wave = CovenWaves::Square;
};


// Class to store the 4 LFO settings + the main clock divider, also a float
class CovenSettings {
public:
    CovenSettings(std::array<SingleLFOSettings,4> settings, float d=1.0f)
        : lfos(settings), div(d)  {}

    float div;  // main clock divider (actually, multiplies the frequency)

    SingleLFOSettings&       operator[](size_t i)       { return lfos[i]; }
    const SingleLFOSettings& operator[](size_t i) const { return lfos[i]; }

private:
    std::array<SingleLFOSettings,4> lfos;
};


constexpr uint32_t magic_value = 0x43574D33;

struct SettingsHeader {
    uint32_t magic = magic_value;   // "CVM1"
};


// Class to store the settings banks. The template things allows us to
// store automatically the bank size upon object declaration
template <size_t N>
class CovenPresets {
public:
    CovenPresets(CovenSettings (&array)[N]) : presets(array) {}

    CovenSettings& curr() { return presets[cursor]; }
      // returns the currently selected set of settings

    const CovenSettings& curr() const { return presets[cursor]; }
      // same as above but const version

    void next() { cursor = (cursor + 1) % N; }
      // moves the cursor to the next settings

    constexpr size_t size() const { return N; }
      // returns the number of different settings stored

    size_t getPos() const { return cursor; }
      // current settings position

    void save()
    {
        int addr = 0;

        SettingsHeader h;
        EEPROM.put(addr, h);
        addr += sizeof(h);

        EEPROM.put(addr, cursor);
        addr += sizeof(cursor);

        for (size_t i = 0; i < N; i++)
        {
            EEPROM.put(addr, presets[i]);
            addr += sizeof(CovenSettings);
        }

        EEPROM.commit();
    }

    bool load()
    {
        int addr = 0;

        SettingsHeader h;
        EEPROM.get(addr, h);

        if (h.magic != magic_value)
            return false;          // première mise sous tension ou format différent

        addr += sizeof(h);

        // Restauration du curseur
        EEPROM.get(addr, cursor);
        addr += sizeof(cursor);

        for (size_t i = 0; i < N; i++)
        {
            EEPROM.get(addr, presets[i]);
            addr += sizeof(CovenSettings);
        }

        return true;
    }


private:
    CovenSettings (&presets)[N];  // automatically sets the number of settings
    size_t cursor = 0;
};





constexpr uint32_t CVInHandler_lock_time = 8000;    // time in millisec during which the CV must be stable to decide that it locks
constexpr int16_t CVInHandler_lock_threshold = 4;   // the value has to be within +- this value to stay in lock
                                                    // overall we have noticed that the value stays within a range of 8 values
                                                    // stability is not the same over the full range of values
                                                    // so the goal is to find the right center spot and then lock here


template <uint8_t NBits>
class CVInHandler {
    static_assert(NBits < 8, "TableSize must fit in a uint8_t cursor.");
public:
    static constexpr uint8_t TableSize = 1u << NBits;

    explicit CVInHandler(uint8_t pin) : pin_to_read(pin) {}

    uint16_t readCV()
    {
        uint16_t new_value = analogRead(pin_to_read);

        last_read_value = new_value;
        last_reading_time = millis();

        if (!isInitialized)
        {
            for (uint8_t i = 0; i < TableSize; ++i)
            {
                cv_readings[i] = new_value;
                summed_values += new_value;
            }
            isInitialized = true;

            is_locked = false;
            last_known_outside_value = new_value;

            // also init the lock mechanism
            last_constant_time = millis();
            lock_value = new_value;
            updateLockValue(lock_value);

            return new_value;
        }

        summed_values -= cv_readings[cursor];
        summed_values += new_value;

        cv_readings[cursor] = new_value;
        cursor = (cursor + 1) & (TableSize - 1);

        uint16_t ret_value = summed_values >> NBits;

        last_filtered_value = ret_value;


        // now looking at the locking mechanism
        if ( (ret_value>=lock_interval_lower_bound) && (ret_value<=lock_interval_upper_bound) )
        {
            // we are within the lock area, check the timing

            uint32_t curr_time = millis();
            if (curr_time - last_constant_time > CVInHandler_lock_time)
            {
                // time is higher than the lock time, update it
                if (!is_locked)
                {
                    //Serial.println("========== Just locked ==========");
                    // increase the boundaries by 2 to stabilize well
                    lock_interval_lower_bound -= 2;
                    lock_interval_upper_bound += 2;

                }

                is_locked = true;
                // update the time to avoid an overflow
                // last_constant_time = curr_time - 3 * CVInHandler_lock_time;

                return lock_value;
            }
            else
            {
                is_locked = false;
                // update the lock value to get is close to the retval in order to avoid a gap when locking
                updateLockValue(ret_value);

                return ret_value;
            }
        }
        else
        {
            //if (is_locked)
            //    Serial.println("========== Just UNlocked ==========");

            // not within the interval, reset everything and make the lock value track the current value
            is_locked = false;

            updateLockValue(ret_value);
            last_constant_time = millis();

            return ret_value;
        }

        return ret_value;
    }

    bool isLocked() const { return is_locked; }

    uint16_t getLockValue() const { return lock_value; }
    int16_t getLockLowerBound() const { return lock_interval_lower_bound; }
    int16_t getLockUpperBound() const { return lock_interval_upper_bound; }

    void updateLockValue(uint16_t curr_value)
    {
        //static uint16_t corr_counter = 0;
        // ret_val is used to know if we reset the timer
        // bool ret_val = false;

        


        if ((curr_value<lock_interval_lower_bound) || (curr_value>lock_interval_upper_bound))
        {
            // the lock value is well outside the boundaries, update it strongly
            if (curr_value<lock_interval_lower_bound)
                // coming from above, we need to decrease the lock value
                // just move it to the higher value that allows the current value inside the boundaries
                lock_value = curr_value + CVInHandler_lock_threshold;
            else
                // coming from below, we need to increase the lock value
                lock_value = curr_value - CVInHandler_lock_threshold;
            // corr_counter = 0;
            last_known_outside_value = curr_value;
        }
        else
        {
            // update the value as long as it is not too far from last_known_outside_value
            uint16_t prev_lock = lock_value;

            // not sure this mean calculation is actually useful
            lock_value = (lock_value + curr_value) >> 1;

            
            if ((prev_lock==lock_value) && (lock_value!=curr_value))
                // try to update anyway, this is for the extreme values mostly
                lock_value = (curr_value>lock_value) ? lock_value+1 : lock_value-1;
            

            if (lock_value > last_known_outside_value + CVInHandler_lock_threshold)
                lock_value = last_known_outside_value + CVInHandler_lock_threshold;
            else if (lock_value < last_known_outside_value - CVInHandler_lock_threshold)
                lock_value = last_known_outside_value - CVInHandler_lock_threshold;
            
            lock_value &= 0x3FF;  // ensuring that we stay within the boundaries
        }

        // determining the interval in which we consider that the input can be locked
        // &0x03ff is there to ensure that we're within a 10 bits value bound
        lock_interval_lower_bound = (lock_value - CVInHandler_lock_threshold);// & 0x3FF;
        lock_interval_upper_bound = (lock_value + CVInHandler_lock_threshold);// & 0x3FF;

        // return ret_val;
    }

    uint16_t lastReadValue() const { return last_read_value; }
    uint16_t lastFilteredValue() const { return last_filtered_value; }
    uint32_t lastReadingTime() const { return last_reading_time; }

private:

    uint16_t last_read_value;
    uint16_t last_filtered_value;
    uint32_t last_reading_time;

    bool isInitialized = false;
    uint8_t pin_to_read;
    uint8_t cursor = 0;
    uint32_t summed_values = 0;
    uint16_t cv_readings[TableSize];
    bool is_locked = false;
    bool lockable = true;
    uint16_t lock_value;
    int16_t last_known_outside_value; // this is to prevent the lock value from getting too far of an occasional extremum
    int16_t lock_interval_lower_bound;
    int16_t lock_interval_upper_bound;
    uint32_t last_constant_time;
};








constexpr size_t CVInHandler_DualQuantile_window_1 = 7;
constexpr size_t CVInHandler_DualQuantile_quant_1 = 4;
constexpr size_t CVInHandler_DualQuantile_window_2 = 5;
constexpr size_t CVInHandler_DualQuantile_quant_2 = 3;
constexpr uint32_t CVInHandler_DualQuantile_lock_time = 4000;
constexpr uint32_t CVInHandler_DualQuantile_unlock_time = 18;
constexpr uint16_t CVInHandler_DualQuantile_lock_window = 3;



class CVInHandler_DualQuantile {
public:
    explicit CVInHandler_DualQuantile(uint8_t pin) : pin_to_read(pin) {}

    uint16_t readCV()
    {
        uint16_t new_value = analogRead(pin_to_read);

        last_read_value = new_value;
        last_reading_time = millis();

        if (!isInitialized)
        {
            for (size_t i=0; i<CVInHandler_DualQuantile_window_1; ++i)
            {
                cv_readings_1[i] = new_value;
                ordered_readings_1[i] = new_value;
            }
            for (size_t i=0; i<CVInHandler_DualQuantile_window_2; ++i)
            {
                cv_readings_2[i] = new_value;
                ordered_readings_2[i] = new_value;
            }
            isInitialized = true;

            is_locked = false;

            // also init the lock mechanism
            last_new_value_time = millis();
            updateLockWindow(new_value);

            return new_value;
        }

        uint16_t old_value_1 = cv_readings_1[cursor_1];
        cv_readings_1[cursor_1] = new_value;
        cursor_1 = (cursor_1 + 1) % CVInHandler_DualQuantile_window_1;

        // remove the old value
        for (size_t k_1=0; k_1<CVInHandler_DualQuantile_window_1; ++k_1)
        {
            if (ordered_readings_1[k_1]==old_value_1)
            {
                for (size_t k=k_1; k<CVInHandler_DualQuantile_window_1-1; ++k)
                {
                    ordered_readings_1[k] = ordered_readings_1[k+1];
                }
                break;
            }
        }

        // insert the new value
        bool inserted_1 = false;
        for (size_t k_1=0; k_1<CVInHandler_DualQuantile_window_1-1; ++k_1)
        {
            if (ordered_readings_1[k_1]>new_value)
            {
                for (size_t k=CVInHandler_DualQuantile_window_1-1; k>k_1; --k)
                {
                    ordered_readings_1[k] = ordered_readings_1[k-1];
                }
                ordered_readings_1[k_1] = new_value;
                inserted_1 = true;
                break;
            }
        }
        if (!inserted_1)
            // the new value wasn't inserted, it should end up at the last position
            ordered_readings_1[CVInHandler_DualQuantile_window_1-1] = new_value;

        // result of the first layer of quantiles
        uint16_t quantile_1 = ordered_readings_1[CVInHandler_DualQuantile_quant_1];


        // now compute the second quantile layer

        uint16_t old_value_2 = cv_readings_2[cursor_2];
        cv_readings_2[cursor_2] = quantile_1;
        cursor_2 = (cursor_2 + 1) % CVInHandler_DualQuantile_window_2;

        // remove the old value
        for (size_t k_2=0; k_2<CVInHandler_DualQuantile_window_2; ++k_2)
        {
            if (ordered_readings_2[k_2]==old_value_2)
            {
                for (size_t k=k_2; k<CVInHandler_DualQuantile_window_2-1; ++k)
                {
                    ordered_readings_2[k] = ordered_readings_2[k+1];
                }
                break;
            }
        }

        // insert the new value
        bool inserted_2 = false;
        for (size_t k_2=0; k_2<CVInHandler_DualQuantile_window_2-1; ++k_2)
        {
            if (ordered_readings_2[k_2]>quantile_1)
            {
                for (size_t k=CVInHandler_DualQuantile_window_2-1; k>k_2; --k)
                {
                    ordered_readings_2[k] = ordered_readings_2[k-1];
                }
                ordered_readings_2[k_2] = quantile_1;
                inserted_2 = true;
                break;
            }
        }
        if (!inserted_2)
            // the new value wasn't inserted, it should end up at the last position
            ordered_readings_2[CVInHandler_DualQuantile_window_2-1] = quantile_1;

        // output of the second layer of quantiles
        uint16_t ret_value = ordered_readings_1[CVInHandler_DualQuantile_quant_2];


        // now looking at the locking mechanism

        bool stable = (ret_value>=lock_interval_lower_bound) && (ret_value<=lock_interval_upper_bound);

        if (!stable && !is_locked)
        {
            // unstable and not locked : record the new value, update the time
            updateLockWindow(ret_value);
            last_new_value_time = millis();
            return ret_value;
        }
        else if (stable && !is_locked)
        {
            // we may be in a stable mode and switch to lock mode
            uint32_t time = millis();
            if (time - last_new_value_time > CVInHandler_DualQuantile_lock_time)
            {
                // we've been stable for long enough, we switch to lock mode
                // update the lock value to reflect the exact current value
                is_locked = true;
                updateLockWindow(ret_value);
                last_stable_value_time = time;
                return lock_value;
            }
            else
                return ret_value;
        }
        else if (stable and is_locked)
        {
            // stable and in lock mode : return the lock value
            last_stable_value_time = millis();
            return lock_value;
        }
        else
        {
            // not stable and locked
            uint32_t time = millis();
            if (time > last_stable_value_time + CVInHandler_DualQuantile_unlock_time)
            {
                // unstable for too long, leave the lock mode
                is_locked = false;
                last_new_value_time = time;
                return ret_value;
            }
            else
                // not unstable for long enough, return the lock value
                return lock_value;
        }
    }

    bool isLocked() const { return is_locked; }

    uint16_t getLockValue() const { return lock_value; }
    int16_t getLockLowerBound() const { return lock_interval_lower_bound; }
    int16_t getLockUpperBound() const { return lock_interval_upper_bound; }

    void updateLockWindow(uint16_t val)
    {
        lock_value = val;
        lock_interval_lower_bound = (lock_value - CVInHandler_DualQuantile_lock_window) & 0x3FF;
        lock_interval_upper_bound = (lock_value + CVInHandler_DualQuantile_lock_window) & 0x3FF;
    }

    uint16_t lastReadValue() const { return last_read_value; }
    uint16_t lastFilteredValue() const { return last_filtered_value; }
    uint32_t lastReadingTime() const { return last_reading_time; }

private:

    uint16_t last_read_value;
    uint16_t last_filtered_value;
    uint32_t last_reading_time;

    bool isInitialized = false;
    uint8_t pin_to_read;
    uint16_t cv_readings_1[CVInHandler_DualQuantile_window_1];
    uint16_t cv_readings_2[CVInHandler_DualQuantile_window_2];
    uint16_t ordered_readings_1[CVInHandler_DualQuantile_window_1];
    uint16_t ordered_readings_2[CVInHandler_DualQuantile_window_2];
    size_t cursor_1 = 0;
    size_t cursor_2 = 0;

    bool is_locked = false;
    bool lockable = true;

    uint16_t lock_value;
    int16_t lock_interval_lower_bound;
    int16_t lock_interval_upper_bound;
    
    uint32_t last_stable_value_time;
    uint32_t last_new_value_time;
};












// Class to store the settings banks. The template things allows us to
// store automatically the bank size upon object declaration
template <size_t N>
class CovenDice {
public:
    CovenDice(uint16_t max) : max_value(max)
    {
        for (size_t i=0; i<N; i++)
        {
            rd_values[i] = random(max);
            used_values[i] = false;
        }
        read_cursor = 0;
        write_cursor = 0;
    }

    void draw()
    {
        if (used_values[write_cursor])
        {
            rd_values[write_cursor] = random(max_value);
            used_values[write_cursor] = false;
            write_cursor = (write_cursor + 1) % N;
        }
    }

    uint16_t get_value()
    {
        uint16_t ret_value = rd_values[read_cursor];
        used_values[read_cursor] = true;
        read_cursor = (read_cursor + 1) % N;
        return ret_value;
    }

private:
    uint16_t max_value;
    size_t write_cursor = 0;
    uint16_t rd_values[N];
    volatile bool used_values[N];
    volatile size_t read_cursor = 0;
};





uint16_t euclidean_gcd(uint16_t a, uint16_t b)
{
  // really standard algorithm, found on https://www.cppbetterexplained.com/posts/cpp-gcd-lcm/
  while (b != 0)
  {
    uint16_t temp = b;
    b = a % b;    // remainder becomes the new b
    a = temp;     // old b becomes the new a
  }
  return a;
}

uint16_t get_lcd(uint16_t A, uint16_t B, uint16_t C, uint16_t D)
{
  // apply the euclidean algorithm to calculate the least common divider
  // useful to know when to sync relatively to the first wave, which is used as a ref
  // relatively heavy on the computational side, but it's only applied when changing the
  // divs, so this must not be common. It's essential that this function
  // only gets called within the main loop.
  uint16_t lcd_a_b = (A * B) / euclidean_gcd(A, B);
  uint16_t lcd_c_d = (C * D) / euclidean_gcd(C, D);
  return (lcd_a_b * lcd_c_d) / euclidean_gcd(lcd_a_b, lcd_c_d);
}





#endif