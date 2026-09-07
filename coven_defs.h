// ButtonState.h
#ifndef COVEN_DEFS_H
#define COVEN_DEFS_H



#include <stdint.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <array>



// available waveforms.
// Count is not one of them, but it is used to handle more easily the cycling through different waveforms
// always leave it at the end

enum class CovenWaves: uint8_t {
    Square,
    Triangle,
    RampDown,
    RampUp,
    Random,
    Count
};



// timings in milliseconds to distinguish between different types of presses
constexpr uint32_t ButtonPressTimeDebounce = 80;
constexpr uint32_t ButtonPressTimeMid = 1000;
constexpr uint32_t ButtonPressTimeLong = 3000;


// hopefully easier to understand and cleaner way to handle the button state
enum class ButtonState : uint8_t {
    Undefined,
    Short_Press,
    Medium_Press,
    Long_Press,
    Released,
    Debounce_Check,
    Pressed
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
  volatile CovenWaves wave = CovenWaves::Triangle;
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

private:
    CovenSettings (&presets)[N];  // automatically sets the number of settings
    size_t cursor = 0;
};





template <uint8_t NBits>
class CVInHandler {
    static_assert(NBits < 8, "TableSize must fit in a uint8_t cursor.");
public:
    static constexpr uint8_t TableSize = 1u << NBits;

    explicit CVInHandler(uint8_t pin) : pin_to_read(pin) {}

    uint16_t readCV()
    {
        uint16_t new_value = analogRead(pin_to_read);

        if (!isInitialized)
        {
            for (uint8_t i = 0; i < TableSize; ++i)
            {
                cv_readings[i] = new_value;
                summed_values += new_value;
            }
            isInitialized = true;
            return new_value;
        }

        summed_values -= cv_readings[cursor];
        summed_values += new_value;

        cv_readings[cursor] = new_value;
        cursor = (cursor + 1) & (TableSize - 1);

        return summed_values >> NBits;
    }

private:
    bool isInitialized = false;
    uint8_t pin_to_read;
    uint8_t cursor = 0;
    uint32_t summed_values = 0;
    uint16_t cv_readings[TableSize];
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