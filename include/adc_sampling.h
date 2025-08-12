#include <atomic>
#include <array>
#include <thread>

#include <Arduino.h>

struct AnalogValue
{
    int raw;
    float value;
    unsigned long lastMillis;
};

class Joystick
{
public:
    static void startSampling(int intervallMs);

    Joystick(int adcPin, float minValue, float maxValue, float range);

    ~Joystick();

    AnalogValue read();

private:
    static constexpr auto AVG_BUFFER_SIZE = 32;
    static std::thread sThread;

    const int mAdcPin;
    const float mMinValue;
    const float mMaxValue;
    const float mRange;

    uint32_t mAvgSum = 0;
    std::array<uint32_t, AVG_BUFFER_SIZE> mAvgBuffer;
    size_t mBufferIndex = 0;

    std::atomic<uint32_t> mLastAvgValue;
    std::atomic<unsigned long> mLastSamplingMillis{0}; // such that main thread knows that sampling thread did not die

    static std::vector<Joystick *>& allJoysticks();

    void sampleAdc();

    static void runSamplingLoop(unsigned long delay_ms);
};