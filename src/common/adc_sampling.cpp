#include "adc_sampling.h"

Joystick::Joystick(int adcPin, float minValue, float maxValue, float range) : mAdcPin{adcPin}, mMinValue{minValue}, mMaxValue{maxValue}, mRange{range}
{
    allJoysticks().push_back(this);
}

Joystick::~Joystick()
{
    auto &vec = allJoysticks();
    vec.erase(std::remove(vec.begin(), vec.end(), this), vec.end());
}

AnalogValue Joystick::read()
{
    const auto raw = mLastAvgValue.load();
    const auto lastMillis = mLastSamplingMillis.load();

    const auto minValue = std::min(mMinValue, mMaxValue);
    const auto maxValue = std::max(mMinValue, mMaxValue);

    auto value = (raw - minValue) * mRange / (maxValue - minValue);
    if (raw <= minValue)
    {
        value = 0;
    }
    else if (raw >= maxValue)
    {
        value = mRange;
    }

    if (mMaxValue < mMinValue)
    {
        value = mRange - value;
    }

    return {raw, value, lastMillis};
}

void Joystick::runSamplingLoop(unsigned long delay_ms)
{
    while (true)
    {
        for (Joystick *j : allJoysticks())
        {
            j->sampleAdc();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
}

void Joystick::sampleAdc()
{
    const uint32_t raw = analogRead(mAdcPin);
    if (mLastSamplingMillis == 0)
    {
        // Init buffer if sampling for first time
        std::fill(mAvgBuffer.begin(), mAvgBuffer.end(), raw);
        mAvgSum = raw * mAvgBuffer.size();
    }
    else
    {

        mAvgSum -= mAvgBuffer.at(mBufferIndex);
        mAvgSum += raw;
        mAvgBuffer.at(mBufferIndex) = raw;

        mBufferIndex = (mBufferIndex + 1) % mAvgBuffer.size();
    }

    mLastAvgValue = (mAvgSum + mAvgBuffer.size() / 2) / mAvgBuffer.size();
    mLastSamplingMillis = millis();
}

void Joystick::startSampling(int intervallMs)
{
    if (!sThread.joinable())
    {
        sThread = std::thread{Joystick::runSamplingLoop, intervallMs};
    }
    else
    {
        Serial.println("Warning: Joystick::startSampling() called while a sampling thread is already running");
    }
}

std::vector<Joystick *> &Joystick::allJoysticks()
{
    static std::vector<Joystick *> all;
    return all;
}

std::thread Joystick::sThread;