#pragma once
#include <JuceHeader.h>
#include "LevelMatcher.h"
#include "Saturation1073ish.h"

class YourPluginAudioProcessor  : public juce::AudioProcessor
{
public:
    YourPluginAudioProcessor();
    ~YourPluginAudioProcessor() override = default;

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override {}
    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    // ... resto JUCE boilerplate ...

    juce::AudioProcessorValueTreeState apvts;

private:
    juce::AudioProcessorValueTreeState::ParameterLayout createParams();

    // Parámetros (cache)
    std::atomic<float>* pDrive = nullptr; // 0..1
    std::atomic<float>* pTone  = nullptr; // 0..1 (dark->bright)
    std::atomic<float>* pMix   = nullptr; // 0..1

    // Smoothers (para evitar zipper)
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> driveSm;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> toneSm;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> mixSm;

    // Tilt EQ (pre-saturación): low shelf + high shelf (opuestos)
    juce::dsp::IIR::Filter<float> lowShelfL, lowShelfR;
    juce::dsp::IIR::Filter<float> highShelfL, highShelfR;

    Sat1073ish sat;
    LevelMatcher levelMatch;

    double sr = 48000.0;

    void updateTiltCoeffs (float tone01);

    static float mapDriveDb (float drive01); // 0..1 -> 0..+30 dB aprox
    static float equalPowerMix (float dry, float wet, float mix01);
};
