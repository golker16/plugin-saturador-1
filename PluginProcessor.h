// PluginProcessor.h
#pragma once

#include <JuceHeader.h>
#include "LevelMatcher.h"
#include "Saturation1073ish.h"

class YourPluginAudioProcessor : public juce::AudioProcessor
{
public:
    YourPluginAudioProcessor();
    ~YourPluginAudioProcessor() override = default;

    //==============================================================================
    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override {}

    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;

    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    //==============================================================================
    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    //==============================================================================
    const juce::String getName() const override;

    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect() const override;

    double getTailLengthSeconds() const override;

    //==============================================================================
    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram (int index) override;
    const juce::String getProgramName (int index) override;
    void changeProgramName (int index, const juce::String& newName) override;

    //==============================================================================
    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

    //==============================================================================
    juce::AudioProcessorValueTreeState apvts;

private:
    // Helpers DSP
    void updateTiltCoeffs (float tone01);
    static float mapDriveDb (float drive01);
    static float equalPowerMix (float dry, float wet, float mix01);

    // Parameter pointers (cache)
    std::atomic<float>* pDrive = nullptr; // 0..1
    std::atomic<float>* pTone  = nullptr; // 0..1
    std::atomic<float>* pMix   = nullptr; // 0..1

    // Smoothers (evita zipper noise)
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> driveSm;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> toneSm;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> mixSm;

    // Tilt EQ (pre-saturación): low shelf + high shelf (opuestos)
    juce::dsp::IIR::Filter<float> lowShelfL, lowShelfR;
    juce::dsp::IIR::Filter<float> highShelfL, highShelfR;

    Sat1073ish sat;
    LevelMatcher levelMatch;

    double sr = 48000.0;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (YourPluginAudioProcessor)
};

