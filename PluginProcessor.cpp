// PluginProcessor.cpp
#include "PluginProcessor.h"

//==============================================================================
// Parameters
static juce::AudioProcessorValueTreeState::ParameterLayout makeLayout()
{
    std::vector<std::unique_ptr<juce::RangedAudioParameter>> params;

    params.push_back (std::make_unique<juce::AudioParameterFloat>(
        "drive", "Drive",
        juce::NormalisableRange<float>(0.0f, 1.0f, 0.0001f),
        0.25f));

    params.push_back (std::make_unique<juce::AudioParameterFloat>(
        "tone", "Tone",
        juce::NormalisableRange<float>(0.0f, 1.0f, 0.0001f),
        0.5f));

    params.push_back (std::make_unique<juce::AudioParameterFloat>(
        "mix", "Mix",
        juce::NormalisableRange<float>(0.0f, 1.0f, 0.0001f),
        1.0f));

    return { params.begin(), params.end() };
}

//==============================================================================
// IMPORTANT: Declara buses, si no FL puede no detectarlo
YourPluginAudioProcessor::YourPluginAudioProcessor()
    : juce::AudioProcessor (BusesProperties()
                            .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                            .withOutput ("Output", juce::AudioChannelSet::stereo(), true))
    , apvts (*this, nullptr, "PARAMS", makeLayout())
{
    pDrive = apvts.getRawParameterValue ("drive");
    pTone  = apvts.getRawParameterValue ("tone");
    pMix   = apvts.getRawParameterValue ("mix");
}

//==============================================================================
// Required JUCE overrides (no GUI custom: usamos Generic editor)
const juce::String YourPluginAudioProcessor::getName() const
{
    return JucePlugin_Name;
}

bool YourPluginAudioProcessor::acceptsMidi() const
{
    return false;
}

bool YourPluginAudioProcessor::producesMidi() const
{
    return false;
}

bool YourPluginAudioProcessor::isMidiEffect() const
{
    return false;
}

double YourPluginAudioProcessor::getTailLengthSeconds() const
{
    return 0.0;
}

int YourPluginAudioProcessor::getNumPrograms()
{
    return 1;
}

int YourPluginAudioProcessor::getCurrentProgram()
{
    return 0;
}

void YourPluginAudioProcessor::setCurrentProgram (int) {}

const juce::String YourPluginAudioProcessor::getProgramName (int)
{
    return {};
}

void YourPluginAudioProcessor::changeProgramName (int, const juce::String&) {}

bool YourPluginAudioProcessor::hasEditor() const
{
    return true;
}

juce::AudioProcessorEditor* YourPluginAudioProcessor::createEditor()
{
    return new juce::GenericAudioProcessorEditor (*this);
}

bool YourPluginAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    // Solo mono o stereo, y mismo layout en input/output
    const auto& in  = layouts.getMainInputChannelSet();
    const auto& out = layouts.getMainOutputChannelSet();

    if (in.isDisabled() || out.isDisabled())
        return false;

    if (in != out)
        return false;

    return (in == juce::AudioChannelSet::mono()
         || in == juce::AudioChannelSet::stereo());
}

//==============================================================================
// State save/load
void YourPluginAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    auto state = apvts.copyState();
    std::unique_ptr<juce::XmlElement> xml (state.createXml());
    copyXmlToBinary (*xml, destData);
}

void YourPluginAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    std::unique_ptr<juce::XmlElement> xmlState (getXmlFromBinary (data, sizeInBytes));
    if (xmlState != nullptr)
        if (xmlState->hasTagName (apvts.state.getType()))
            apvts.replaceState (juce::ValueTree::fromXml (*xmlState));
}

//==============================================================================
// DSP
void YourPluginAudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    sr = sampleRate;

    // Smoothing 20ms
    driveSm.reset (sr, 0.02);
    toneSm .reset (sr, 0.02);
    mixSm  .reset (sr, 0.02);

    driveSm.setCurrentAndTargetValue (*pDrive);
    toneSm .setCurrentAndTargetValue (*pTone);
    mixSm  .setCurrentAndTargetValue (*pMix);

    // Coefs iniciales de Tilt
    updateTiltCoeffs (*pTone);

    sat.reset();

    // Level matcher
    levelMatch.prepare (sr);
    levelMatch.setGateDb (-60.0f);
    levelMatch.setClampDb (-12.0f, +12.0f);
    levelMatch.setMeasurementWindowMs (300.0f);
    levelMatch.setGainSmoothingMs (120.0f, 600.0f);

    juce::ignoreUnused (samplesPerBlock);
}

float YourPluginAudioProcessor::mapDriveDb (float drive01)
{
    const float shaped = std::pow (juce::jlimit (0.0f, 1.0f, drive01), 0.65f);
    return 30.0f * shaped;
}

float YourPluginAudioProcessor::equalPowerMix (float dry, float wet, float mix01)
{
    const float m = juce::jlimit (0.0f, 1.0f, mix01);
    const float a = std::cos (0.5f * juce::MathConstants<float>::pi * m);
    const float b = std::sin (0.5f * juce::MathConstants<float>::pi * m);
    return dry * a + wet * b;
}

void YourPluginAudioProcessor::updateTiltCoeffs (float tone01)
{
    const float t = juce::jlimit (0.0f, 1.0f, tone01);
    const float tiltDb = (t * 2.0f - 1.0f) * 6.0f; // +/- 6 dB
    const float pivotHz = 1100.0f;
    const float Q = 0.707f;

    auto low  = juce::dsp::IIR::Coefficients<float>::makeLowShelf  (sr, pivotHz, Q,
                    juce::Decibels::decibelsToGain (-tiltDb));
    auto high = juce::dsp::IIR::Coefficients<float>::makeHighShelf (sr, pivotHz, Q,
                    juce::Decibels::decibelsToGain (+tiltDb));

    lowShelfL.coefficients  = low;
    lowShelfR.coefficients  = low;
    highShelfL.coefficients = high;
    highShelfR.coefficients = high;

    lowShelfL.reset(); lowShelfR.reset();
    highShelfL.reset(); highShelfR.reset();
}

void YourPluginAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;

    const int numCh = buffer.getNumChannels();
    const int numSamples = buffer.getNumSamples();

    // Si vinieran canales extra, los limpiamos
    for (int ch = 2; ch < numCh; ++ch)
        buffer.clear (ch, 0, numSamples);

    auto* ch0 = buffer.getWritePointer (0);
    auto* ch1 = (numCh > 1) ? buffer.getWritePointer (1) : nullptr;

    driveSm.setTargetValue (*pDrive);
    toneSm .setTargetValue (*pTone);
    mixSm  .setTargetValue (*pMix);

    updateTiltCoeffs (toneSm.getTargetValue()); // MVP: update por bloque

    for (int i = 0; i < numSamples; ++i)
    {
        const float drive01 = driveSm.getNextValue();
        const float mix01   = mixSm.getNextValue();

        const float driveDb = mapDriveDb (drive01);
        const float pregain = juce::Decibels::decibelsToGain (driveDb);

        // L
        const float dryL = ch0[i];
        float xL = dryL * pregain;

        xL = lowShelfL.processSample (xL);
        xL = highShelfL.processSample (xL);

        float wetL = sat.process (xL);

        const float soften = 1.0f / (1.0f + 0.12f * driveDb);
        wetL *= (0.9f + 0.1f * soften);

        const float mixedL = equalPowerMix (dryL, wetL, mix01);

        // R
        float dryR = dryL;
        float mixedR = mixedL;

        if (ch1 != nullptr)
        {
            dryR = ch1[i];
            float xR = dryR * pregain;

            xR = lowShelfR.processSample (xR);
            xR = highShelfR.processSample (xR);

            float wetR = sat.process (xR);
            wetR *= (0.9f + 0.1f * soften);

            mixedR = equalPowerMix (dryR, wetR, mix01);
        }

        const float dryAvg   = 0.5f * (dryL   + dryR);
        const float mixedAvg = 0.5f * (mixedL + mixedR);

        const float g = levelMatch.process (dryAvg, mixedAvg);

        float outL = Sat1073ish::softClipSafety (mixedL * g);
        float outR = Sat1073ish::softClipSafety (mixedR * g);

        ch0[i] = outL;
        if (ch1 != nullptr) ch1[i] = outR;
    }
}

//==============================================================================
// This creates new instances of the plugin.
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new YourPluginAudioProcessor();
}


