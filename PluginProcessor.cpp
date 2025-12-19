#include "PluginProcessor.h"

static juce::AudioProcessorValueTreeState::ParameterLayout makeLayout()
{
    std::vector<std::unique_ptr<juce::RangedAudioParameter>> params;

    params.push_back (std::make_unique<juce::AudioParameterFloat> (
        "drive", "Drive", juce::NormalisableRange<float> (0.0f, 1.0f, 0.0001f), 0.25f));

    params.push_back (std::make_unique<juce::AudioParameterFloat> (
        "tone", "Tone", juce::NormalisableRange<float> (0.0f, 1.0f, 0.0001f), 0.5f));

    params.push_back (std::make_unique<juce::AudioParameterFloat> (
        "mix", "Mix", juce::NormalisableRange<float> (0.0f, 1.0f, 0.0001f), 1.0f));

    return { params.begin(), params.end() };
}

YourPluginAudioProcessor::YourPluginAudioProcessor()
: apvts (*this, nullptr, "PARAMS", makeLayout())
{
    pDrive = apvts.getRawParameterValue ("drive");
    pTone  = apvts.getRawParameterValue ("tone");
    pMix   = apvts.getRawParameterValue ("mix");
}

void YourPluginAudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    sr = sampleRate;

    // Smoothing 20ms para knobs
    driveSm.reset (sr, 0.02);
    toneSm .reset (sr, 0.02);
    mixSm  .reset (sr, 0.02);

    driveSm.setCurrentAndTargetValue (*pDrive);
    toneSm .setCurrentAndTargetValue (*pTone);
    mixSm  .setCurrentAndTargetValue (*pMix);

    // Tilt EQ init
    juce::dsp::ProcessSpec spec;
    spec.sampleRate = sr;
    spec.maximumBlockSize = (juce::uint32) samplesPerBlock;
    spec.numChannels = 1;

    lowShelfL.reset(); lowShelfR.reset();
    highShelfL.reset(); highShelfR.reset();

    // Coef iniciales
    updateTiltCoeffs (*pTone);

    sat.reset();

    // Level matcher
    levelMatch.prepare (sr);
    levelMatch.setGateDb (-60.0f);
    levelMatch.setClampDb (-12.0f, +12.0f);
    levelMatch.setMeasurementWindowMs (300.0f);
    levelMatch.setGainSmoothingMs (120.0f, 600.0f);
}

float YourPluginAudioProcessor::mapDriveDb (float drive01)
{
    // Curva musical: 0..1 -> 0..+30 dB con más resolución al inicio
    // (pow < 1 expande el rango bajo)
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
    // Tone: 0=dark, 0.5=neutral, 1=bright
    const float t = juce::jlimit (0.0f, 1.0f, tone01);
    const float tiltDb = (t * 2.0f - 1.0f) * 6.0f; // +/- 6 dB
    const float pivotHz = 1100.0f;
    const float Q = 0.707f;

    auto low  = juce::dsp::IIR::Coefficients<float>::makeLowShelf  (sr, pivotHz, Q,
                    juce::Decibels::decibelsToGain (-tiltDb));
    auto high = juce::dsp::IIR::Coefficients<float>::makeHighShelf (sr, pivotHz, Q,
                    juce::Decibels::decibelsToGain (+tiltDb));

    *lowShelfL.coefficients  = *low;
    *lowShelfR.coefficients  = *low;
    *highShelfL.coefficients = *high;
    *highShelfR.coefficients = *high;
}

void YourPluginAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;

    const int numCh = buffer.getNumChannels();
    const int numSamples = buffer.getNumSamples();

    // Cache targets
    driveSm.setTargetValue (*pDrive);
    toneSm .setTargetValue (*pTone);
    mixSm  .setTargetValue (*pMix);

    // Si tone cambia, actualizamos coef (con smoothing)
    // (Actualizamos cada bloque: suficiente para MVP)
    updateTiltCoeffs (toneSm.getTargetValue());

    auto* ch0 = buffer.getWritePointer (0);
    auto* ch1 = (numCh > 1) ? buffer.getWritePointer (1) : nullptr;

    for (int i = 0; i < numSamples; ++i)
    {
        const float drive01 = driveSm.getNextValue();
        const float tone01  = toneSm.getNextValue();
        const float mix01   = mixSm.getNextValue();

        // (Si quieres ultra-suave, podrías recalcular tilt a menor tasa; por ahora bloque OK)
        (void) tone01;

        const float driveDb  = mapDriveDb (drive01);
        const float pregain  = juce::Decibels::decibelsToGain (driveDb);

        // L
        float dryL = ch0[i];
        float xL = dryL * pregain;

        xL = lowShelfL.processSample (xL);
        xL = highShelfL.processSample (xL);

        float wetL = sat.process (xL);

        // Sin knob extra: micro “soften” dependiente de drive (muy leve)
        // (Evita fizz si empujas fuerte)
        const float soften = 1.0f / (1.0f + 0.12f * driveDb); // simple
        wetL *= (0.9f + 0.1f * soften);

        float mixedL = equalPowerMix (dryL, wetL, mix01);

        // R (si mono, copiamos L)
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

        // Level match basado en señal MIXED (para que Mix no cambie volumen)
        // Referencia: DRY (entrada)
        // Para estéreo: promediamos la medición para un gain común y coherente
        const float dryAvg   = 0.5f * (dryL   + dryR);
        const float mixedAvg = 0.5f * (mixedL + mixedR);

        const float g = levelMatch.process (dryAvg, mixedAvg);

        float outL = mixedL * g;
        float outR = mixedR * g;

        // Safety final
        outL = Sat1073ish::softClipSafety (outL);
        outR = Sat1073ish::softClipSafety (outR);

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
