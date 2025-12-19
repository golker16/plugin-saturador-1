#pragma once
#include <JuceHeader.h>

// Level matching simple y estable: RMS con EMA + gate + clamp + smoothing attack/release
class LevelMatcher
{
public:
    void prepare (double sampleRate)
    {
        sr = sampleRate;

        // Ventanas de medición (ms) - ajusta si quieres:
        setMeasurementWindowMs (300.0f);

        // Smoothing del gain (ms)
        setGainSmoothingMs (100.0f, 500.0f); // attack, release

        // Reset estados
        reset();
    }

    void reset()
    {
        refEnv = 0.0f;
        outEnv = 0.0f;
        compGain = 1.0f;
        targetGain = 1.0f;
    }

    // Gate en dBFS RMS aprox
    void setGateDb (float gateDb_)
    {
        gateLin = juce::Decibels::decibelsToGain (gateDb_);
        gatePow = gateLin * gateLin;
    }

    // Clamp del auto-gain (en dB)
    void setClampDb (float minDb, float maxDb)
    {
        minGain = juce::Decibels::decibelsToGain (minDb);
        maxGain = juce::Decibels::decibelsToGain (maxDb);
    }

    void setMeasurementWindowMs (float ms)
    {
        // EMA alpha para potencia: alpha = exp(-1/(tau*sr))
        // tau ~ ms/1000
        const float tau = juce::jmax (1.0e-4f, ms * 0.001f);
        measAlpha = std::exp (-1.0f / (tau * (float) sr));
    }

    void setGainSmoothingMs (float attackMs, float releaseMs)
    {
        gainAttackAlpha  = alphaFromMs (attackMs);
        gainReleaseAlpha = alphaFromMs (releaseMs);
    }

    // Llama 1 vez por sample (o dentro de tu bucle). Te devuelve el gain actualizado.
    float process (float drySample, float mixedSample)
    {
        const float dryP = drySample * drySample;
        const float outP = mixedSample * mixedSample;

        // EMA de potencia
        refEnv = measAlpha * refEnv + (1.0f - measAlpha) * dryP;
        outEnv = measAlpha * outEnv + (1.0f - measAlpha) * outP;

        // Gate: si estamos casi en silencio, NO actualizamos target
        if (refEnv > gatePow && outEnv > gatePow)
        {
            const float refRms = std::sqrt (refEnv + 1.0e-12f);
            const float outRms = std::sqrt (outEnv + 1.0e-12f);

            float t = refRms / outRms;

            // Clamp
            t = juce::jlimit (minGain, maxGain, t);
            targetGain = t;
        }

        // Suavizado attack/release del gain
        const bool needDown = (targetGain < compGain);
        const float a = needDown ? gainAttackAlpha : gainReleaseAlpha;
        compGain = a * compGain + (1.0f - a) * targetGain;

        return compGain;
    }

private:
    float alphaFromMs (float ms) const
    {
        const float tau = juce::jmax (1.0e-4f, ms * 0.001f);
        return std::exp (-1.0f / (tau * (float) sr));
    }

    double sr = 48000.0;

    // EMA de potencia
    float measAlpha = 0.999f;
    float refEnv = 0.0f;
    float outEnv = 0.0f;

    // Gate
    float gateLin = juce::Decibels::decibelsToGain (-60.0f);
    float gatePow = gateLin * gateLin;

    // Clamp
    float minGain = juce::Decibels::decibelsToGain (-12.0f);
    float maxGain = juce::Decibels::decibelsToGain (+12.0f);

    // Gain smoothing
    float gainAttackAlpha  = 0.999f;
    float gainReleaseAlpha = 0.9995f;

    float compGain = 1.0f;
    float targetGain = 1.0f;
};
