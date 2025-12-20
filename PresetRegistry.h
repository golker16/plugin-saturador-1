#pragma once

#include <JuceHeader.h>
#include <array>
#include <type_traits>
#include <cstring>
#include <cmath>

#include "funciones.h"
#include "PresetRegistry.h"

class YourPluginAudioProcessor : public juce::AudioProcessor
{
public:
    YourPluginAudioProcessor();
    ~YourPluginAudioProcessor() override = default;

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override {}

    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    const juce::String getName() const override;

    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect() const override;

    double getTailLengthSeconds() const override;

    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram (int index) override;
    const juce::String getProgramName (int index) override;
    void changeProgramName (int index, const juce::String& newName) override;

    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

    juce::AudioProcessorValueTreeState apvts;

private:
    void updateTiltCoeffs (float tone01);

    // ============================================================
    // ✅ Stereo interaction PRO (solo dentro del oversampled block)
    // - Crosstalk dependiente de frecuencia (más mids, menos highs)
    // - Crosstalk dependiente de nivel (sube con drive)
    // - Micro fase/latencia via allpass muy leve (L/R distinto)
    // ============================================================
    struct StereoInteract
    {
        float sr = 48000.0f;

        // env para nivel-dependencia
        float env = 0.0f;

        // filtro para quitar highs al crossfeed (LP ~8k)
        float ct_lpL = 0.0f, ct_lpR = 0.0f;

        // band “mid” para más consola vibe
        float midHiL = 0.0f, midHiR = 0.0f;
        float midLoL = 0.0f, midLoR = 0.0f;

        // allpass states (2 en cascada por canal)
        float ap1_x1L = 0.0f, ap1_y1L = 0.0f;
        float ap1_x1R = 0.0f, ap1_y1R = 0.0f;

        float ap2_x1L = 0.0f, ap2_y1L = 0.0f;
        float ap2_x1R = 0.0f, ap2_y1R = 0.0f;

        void prepare (float sampleRate) noexcept
        {
            sr = (sampleRate > 1000.0f ? sampleRate : 48000.0f);
            reset();
        }

        void reset() noexcept
        {
            env = 0.0f;
            ct_lpL = ct_lpR = 0.0f;
            midHiL = midHiR = 0.0f;
            midLoL = midLoR = 0.0f;

            ap1_x1L = ap1_y1L = 0.0f;
            ap1_x1R = ap1_y1R = 0.0f;
            ap2_x1L = ap2_y1L = 0.0f;
            ap2_x1R = ap2_y1R = 0.0f;
        }

        static inline float clamp01 (float v) noexcept
        {
            return (v < 0.0f) ? 0.0f : (v > 1.0f) ? 1.0f : v;
        }

        static inline float onePoleLP (float y1, float x, float a) noexcept
        {
            return (1.0f - a) * x + a * y1;
        }

        static inline float alphaFromHz (float fc, float sr) noexcept
        {
            constexpr float twoPi = 6.2831853071795864769f;
            const float safeSr = (sr > 1.0f ? sr : 1.0f);
            return std::exp (-twoPi * (fc / safeSr));
        }

        static inline float allpass1 (float x, float a, float& x1, float& y1) noexcept
        {
            // y = -a*x + x1 + a*y1
            const float y = (-a * x) + x1 + (a * y1);
            x1 = x;
            y1 = y;
            return y;
        }

        inline void processSample (float& xL, float& xR) noexcept
        {
            // ---------- nivel ----------
            const float lvl = 0.5f * (std::fabs (xL) + std::fabs (xR));

            // env rápido (~8ms)
            const float aEnv = std::exp (-1.0f / (0.008f * sr));
            env = aEnv * env + (1.0f - aEnv) * lvl;

            // mapea env -> 0..1 (ajuste)
            const float env01 = clamp01 (env * 2.5f);

            // ---------- freq shaping ----------
            // LP para quitar highs del crossfeed (menos en highs)
            const float aLP = alphaFromHz (8000.0f, sr);
            ct_lpL = onePoleLP (ct_lpL, xL, aLP);
            ct_lpR = onePoleLP (ct_lpR, xR, aLP);

            // mids band: LP(2.5k) - LP(250)
            const float aHi = alphaFromHz (2500.0f, sr);
            const float aLo = alphaFromHz (250.0f,  sr);

            midHiL = onePoleLP (midHiL, xL, aHi);
            midLoL = onePoleLP (midLoL, xL, aLo);
            midHiR = onePoleLP (midHiR, xR, aHi);
            midLoR = onePoleLP (midLoR, xR, aLo);

            const float midBandL = midHiL - midLoL;
            const float midBandR = midHiR - midLoR;

            // mezcla: 65% low/mid (LP8k) + 35% mid band (más consola)
            const float srcL = 0.65f * ct_lpL + 0.35f * midBandL;
            const float srcR = 0.65f * ct_lpR + 0.35f * midBandR;

            // ---------- crosstalk amount ----------
            // base ~ -56 dB + sube con env (cuando saturas)
            const float base = 0.0016f;
            const float ct   = base * (1.0f + 1.6f * env01);

            // ---------- micro fase ----------
            // 2 allpass cascada, coef leve y distinto L/R
            float a1 = 0.33f + 0.10f * env01;
            float a2 = 0.18f + 0.06f * env01;

            a1 = juce::jlimit (0.05f, 0.75f, a1);
            a2 = juce::jlimit (0.05f, 0.75f, a2);

            // offsets pequeñísimos L/R
            const float a1L = a1 * 1.01f;
            const float a1R = a1 * 0.99f;
            const float a2L = a2 * 1.01f;
            const float a2R = a2 * 0.99f;

            float feedToL = srcR;
            float feedToR = srcL;

            feedToL = allpass1 (feedToL, a1L, ap1_x1L, ap1_y1L);
            feedToR = allpass1 (feedToR, a1R, ap1_x1R, ap1_y1R);

            feedToL = allpass1 (feedToL, a2L, ap2_x1L, ap2_y1L);
            feedToR = allpass1 (feedToR, a2R, ap2_x1R, ap2_y1R);

            // ---------- aplicar crossfeed ----------
            xL = xL + ct * feedToL;
            xR = xR + ct * feedToR;
        }
    };

    StereoInteract stereoInteract;

    // ============================================================
    // Parameter pointers
    std::atomic<float>* pDrive  = nullptr;
    std::atomic<float>* pTone   = nullptr;
    std::atomic<float>* pMix    = nullptr;
    std::atomic<float>* pPreamp = nullptr;

    // Smoothers
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> driveSm;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> toneSm;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> mixSm;

    // Tilt EQ (pre)
    juce::dsp::IIR::Filter<float> lowShelfL, lowShelfR;
    juce::dsp::IIR::Filter<float> highShelfL, highShelfR;

    // Oversampling
    static constexpr int kOversamplingExponent = 3; // 8x
    std::unique_ptr<juce::dsp::Oversampling<float>> oversampling;
    juce::AudioBuffer<float> wetBuffer;

    // Preset activo (stateful)
    const PresetRegistry::Item* activePreset = nullptr;
    int activePresetIndex = -1;

    using PresetStateStorage =
        std::aligned_storage_t<PresetRegistry::kMaxStateSize, PresetRegistry::kMaxStateAlign>;
    std::array<PresetStateStorage, 2> presetState {}; // hasta estéreo

    plugin::LevelMatcher levelMatch;

    double sr = 48000.0;
    float  osSr = 384000.0f; // sr * oversamplingFactor (se recalcula en prepareToPlay)

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (YourPluginAudioProcessor)
};
