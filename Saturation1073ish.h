#pragma once
#include <cmath>
#include <algorithm>

struct Sat1073ish
{
    void reset() {}

    // driveLin: pre-gain (lineal), pero aquí ya te llega la señal pre-ganada.
    inline float process (float x) noexcept
    {
        // Etapa A: "transformer-ish" suave (tanh normalizada con un pelín de bias)
        x = transformerStage (x);

        // Etapa B: "amp" soft clip asimétrico (umbral distinto +/-, musical)
        x = ampStage (x);

        return x;
    }

    static inline float softClipSafety (float x) noexcept
    {
        // safety clip transparente
        const float k = 2.0f;
        return std::tanh (k * x) / std::tanh (k);
    }

private:
    static inline float transformerStage (float x) noexcept
    {
        // bias muy leve para armónicos pares (asimetría sutil)
        const float bias = 0.015f;
        const float k = 1.6f;

        const float y  = std::tanh (k * (x + bias)) - std::tanh (k * bias);
        const float yn = y / std::tanh (k); // normaliza aprox

        return yn;
    }

    static inline float ampStage (float x) noexcept
    {
        // Soft clip asimétrico: umbrales y curvas distintas
        const float posTh = 0.85f;
        const float negTh = -0.78f;

        if (x > posTh)
        {
            const float over = x - posTh;
            // curva suave tipo exp
            return posTh + (1.0f - std::exp (-3.0f * over)) / 3.0f;
        }

        if (x < negTh)
        {
            const float over = negTh - x;
            return negTh - (1.0f - std::exp (-2.5f * over)) / 2.5f;
        }

        return x;
    }
};
