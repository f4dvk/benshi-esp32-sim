#pragma once
#include <Arduino.h>
#include <math.h>
#include "config.h"

#if DISPLAY_ENABLE

// ============================================================================
// Analyseur de spectre audio minimal pour l'écran de façade.
//
// FFT radix-2 en float, N points, entrée réelle (fenêtre de Hann). Les bins
// utiles sont regroupés linéairement en `nbars` barres, magnitude mise à une
// échelle racine puis lissée (attaque rapide, descente lente) façon analyseur.
//
// N = 256 @ 32 kHz  -> 125 Hz / bin. Léger : ~50 µs par calcul, appelé par la
// tâche d'affichage (jamais depuis l'audio).
// ============================================================================

template <int N>
class AudioSpectrum {
public:
    // Échelle dB : 0 dB = raie pleine échelle. On mappe [FLOOR, TOP] -> [0, 255].
    // L'audio reçu passe par l'AGC (cible ~ -12 dB) -> une voix forte pointe
    // vers le haut, le bruit reste en bas. Ajuster si trop haut / trop bas.
    static constexpr float SPEC_DB_FLOOR = -44.0f;   // en dessous -> barre à zéro (bruit)
    static constexpr float SPEC_DB_TOP   = -8.0f;
    // Descente des barres (par trame) : attaque immédiate ; chute = on divise
    // l'écart à la cible par 2, avec AU MOINS SPEC_DECAY_MIN pour garantir le
    // retour à zéro (sinon une barre "colle" sur le plancher de bruit).
    static constexpr int   SPEC_DECAY_MIN = 10;
    // Compensation de pente : l'audio FM reçu est désaccentué (~-6 dB/octave) et
    // la voix a peu d'énergie dans les aigus -> les barres hautes restent plates.
    // SPEC_TILT relève les barres proportionnellement à leur fréquence
    // (0 = brut, 4 -> +~14 dB sur la dernière barre). Purement visuel.
    static constexpr float SPEC_TILT = 4.0f;

    // in[N] échantillons ADC -> bars[nbars] (0..255). maxBin = bin le plus haut
    // affiché. Renvoie le pic (dB) pour la trace de mise au point.
    float compute(const int16_t* in, uint8_t* bars, int nbars, int maxBin) {
        static float win[N];
        static bool winReady = false;
        if (!winReady) {
            for (int i = 0; i < N; i++)
                win[i] = 0.5f - 0.5f * cosf(2.0f * (float)PI * i / (N - 1));   // Hann
            winReady = true;
        }
        float mean = 0.0f;
        for (int i = 0; i < N; i++) mean += in[i];
        mean /= N;                                       // retrait du continu résiduel
        for (int i = 0; i < N; i++) {
            re_[i] = ((in[i] - mean) * (1.0f / 32768.0f)) * win[i];
            im_[i] = 0.0f;
        }
        fft();
        if (maxBin > N / 2) maxBin = N / 2;
        const float norm = 2.0f / N;                     // ~1.0 pour une raie pleine échelle
        const float span = SPEC_DB_TOP - SPEC_DB_FLOOR;
        float peakDb = -120.0f;
        for (int b = 0; b < nbars; b++) {
            int lo = 1 + b * maxBin / nbars;
            int hi = 1 + (b + 1) * maxBin / nbars;
            if (hi <= lo) hi = lo + 1;
            float m = 0.0f;
            for (int k = lo; k < hi && k < N / 2; k++) {
                float mag = sqrtf(re_[k] * re_[k] + im_[k] * im_[k]) * norm;
                if (mag > m) m = mag;
            }
            m *= 1.0f + SPEC_TILT * (float)b / nbars;     // compense la désaccentuation FM
            float db = 20.0f * log10f(m + 1e-4f);         // ~ -80 .. 0
            if (db > peakDb) peakDb = db;
            float v = (db - SPEC_DB_FLOOR) / span;
            if (v < 0.0f) v = 0.0f;
            if (v > 1.0f) v = 1.0f;
            int q = (int)(v * 255.0f);
            if (q >= bars[b]) {
                bars[b] = (uint8_t)q;                       // attaque immédiate
            } else {
                int drop = (bars[b] - q) >> 1;
                if (drop < SPEC_DECAY_MIN) drop = SPEC_DECAY_MIN;
                int nv = bars[b] - drop;
                bars[b] = (uint8_t)(nv < q ? q : nv);
            }
        }
        return peakDb;
    }

private:

    void fft() {
        // bit-reversal
        for (int i = 1, j = 0; i < N; i++) {
            int bit = N >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) {
                float t = re_[i]; re_[i] = re_[j]; re_[j] = t;
                t = im_[i]; im_[i] = im_[j]; im_[j] = t;
            }
        }
        for (int len = 2; len <= N; len <<= 1) {
            float ang = -2.0f * (float)PI / len;
            float wr = cosf(ang), wi = sinf(ang);
            for (int i = 0; i < N; i += len) {
                float cr = 1.0f, ci = 0.0f;
                for (int k = 0; k < len / 2; k++) {
                    float xr = re_[i + k + len / 2] * cr - im_[i + k + len / 2] * ci;
                    float xi = re_[i + k + len / 2] * ci + im_[i + k + len / 2] * cr;
                    float ur = re_[i + k], ui = im_[i + k];
                    re_[i + k] = ur + xr;           im_[i + k] = ui + xi;
                    re_[i + k + len / 2] = ur - xr; im_[i + k + len / 2] = ui - xi;
                    float ncr = cr * wr - ci * wi;
                    ci = cr * wi + ci * wr;
                    cr = ncr;
                }
            }
        }
    }

    float re_[N];
    float im_[N];
};

#endif  // DISPLAY_ENABLE
