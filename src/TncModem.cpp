#include "TncModem.h"
#include <new>

#if TNC_ENABLE

TncModem* TncModem::self_ = nullptr;

bool TncModem::begin() {
    self_ = this;
    if (!demod_) demod_ = new (std::nothrow)
        AfskDemodulator(AFSK_SAMPLE_RATE, 2, &TncModem::demodTrampoline);  // décim. 2 comme kv4p-ht
    if (!mod_)   mod_   = new (std::nothrow)
        AfskModulator(AFSK_SAMPLE_RATE, &TncModem::modTrampoline);
    return demod_ && mod_;
}

void TncModem::end() {
    txBusy_ = false;
    if (demod_) { delete demod_; demod_ = nullptr; }
    if (mod_)   { delete mod_;   mod_   = nullptr; }
}

void TncModem::demodTrampoline(const uint8_t* frame, size_t len) {
    if (self_ && self_->rxCb_ && len) self_->rxCb_(frame, len);
}

void TncModem::modTrampoline(const float* s, size_t n) {
    TncModem* m = self_;
    if (!m || !m->txAudioCb_ || !n) return;
    // float [-1..1] -> int16
    static int16_t buf[256];
    size_t i = 0;
    while (i < n) {
        size_t k = 0;
        for (; k < 256 && i < n; k++, i++) {
            float v = s[i] * 30000.0f;
            buf[k] = v > 32767.f ? 32767 : (v < -32768.f ? -32768 : (int16_t)v);
        }
        m->txAudioCb_(buf, k);
    }
}

void TncModem::feedRxAudio(const int16_t* pcm, size_t n) {
    if (demod_ && !txBusy_) demod_->processSamples(pcm, n);
}

void TncModem::txAx25(const uint8_t* frame, size_t len) {
    if (!mod_ || !frame || !len) return;
    txBusy_ = true;
    float chunk[128];
    // La lib appelle modTrampoline au fil de la génération (callback bloquant
    // côté appelant -> cadençage temps réel via le DAC).
    mod_->modulate(frame, len, chunk, 128, 250.0f /*lead ms*/, 60.0f /*tail ms*/);
    txBusy_ = false;
    if (txDoneCb_) txDoneCb_();
}

#endif  // TNC_ENABLE
