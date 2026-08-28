#pragma once
#include <Arduino.h>
#include <functional>
#include <stdint.h>
#include "config.h"

// ============================================================================
// TNC AX.25 / AFSK 1200 pour le canal données APRS.
//
// Utilise dkaukov/esp32-afsk (démod : bandpass -> I/Q -> FM demod -> PLL ->
// NRZI -> HDLC -> CRC ; mod : HDLC + bit-stuffing + FCS + tonalités 1200/2200).
// Mêmes libs que kv4p-ht. GPL-3.
//
//   RX : feedRxAudio(pcm) <- ADC du poste  ->  onRxFrame(ax25, len)  (FCS déjà
//        vérifié et retiré) -> notification dataRxd vers HTCommander.
//   TX : txAx25(ax25, len) <- HT_SEND_DATA réassemblé  ->  onTxAudio(pcm) vers
//        le DAC (PTT géré par l'appelant), puis onTxDone().
//
// Actif uniquement quand la radio est sur le canal "APRS" (voir
// TNC_CHANNEL_NAME) ; sinon le pont audio fonctionne normalement (phonie SBC).
// ============================================================================

#if TNC_ENABLE

extern "C++" {
#include <AfskDemodulator.h>
#include <AfskModulator.h>
}

class TncModem {
public:
    using RxFrameFn = std::function<void(const uint8_t*, size_t)>;
    using TxAudioFn = std::function<void(const int16_t*, size_t)>;
    using TxDoneFn  = std::function<void()>;

    void onRxFrame(RxFrameFn f) { rxCb_ = std::move(f); }
    void onTxAudio(TxAudioFn f) { txAudioCb_ = std::move(f); }
    void onTxDone(TxDoneFn f)   { txDoneCb_ = std::move(f); }

    bool begin();
    void feedRxAudio(const int16_t* pcm, size_t n);
    void txAx25(const uint8_t* frame, size_t len);
    bool transmitting() const { return txBusy_; }

private:
    static TncModem* self_;
    static void demodTrampoline(const uint8_t* frame, size_t len);
    static void modTrampoline(const float* s, size_t n);

    AfskDemodulator* demod_ = nullptr;
    AfskModulator*   mod_    = nullptr;

    RxFrameFn rxCb_;
    TxAudioFn txAudioCb_;
    TxDoneFn  txDoneCb_;
    volatile bool txBusy_ = false;
};

#endif  // TNC_ENABLE
