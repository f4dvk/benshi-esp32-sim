// ============================================================================
// Implementation de sbc::Decoder - unite isolee : c'est la SEULE ou l'on
// inclut oi_codec_sbc.h (macros en conflit avec l'encodeur).
// ============================================================================
#include "SbcCodec.h"

#include <string.h>
#include <stdlib.h>

extern "C" {
#include "oi_codec_sbc.h"   // lib/sbc/include (symboles dans libbt.a)
}

namespace sbc {

Decoder::~Decoder() {
    if (ctx_)  { free(ctx_);  ctx_  = nullptr; }
    if (data_) { free(data_); data_ = nullptr; }
}

bool Decoder::begin() {
    if (!ctx_) {
        ctx_ = calloc(1, sizeof(OI_CODEC_SBC_DECODER_CONTEXT));
        if (!ctx_) return false;
    }
    if (!data_) {
        dataWords_ = sizeof(OI_CODEC_SBC_CODEC_DATA_MONO) / sizeof(uint32_t);
        data_ = static_cast<uint32_t*>(calloc(dataWords_, sizeof(uint32_t)));
        if (!data_) { free(ctx_); ctx_ = nullptr; return false; }
    }
    reset();
    return ready();
}

void Decoder::reset() {
    if (!ctx_ || !data_) return;
    OI_STATUS st = OI_CODEC_SBC_DecoderReset(
        static_cast<OI_CODEC_SBC_DECODER_CONTEXT*>(ctx_),
        reinterpret_cast<OI_UINT32*>(data_),
        static_cast<OI_UINT32>(dataWords_ * sizeof(uint32_t)),
        /*maxChannels*/ 1,
        /*pcmStride*/   1,
        /*enhanced*/    FALSE,
        /*msbc_enable*/ FALSE);
    if (!OI_SUCCESS(st)) {
        free(ctx_);  ctx_  = nullptr;
        free(data_); data_ = nullptr;
    }
}

size_t Decoder::decode(const uint8_t* in, size_t* inLen,
                       int16_t* pcmOut, size_t pcmOutCap) {
    if (!ready() || !in || !inLen) return 0;

    const OI_BYTE* frameData  = reinterpret_cast<const OI_BYTE*>(in);
    OI_UINT32      frameBytes = static_cast<OI_UINT32>(*inLen);
    size_t         totalSamples = 0;

    while (frameBytes > 0) {
        OI_UINT32 pcmBytes = static_cast<OI_UINT32>(
            (pcmOutCap - totalSamples) * sizeof(int16_t));
        if (pcmBytes < kPcmBytesPerFrame) break;   // plus de place en sortie

        const OI_BYTE* before      = frameData;
        OI_UINT32      beforeBytes = frameBytes;

        OI_STATUS st = OI_CODEC_SBC_DecodeFrame(
            static_cast<OI_CODEC_SBC_DECODER_CONTEXT*>(ctx_),
            &frameData, &frameBytes,
            reinterpret_cast<OI_INT16*>(pcmOut + totalSamples),
            &pcmBytes);

        if (st == OI_STATUS_SUCCESS || st == OI_CODEC_SBC_PARTIAL_DECODE) {
            totalSamples += pcmBytes / sizeof(int16_t);
            if (frameData == before && frameBytes == beforeBytes && pcmBytes == 0) {
                break;                              // garde anti-blocage
            }
            continue;
        }
        if (st == OI_CODEC_SBC_NOT_ENOUGH_HEADER_DATA ||
            st == OI_CODEC_SBC_NOT_ENOUGH_BODY_DATA   ||
            st == OI_CODEC_SBC_NOT_ENOUGH_AUDIO_DATA) {
            frameData  = before;                    // trame incomplete : on garde
            frameBytes = beforeBytes;               // le reste pour le prochain appel
            break;
        }
        // NO_SYNCWORD / CHECKSUM_MISMATCH : on jette 1 octet et on resynchronise.
        frameData  = before + 1;
        frameBytes = beforeBytes - 1;
    }

    *inLen = frameBytes;   // reste non consomme
    return totalSamples;
}

} // namespace sbc
