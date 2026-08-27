// ============================================================================
// Implementation de sbc::Encoder - unite isolee : c'est la SEULE ou l'on
// inclut sbc_encoder.h (macros en conflit avec le decodeur).
// ============================================================================
#include "SbcCodec.h"

#include <string.h>
#include <stdlib.h>

extern "C" {
#include "sbc_encoder.h"   // lib/sbc/include (symboles dans libbt.a)
}

namespace sbc {

Encoder::~Encoder() {
    if (params_) { free(params_); params_ = nullptr; }
}

bool Encoder::begin(int bitpool) {
    if (!params_) {
        params_ = calloc(1, sizeof(SBC_ENC_PARAMS));
        if (!params_) return false;
    }
    SBC_ENC_PARAMS* p = static_cast<SBC_ENC_PARAMS*>(params_);
    memset(p, 0, sizeof(*p));

    p->s16SamplingFreq     = SBC_sf32000;
    p->s16ChannelMode      = SBC_MONO;
    p->s16NumOfSubBands    = kSubbands;
    p->s16NumOfChannels    = 1;
    p->s16NumOfBlocks      = kBlocks;
    p->s16AllocationMethod = SBC_LOUDNESS;
    p->s16BitPool          = bitpool;
    p->sbc_mode            = SBC_MODE_STD;
    p->u8NumPacketToEncode = 1;

    SBC_Encoder_Init(p);

    // Repli si u16PacketLength n'est pas renseigne par SBC_Encoder_Init :
    // longueur d'une trame SBC mono = 4 (header) + ceil(4*subbands/8)
    // (scale factors) + ceil(blocks*bitpool/8) (echantillons).
    frameLen_ = p->u16PacketLength
                    ? p->u16PacketLength
                    : (4
                       + ((4 * kSubbands + 7) / 8)
                       + ((kBlocks * bitpool + 7) / 8));
    return true;
}

size_t Encoder::encodeFrame(const int16_t* pcm, uint8_t* out, size_t outCap) {
    if (!params_ || outCap < static_cast<size_t>(frameLen_)) return 0;
    SBC_ENC_PARAMS* p = static_cast<SBC_ENC_PARAMS*>(params_);

    // Avec la config par defaut (SBC_NO_PCM_CPY_OPTION==FALSE), l'encodeur lit
    // toujours depuis son buffer interne as16PcmBuffer : on y recopie le PCM.
    memcpy(p->as16PcmBuffer, pcm, kSamplesPerFrame * sizeof(int16_t));
    p->pu8Packet           = out;
    p->u8NumPacketToEncode = 1;

    SBC_Encoder(p);

    return p->u16PacketLength ? p->u16PacketLength : static_cast<size_t>(frameLen_);
}

} // namespace sbc
