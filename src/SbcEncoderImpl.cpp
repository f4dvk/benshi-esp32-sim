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

// PIEGE : SBC_Encoder_Init() de Bluedroid, pour un flux mono/dual, IGNORE
// s16BitPool et le RECALCULE a partir de u16BitRate (kbps). Avec u16BitRate=0
// on obtient bitpool = 0 -> trames SBC de ~9 octets = inexploitables.
// On calcule donc le bitrate qui redonne le bitpool voulu, puis on re-force
// s16BitPool apres l'init et avant chaque encodage.
static uint16_t bitrateForBitpool(int bitpool, int freqHz, int subbands,
                                  int blocks, int channels) {
    // s16Bitpool = subbands*bitrate*1000/(freq*ch) - ((32/ch + 4*subbands)/blocks)
    int corr = ((32 / channels) + 4 * subbands) / blocks;
    long r = (long)(bitpool + corr) * freqHz * channels;
    r /= (long)subbands * 1000;
    if (r < 1) r = 1;
    if (r > 0xFFFF) r = 0xFFFF;
    return (uint16_t)r;
}

Encoder::~Encoder() {
    if (params_) { free(params_); params_ = nullptr; }
}

bool Encoder::begin(int bitpool) {
    if (!params_) {
        params_ = calloc(1, sizeof(SBC_ENC_PARAMS));
        if (!params_) return false;
    }
    bitpool_ = bitpool;
    SBC_ENC_PARAMS* p = static_cast<SBC_ENC_PARAMS*>(params_);
    memset(p, 0, sizeof(*p));

    p->s16SamplingFreq     = SBC_sf32000;
    p->s16ChannelMode      = SBC_MONO;
    p->s16NumOfSubBands    = kSubbands;
    p->s16NumOfChannels    = 1;
    p->s16NumOfBlocks      = kBlocks;
    p->s16AllocationMethod = SBC_LOUDNESS;
    p->s16BitPool          = bitpool;
    p->u16BitRate          = bitrateForBitpool(bitpool, 32000, kSubbands, kBlocks, 1);
    p->sbc_mode            = SBC_MODE_STD;
    p->u8NumPacketToEncode = 1;

    SBC_Encoder_Init(p);
    p->s16BitPool = bitpool;             // re-force apres le recalcul interne

    // Longueur d'une trame SBC mono = 4 (header) + ceil(4*subbands/8)
    // (scale factors) + ceil(blocks*bitpool/8) (echantillons).
    frameLen_ = 4 + ((4 * kSubbands + 7) / 8) + ((kBlocks * bitpool + 7) / 8);
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
    p->s16BitPool          = bitpool_;   // garde-fou

    SBC_Encoder(p);

    return p->u16PacketLength ? p->u16PacketLength : static_cast<size_t>(frameLen_);
}

} // namespace sbc
