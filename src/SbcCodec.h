#pragma once
#include <stdint.h>
#include <stddef.h>

// ============================================================================
// Enrobage C++ minimal autour du codec SBC de Bluedroid (symboles deja
// presents dans libbt.a - voir lib/sbc/README.md).
//
// Cet en-tete n'inclut AUCUN en-tete vendeur : l'encodeur et le decodeur
// definissent des macros qui se chevauchent (SBC_MONO, SBC_LOUDNESS, ...),
// donc chaque cote est compile dans sa propre unite (SbcEncoderImpl.cpp /
// SbcDecoderImpl.cpp) et n'expose ici qu'une API propre.
//
// Format fige pour le flux Benshi : mono, 8 sous-bandes, 16 blocs,
// allocation "loudness". Seul le bitpool est parametrable.
// ============================================================================

namespace sbc {

// Geometrie d'une trame pour ce format (mono / 8 sous-bandes / 16 blocs).
static constexpr int   kSubbands        = 8;
static constexpr int   kBlocks          = 16;
static constexpr size_t kSamplesPerFrame = kSubbands * kBlocks;      // 128
static constexpr size_t kPcmBytesPerFrame = kSamplesPerFrame * 2;    // 256
// Majorant large d'une trame SBC encodee (bitpool <= 64 : 4 + 4 + 128 = 136).
static constexpr size_t kMaxSbcFrameLen = 160;

// ----------------------------------------------------------------------------
class Encoder {
public:
    ~Encoder();
    // bitpool : cf. AUDIO_SBC_BITPOOL (40 pour le flux Benshi standard).
    bool begin(int bitpool);
    // Encode EXACTEMENT kSamplesPerFrame echantillons int16 mono.
    // Retourne le nombre d'octets ecrits dans `out`, 0 si erreur.
    size_t encodeFrame(const int16_t* pcm, uint8_t* out, size_t outCap);
    int  frameLen() const { return frameLen_; }
    bool ready()    const { return params_ != nullptr; }

private:
    void* params_ = nullptr;   // SBC_ENC_PARAMS*
    int   frameLen_ = 0;
    int   bitpool_ = 0;
};

// ----------------------------------------------------------------------------
class Decoder {
public:
    ~Decoder();
    bool begin();
    void reset();
    // Decode une (ou plusieurs) trame(s) SBC concatenee(s). `*inLen` est mis a
    // jour avec le nombre d'octets NON consommes (0 = tout consomme).
    // Retourne le nombre d'echantillons int16 mono ecrits dans `pcmOut`.
    size_t decode(const uint8_t* in, size_t* inLen, int16_t* pcmOut, size_t pcmOutCap);
    bool ready() const { return ctx_ != nullptr; }

private:
    void*     ctx_  = nullptr;  // OI_CODEC_SBC_DECODER_CONTEXT*
    uint32_t* data_ = nullptr;  // buffer de travail du decodeur
    size_t    dataWords_ = 0;
};

} // namespace sbc
