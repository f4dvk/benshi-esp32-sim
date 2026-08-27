#pragma once
#include <Arduino.h>
#include <vector>

// ============================================================================
// BitWriter / BitReader
// Les structures du protocole Benshi (RfCh, Settings, DevInfo, StatusExt...)
// sont des "bitfields" : les champs ne sont PAS alignés sur l'octet
// (ex: tx_freq sur 30 bits). On empile donc les bits en MSB-first, comme le
// fait la lib Python "bydantic" utilisée par benlink.
//
// IMPORTANT (transparence) : la doc de référence ne liste que les champs
// "principaux" de chaque structure ("...and many more"). Les tailles totales
// exactes en octets des structures Settings/DevInfo complètes ne sont donc
// PAS garanties ici. Si HTCommander rejette une réponse ou se comporte
// bizarrement après READ_SETTINGS/GET_DEV_INFO, c'est probablement là qu'il
// faut ajuster (voir README.md - section "Ajuster le protocole").
// ============================================================================

class BitWriter {
public:
    void writeBits(uint32_t value, uint8_t nbits) {
        for (int i = nbits - 1; i >= 0; i--) {
            bool bit = (value >> i) & 0x1;
            pushBit(bit);
        }
    }

    // Ecrit une chaine ASCII sur nbytes octets (padding 0x00 si plus court)
    void writeString(const char* str, uint8_t nbytes) {
        size_t len = strlen(str);
        for (uint8_t i = 0; i < nbytes; i++) {
            uint8_t c = (i < len) ? (uint8_t)str[i] : 0x00;
            writeBits(c, 8);
        }
    }

    std::vector<uint8_t>& bytes() { return buf_; }

private:
    void pushBit(bool bit) {
        if (bitPos_ == 0) buf_.push_back(0);
        if (bit) buf_.back() |= (1 << (7 - bitPos_));
        bitPos_ = (bitPos_ + 1) % 8;
    }
    std::vector<uint8_t> buf_;
    uint8_t bitPos_ = 0;
};

class BitReader {
public:
    BitReader(const uint8_t* data, size_t len) : data_(data), len_(len) {}

    uint32_t readBits(uint8_t nbits) {
        uint32_t v = 0;
        for (uint8_t i = 0; i < nbits; i++) {
            v = (v << 1) | readBit();
        }
        return v;
    }

    bool eof() const { return bytePos_ >= len_; }

private:
    bool readBit() {
        if (bytePos_ >= len_) return 0;
        bool bit = (data_[bytePos_] >> (7 - bitPos_)) & 0x1;
        bitPos_++;
        if (bitPos_ == 8) { bitPos_ = 0; bytePos_++; }
        return bit;
    }
    const uint8_t* data_;
    size_t len_;
    size_t bytePos_ = 0;
    uint8_t bitPos_ = 0;
};
