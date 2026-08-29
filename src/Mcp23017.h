#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// ============================================================================
// Pilote minimal MCP23017 (expandeur GPIO 16 bits sur I2C).
//
// Utilisé ici pour porter les lignes de l'écran ILI9225 (SPI bit-bangé, voir
// Ili9225.h).
//
// MODE BANK = 1 + SEQOP = 1 : le pointeur de registre reste FIGÉ sur GPIOA. On
// peut alors envoyer une rafale d'octets qui tombent tous dans GPIOA en une
// seule transaction I2C -> bit-bang rapide.
// (En BANK = 0, SEQOP = 1 fait BASCULER le pointeur entre GPIOA et GPIOB, ce
//  qui n'est PAS ce qu'on veut.)
//
// Carte des registres en BANK = 1 : IODIRA=0x00, IODIRB=0x10, IOCON=0x05,
// GPIOA=0x09, GPIOB=0x19.
// ============================================================================

#if DISPLAY_ENABLE

class Mcp23017 {
public:
    static const size_t kWireBuf = 1024;   // cf Wire.setBufferSize() de l'appelant

    bool begin(TwoWire& wire, uint8_t addr, size_t wireBuf = kWireBuf) {
        wire_ = &wire;
        addr_ = addr;
        chunk_ = wireBuf > 8 ? wireBuf - 4 : 4;

        wire_->beginTransmission(addr_);
        if (wire_->endTransmission() != 0) return false;

        // Passe en BANK=1 : l'écriture se fait à l'ADRESSE COURANTE de IOCON
        // (0x0A tant que BANK=0). bit7 BANK=1, bit5 SEQOP=1.
        writeReg(0x0A, 0xA0);
        // À partir d'ici : BANK=1.
        writeReg(REG_IODIRA, 0x00);   // port A = sorties
        writeReg(REG_IODIRB, 0x00);   // port B = sorties
        olatA_ = olatB_ = 0x00;
        writeReg(REG_GPIOA, 0x00);
        writeReg(REG_GPIOB, 0x00);
        return true;
    }

    void writeA(uint8_t v) { olatA_ = v; writeReg(REG_GPIOA, v); }
    void writeB(uint8_t v) { olatB_ = v; writeReg(REG_GPIOB, v); }
    uint8_t imageA() const { return olatA_; }

    // pin 0..7 = GPA0..7, 8..15 = GPB0..7.
    void digitalWrite(uint8_t pin, bool level) {
        if (pin < 8) {
            uint8_t v = level ? (olatA_ | (1 << pin)) : (olatA_ & ~(1 << pin));
            if (v != olatA_) writeA(v);
        } else {
            pin -= 8;
            uint8_t v = level ? (olatB_ | (1 << pin)) : (olatB_ & ~(1 << pin));
            if (v != olatB_) writeB(v);
        }
    }

    // Rafale d'états sur GPIOA (BANK=1, SEQOP=1 -> pointeur figé sur GPIOA).
    void burstA(const uint8_t* buf, size_t n) {
        size_t off = 0;
        while (off < n) {
            size_t k = (n - off) < chunk_ ? (n - off) : chunk_;
            wire_->beginTransmission(addr_);
            wire_->write(REG_GPIOA);
            wire_->write(buf + off, k);
            wire_->endTransmission();
            off += k;
        }
        if (n) olatA_ = buf[n - 1];
    }

    uint8_t readReg(uint8_t reg) {
        wire_->beginTransmission(addr_);
        wire_->write(reg);
        wire_->endTransmission(false);
        wire_->requestFrom((int)addr_, 1);
        return wire_->available() ? wire_->read() : 0xFF;
    }

    // Adresses en BANK = 1
    static const uint8_t REG_IODIRA = 0x00, REG_IODIRB = 0x10;
    static const uint8_t REG_IOCON  = 0x05;
    static const uint8_t REG_GPIOA  = 0x09, REG_GPIOB  = 0x19;

private:
    void writeReg(uint8_t reg, uint8_t val) {
        wire_->beginTransmission(addr_);
        wire_->write(reg);
        wire_->write(val);
        wire_->endTransmission();
    }

    TwoWire* wire_ = nullptr;
    uint8_t  addr_ = 0x20;
    uint8_t  olatA_ = 0, olatB_ = 0;
    size_t   chunk_ = 120;
};

#endif  // DISPLAY_ENABLE
