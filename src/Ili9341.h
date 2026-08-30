#pragma once
#include <Arduino.h>
#include "config.h"
#include "Font5x7.h"

#if DISPLAY_HAS_ILI9341

#include <SPI.h>
#include "TftColors.h"

// ============================================================================
// Pilote minimal ILI9341 (TFT 320x240, RGB565) en SPI MATÉRIEL.
//
// SCK / MOSI sur l'ESP (partagés avec l'ILI9225), MISO obligatoire (lecture de
// l'ID + tactile XPT2046), CS + DC sur GPIO, RST optionnel (sinon câblé).
// La rotation est faite par le contrôleur (MADCTL) : aucun remap de pixel.
// Même API de dessin que Ili9225 (fillRect / blit / text / drawRect...).
// ============================================================================

class Ili9341 {
public:
    // Lecture de l'ID (0xD3 -> 00 93 41). Utilisée pour la détection AVANT
    // begin() : ouvre le SPI à basse vitesse, le referme.
    // `spi` doit être DÉJÀ initialisé (bus partagé, voir FaceDisplay).
    static uint32_t probeId(SPIClass* spi) {
        pinMode(DISPLAY_CS_PIN, OUTPUT);  digitalWrite(DISPLAY_CS_PIN, HIGH);
        pinMode(DISPLAY_DC_PIN, OUTPUT);  digitalWrite(DISPLAY_DC_PIN, HIGH);
        if (DISPLAY_RST_PIN >= 0) {
            pinMode(DISPLAY_RST_PIN, OUTPUT);
            digitalWrite(DISPLAY_RST_PIN, LOW);  delay(10);
            digitalWrite(DISPLAY_RST_PIN, HIGH); delay(120);
        }
        spi->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
        digitalWrite(DISPLAY_CS_PIN, LOW);
        digitalWrite(DISPLAY_DC_PIN, LOW);  spi->transfer(0xD3);
        digitalWrite(DISPLAY_DC_PIN, HIGH);
        spi->transfer(0);                                  // octet muet
        uint8_t a = spi->transfer(0), b = spi->transfer(0), c = spi->transfer(0);
        digitalWrite(DISPLAY_CS_PIN, HIGH);
        spi->endTransaction();
        return ((uint32_t)a << 16) | ((uint32_t)b << 8) | c;   // attendu 0x009341
    }

    void begin(SPIClass* spi) {
        spi_ = spi;                                        // bus partagé, déjà begin()
        pinMode(DISPLAY_CS_PIN, OUTPUT); digitalWrite(DISPLAY_CS_PIN, HIGH);
        pinMode(DISPLAY_DC_PIN, OUTPUT); digitalWrite(DISPLAY_DC_PIN, HIGH);
        if (DISPLAY_LED_PIN >= 0) { pinMode(DISPLAY_LED_PIN, OUTPUT); digitalWrite(DISPLAY_LED_PIN, HIGH); }
        if (DISPLAY_RST_PIN >= 0) {
            pinMode(DISPLAY_RST_PIN, OUTPUT);
            digitalWrite(DISPLAY_RST_PIN, HIGH); delay(5);
            digitalWrite(DISPLAY_RST_PIN, LOW);  delay(15);
            digitalWrite(DISPLAY_RST_PIN, HIGH); delay(120);
        }
        rot_ = ILI9341_ROTATION & 3;
        initSeq();
        Serial.printf("[ILI9341] init %dx%d rot %d\n", width(), height(), rot_);
    }

    int width()  const { return (rot_ & 1) ? 320 : 240; }
    int height() const { return (rot_ & 1) ? 240 : 320; }
    SPIClass* spi() const { return spi_; }   // bus partagé avec le tactile

    void fillScreen(uint16_t c) { fillRect(0, 0, width(), height(), c); }

    void fillRect(int x, int y, int w, int h, uint16_t c) {
        if (w <= 0 || h <= 0 || x >= width() || y >= height()) return;
        if (x < 0) { w += x; x = 0; }
        if (y < 0) { h += y; y = 0; }
        if (x + w > width())  w = width() - x;
        if (y + h > height()) h = height() - y;
        if (w <= 0 || h <= 0) return;
        setWindow(x, y, x + w - 1, y + h - 1);
        beginData();
        uint32_t n = (uint32_t)w * h;
        while (n--) spi_->write16(c);
        endData();
    }

    void hLine(int x, int y, int w, uint16_t c) { fillRect(x, y, w, 1, c); }
    void vLine(int x, int y, int h, uint16_t c) { fillRect(x, y, 1, h, c); }
    void drawRect(int x, int y, int w, int h, uint16_t c) {
        hLine(x, y, w, c); hLine(x, y + h - 1, w, c);
        vLine(x, y, h, c); vLine(x + w - 1, y, h, c);
    }

    void blit(int x, int y, int w, int h, const uint16_t* px) {
        if (w <= 0 || h <= 0) return;
        setWindow(x, y, x + w - 1, y + h - 1);
        beginData();
        uint32_t n = (uint32_t)w * h;
        for (uint32_t i = 0; i < n; i++) spi_->write16(px[i]);
        endData();
    }

    void text(int x, int y, const char* s, uint16_t fg, uint16_t bg, int scale = 1) {
        for (; *s; s++, x += 6 * scale) glyph(x, y, *s, fg, bg, scale);
    }
    void textRight(int xr, int y, const char* s, uint16_t fg, uint16_t bg, int scale = 1) {
        text(xr - (int)strlen(s) * 6 * scale, y, s, fg, bg, scale);
    }
    int textWidth(const char* s, int scale = 1) const { return (int)strlen(s) * 6 * scale; }
    static const int CHAR_H = 8;

    void glyph(int x, int y, char ch, uint16_t fg, uint16_t bg, int sc) {
        const int w = 6 * sc, h = 8 * sc;
        if (x + w <= 0 || y + h <= 0 || x >= width() || y >= height()) return;
        setWindow(x, y, x + w - 1, y + h - 1);
        beginData();
        for (int ry = 0; ry < h; ry++) {
            int gy = ry / sc;
            for (int rx = 0; rx < w; rx++) {
                int gx = rx / sc;
                bool on = (gx < 5 && gy < 7) && (font5x7Col(ch, gx) & (1 << gy));
                spi_->write16(on ? fg : bg);
            }
        }
        endData();
    }

private:
    void cmd(uint8_t c) {
        spi_->beginTransaction(SPISettings(ILI9341_SPI_HZ, MSBFIRST, SPI_MODE0));
        digitalWrite(DISPLAY_CS_PIN, LOW);
        digitalWrite(DISPLAY_DC_PIN, LOW);
        spi_->write(c);
        digitalWrite(DISPLAY_DC_PIN, HIGH);
        digitalWrite(DISPLAY_CS_PIN, HIGH);
        spi_->endTransaction();
    }
    void cmdData(uint8_t c, const uint8_t* d, size_t n) {
        spi_->beginTransaction(SPISettings(ILI9341_SPI_HZ, MSBFIRST, SPI_MODE0));
        digitalWrite(DISPLAY_CS_PIN, LOW);
        digitalWrite(DISPLAY_DC_PIN, LOW);  spi_->write(c);
        digitalWrite(DISPLAY_DC_PIN, HIGH); for (size_t i = 0; i < n; i++) spi_->write(d[i]);
        digitalWrite(DISPLAY_CS_PIN, HIGH);
        spi_->endTransaction();
    }
    // Ouvre la fenêtre GRAM ; laisse CS bas + transaction ouverte (beginData).
    void setWindow(int x0, int y0, int x1, int y1) {
        uint8_t ca[4] = { (uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1 };
        uint8_t pa[4] = { (uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint8_t)y1 };
        cmdData(0x2A, ca, 4);
        cmdData(0x2B, pa, 4);
        cmd(0x2C);   // RAMWR
    }
    void beginData() {
        spi_->beginTransaction(SPISettings(ILI9341_SPI_HZ, MSBFIRST, SPI_MODE0));
        digitalWrite(DISPLAY_CS_PIN, LOW);
        digitalWrite(DISPLAY_DC_PIN, HIGH);
    }
    void endData() {
        digitalWrite(DISPLAY_CS_PIN, HIGH);
        spi_->endTransaction();
    }

    void initSeq() {
        cmd(0x01); delay(120);                              // SWRESET
        { uint8_t d[] = {0x23};            cmdData(0xC0, d, 1); }   // PWCTR1
        { uint8_t d[] = {0x10};            cmdData(0xC1, d, 1); }   // PWCTR2
        { uint8_t d[] = {0x3E, 0x28};      cmdData(0xC5, d, 2); }   // VMCTR1
        { uint8_t d[] = {0x86};            cmdData(0xC7, d, 1); }   // VMCTR2
        { uint8_t d[] = {0x55};            cmdData(0x3A, d, 1); }   // PIXFMT 16 bit
        { uint8_t d[] = {0x00, 0x18};      cmdData(0xB1, d, 2); }   // FRMCTR1
        { uint8_t d[] = {0x08, 0x82, 0x27};cmdData(0xB6, d, 3); }   // DFUNCTR
        setMadctl();
        cmd(0x11); delay(120);                              // SLPOUT
        cmd(0x29);                                          // DISPON
        if (ILI9341_INVERT) cmd(0x21);
    }
    void setMadctl() {
        // MY MX MV ML BGR MH .. ..   (BGR=1 pour ces dalles)
        uint8_t m;
        switch (rot_) {
            default:
            case 0: m = 0x48; break;   // 240x320
            case 1: m = 0x28; break;   // 320x240 (paysage)
            case 2: m = 0x88; break;
            case 3: m = 0xE8; break;
        }
        cmdData(0x36, &m, 1);
    }

    SPIClass* spi_ = nullptr;
    uint8_t   rot_ = 1;
};

#endif  // DISPLAY_HAS_ILI9341
