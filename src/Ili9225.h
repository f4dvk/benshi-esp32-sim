#pragma once
#include <Arduino.h>
#include "config.h"
#include "Font5x7.h"
#include "TftColors.h"

#if DISPLAY_HAS_ILI9225

#include <SPI.h>

// ============================================================================
// Pilote minimal ILI9225 (TFT 176x220, RGB565) en SPI MATÉRIEL DIRECT.
//
// SCK / MOSI / MISO + CS + DC(RS) + RST sur des GPIO de l'ESP (plus de
// MCP23017). Registres 16 bits (index + valeur). La rotation est faite en
// logiciel (toNative/fromNative) car l'ILI9225 n'a pas de MADCTL complet.
// Même API de dessin que Ili9341.
// ============================================================================

class Ili9225 {
public:
    // Lecture du "device code" (registre 0x00 -> 0x9225) pour la détection.
    // `spi` doit être DÉJÀ initialisé (bus partagé, voir FaceDisplay).
    static uint32_t probeId(SPIClass* spi) {
        pinMode(DISPLAY_CS_PIN, OUTPUT); digitalWrite(DISPLAY_CS_PIN, HIGH);
        pinMode(DISPLAY_DC_PIN, OUTPUT); digitalWrite(DISPLAY_DC_PIN, HIGH);
        if (DISPLAY_RST_PIN >= 0) {
            pinMode(DISPLAY_RST_PIN, OUTPUT);
            digitalWrite(DISPLAY_RST_PIN, LOW);  delay(10);
            digitalWrite(DISPLAY_RST_PIN, HIGH); delay(50);
        }
        spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        digitalWrite(DISPLAY_CS_PIN, LOW);
        digitalWrite(DISPLAY_DC_PIN, LOW);  spi->transfer16(0x0000);   // index reg 0
        digitalWrite(DISPLAY_DC_PIN, HIGH);
        spi->transfer(0);                                              // octet muet
        uint16_t id = spi->transfer16(0x0000);
        digitalWrite(DISPLAY_CS_PIN, HIGH);
        spi->endTransaction();
        return id;                                                    // attendu 0x9225
    }

    void begin(SPIClass* spi) {
        spi_ = spi;                                     // bus partagé, déjà begin()
        pinMode(DISPLAY_CS_PIN, OUTPUT); digitalWrite(DISPLAY_CS_PIN, HIGH);
        pinMode(DISPLAY_DC_PIN, OUTPUT); digitalWrite(DISPLAY_DC_PIN, HIGH);
        if (DISPLAY_LED_PIN >= 0) { pinMode(DISPLAY_LED_PIN, OUTPUT); digitalWrite(DISPLAY_LED_PIN, HIGH); }
        rot_ = ILI9225_ROTATION & 3;
        if (DISPLAY_RST_PIN >= 0) {
            pinMode(DISPLAY_RST_PIN, OUTPUT);
            digitalWrite(DISPLAY_RST_PIN, HIGH); delay(10);
            digitalWrite(DISPLAY_RST_PIN, LOW);  delay(20);
            digitalWrite(DISPLAY_RST_PIN, HIGH); delay(150);
        }
        initSeq();
        Serial.printf("[ILI9225] init %dx%d rot %d @ %d Hz\n",
                      width(), height(), rot_, ILI9225_SPI_HZ);
        fillRect(0, 0, 16, 16, C_RED);
        fillRect(width() - 16, height() - 16, 16, 16, C_GREEN);
        Serial.println("[ILI9225] carres test : ROUGE haut-gauche + VERT bas-droite ?");
    }

    int width()  const { return (rot_ & 1) ? 220 : 176; }
    int height() const { return (rot_ & 1) ? 176 : 220; }

    void fillScreen(uint16_t c) { fillRect(0, 0, width(), height(), c); }

    void fillRect(int x, int y, int w, int h, uint16_t c) {
        if (w <= 0 || h <= 0) return;
        int nx0, ny0, nx1, ny1;
        nativeBox(x, y, w, h, nx0, ny0, nx1, ny1);
        setWindow(nx0, ny0, nx1, ny1);
        gramStart();
        uint32_t n = (uint32_t)(nx1 - nx0 + 1) * (ny1 - ny0 + 1);
        while (n--) spi_->write16(c);
        gramEnd();
    }

    void hLine(int x, int y, int w, uint16_t c) { fillRect(x, y, w, 1, c); }
    void vLine(int x, int y, int h, uint16_t c) { fillRect(x, y, 1, h, c); }
    void drawRect(int x, int y, int w, int h, uint16_t c) {
        hLine(x, y, w, c); hLine(x, y + h - 1, w, c);
        vLine(x, y, h, c); vLine(x + w - 1, y, h, c);
    }

    // Copie un bloc de pixels (orientation logique, ligne par ligne).
    void blit(int x, int y, int w, int h, const uint16_t* px) {
        if (w <= 0 || h <= 0) return;
        int nx0, ny0, nx1, ny1;
        nativeBox(x, y, w, h, nx0, ny0, nx1, ny1);
        setWindow(nx0, ny0, nx1, ny1);
        gramStart();
        for (int ny = ny0; ny <= ny1; ny++)
            for (int nx = nx0; nx <= nx1; nx++) {
                int lx, ly; fromNative(nx, ny, lx, ly);
                spi_->write16(px[(ly - y) * w + (lx - x)]);
            }
        gramEnd();
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
        int w = 6 * sc, h = 8 * sc;
        if (x + w <= 0 || y + h <= 0 || x >= width() || y >= height()) return;
        int nx0, ny0, nx1, ny1;
        nativeBox(x, y, w, h, nx0, ny0, nx1, ny1);
        setWindow(nx0, ny0, nx1, ny1);
        gramStart();
        for (int ny = ny0; ny <= ny1; ny++)
            for (int nx = nx0; nx <= nx1; nx++) {
                int lx, ly; fromNative(nx, ny, lx, ly);
                int gx = (lx - x) / sc, gy = (ly - y) / sc;
                bool on = (gx >= 0 && gx < 5 && gy >= 0 && gy < 7) &&
                          (font5x7Col(ch, gx) & (1 << gy));
                spi_->write16(on ? fg : bg);
            }
        gramEnd();
    }

private:
    void beginSpi() { spi_->beginTransaction(SPISettings(ILI9225_SPI_HZ, MSBFIRST, SPI_MODE0)); }

    void reg(uint16_t r, uint16_t v) {
        beginSpi();
        digitalWrite(DISPLAY_CS_PIN, LOW);
        digitalWrite(DISPLAY_DC_PIN, LOW);  spi_->write16(r);
        digitalWrite(DISPLAY_DC_PIN, HIGH); spi_->write16(v);
        digitalWrite(DISPLAY_CS_PIN, HIGH);
        spi_->endTransaction();
    }
    void winReg(uint16_t r, uint16_t v) {   // CS déjà bas, transaction ouverte
        digitalWrite(DISPLAY_DC_PIN, LOW);  spi_->write16(r);
        digitalWrite(DISPLAY_DC_PIN, HIGH); spi_->write16(v);
    }
    void setWindow(int nx0, int ny0, int nx1, int ny1) {
        beginSpi();
        digitalWrite(DISPLAY_CS_PIN, LOW);
        winReg(0x0036, nx1); winReg(0x0037, nx0);
        winReg(0x0038, ny1); winReg(0x0039, ny0);
        winReg(0x0020, nx0); winReg(0x0021, ny0);
        spi_->endTransaction();
        digitalWrite(DISPLAY_CS_PIN, HIGH);
    }
    void gramStart() {
        beginSpi();
        digitalWrite(DISPLAY_CS_PIN, LOW);
        digitalWrite(DISPLAY_DC_PIN, LOW);  spi_->write16(0x0022);
        digitalWrite(DISPLAY_DC_PIN, HIGH);   // flux pixels : CS bas, DC haut, transaction ouverte
    }
    void gramEnd() {
        digitalWrite(DISPLAY_CS_PIN, HIGH);
        spi_->endTransaction();
    }

    // ---- rotation logicielle -----------------------------------------------
    void toNative(int x, int y, int& nx, int& ny) const {
        switch (rot_) {
            default:
            case 0: nx = x;        ny = y;        break;
            case 1: nx = y;        ny = 219 - x;  break;
            case 2: nx = 175 - x;  ny = 219 - y;  break;
            case 3: nx = 175 - y;  ny = x;        break;
        }
    }
    void fromNative(int nx, int ny, int& x, int& y) const {
        switch (rot_) {
            default:
            case 0: x = nx;        y = ny;        break;
            case 1: x = 219 - ny;  y = nx;        break;
            case 2: x = 175 - nx;  y = 219 - ny;  break;
            case 3: x = ny;        y = 175 - nx;  break;
        }
    }
    void nativeBox(int x, int y, int w, int h, int& nx0, int& ny0, int& nx1, int& ny1) const {
        int ax, ay, bx, by;
        toNative(x, y, ax, ay);
        toNative(x + w - 1, y + h - 1, bx, by);
        nx0 = min(ax, bx); nx1 = max(ax, bx);
        ny0 = min(ay, by); ny1 = max(ay, by);
        nx0 = constrain(nx0, 0, 175); nx1 = constrain(nx1, 0, 175);
        ny0 = constrain(ny0, 0, 219); ny1 = constrain(ny1, 0, 219);
    }

    void initSeq() {
        // Séquence de TFT_22_ILI9225 (Nkawu).
        reg(0x0010, 0x0000); reg(0x0011, 0x0000); reg(0x0012, 0x0000);
        reg(0x0013, 0x0000); reg(0x0014, 0x0000); delay(40);
        reg(0x0011, 0x0018); reg(0x0012, 0x6121); reg(0x0013, 0x006F);
        reg(0x0014, 0x495F); reg(0x0010, 0x0800); delay(10);
        reg(0x0011, 0x103B); delay(50);
        reg(0x0001, 0x011C); reg(0x0002, 0x0100); reg(0x0003, 0x1030);
        reg(0x0007, 0x0000); reg(0x0008, 0x0808); reg(0x000B, 0x1100);
        reg(0x000C, 0x0000); reg(0x000F, 0x0D01); reg(0x0015, 0x0020);
        reg(0x0020, 0x0000); reg(0x0021, 0x0000);
        reg(0x0030, 0x0000); reg(0x0031, 0x00DB); reg(0x0032, 0x0000);
        reg(0x0033, 0x0000); reg(0x0034, 0x00DB); reg(0x0035, 0x0000);
        reg(0x0036, 0x00AF); reg(0x0037, 0x0000); reg(0x0038, 0x00DB);
        reg(0x0039, 0x0000);
        reg(0x0050, 0x0000); reg(0x0051, 0x0808); reg(0x0052, 0x080A);
        reg(0x0053, 0x000A); reg(0x0054, 0x0A08); reg(0x0055, 0x0808);
        reg(0x0056, 0x0000); reg(0x0057, 0x0A00); reg(0x0058, 0x0710);
        reg(0x0059, 0x0710);
        reg(0x0007, 0x0012); delay(50);
        reg(0x0007, ILI9225_INVERT ? 0x1013 : 0x1017);   // display ON
    }

    SPIClass* spi_ = nullptr;
    uint8_t   rot_ = 1;
};

#endif  // DISPLAY_HAS_ILI9225
