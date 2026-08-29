#pragma once
#include <Arduino.h>
#include "config.h"
#include "Mcp23017.h"
#include "Font5x7.h"
#if ILI9225_HW_SPI
#include <SPI.h>
#endif

// ============================================================================
// Pilote minimal ILI9225 (TFT 176x220, RGB565) piloté en SPI LOGICIEL bit-bangé
// à travers un MCP23017 (I2C). Lent -> réservé à un affichage statut.
//
// Toutes les lignes de l'écran sont sur le PORT A du MCP23017. Le flux SPI est
// envoyé par rafales (Mcp23017::burstA, SEQOP=1) : une transaction I2C pour
// plusieurs états de port.
// ============================================================================

#if DISPLAY_ENABLE

#if !ILI9225_HW_SPI && (ILI9225_CS_PIN >= 8 || ILI9225_RS_PIN >= 8)
#error "En bit-bang (ILI9225_HW_SPI=false) toutes les lignes ecran doivent etre sur le port A (pins 0..7)."
#endif

// Couleurs RGB565
static const uint16_t C_BLACK  = 0x0000, C_WHITE = 0xFFFF, C_RED = 0xF800;
static const uint16_t C_GREEN  = 0x07E0, C_BLUE  = 0x001F, C_YELLOW = 0xFFE0;
static const uint16_t C_CYAN   = 0x07FF, C_GREY  = 0x8410, C_DKGREY = 0x39E7;
static const uint16_t C_ORANGE = 0xFD20;

class Ili9225 {
public:
    void begin(Mcp23017& mcp) {
        mcp_ = &mcp;
#if ILI9225_HW_SPI
        SCK_ = 0; SDI_ = 0;                       // hors du port MCP
        spi_ = new SPIClass(HSPI);
        spi_->begin(ILI9225_SPI_SCK, -1, ILI9225_SPI_MOSI, -1);
        Serial.printf("[DISP] SPI materiel : SCK=%d MOSI=%d @ %d Hz\n",
                      ILI9225_SPI_SCK, ILI9225_SPI_MOSI, ILI9225_SPI_HZ);
#else
        SCK_ = 1 << (ILI9225_SCK_PIN & 7);
        SDI_ = 1 << (ILI9225_SDI_PIN & 7);
#endif
        // CS/RS/RST/LED : sur le port MCP (A si pins 0..7, B si 8..15).
        // RS peut être déporté sur un GPIO de l'ESP (ILI9225_SPI_RS >= 0) -> les
        // bascules commande/donnée ne passent alors plus par l'I2C (bien plus rapide).
        ctrlOnB_ = (ILI9225_CS_PIN >= 8);
#if ILI9225_HW_SPI
        rsPin_ = ILI9225_SPI_RS;
#else
        rsPin_ = -1;
#endif
        CS_  = 1 << (ILI9225_CS_PIN & 7);
        RS_  = (rsPin_ >= 0) ? 0 : (1 << (ILI9225_RS_PIN & 7));
        RST_ = (ILI9225_RST_PIN >= 0) ? (1 << (ILI9225_RST_PIN & 7)) : 0;
        LED_ = (ILI9225_LED_PIN >= 0) ? (1 << (ILI9225_LED_PIN & 7)) : 0;
        rot_ = ILI9225_ROTATION & 3;
        if (rsPin_ >= 0) { pinMode(rsPin_, OUTPUT); ::digitalWrite(rsPin_, HIGH); }
        // État de repos : CS haut, RS haut, RST relâché, RÉTRO-ÉCLAIRAGE ALLUMÉ.
        // (le rétro-éclairage doit rester allumé pendant tous les transferts SPI)
        idle_ = CS_ | RS_ | RST_;
#if ILI9225_LED_ON_LEVEL
        idle_ |= LED_;
#endif
        // Rétro-éclairage d'abord : si le reste échoue, on voit au moins l'écran allumé.
        cur_ = ~idle_;                 // force la 1re écriture
        port(idle_);
        delay(20);

        // Reset matériel : /RST bas ~10 ms puis relâché ~50 ms
        port(idle_ & ~RST_);
        delay(10);
        port(idle_);
        delay(50);

        initSeq();
        Serial.printf("[DISP] ILI9225 initialise (%dx%d, rot %d)\n", width(), height(), rot_);
        // Test rapide : carré rouge en haut-gauche + carré vert en bas-droite.
        // S'ils apparaissent -> SPI/init/rétro-éclairage OK, le reste n'est que lent.
        fillRect(0, 0, 20, 20, C_RED);
        fillRect(width() - 20, height() - 20, 20, 20, C_GREEN);
        Serial.println("[DISP] test : carre ROUGE haut-gauche + VERT bas-droite ?");
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
        pushColor(c, n);
        gramEnd();
    }

    void hLine(int x, int y, int w, uint16_t c) { fillRect(x, y, w, 1, c); }
    void vLine(int x, int y, int h, uint16_t c) { fillRect(x, y, 1, h, c); }
    void drawRect(int x, int y, int w, int h, uint16_t c) {
        hLine(x, y, w, c); hLine(x, y + h - 1, w, c);
        vLine(x, y, h, c); vLine(x + w - 1, y, h, c);
    }

    // Copie un bloc de pixels (orientation logique, ligne par ligne) en UNE
    // fenêtre GRAM -> bien plus rapide que N petits fillRect (ex. un chiffre).
    void blit(int x, int y, int w, int h, const uint16_t* px) {
        if (w <= 0 || h <= 0) return;
        int nx0, ny0, nx1, ny1;
        nativeBox(x, y, w, h, nx0, ny0, nx1, ny1);
        setWindow(nx0, ny0, nx1, ny1);
        gramStart();
        for (int ny = ny0; ny <= ny1; ny++)
            for (int nx = nx0; nx <= nx1; nx++) {
                int lx, ly; fromNative(nx, ny, lx, ly);
                pushPixel(px[(ly - y) * w + (lx - x)]);
            }
        gramEnd();
    }

    // Texte 5x7 mis à l'échelle. `bg` = fond (opaque).
    void text(int x, int y, const char* s, uint16_t fg, uint16_t bg, int scale = 1) {
        for (; *s; s++, x += 6 * scale) glyph(x, y, *s, fg, bg, scale);
    }
    void textRight(int xr, int y, const char* s, uint16_t fg, uint16_t bg, int scale = 1) {
        text(xr - (int)strlen(s) * 6 * scale, y, s, fg, bg, scale);
    }
    int textWidth(const char* s, int scale = 1) const { return (int)strlen(s) * 6 * scale; }
    static const int CHAR_H = 8;

private:
    // ---- bas niveau ------------------------------------------------------
    // Écrit l'état des lignes de contrôle (CS/RS/RST/LED) sur le bon port MCP.
    // Déduplication : une écriture I2C n'a lieu que si l'état change réellement.
    void port(uint8_t v) {
        if (v == cur_) return;
        cur_ = v;
        ctrlOnB_ ? mcp_->writeB(v) : mcp_->writeA(v);
    }
    void csLow()  { port(cur_ & ~CS_); }
    void csHigh() { port(cur_ | CS_); }
    void rsLow()  { if (rsPin_ >= 0) ::digitalWrite(rsPin_, LOW);  else port(cur_ & ~RS_); }
    void rsHigh() { if (rsPin_ >= 0) ::digitalWrite(rsPin_, HIGH); else port(cur_ | RS_); }

#if ILI9225_HW_SPI
    // ---- SPI matériel : SCK/MOSI sur l'ESP ; CS (+ RS si non déporté) sur MCP.
    // Vitesse : garder CS bas et UNE transaction SPI ouverte sur toute une
    // fenêtre GRAM ; ne toggler que RS (idéalement un GPIO de l'ESP).
    void beginSpi() { spi_->beginTransaction(SPISettings(ILI9225_SPI_HZ, MSBFIRST, SPI_MODE0)); }
    void endSpi()   { spi_->endTransaction(); }

    // Registre isolé (init) : gère son propre CS + sa transaction.
    void cmd16(uint16_t c)  { csLow(); rsLow();  beginSpi(); spi_->write16(c); endSpi(); csHigh(); }
    void data16(uint16_t d) { csLow(); rsHigh(); beginSpi(); spi_->write16(d); endSpi(); csHigh(); }
    void reg(uint16_t r, uint16_t v) { cmd16(r); data16(v); }

    // Registre à l'intérieur d'un setWindow (CS déjà bas, transaction ouverte).
    void winReg(uint16_t r, uint16_t v) {
        rsLow();  spi_->write16(r);
        rsHigh(); spi_->write16(v);
    }

    void gramStart() {
        csLow();
        rsLow();  beginSpi(); spi_->write16(0x0022); endSpi();
        rsHigh(); beginSpi();          // CS bas + RS haut + transaction : flux pixels
    }
    void gramEnd() { endSpi(); csHigh(); }

    void pushColor(uint16_t c, uint32_t count) { while (count--) spi_->write16(c); }
    void pushPixel(uint16_t c) { spi_->write16(c); }
#else
    // ---- SPI logiciel bit-bangé à travers le MCP23017 ------------------
    // Envoie une suite de mots 16 bits (MSB d'abord) ; RS/CS déjà positionnés.
    void shift16(const uint16_t* w, size_t n, uint8_t base) {
        size_t bi = 0;
        for (size_t k = 0; k < n; k++) {
            uint16_t val = w[k];
            for (int b = 15; b >= 0; b--) {
                uint8_t p = base | (((val >> b) & 1) ? SDI_ : 0);
                bb_[bi++] = p;            // SCK bas + donnée
                bb_[bi++] = p | SCK_;     // front montant SCK
                if (bi >= sizeof(bb_) - 2) { mcp_->burstA(bb_, bi); bi = 0; }
            }
        }
        if (bi) mcp_->burstA(bb_, bi);
    }
    void cmd16(uint16_t c) {
        uint8_t b = (idle_ & ~CS_ & ~RS_ & ~SCK_ & ~SDI_);
        port(b); shift16(&c, 1, b); port(idle_);
    }
    void data16(uint16_t d) {
        uint8_t b = (idle_ & ~CS_ & ~SCK_ & ~SDI_) | RS_;
        port(b); shift16(&d, 1, b); port(idle_);
    }
    void reg(uint16_t r, uint16_t v) { cmd16(r); data16(v); }

    void gramStart() {
        cmd16(0x0022);
        gramBase_ = (idle_ & ~CS_ & ~SCK_ & ~SDI_) | RS_;
        port(gramBase_);
        bbLen_ = 0;
    }
    void gramEnd() { if (bbLen_) { mcp_->burstA(bb_, bbLen_); bbLen_ = 0; } port(idle_); }

    void pushColor(uint16_t c, uint32_t count) { while (count--) pushPixel(c); }
    void pushPixel(uint16_t c) {
        for (int b = 15; b >= 0; b--) {
            uint8_t p = gramBase_ | (((c >> b) & 1) ? SDI_ : 0);
            bb_[bbLen_++] = p;
            bb_[bbLen_++] = p | SCK_;
        }
        if (bbLen_ > sizeof(bb_) - 32) { mcp_->burstA(bb_, bbLen_); bbLen_ = 0; }
    }
#endif

    // ---- fenêtre GRAM + rotation --------------------------------------
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
    void setWindow(int nx0, int ny0, int nx1, int ny1) {
#if ILI9225_HW_SPI
        csLow(); beginSpi();
        winReg(0x0036, nx1); winReg(0x0037, nx0);
        winReg(0x0038, ny1); winReg(0x0039, ny0);
        winReg(0x0020, nx0); winReg(0x0021, ny0);
        endSpi();                     // CS reste bas : gramStart enchaîne
#else
        reg(0x0036, nx1); reg(0x0037, nx0);
        reg(0x0038, ny1); reg(0x0039, ny0);
        reg(0x0020, nx0); reg(0x0021, ny0);
#endif
    }

    // ---- glyphe --------------------------------------------------------
    void glyph(int x, int y, char ch, uint16_t fg, uint16_t bg, int sc) {
        int w = 6 * sc, h = 8 * sc;
        if (x + w <= 0 || y + h <= 0 || x >= width() || y >= height()) return;
        int nx0, ny0, nx1, ny1;
        nativeBox(x, y, w, h, nx0, ny0, nx1, ny1);
        setWindow(nx0, ny0, nx1, ny1);
        gramStart();
        for (int ny = ny0; ny <= ny1; ny++) {
            for (int nx = nx0; nx <= nx1; nx++) {
                int lx, ly; fromNative(nx, ny, lx, ly);
                int gx = (lx - x) / sc, gy = (ly - y) / sc;
                bool on = (gx >= 0 && gx < 5 && gy >= 0 && gy < 7) &&
                          (font5x7Col(ch, gx) & (1 << gy));
                pushPixel(on ? fg : bg);
            }
        }
        gramEnd();
    }

    void initSeq() {
        // Séquence de TFT_22_ILI9225 (Nkawu), référence connue pour fonctionner.
        // -- Power-on --
        reg(0x0010, 0x0000); reg(0x0011, 0x0000); reg(0x0012, 0x0000);
        reg(0x0013, 0x0000); reg(0x0014, 0x0000); delay(40);
        reg(0x0011, 0x0018); reg(0x0012, 0x6121); reg(0x0013, 0x006F);
        reg(0x0014, 0x495F); reg(0x0010, 0x0800); delay(10);
        reg(0x0011, 0x103B); delay(50);
        // -- Réglages écran --
        reg(0x0001, 0x011C); reg(0x0002, 0x0100); reg(0x0003, 0x1030);
        reg(0x0007, 0x0000); reg(0x0008, 0x0808); reg(0x000B, 0x1100);
        reg(0x000C, 0x0000); reg(0x000F, 0x0D01); reg(0x0015, 0x0020);
        reg(0x0020, 0x0000); reg(0x0021, 0x0000);
        // -- Zone GRAM --
        reg(0x0030, 0x0000); reg(0x0031, 0x00DB); reg(0x0032, 0x0000);
        reg(0x0033, 0x0000); reg(0x0034, 0x00DB); reg(0x0035, 0x0000);
        reg(0x0036, 0x00AF); reg(0x0037, 0x0000); reg(0x0038, 0x00DB);
        reg(0x0039, 0x0000);
        // -- Gamma --
        reg(0x0050, 0x0000); reg(0x0051, 0x0808); reg(0x0052, 0x080A);
        reg(0x0053, 0x000A); reg(0x0054, 0x0A08); reg(0x0055, 0x0808);
        reg(0x0056, 0x0000); reg(0x0057, 0x0A00); reg(0x0058, 0x0710);
        reg(0x0059, 0x0710);
        reg(0x0007, 0x0012); delay(50);
        reg(0x0007, ILI9225_INVERT ? 0x1013 : 0x1017);   // display ON
    }

    Mcp23017* mcp_ = nullptr;
    uint8_t SCK_ = 0, SDI_ = 0, CS_ = 0, RS_ = 0, RST_ = 0, LED_ = 0;
    uint8_t idle_ = 0, gramBase_ = 0, rot_ = 1, cur_ = 0;
    int     rsPin_ = -1;
    bool    ctrlOnB_ = false;
#if ILI9225_HW_SPI
    SPIClass* spi_ = nullptr;
#else
    size_t  bbLen_ = 0;
    uint8_t bb_[Mcp23017::kWireBuf];
#endif
};

#endif  // DISPLAY_ENABLE
