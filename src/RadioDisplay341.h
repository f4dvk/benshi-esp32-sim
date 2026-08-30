#pragma once
#include <Arduino.h>
#include "config.h"

#if DISPLAY_HAS_ILI9341

#include <functional>
#include "RadioFace.h"
#include "Ili9341.h"
#include "Xpt2046.h"
#include "DigitMatrix.h"
#include "AudioSpectrum.h"

// ============================================================================
// Écran de façade sur ILI9341 320x240, look Icom. Rendu delta (seuls les
// champs qui changent sont retracés). Tactile XPT2046 optionnel : tap dans le
// 1/3 gauche / droit = canal -1 / +1 (auto-désactivé si la puce ne répond pas).
// ============================================================================

class RadioDisplay341 {
public:
    using PcmSource   = std::function<void(int16_t*)>;
    using TouchAction = std::function<void(int)>;   // -1 / +1 = canal
    void setPcmSource(PcmSource fn)   { pcm_ = std::move(fn); }
    void setTouchAction(TouchAction fn) { touchCb_ = std::move(fn); }

    bool begin(SPIClass* spi) {
        if (ESP.getFreeHeap() < 20000) {
            Serial.println("[341] tas trop juste -> ecran non demarre");
            return false;
        }
        mtx_ = xSemaphoreCreateMutex();
        tft_.begin(spi);
#if TOUCH_ENABLE
        touch_.begin(spi);
        touchOk_ = touch_.present();
        Serial.printf("[341] tactile XPT2046 : %s\n", touchOk_ ? "detecte" : "absent -> desactive");
#endif
        xTaskCreatePinnedToCore(&RadioDisplay341::trampoline, "disp341", 4608, this, 1, nullptr, 1);
        return true;
    }

    void set(const RadioFace& f) {
        if (!mtx_) return;
        xSemaphoreTake(mtx_, portMAX_DELAY);
        pending_ = f;
        xSemaphoreGive(mtx_);
    }

private:
    static void trampoline(void* s) { static_cast<RadioDisplay341*>(s)->loop(); }

    // ---- palette ----
    static const uint16_t BG=0x0000, LINE=0x03B6, LBL=0x9CD3, SEG=0xFFFF, SEGD=0x8C71;
    static const uint16_t REDX=0xF800, AMBER=0xFD20, MTRG=0x2FEB, OKG=0x07E6, DIM=0x39E7, PANEL=0x0841;
    static const uint16_t BT_ON=0x057F;   // bleu (BT connecté à HTCommander)

    // ---- géométrie 320x240 ----
    static const int W=320, H=240;
    static const int BAR_Y=3, SEP1=22;
    static const int DM_CELL=8, DM_SQ=7, DM_SCELL=5, DM_SSQ=4;
    static const int FPY=25, FPH=80, FY=29;
    static const int DGW = digitmatrix::COLS*DM_CELL;      // 40
    static const int DGH = digitmatrix::ROWS*DM_CELL;      // 72
    static const int SGW = digitmatrix::COLS*DM_SCELL;     // 25
    static const int FX0=6, FGAP=3, DOTGAP=13;
    static const int MODE_Y=113;
    static const int TICK_Y=138, BAR_X=20, BAR_Y2=150, BAR_W=284, BAR_H=15;
    static const int SEP3=170;
    static const int SPX=6, SPY=174, SPW=308, SPH=34;      // cadre spectre
    static const int ST_Y=214;
    static const int SP_N=256, SP_BARS=32, SP_MAXBIN=24;

    int digX(int i) const { return FX0 + i*(DGW+FGAP) + (i>=3 ? DOTGAP : 0); }
    int dotX()      const { return digX(2)+DGW+4; }
    int smallX()    const { return digX(5)+DGW+5; }

    void loop() {
        tft_.fillScreen(BG);
        drawChrome();
        uint32_t lastSpec = 0;
        for (;;) {
            RadioFace f;
            xSemaphoreTake(mtx_, portMAX_DELAY);
            f = pending_;
            xSemaphoreGive(mtx_);
            render(f);
#if DISPLAY_SPECTRUM
            if (pcm_ && millis() - lastSpec >= DISPLAY_SPECTRUM_MS) {
                lastSpec = millis();
                renderSpectrum(f.sqOpen && !f.tx);
            }
#endif
#if TOUCH_ENABLE
            if (touchOk_) pollTouch();
#endif
            vTaskDelay(pdMS_TO_TICKS(DISPLAY_REFRESH_MS));
        }
    }

    void drawChrome() {
        tft_.hLine(0, SEP1, W, LINE);
        tft_.hLine(0, MODE_Y - 4, W, LINE);
        tft_.hLine(0, SEP3, W, LINE);
        tft_.hLine(0, H - 2, W, LINE);
        tft_.drawRect(2, FPY, W - 4, FPH, LINE);
        tft_.fillRect(dotX(), FY + DGH - 9, 9, 9, SEG);                 // séparateur MHz/kHz
        tft_.text(smallX() + SGW + 4, FY + DGH - 13, "MHz", LBL, BG, 2);
        // échelle S-mètre (Icom)
        static const struct { const char* t; uint8_t pc; bool r; } SC[] = {
            {"1",0,0},{"3",15,0},{"5",30,0},{"7",45,0},{"9",60,0},{"20",75,1},{"40",87,1},{"60",99,1} };
        for (auto& m : SC)
            tft_.text(BAR_X + m.pc*(BAR_W-12)/100, TICK_Y, m.t, m.r?REDX:LBL, BG, 1);
        tft_.text(4, BAR_Y2 - 2, "S", C_WHITE, BG, 2);
        tft_.drawRect(BAR_X - 2, BAR_Y2 - 2, BAR_W + 4, BAR_H + 4, LINE);
#if DISPLAY_SPECTRUM
        tft_.drawRect(SPX, SPY, SPW, SPH, LINE);
#endif
    }

    void render(const RadioFace& f) {
        bool F = first_;
        char b[28];

        // top bar : n° canal + nom | shift | puissance | BT | GPS
        if (F || f.channelId != c_.channelId || strcmp(f.channel, c_.channel)) {
            snprintf(b, sizeof(b), "%02u %s", f.channelId, f.channel);
            tft_.fillRect(4, BAR_Y, 168, 16, BG);
            tft_.text(4, BAR_Y, b, AMBER, BG, 2);
        }
        if (F || f.shift != c_.shift) {
            tft_.fillRect(174, BAR_Y, 14, 16, BG);
            tft_.text(174, BAR_Y, f.shift > 0 ? "+" : (f.shift < 0 ? "-" : ""), C_WHITE, BG, 2);
        }
        if (F || f.highPower != c_.highPower) {
            tft_.fillRect(196, BAR_Y, 14, 16, BG);
            tft_.text(196, BAR_Y, f.highPower ? "H" : "L", f.highPower ? AMBER : SEG, BG, 2);
        }
        if (F || f.bt != c_.bt) tft_.text(230, BAR_Y, "BT", f.bt ? BT_ON : DIM, BG, 2);
        if (F || f.gpsFix != c_.gpsFix || f.gpsSats != c_.gpsSats) {
            if (f.gpsFix >= 2) snprintf(b, sizeof(b), "%uD%02u", f.gpsFix, f.gpsSats);
            else               snprintf(b, sizeof(b), "--");
            tft_.fillRect(266, BAR_Y, W - 266, 16, BG);
            tft_.textRight(W - 3, BAR_Y, b, f.gpsFix >= 2 ? OKG : DIM, BG, 2);
        }

        // fréquence (matrice de carrés)
        {
            char d[12]; snprintf(d, sizeof(d), "%08.4f", f.rxMHz);
            char digs[8]; int k = 0;
            for (const char* p = d; *p && k < 7; p++) if (*p != '.') digs[k++] = *p;
            uint16_t on = f.tx ? REDX : SEG, onSm = f.tx ? 0x8000 : SEGD;
            bool recol = (f.tx != c_.tx);
            for (int i = 0; i < 6; i++) {
                const uint8_t* g = digitmatrix::glyph(digs[i]);
                if (F || recol || memcmp(g, dprev_[i], digitmatrix::ROWS)) {
                    drawDigit(digX(i), FY, DM_CELL, DM_SQ, g, on);
                    memcpy(dprev_[i], g, digitmatrix::ROWS);
                }
            }
            const uint8_t* gs = digitmatrix::glyph(digs[6]);
            if (F || recol || memcmp(gs, dprev_[6], digitmatrix::ROWS)) {
                drawDigit(smallX(), FY + DGH - digitmatrix::ROWS*DM_SCELL, DM_SCELL, DM_SSQ, gs, onSm);
                memcpy(dprev_[6], gs, digitmatrix::ROWS);
            }
            if (F || recol) tft_.fillRect(dotX(), FY + DGH - 9, 9, 9, on);
        }

        // mode + UTC (taille 2)
        if (F || f.wide != c_.wide) {
            tft_.fillRect(4, MODE_Y, 170, 16, BG);
            tft_.text(4, MODE_Y, "FM", C_WHITE, BG, 2);
            tft_.text(40, MODE_Y, f.wide ? "WIDE 25k" : "NARR 12k", LBL, BG, 2);
        }
        if (F || strcmp(f.utc, c_.utc)) {
            snprintf(b, sizeof(b), "UTC %s", f.utc[0] ? f.utc : "--:--:--");
            tft_.fillRect(W - 150, MODE_Y, 150, 16, BG);
            tft_.textRight(W - 3, MODE_Y, b, f.utc[0] ? OKG : DIM, BG, 2);
        }

        // S-mètre
        if (F || f.sMeter != c_.sMeter) {
            int span = BAR_W * 60 / 100;
            int fill = (int)f.sMeter * span / 9, oldf = (int)c_.sMeter * span / 9;
            uint16_t col = f.sMeter >= 9 ? REDX : (f.sMeter >= 8 ? AMBER : MTRG);
            if (F) { tft_.fillRect(BAR_X, BAR_Y2, BAR_W, BAR_H, PANEL); oldf = 0; }
            if (fill > oldf) tft_.fillRect(BAR_X + oldf, BAR_Y2, fill - oldf, BAR_H, col);
            if (fill < oldf) tft_.fillRect(BAR_X + fill, BAR_Y2, oldf - fill, BAR_H, PANEL);
            if (!F && fill > 0) tft_.fillRect(BAR_X, BAR_Y2, fill, BAR_H, col);
        }

        // bloc statut (taille 2) + indicatif (onglet Licence HTCommander)
        if (F || f.tx != c_.tx || f.txAprs != c_.txAprs || f.sqOpen != c_.sqOpen) {
            const char* s; uint16_t fg;
            if (f.tx && f.txAprs) { s = "APRS"; fg = REDX; }
            else if (f.tx)        { s = "TX";   fg = REDX; }
            else if (f.sqOpen)    { s = "RX";   fg = OKG;  }
            else                  { s = "STBY"; fg = DIM;  }
            tft_.fillRect(4, ST_Y, 60, 16, BG);
            tft_.text(4, ST_Y, s, fg, BG, 2);
        }
        // indicatif : centré dans l'espace libre entre le statut et SQ
        if (F || strcmp(f.callsign, c_.callsign)) {
            const int x0 = 72, x1 = W - 40;
            tft_.fillRect(x0, ST_Y, x1 - x0, 16, BG);
            if (f.callsign[0]) {
                int cw = tft_.textWidth(f.callsign, 2);
                int cx = x0 + (x1 - x0 - cw) / 2;
                if (cx < x0) cx = x0;
                tft_.text(cx, ST_Y, f.callsign, AMBER, BG, 2);
            }
        }
        if (F || f.sqOpen != c_.sqOpen) {
            tft_.fillRect(W - 32, ST_Y, 32, 16, BG);
            tft_.textRight(W - 4, ST_Y, f.sqOpen ? "SQ" : "", f.sqOpen ? OKG : DIM, BG, 2);
        }

        c_ = f;
        first_ = false;
    }

    // chiffre matrice : efface la case puis pose les carrés allumés
    void drawDigit(int x, int y, int cell, int sq, const uint8_t* want, uint16_t on) {
        tft_.fillRect(x, y, digitmatrix::COLS * cell, digitmatrix::ROWS * cell, BG);
        for (int r = 0; r < digitmatrix::ROWS; r++)
            for (int col = 0; col < digitmatrix::COLS; col++)
                if ((want[r] >> (digitmatrix::COLS - 1 - col)) & 1)
                    tft_.fillRect(x + col * cell, y + r * cell, sq, sq, on);
    }

#if DISPLAY_SPECTRUM
    void renderSpectrum(bool live) {
        if (live) {
            pcm_(specPcm_);
            spectrum_.compute(specPcm_, specVal_, SP_BARS, SP_MAXBIN);
        } else {
            bool any = false;
            for (int i = 0; i < SP_BARS; i++) { if (specVal_[i]) any = true;
                specVal_[i] = specVal_[i] > 30 ? (uint8_t)(specVal_[i] - 30) : 0; }
            if (!any && !specDrawn_) return;
        }
        const int px0 = SPX + 3, pw = SPW - 6, base = SPY + SPH - 3, ph = SPH - 6;
        bool empty = true;
        for (int x = 0; x < pw; x++) {
            int bi = x * (SP_BARS - 1) / (pw - 1);
            int h = (int)specVal_[bi] * ph / 255;
            if (h) empty = false;
            int ny = base - h;
            int oy = colY_[x] ? colY_[x] : base;
            if (ny == oy) continue;
            uint16_t col = specVal_[bi] > 195 ? REDX : (specVal_[bi] > 115 ? AMBER : MTRG);
            if (ny < oy) tft_.fillRect(px0 + x, ny, 1, oy - ny, col);   // monte
            else         tft_.fillRect(px0 + x, oy, 1, ny - oy, BG);    // descend
            colY_[x] = ny;
        }
        specDrawn_ = !empty;
    }
#endif

#if TOUCH_ENABLE
    void pollTouch() {
        int x, y;
        bool down = touch_.read(x, y);
        uint32_t now = millis();
        if (down && !touchDown_) { touchDown_ = true; touchT0_ = now; touchX0_ = x; }
        else if (!down && touchDown_) {
            touchDown_ = false;
            if (now - touchT0_ <= TOUCH_TAP_MAX_MS && touchCb_) {
                if (touchX0_ < W / 3)       touchCb_(-1);
                else if (touchX0_ > 2*W/3)  touchCb_(+1);
            }
        }
    }
#endif

    Ili9341 tft_;
#if TOUCH_ENABLE
    Xpt2046 touch_;
    bool    touchOk_ = false, touchDown_ = false;
    uint32_t touchT0_ = 0; int touchX0_ = 0;
#endif
    SemaphoreHandle_t mtx_ = nullptr;
    RadioFace pending_, c_;
    bool first_ = true;
    uint8_t dprev_[7][digitmatrix::ROWS] = {{0}};
    PcmSource   pcm_;
    TouchAction touchCb_;
#if DISPLAY_SPECTRUM
    AudioSpectrum<SP_N> spectrum_;
    int16_t  specPcm_[SP_N];
    uint8_t  specVal_[SP_BARS] = {0};
    int16_t  colY_[SPW] = {0};
    bool     specDrawn_ = false;
#endif
};

#endif  // DISPLAY_HAS_ILI9341
