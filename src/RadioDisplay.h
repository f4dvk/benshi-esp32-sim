#pragma once
#include <Arduino.h>
#include "config.h"

#if DISPLAY_HAS_ILI9225

#include <functional>
#include "RadioFace.h"
#include "Ili9225.h"
#include "AudioSpectrum.h"

// ============================================================================
// Écran de façade "portatif VHF", look Icom (ILI9225, SPI direct).
//
// Affichage passif, rendu delta (seuls les champs qui changent sont retracés).
// Tâche FreeRTOS dédiée, priorité basse.
// ============================================================================

class RadioDisplay {
public:
    // Source PCM pour l'analyseur de spectre : remplit AudioBridge::kSpecN
    // échantillons (les plus récents de l'audio reçu). Optionnelle.
    using PcmSource = std::function<void(int16_t*)>;
    void setPcmSource(PcmSource fn) { pcm_ = std::move(fn); }

    bool begin(SPIClass* spi) {
        if (ESP.getFreeHeap() < 20000) {
            Serial.println("[DISP] tas trop juste -> ecran non demarre");
            return false;
        }
        mtx_ = xSemaphoreCreateMutex();
        tft_.begin(spi);
        xTaskCreatePinnedToCore(&RadioDisplay::trampoline, "display", 4096, this, 1, nullptr, 1);
        return true;
    }

    void set(const RadioFace& f) {
        if (!mtx_) return;
        xSemaphoreTake(mtx_, portMAX_DELAY);
        pending_ = f;
        xSemaphoreGive(mtx_);
    }

private:
    static void trampoline(void* self) { static_cast<RadioDisplay*>(self)->loop(); }

    void loop() {
        uint32_t t0 = millis();
        drawFrame();
        Serial.printf("[DISP] chassis dessine (%lu ms)\n", (unsigned long)(millis() - t0));
        uint32_t lastSpec = 0;
        for (;;) {
            RadioFace f;
            xSemaphoreTake(mtx_, portMAX_DELAY);
            f = pending_;
            xSemaphoreGive(mtx_);
            bool wf = first_;
            render(f);
            if (wf) Serial.println("[DISP] premier rendu OK");
#if DISPLAY_SPECTRUM
            if (pcm_ && millis() - lastSpec >= DISPLAY_SPECTRUM_MS) {
                lastSpec = millis();
                renderSpectrum(f.sqOpen && !f.tx);
            }
#endif
            vTaskDelay(pdMS_TO_TICKS(DISPLAY_REFRESH_MS));
        }
    }

    // ---- palette (Icom IC-7760 : fond noir, cadres cyan, VFO blanc) ----
    static const uint16_t BG    = 0x0000;   // noir
    static const uint16_t PANEL = 0x0841;   // navy très foncé (fonds encastrés)
    static const uint16_t LINE  = 0x03B6;   // cyan foncé (cadres, grille)
    static const uint16_t LBL   = 0x9CD3;   // bleu-gris clair (libellés)
    static const uint16_t SEG   = 0xFFFF;   // blanc (fréquence VFO en réception)
    static const uint16_t SEGD  = 0x8C71;   // gris (unités kHz)
    static const uint16_t SEGX  = 0x0000;   // segment éteint = noir
    static const uint16_t TXD   = 0x8000;   // rouge atténué (unités kHz en TX)
    static const uint16_t REDX  = 0xF800;   // rouge émission
    static const uint16_t AMBER = 0xFD20;
    static const uint16_t MTRG  = 0x2FEB;   // vert S-mètre Icom
    static const uint16_t OKG   = 0x07E6;
    static const uint16_t DIM   = 0x39E7;
    static const uint16_t BT_ON = 0x057F;   // bleu (BT connecté à HTCommander)

    // ---- géométrie (220 x 176, tout vérifié sans chevauchement) -----
    static const int W = 220, H = 176;
    static const int BAR_Y = 2;
    static const int SEP1  = 12;
    // Panneau fréquence encastré : 6 GROS chiffres en MATRICE DE CARRÉS (LED
    // dot-matrix 5x9) + 1 chiffre kHz plus petit. "MHz" est en bout de ligne mode.
    static const int DM_COLS = 5, DM_ROWS = 9;
    static const int DM_BIG = 5, DM_BIG_SQ = 4;   // pas / côté du carré (gros)
    static const int DM_SM  = 3, DM_SM_SQ  = 2;   // idem (petit chiffre kHz)
    static const int FPX = 2, FPY = 14, FPW = 216, FPH = 52;
    static const int FDW = DM_COLS * DM_BIG, FDH = DM_ROWS * DM_BIG;   // 25 x 45
    static const int FSW = DM_COLS * DM_SM,  FSH = DM_ROWS * DM_SM;    // 15 x 27
    static const int FX0 = 6, FY = 17, FGAP = 5, DOTGAP = 12;
    static const int MODE_Y = 71;
    static const int TICK_Y = 85;
    static const int BAR_X = 16, BAR_Y2 = 95, BAR_W = 196, BAR_H = 10;
    // Analyseur de spectre audio : courbe (polyligne) tracée dans un tampon RAM
    // puis poussée en une passe (Ili9225::blit).
    static const int SP_N = 256;                 // = AudioBridge::kSpecN
    static const int SPX = 3, SPY = 112, SPW = 214, SPH = 26;   // cadre
    static const int SP_PX = SPX + 2, SP_PY = SPY + 2;          // zone de tracé
    static const int SP_PW = SPW - 4, SP_PH = SPH - 4;          // 210 x 22
    static const int SP_STRIP = 6;                              // hauteur de bande (blit)
    static const int SP_BARS = 24, SP_MAXBIN = 24;   // 24 points -> ~3 kHz @ 32 kHz
    static const uint32_t SPEC_IDLE_CLEAR_MS = 2500; // efface le spectre N ms après la perte du signal
    static const int SEP3  = 141;
    static const int ST_Y  = 144, ST_BOX_W = 70, ST_BOX_H = 28;   // réduit pour loger l'indicatif

    int digX(int i)  const { return FX0 + i * (FDW + FGAP) + (i >= 3 ? DOTGAP : 0); }
    int dotX()       const { return digX(2) + FDW + 3; }
    int smallX()     const { return digX(5) + FDW + 3; }

    // Chiffres 0..9 en matrice 5 colonnes x 9 lignes (bit4 = colonne de gauche).
    static const uint8_t* digGlyph(char c) {
        static const uint8_t g[10][DM_ROWS] = {
            {0x0E,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x0E}, // 0
            {0x04,0x0C,0x04,0x04,0x04,0x04,0x04,0x04,0x0E}, // 1
            {0x0E,0x11,0x01,0x01,0x02,0x04,0x08,0x10,0x1F}, // 2
            {0x0E,0x11,0x01,0x01,0x06,0x01,0x01,0x11,0x0E}, // 3
            {0x02,0x06,0x0A,0x12,0x12,0x1F,0x02,0x02,0x02}, // 4
            {0x1F,0x10,0x10,0x1E,0x01,0x01,0x01,0x11,0x0E}, // 5
            {0x06,0x08,0x10,0x1E,0x11,0x11,0x11,0x11,0x0E}, // 6
            {0x1F,0x01,0x02,0x02,0x04,0x04,0x08,0x08,0x08}, // 7
            {0x0E,0x11,0x11,0x11,0x0E,0x11,0x11,0x11,0x0E}, // 8
            {0x0E,0x11,0x11,0x11,0x0F,0x01,0x01,0x02,0x0C}, // 9
        };
        static const uint8_t blank[DM_ROWS] = {0};
        return (c >= '0' && c <= '9') ? g[c - '0'] : blank;
    }

    // Trace un chiffre matrice en UNE fenêtre GRAM (via Ili9225::blit) : ~30x
    // plus rapide que N petits fillRect. L'appelant ne le fait que si le chiffre
    // a changé (comparaison avec dgPrev_). Utilise le tampon partagé scratch_.
    void digMat(int x, int y, int cell, int sq, const uint8_t* want, uint16_t on) {
        const int w = DM_COLS * cell, h = DM_ROWS * cell;
        for (int i = 0; i < w * h; i++) scratch_[i] = BG;
        for (int r = 0; r < DM_ROWS; r++)
            for (int c = 0; c < DM_COLS; c++) {
                if (!((want[r] >> (DM_COLS - 1 - c)) & 1)) continue;
                for (int yy = 0; yy < sq; yy++) {
                    uint16_t* row = &scratch_[(r * cell + yy) * w + c * cell];
                    for (int xx = 0; xx < sq; xx++) row[xx] = on;
                }
            }
        tft_.blit(x, y, w, h, scratch_);
    }

    // Échelle S-mètre façon Icom : S1..S9 en gris, +20/+40/+60 en rouge.
    struct SMark { const char* t; uint8_t pc; bool red; };
    static const SMark* sScale(int& n) {
        static const SMark m[] = {
            {"1", 0, false}, {"3", 15, false}, {"5", 30, false},
            {"7", 45, false}, {"9", 60, false},
            {"20", 74, true}, {"40", 86, true}, {"60", 98, true},
        };
        n = sizeof(m) / sizeof(m[0]);
        return m;
    }
    int sMarkX(const SMark& m) const { return BAR_X + m.pc * (BAR_W - 8) / 100; }

    void drawFrame() {
#if DISPLAY_FULL_CLEAR
        tft_.fillScreen(BG);
#endif
        tft_.hLine(0, SEP1, W, LINE);
        tft_.hLine(0, SEP3, W, LINE);
        tft_.hLine(0, H - 3, W, LINE);

        // panneau fréquence encastré (bord cyan), "MHz" en bout de ligne mode
        tft_.drawRect(FPX, FPY, FPW, FPH, LINE);
        tft_.textRight(W - 3, MODE_Y, "MHz", LBL, BG, 1);
        tft_.fillRect(dotX(), FY + FDH - 7, 7, 7, SEG);   // séparateur MHz/kHz

        // S-mètre : "S" + échelle Icom + cadre encastré
        tft_.text(2, BAR_Y2 - 2, "S", C_WHITE, BG, 2);
        int n; const SMark* sc = sScale(n);
        for (int i = 0; i < n; i++)
            tft_.text(sMarkX(sc[i]), TICK_Y, sc[i].t, sc[i].red ? REDX : LBL, BG, 1);
        tft_.drawRect(BAR_X - 2, BAR_Y2 - 2, BAR_W + 4, BAR_H + 4, LINE);

#if DISPLAY_SPECTRUM
        tft_.drawRect(SPX, SPY, SPW, SPH, LINE);   // cadre de l'analyseur de spectre
#endif
    }

    void render(const RadioFace& f) {
        bool F = first_;

        // ---- barre haute : n° canal + nom | H/L +/- CTCSS | BT | GPS ----
        if (F || f.channelId != c_.channelId || strcmp(f.channel, c_.channel)) {
            char s[16]; snprintf(s, sizeof(s), "%02u %s", f.channelId, f.channel);
            tft_.fillRect(0, BAR_Y, 76, 8, BG);
            tft_.text(4, BAR_Y, s, AMBER, BG, 1);
        }
        if (F || f.highPower != c_.highPower || f.shift != c_.shift || f.tone != c_.tone) {
            char s[10];
            snprintf(s, sizeof(s), "%s%s%s", f.highPower ? "H" : "L",
                     f.shift > 0 ? " +" : (f.shift < 0 ? " -" : ""),
                     f.tone == 2 ? " CT" : (f.tone == 1 ? " T" : ""));
            tft_.fillRect(80, BAR_Y, 40, 8, BG);
            tft_.text(80, BAR_Y, s, f.highPower ? AMBER : C_WHITE, BG, 1);
        }
        if (F || f.bt != c_.bt)
            tft_.text(158, BAR_Y, "BT", f.bt ? BT_ON : DIM, BG, 1);
        if (F || f.gpsFix != c_.gpsFix || f.gpsSats != c_.gpsSats) {
            char s[10];
            if (f.gpsFix >= 2) snprintf(s, sizeof(s), "%uD %02u", f.gpsFix, f.gpsSats);
            else               snprintf(s, sizeof(s), "-- --");
            tft_.fillRect(180, BAR_Y, W - 180, 8, BG);
            tft_.textRight(W - 2, BAR_Y, s, f.gpsFix >= 2 ? OKG : DIM, BG, 1);
        }

        // ---- fréquence : matrice de carrés (LED dot-matrix), style Icom ----
        {
            char d[12]; snprintf(d, sizeof(d), "%08.4f", f.rxMHz);   // "145.5000"
            char digs[8]; int k = 0;
            for (const char* p = d; *p && k < 7; p++) if (*p != '.') digs[k++] = *p;
            uint16_t on   = f.tx ? REDX : SEG;
            uint16_t onSm = f.tx ? TXD  : SEGD;
            bool recolor  = (f.tx != c_.tx);
            for (int i = 0; i < 6; i++) {                            // "145.500"
                const uint8_t* want = digGlyph(digs[i]);
                if (F || recolor || memcmp(want, dgPrev_[i], DM_ROWS)) {
                    digMat(digX(i), FY, DM_BIG, DM_BIG_SQ, want, on);
                    memcpy(dgPrev_[i], want, DM_ROWS);
                }
            }
            {                                                       // unités kHz
                const uint8_t* want = digGlyph(digs[6]);
                if (F || recolor || memcmp(want, dgPrev_[6], DM_ROWS)) {
                    digMat(smallX(), FY + FDH - FSH, DM_SM, DM_SM_SQ, want, onSm);
                    memcpy(dgPrev_[6], want, DM_ROWS);
                }
            }
            if (F || recolor) tft_.fillRect(dotX(), FY + FDH - 7, 7, 7, on);
        }

        // ---- badge mode (encadré, style Icom) ----
        if (F || f.wide != c_.wide) {
            tft_.drawRect(2, MODE_Y - 2, 82, 13, LINE);
            tft_.fillRect(4, MODE_Y, 78, 9, BG);
            tft_.text(6,  MODE_Y, "FM", C_WHITE, BG, 1);
            tft_.text(28, MODE_Y, f.wide ? "W 25k" : "N 12k", LBL, BG, 1);
        }
        // ---- heure UTC (GPS) sur la ligne mode ----
        if (F || strcmp(f.utc, c_.utc)) {
            char t[16];
            snprintf(t, sizeof(t), "UTC %s", f.utc[0] ? f.utc : "--:--:--");
            tft_.fillRect(92, MODE_Y, 100, 8, BG);
            tft_.text(94, MODE_Y, t, f.utc[0] ? OKG : DIM, BG, 1);
        }

        // ---- S-mètre : barre proportionnelle, calée sur l'échelle Icom ----
        //  (S0..S9 occupe les 60 % gauche de la barre, cf. sScale()).
        if (F || f.sMeter != c_.sMeter) {
            int span = BAR_W * 60 / 100;
            int fill  = (int)f.sMeter * span / 9;
            int oldf  = (int)c_.sMeter * span / 9;
            uint16_t col = f.sMeter >= 9 ? REDX : (f.sMeter >= 8 ? AMBER : MTRG);
            if (F) { tft_.fillRect(BAR_X, BAR_Y2, BAR_W, BAR_H, PANEL); oldf = 0; }
            if (fill > oldf) tft_.fillRect(BAR_X + oldf, BAR_Y2, fill - oldf, BAR_H, col);
            if (fill < oldf) tft_.fillRect(BAR_X + fill, BAR_Y2, oldf - fill, BAR_H, PANEL);
            if (!F && fill > 0) tft_.fillRect(BAR_X, BAR_Y2, fill, BAR_H, col);
        }

        // ---- statut : bloc TX / RX (réduit) ----
        bool stChg = F || f.tx != c_.tx || f.txAprs != c_.txAprs || f.sqOpen != c_.sqOpen;
        if (stChg) {
            const char* lbl; uint16_t fg, fill;
            if (f.tx && f.txAprs) { lbl = "APRS"; fg = C_WHITE; fill = REDX; }
            else if (f.tx)        { lbl = "TX";   fg = C_WHITE; fill = REDX; }
            else if (f.sqOpen)    { lbl = "RX";   fg = BG;      fill = OKG;  }
            else                  { lbl = "STBY"; fg = DIM;     fill = BG;   }
            tft_.fillRect(6, ST_Y, ST_BOX_W, ST_BOX_H, fill);
            tft_.drawRect(6, ST_Y, ST_BOX_W, ST_BOX_H, fill == BG ? LINE : fill);
            int tw = tft_.textWidth(lbl, 2);
            tft_.text(6 + (ST_BOX_W - tw) / 2, ST_Y + (ST_BOX_H - 16) / 2, lbl, fg,
                      fill == BG ? BG : fill, 2);
        }
        // ---- indicatif : centré dans l'espace libre entre le bloc statut et SQ ----
        if (F || strcmp(f.callsign, c_.callsign)) {
            const int x0 = ST_BOX_W + 12, x1 = W - 34;      // zone libre
            tft_.fillRect(x0, ST_Y + 8, x1 - x0, 14, BG);
            if (f.callsign[0]) {
                int cw = tft_.textWidth(f.callsign, 2);
                int cx = x0 + (x1 - x0 - cw) / 2;
                if (cx < x0) cx = x0;
                tft_.text(cx, ST_Y + 8, f.callsign, AMBER, BG, 2);
            }
        }
        if (F || f.sqOpen != c_.sqOpen) {
            tft_.fillRect(W - 30, ST_Y + 8, 30, 16, BG);
            tft_.textRight(W - 4, ST_Y + 8, f.sqOpen ? "SQ" : "", f.sqOpen ? OKG : DIM, BG, 2);
        }

        c_ = f;
        first_ = false;
    }

#if DISPLAY_SPECTRUM
    // Spectre sous forme de COURBE : la polyligne est tracée dans un tampon RAM
    // (couleur = amplitude : vert -> ambre -> rouge) puis poussée en une seule
    // fenêtre GRAM. `live` : signal reçu présent -> FFT ; sinon la courbe
    // retombe vers le bas puis est effacée.
    void renderSpectrum(bool live) {
        if (live) {
            specIdleMs_ = 0;
            pcm_(specPcm_);
            float pk = spectrum_.compute(specPcm_, specVal_, SP_BARS, SP_MAXBIN);
#if AUDIO_DEBUG
            if (millis() - specDbgMs_ > 2000) {
                specDbgMs_ = millis();
                int mx = 0;
                for (int b = 0; b < SP_BARS; b++) if (specVal_[b] > mx) mx = specVal_[b];
                Serial.printf("[SPEC] pic %.0f dB (plage %.0f..%.0f dB), max %d%% | "
                              "affichage 0-%d Hz, %d Hz/point\n",
                              pk, (double)AudioSpectrum<SP_N>::SPEC_DB_FLOOR,
                              (double)AudioSpectrum<SP_N>::SPEC_DB_TOP, mx * 100 / 255,
                              SP_MAXBIN * AUDIO_I2S_RATE / SP_N,
                              (SP_MAXBIN * AUDIO_I2S_RATE / SP_N) / SP_BARS);
            }
#endif
        } else {
            if (!specIdleMs_) specIdleMs_ = millis();
            bool hard = millis() - specIdleMs_ > SPEC_IDLE_CLEAR_MS;
            for (int b = 0; b < SP_BARS; b++)
                specVal_[b] = (hard || specVal_[b] <= 30) ? 0 : (uint8_t)(specVal_[b] - 30);
        }

        bool empty = true;
        int px[SP_BARS], py[SP_BARS];
        for (int b = 0; b < SP_BARS; b++) {
            if (specVal_[b]) empty = false;
            px[b] = (SP_BARS > 1) ? b * (SP_PW - 1) / (SP_BARS - 1) : 0;
            py[b] = (SP_PH - 1) - (int)specVal_[b] * (SP_PH - 1) / 255;
        }
        if (empty && !specDrawn_) return;   // déjà vide, rien à faire

        // Tracé + envoi par bandes horizontales (tampon partagé scratch_ ->
        // ~9 Ko de RAM statique économisés vs un tampon plein écran).
        for (int y0 = 0; y0 < SP_PH; y0 += SP_STRIP) {
            int sh = (y0 + SP_STRIP <= SP_PH) ? SP_STRIP : (SP_PH - y0);
            memset(scratch_, 0, (size_t)SP_PW * sh * sizeof(uint16_t));
            for (int b = 0; b + 1 < SP_BARS; b++)
                plotSeg(px[b], py[b], px[b + 1], py[b + 1], y0, sh);
            tft_.blit(SP_PX, SP_PY + y0, SP_PW, sh, scratch_);
        }
        specDrawn_ = !empty;
    }

    static uint16_t specColor(int y) {          // y : 0 = haut du tracé
        int pc = (SP_PH - 1 - y) * 100 / (SP_PH - 1);
        return pc > 75 ? REDX : (pc > 45 ? AMBER : MTRG);
    }
    void plotSeg(int x0, int y0, int x1, int y1, int sy0, int sh) {   // Bresenham, trait 2 px
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dyy = -abs(y1 - y0), sdy = y0 < y1 ? 1 : -1;
        int err = dx + dyy;
        for (;;) {
            uint16_t c = specColor(y0);
            putScratch(x0, y0 - sy0, sh, c);
            putScratch(x0, y0 + 1 - sy0, sh, c);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dyy) { err += dyy; x0 += sx; }
            if (e2 <= dx)  { err += dx;  y0 += sdy; }
        }
    }
    void putScratch(int x, int ly, int sh, uint16_t c) {
        if (x >= 0 && x < SP_PW && ly >= 0 && ly < sh) scratch_[ly * SP_PW + x] = c;
    }
#endif

    Ili9225  tft_;
    SemaphoreHandle_t mtx_ = nullptr;
    RadioFace pending_, c_;
    bool     first_ = true;
    uint8_t  dgPrev_[7][DM_ROWS] = {{0}};   // état matrice de chaque chiffre
    // Tampon de tracé partagé : un chiffre (25x45) OU une bande de spectre
    // (SP_PW x SP_STRIP). Dimensionné pour le plus grand des deux.
    static const int DIG_AREA = (DM_COLS * DM_BIG) * (DM_ROWS * DM_BIG);
    static const int STRIP_AREA = SP_PW * SP_STRIP;
    static const int SCRATCH_N = DIG_AREA > STRIP_AREA ? DIG_AREA : STRIP_AREA;
    uint16_t scratch_[SCRATCH_N];
    PcmSource pcm_;
#if DISPLAY_SPECTRUM
    AudioSpectrum<SP_N> spectrum_;
    int16_t  specPcm_[SP_N];
    uint8_t  specVal_[SP_BARS] = {0};
    bool     specDrawn_ = false;
    uint32_t specDbgMs_ = 0;
    uint32_t specIdleMs_ = 0;
#endif
};

#endif  // DISPLAY_HAS_ILI9225
