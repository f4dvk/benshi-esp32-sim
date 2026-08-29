#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <string.h>
#include <math.h>
#include "config.h"

// ============================================================================
// Réglages APRS de la "radio", pilotables depuis HTCommander :
//
//   READ_BSS_SETTINGS  (33) / WRITE_BSS_SETTINGS (34)  -> structure BSS 46 o
//        indicatif, SSID, symbole (icône), intervalle de balise, message de
//        balise, ID station (ident au relâché de PTT), drapeaux...
//   GET_APRS_PATH      (72) / SET_APRS_PATH      (71)  -> chaîne de digis
//   GET_POSITION       (76) / SET_POSITION       (32)  -> position fixe
//
// Tout est persisté en NVS (namespace "aprs") et survit au reboot. Au premier
// démarrage, les valeurs sont semées depuis les APRS_* de config.h.
//
// La balise autonome de l'ESP (AprsBeacon.h) lit ces réglages : configurer
// une fois dans HTCommander suffit, la radio continue de baliser sans lui.
// ============================================================================

class AprsConfig {
public:
    static const size_t BSS_LEN = 46;

    void begin() {
        nvsOk_ = prefs_.begin("aprs", false);

        // --- BSS : défaut construit depuis config.h ---
        buildDefaultBss(bss_);
        if (nvsOk_ && prefs_.getBytesLength("bss") == BSS_LEN)
            prefs_.getBytes("bss", bss_, BSS_LEN);

        // --- Chemin de digipeaters ---
        path_ = APRS_PATH;
        if (nvsOk_ && prefs_.isKey("path")) path_ = prefs_.getString("path", path_);

        // --- Position fixe (raw = degrés * 30000, comme le protocole Benshi) ---
        latRaw_ = degToRaw((double)APRS_FIXED_LAT);
        lonRaw_ = degToRaw((double)APRS_FIXED_LON);
        if (nvsOk_) {
            latRaw_ = prefs_.getInt("lat", latRaw_);
            lonRaw_ = prefs_.getInt("lon", lonRaw_);
        }

        // --- Canal de balise (auto_share_loc_ch, onglet Beacon) : 0 = courant,
        //     N = canal mémoire N-1. Persisté à part de la structure Settings
        //     (que HTCommander réécrit sans ce champ lors de ses réconciliations).
        if (nvsOk_) beaconChannel_ = prefs_.getUChar("bcnch", 0);

        Serial.printf("[APRS] config : %s-%d  sym '%c%c'  balise %ds  path \"%s\"  pos %.5f,%.5f  (%s)\n",
                      callsign().c_str(), ssid(), symbolTable(), symbolCode(),
                      intervalSec(), path_.c_str(), lat(), lon(),
                      nvsOk_ ? "NVS" : "config.h seule");
    }

    // ---- BSS (structure 46 octets, telle qu'échangée avec HTCommander) ----
    const uint8_t* bss() const { return bss_; }

    // -1 = rejeté (taille), 0 = accepté mais inchangé, 1 = modifié + sauvé.
    int setBss(const uint8_t* data, size_t len) {
        if (!data || len < BSS_LEN) return -1;
        if (!memcmp(bss_, data, BSS_LEN)) { Serial.println("[APRS] BSS inchange"); return 0; }
        memcpy(bss_, data, BSS_LEN);
        size_t w = nvsOk_ ? prefs_.putBytes("bss", bss_, BSS_LEN) : 0;
        Serial.printf("[APRS] BSS mis a jour : %s-%d  sym '%c%c'  balise %ds  partage=%d  ID \"%s\"  msg \"%s\"  [NVS %u o]\n",
                      callsign().c_str(), ssid(), symbolTable(), symbolCode(),
                      intervalSec(), shouldShareLocation() ? 1 : 0,
                      stationId().c_str(), beaconMessage().c_str(), (unsigned)w);
        if (nvsOk_ && w != BSS_LEN) Serial.println("[APRS] !!! ecriture NVS BSS INCOMPLETE (NVS pleine ?)");
        return 1;
    }

    // ---- Chemin ----------------------------------------------------------
    const String& path() const { return path_; }
    bool setPath(const String& p) {
        if (p == path_) return true;
        path_ = p;
        if (nvsOk_) prefs_.putString("path", path_);
        Serial.printf("[APRS] chemin (path) : \"%s\"\n", path_.c_str());
        return true;
    }

    // ---- Canal de balise ("auto_share_loc_ch") -----------------------
    // 0 = canal courant ; N = canal mémoire N-1.
    uint8_t beaconChannel() const { return beaconChannel_; }

    // Signale une écriture WRITE_BSS_SETTINGS : l'onglet Beacon envoie le BSS
    // PUIS, dans la foulée, le WRITE_SETTINGS qui porte auto_share_loc_ch.
    void noteBssWrite() { bssWriteMs_ = millis(); bssWriteConsumed_ = false; }

    // Appelé à CHAQUE WRITE_SETTINGS. Seul le PREMIER dans les 1,5 s suivant un
    // WRITE_BSS_SETTINGS fait foi (= l'écriture de l'onglet Beacon, valeur 0 ou
    // canal). Les suivants (verrou/déverrou APRS, réconciliation VFO) portent
    // une valeur périmée -> on les ignore et on garde le canal choisi.
    void noteAutoShareLocCh(uint8_t asc) {
        bool authoritative = !bssWriteConsumed_ && (millis() - bssWriteMs_ < 1500);
        if (!authoritative) return;
        bssWriteConsumed_ = true;
        if (asc != beaconChannel_) {
            beaconChannel_ = asc;
            if (nvsOk_) prefs_.putUChar("bcnch", asc);
            Serial.printf("[APRS] canal de balise = %u%s\n",
                          asc, asc ? "" : " (canal courant)");
        }
    }

    // ---- Position fixe -------------------------------------------------
    int32_t latRaw() const { return latRaw_; }
    int32_t lonRaw() const { return lonRaw_; }
    double  lat()    const { return latRaw_ / 30000.0; }
    double  lon()    const { return lonRaw_ / 30000.0; }
    bool setPositionRaw(int32_t la, int32_t lo) {
        if (la == latRaw_ && lo == lonRaw_) return true;
        latRaw_ = la; lonRaw_ = lo;
        if (nvsOk_) { prefs_.putInt("lat", latRaw_); prefs_.putInt("lon", lonRaw_); }
        Serial.printf("[APRS] position fixe : %.5f, %.5f\n", lat(), lon());
        return true;
    }

    // ---- Champs décodés du BSS (pour la balise autonome) --------------
    uint8_t ssid()        const { return (bss_[2] >> 4) & 0x0F; }
    char    symbolTable() const { return bss_[38] ? (char)bss_[38] : '/'; }
    char    symbolCode()  const { return bss_[39] ? (char)bss_[39] : '-'; }
    String  callsign()    const { return trimmed(bss_ + 40, 6); }
    String  beaconMessage() const { return trimmed(bss_ + 20, 18); }
    String  stationId()   const { return trimmed(bss_ + 8, 12); }   // ident au relâché de PTT
    uint16_t intervalSec() const {
        uint16_t s = (uint16_t)bss_[3] * 10;
        return s ? s : (uint16_t)(APRS_BEACON_INTERVAL_MIN * 60);
    }
    bool shouldShareLocation() const { return bss_[1] & 0x10; }

    static int32_t degToRaw(double d) { return (int32_t)lround(d * 30000.0); }

private:
    static String trimmed(const uint8_t* p, size_t n) {
        char b[24]; if (n > sizeof(b) - 1) n = sizeof(b) - 1;
        size_t k = 0;
        for (size_t i = 0; i < n; i++) {
            char c = (char)p[i];
            if (c == 0) break;
            b[k++] = c;
        }
        while (k && (b[k - 1] == ' ')) k--;
        b[k] = 0;
        return String(b);
    }

    static void putPadded(uint8_t* dst, const char* s, size_t n) {
        size_t l = strnlen(s, n);
        for (size_t i = 0; i < n; i++) dst[i] = (i < l) ? (uint8_t)s[i] : ' ';
    }

    static void buildDefaultBss(uint8_t* b) {
        memset(b, 0, BSS_LEN);
        b[0] = 0x31;                                   // maxFwdTimes=3, ttl=1
        b[1] = 0x10;                                   // shouldShareLocation
        b[2] = (uint8_t)((APRS_SSID & 0x0F) << 4);
        b[3] = (uint8_t)((APRS_BEACON_INTERVAL_MIN * 60) / 10);
        putPadded(b + 8,  APRS_CALLSIGN, 12);          // ID station par défaut = indicatif
        putPadded(b + 20, APRS_COMMENT, 18);          // message de balise
        b[38] = (uint8_t)APRS_SYMBOL_TABLE;
        b[39] = (uint8_t)APRS_SYMBOL;
        putPadded(b + 40, APRS_CALLSIGN, 6);
    }

    Preferences prefs_;
    bool     nvsOk_ = false;
    uint8_t  bss_[BSS_LEN];
    String   path_;
    int32_t  latRaw_ = 0, lonRaw_ = 0;
    uint8_t  beaconChannel_ = 0;
    uint32_t bssWriteMs_ = 0;
    bool     bssWriteConsumed_ = true;
};
