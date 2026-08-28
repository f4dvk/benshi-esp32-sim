#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <vector>
#include <cstring>
#include <cmath>
#include "BitStream.h"
#include "config.h"

// ============================================================================
// Etat RUNTIME modifiable de la radio simulee.
//
// Au boot il est initialise depuis config.h, PUIS il devient modifiable par
// HTCommander :
//   - WRITE_RF_CH      -> reecrit un canal memoire (frequences, CTCSS, nom,
//                         largeur de bande, TX interdit...) = "editer un canal"
//   - WRITE_SETTINGS   -> change le canal VFO A / VFO B, le mode double veille
//                         (VFO actif), le squelch, le scan = "changer de VFO /
//                         de canal memoire"
//   - SET_REGION       -> change la region reglementaire active
//   - WRITE_REGION_NAME-> renomme une region
//
// Les modifications sont ecrites en NVS (Preferences) et SURVIVENT au reboot.
// Pour repartir des valeurs de config.h : mettre FACTORY_RESET_ON_BOOT a true
// (efface la NVS une fois), ou effacer la flash (`pio run -t erase`).
//
// Les structures binaires (canal = 25 octets, reglages = 20 octets) sont
// stockees telles quelles : ce que HTCommander ecrit est exactement ce qu'il
// relit ensuite -> l'editeur de canal se met a jour correctement.
// ============================================================================

class RadioState {
public:
    static const size_t CH_STRUCT_LEN       = 25;
    static const size_t SETTINGS_STRUCT_LEN = 20;

    void begin() {
#if FACTORY_RESET_ON_BOOT
        {
            Preferences p;
            p.begin("benshi", false);
            p.clear();
            p.end();
            Serial.println("[STATE] FACTORY_RESET_ON_BOOT : NVS effacee, retour a config.h");
        }
#endif
        // NVS = memoire flash non volatile (partition "nvs" de la table de
        // partitions). begin() renvoie false si la partition est absente ou
        // pleine : dans ce cas les modifs ne seront PAS conservees au reboot.
        nvsOk_ = prefs_.begin("benshi", false);
        if (!nvsOk_) {
            Serial.println("[STATE] !!! NVS 'benshi' inaccessible -> les modifs de canaux "
                           "NE SERONT PAS conservees apres coupure d'alimentation.");
        }

        uint8_t chFromNvs = 0;
        for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
            chan_[i] = encodeChannelStruct(CHANNELS[i]);
            char key[8];
            snprintf(key, sizeof(key), "ch%u", i);
            if (loadBlob(key, chan_[i], CH_STRUCT_LEN)) chFromNvs++;
        }

        settings_ = encodeSettingsStruct(DEFAULT_CHANNEL_A, DEFAULT_CHANNEL_B,
                                         DEFAULT_SQUELCH, DEFAULT_MIC_GAIN);
        bool settingsFromNvs = loadBlob("settings", settings_, SETTINGS_STRUCT_LEN);

        region_ = nvsOk_ ? prefs_.getUChar("region", DEFAULT_REGION) : DEFAULT_REGION;

        for (uint8_t i = 0; i < REGION_COUNT; i++) {
            char key[10];
            snprintf(key, sizeof(key), "rgn%u", i);
            regionNames_[i] = nvsOk_ ? prefs_.getString(key, REGION_NAMES[i])
                                     : String(REGION_NAMES[i]);
        }

        Serial.printf("[STATE] Restaure depuis la memoire permanente : %u/%u canaux, "
                      "reglages=%s\n", chFromNvs, CHANNEL_COUNT,
                      settingsFromNvs ? "oui" : "non (config.h)");
        Serial.printf("[STATE] pret : VFO-A=%u VFO-B=%u dual=%u squelch=%u region=%u\n",
                      channelA(), channelB(), doubleChannel(), squelch(), region_);
    }

    // ---- Canaux memoire --------------------------------------------------
    const std::vector<uint8_t>& channelStruct(uint8_t id) const { return chan_[id]; }

    // data/len = corps recu de WRITE_RF_CH (struct de 25 octets, data[0] = id).
    bool setChannelStruct(uint8_t id, const uint8_t* data, size_t len) {
        if (id >= CHANNEL_COUNT || data == nullptr || len < CH_STRUCT_LEN) return false;
        std::vector<uint8_t> v(data, data + CH_STRUCT_LEN);
        v[0] = id; // garde le champ channel_id coherent avec l'index
        if (v == chan_[id]) return true;
        chan_[id] = std::move(v);
        char key[8];
        snprintf(key, sizeof(key), "ch%u", id);
        persistBlob(key, chan_[id], "Canal", id, channelName(id));
        return true;
    }

    // ---- Reglages / selection VFO --------------------------------------
    const std::vector<uint8_t>& settingsStruct() const { return settings_; }

    // data/len = corps recu de WRITE_SETTINGS (20 octets ; tolere plus court).
    bool setSettingsStruct(const uint8_t* data, size_t len) {
        if (data == nullptr || len < 2) return false;
        std::vector<uint8_t> v(SETTINGS_STRUCT_LEN, 0);
        memcpy(v.data(), data, len < SETTINGS_STRUCT_LEN ? len : SETTINGS_STRUCT_LEN);
        if (v == settings_) return true;
        settings_ = std::move(v);
        persistBlob("settings", settings_, "Reglages", 0, String());
        Serial.printf("[STATE]   -> VFO-A=%u VFO-B=%u dual=%u squelch=%u scan=%u "
                      "canal actif=%u\n",
                      channelA(), channelB(), doubleChannel(), squelch(),
                      scan() ? 1 : 0, activeChannelId());
        return true;
    }

    uint8_t channelA() const {
        return (uint8_t)(((settings_[0] & 0xF0) >> 4) | (settings_[9] & 0xF0));
    }
    uint8_t channelB() const {
        return (uint8_t)((settings_[0] & 0x0F) | ((settings_[9] & 0x0F) << 4));
    }
    uint8_t doubleChannel() const { return (settings_[1] & 0x30) >> 4; } // 0=off,1=VFO A,2=VFO B
    uint8_t squelch()       const { return settings_[1] & 0x0F; }
    uint8_t micGain()       const { return (settings_[2] & 0x0E) >> 1; }
    bool    scan()          const { return (settings_[1] & 0x80) != 0; }

    // Canal actuellement "actif" -> renvoye par GET_HT_STATUS.
    uint8_t activeChannelId() const {
        uint8_t id = (doubleChannel() == 2) ? channelB() : channelA();
        return (id < CHANNEL_COUNT) ? id : 0;
    }

    // Nom du canal actif (10 octets max, decode de la struct binaire).
    String activeChannelName() const { return channelName(activeChannelId()); }
    String channelNameOf(uint8_t id) const {
        return (id < CHANNEL_COUNT) ? channelName(id) : String();
    }

    // Parametres RF decodes du canal actif (pour piloter un module SA818).
    struct ActiveRf {
        double  tx_mhz = 0, rx_mhz = 0;   // MHz
        double  tx_ctcss_hz = 0, rx_ctcss_hz = 0;
        bool    wide = true;              // bandwidth : 1 = 25 kHz, 0 = 12,5 kHz
        bool    tx_at_max_power = true;   // puissance haute demandee
        bool    emph_bypass = false;      // 1 = pas de pre/de-emphase
        bool    tx_disable = false;
    };
    ActiveRf activeRf() const { return decodeRf(chan_[activeChannelId()]); }

    static ActiveRf decodeRf(const std::vector<uint8_t>& s) {
        ActiveRf r;
        if (s.size() < CH_STRUCT_LEN) return r;
        BitReader br(s.data(), s.size());
        br.readBits(8);                              // channel_id
        br.readBits(2);                              // tx_mod
        r.tx_mhz = br.readBits(30) / 1e6;
        br.readBits(2);                              // rx_mod
        r.rx_mhz = br.readBits(30) / 1e6;
        r.tx_ctcss_hz = br.readBits(16) / 100.0;
        r.rx_ctcss_hz = br.readBits(16) / 100.0;
        br.readBits(1);                              // scan
        r.tx_at_max_power = br.readBits(1) != 0;     // tx_at_max_power
        br.readBits(1);                              // talk_around
        r.wide = br.readBits(1) != 0;                // bandwidth (1 = wide)
        r.emph_bypass = br.readBits(1) != 0;         // pre_de_emph_bypass
        br.readBits(1);                              // sign
        br.readBits(1);                              // tx_at_med_power
        r.tx_disable = br.readBits(1) != 0;          // tx_disable
        return r;
    }

    // ---- Region ----------------------------------------------------------
    uint8_t region() const { return region_; }
    void setRegion(uint8_t r) {
        if (r == region_) return;
        region_ = r;
        if (nvsOk_) prefs_.putUChar("region", r);
        Serial.printf("[STATE] Region active -> %u %s\n", r,
                      nvsOk_ ? "(sauvegardee)" : "(NON persistee)");
    }
    String regionName(uint8_t id) const {
        return (id < REGION_COUNT) ? regionNames_[id] : String();
    }
    void setRegionName(uint8_t id, const String& name) {
        if (id >= REGION_COUNT) return;
        regionNames_[id] = name;
        char key[10];
        snprintf(key, sizeof(key), "rgn%u", id);
        if (nvsOk_) prefs_.putString(key, name);
        Serial.printf("[STATE] Region %u renommee : \"%s\" %s\n", id, name.c_str(),
                      nvsOk_ ? "(sauvegardee)" : "(NON persistee)");
    }

    // ---- Encodeurs config.h -> struct binaire ---------------------------
    static std::vector<uint8_t> encodeChannelStruct(const ChannelConfig& c) {
        BitWriter w;
        w.writeBits(c.channel_id, 8);
        w.writeBits((uint8_t)c.tx_mod, 2);
        w.writeBits((uint32_t)llround(c.tx_freq_mhz * 1e6), 30);
        w.writeBits((uint8_t)c.rx_mod, 2);
        w.writeBits((uint32_t)llround(c.rx_freq_mhz * 1e6), 30);
        w.writeBits((uint32_t)llround(c.tx_sub_audio * 100), 16); // CTCSS Hz x100
        w.writeBits((uint32_t)llround(c.rx_sub_audio * 100), 16);
        w.writeBits(0, 1);                                 // scan
        w.writeBits(1, 1);                                 // tx_at_max_power
        w.writeBits(0, 1);                                 // talk_around
        w.writeBits(c.bandwidth == BW_WIDE ? 1 : 0, 1);    // bandwidth (1 = wide)
        w.writeBits(0, 1);                                 // pre_de_emph_bypass
        w.writeBits(0, 1);                                 // sign
        w.writeBits(0, 1);                                 // tx_at_med_power
        w.writeBits(c.tx_disable ? 1 : 0, 1);              // tx_disable
        w.writeBits(0, 8);                                 // fixed_* / mute
        w.writeString(c.name, 10);
        return w.bytes(); // 25 octets
    }

    static std::vector<uint8_t> encodeSettingsStruct(uint8_t chA, uint8_t chB,
                                                     uint8_t squelch, uint8_t micGain) {
        std::vector<uint8_t> b(SETTINGS_STRUCT_LEN, 0);
        b[0] = (uint8_t)(((chA & 0x0F) << 4) | (chB & 0x0F));
        b[1] = (uint8_t)(squelch & 0x0F); // scan = 0, double_channel = 0 (VFO A seul)
        b[2] = (uint8_t)((micGain & 0x07) << 1);
        b[9] = (uint8_t)((chA & 0xF0) | ((chB >> 4) & 0x0F));
        return b;
    }

private:
    String channelName(uint8_t id) const {
        const std::vector<uint8_t>& v = chan_[id];
        char n[11] = {0};
        for (int i = 0; i < 10 && (15 + i) < (int)v.size(); i++) n[i] = (char)v[15 + i];
        return String(n);
    }
    // Renvoie true si `dst` a bien ete rempli depuis la NVS (cle presente et
    // de la bonne taille), false si on garde la valeur par defaut de config.h.
    bool loadBlob(const char* key, std::vector<uint8_t>& dst, size_t len) {
        if (!nvsOk_) return false;
        if (prefs_.getBytesLength(key) != len) return false;   // absente ou taille differente
        return prefs_.getBytes(key, dst.data(), len) == len;
    }

    // Ecrit `src` en NVS et journalise le resultat (nb d'octets ecrits).
    void persistBlob(const char* key, const std::vector<uint8_t>& src,
                     const char* what, uint8_t idx, const String& label) {
        if (!nvsOk_) {
            Serial.printf("[STATE] %s %u modifie mais NON persiste (NVS KO)%s%s\n",
                          what, idx, label.length() ? " : " : "", label.c_str());
            return;
        }
        size_t n = prefs_.putBytes(key, src.data(), src.size());
        Serial.printf("[STATE] %s %u sauvegarde en memoire permanente (%u octets)%s%s\n",
                      what, idx, (unsigned)n, label.length() ? " : " : "", label.c_str());
    }

    std::vector<uint8_t> chan_[CHANNEL_COUNT];
    std::vector<uint8_t> settings_;
    uint8_t region_ = DEFAULT_REGION;
    String  regionNames_[REGION_COUNT > 0 ? REGION_COUNT : 1];
    Preferences prefs_;
    bool nvsOk_ = false;
};
