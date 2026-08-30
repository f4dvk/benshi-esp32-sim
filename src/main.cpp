#include <Arduino.h>
#include <esp_mac.h>
#include "config.h"

#if USE_DUAL_RFCOMM_SERVERS
    #include "DualRfcommServers.h"
    #include "Sa818.h"
    DualRfcommServers rfcomm;
    Sa818 rfModule;
    #if RF_MODULE_UVK5_ENABLE
        #include "UvK5.h"
        UvK5 uvk5Module;
    #endif
#else
    #include "AudioRfcomm.h"
    BenshiAudioLink rfcomm;
#endif

#if RF_MODULE_UVK5_SELFTEST
    #include "UvK5Link.h"
    static UvK5Link uvk5link;
#endif

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=== Simulateur radio Benshi (VR-N76) sur ESP32 ===");
    Serial.printf("[BUILD] firmware compile le %s %s\n", __DATE__, __TIME__);
    Serial.println("[INFO] Bluetooth Classic uniquement (voir config.h / README)");

#if RF_MODULE_UVK5_SELFTEST
    Serial.println("[UVK5] === TEST DE SANITE LIAISON SERIE (Phase 1) ===");
    Serial.printf("[UVK5] UART RF : RX=GPIO%d TX=GPIO%d @ %d bauds\n",
                  RF_MODULE_UART_RX, RF_MODULE_UART_TX, RF_MODULE_UVK5_BAUD);
    uvk5link.begin(&Serial2, RF_MODULE_UART_RX, RF_MODULE_UART_TX, RF_MODULE_UVK5_BAUD);
    delay(200);
    if (!uvk5link.probe())
        Serial.println("[UVK5] pas de reponse 0x0514 (cable croise ? baud ? firmware ?)");
    for (;;) {
        uint16_t r67 = 0;
        if (uvk5link.readBkReg(0x67, r67))
            Serial.printf("[UVK5] BK4819 REG_67 = 0x%04X (RSSI brut %u)\n", r67, r67 & 0x1FF);
        else
            Serial.println("[UVK5] lecture REG_67 : pas de reponse");
        delay(2000);
    }
#endif

#if OVERRIDE_BT_MAC
    // Sur l'ESP32, la MAC Bluetooth = base MAC avec le dernier octet + 2
    // (cf. esp_read_mac(), ESP_MAC_BT). On remonte donc a la base MAC pour
    // que la MAC BT effective soit EXACTEMENT CUSTOM_BT_MAC.
    {
        uint8_t base[6];
        memcpy(base, CUSTOM_BT_MAC, 6);
        base[5] -= 2; // addition 8 bits sans retenue cote IDF -> soustraction directe
        esp_err_t merr = esp_base_mac_addr_set(base);
        Serial.printf("[BT] MAC Bluetooth cible %02X:%02X:%02X:%02X:%02X:%02X"
                      " (base %02X:%02X:%02X:%02X:%02X:%02X) -> %s\n",
                      CUSTOM_BT_MAC[0], CUSTOM_BT_MAC[1], CUSTOM_BT_MAC[2],
                      CUSTOM_BT_MAC[3], CUSTOM_BT_MAC[4], CUSTOM_BT_MAC[5],
                      base[0], base[1], base[2], base[3], base[4], base[5],
                      merr == ESP_OK ? "OK" : esp_err_to_name(merr));
    }
#endif

#if USE_DUAL_RFCOMM_SERVERS
    // Choix du mode : sonde un module RF SA818/DRA818 sur l'UART RF.
    Sa818* rf = nullptr;
#if RF_MODULE_ENABLE
    Serial.println("[MODE] Recherche d'un module RF SA818/DRA818...");
    if (rfModule.begin(RF_MODULE_UART_RX, RF_MODULE_UART_TX,
                       RF_MODULE_PD_GPIO, RF_MODULE_PROBES)) {
        rf = &rfModule;
        Serial.println("[MODE] === SA818 : pilotage d'un module RF reel ===");
    }
#else
    Serial.println("[MODE] === sonde SA818 desactivee (RF_MODULE_ENABLE=false) ===");
#endif

#if RF_MODULE_UVK5_ENABLE
    UvK5* uvk5 = nullptr;
    if (!rf) {
        // Pas de SA818 -> tente un poste Quansheng UV-K1 / UV-K5 V3 (mode hote)
        // sur le meme UART, a 38400 bauds.
        Serial.println("[MODE] Recherche d'un poste Quansheng UV-K1 / UV-K5 V3...");
        Serial2.end();
        if (uvk5Module.begin(&Serial2)) {
            uvk5 = &uvk5Module;
            Serial.println("[MODE] === UV-K1 : poste Quansheng pilote (mode hote) ===");
        } else {
            Serial2.end();
        }
    }
    if (!rf && !uvk5)
        Serial.println("[MODE] === UV-K1 : simulation + passerelle vers poste externe ===");
    if (!rfcomm.begin(rf, uvk5)) {
#else
    if (!rf)
        Serial.println("[MODE] === UV-K1 : simulation + passerelle vers poste externe ===");
    if (!rfcomm.begin(rf)) {
#endif
        Serial.println("[FATAL] Demarrage Bluetooth Classic impossible (voir erreurs ci-dessus).");
        Serial.println("        Verifie que la carte est bien un ESP32 'classique' (WROOM-32 /");
        Serial.println("        DevKitC / WROVER) : les S3/C3/C6 n'ont pas de Bluetooth Classic.");
        return;
    }
#else
    rfcomm.begin();
#endif

    Serial.println("[OK] En attente de connexion HTCommander...");
    Serial.printf("     Nom Bluetooth : %s\n", BT_CLASSIC_NAME);
    Serial.printf("     Class of Device : 0x%06X\n", DEVICE_CLASS_OF_DEVICE);
#if USE_DUAL_RFCOMM_SERVERS
    Serial.println("     -> 2 canaux RFCOMM (commande GaiaFrame + audio)");
#else
    Serial.println("     -> 1 canal RFCOMM (audio uniquement, BluetoothSerial)");
#endif
}

void loop() {
#if USE_DUAL_RFCOMM_SERVERS
    rfcomm.poll();  // retune différé du module RF (mode SA818) ; no-op sinon
#else
    rfcomm.loop();
#endif
    delay(5);
}
