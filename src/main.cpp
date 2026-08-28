#include <Arduino.h>
#include <esp_mac.h>
#include "config.h"

#if USE_DUAL_RFCOMM_SERVERS
    #include "DualRfcommServers.h"
    #include "Sa818.h"
    DualRfcommServers rfcomm;
    Sa818 rfModule;
#else
    #include "AudioRfcomm.h"
    BenshiAudioLink rfcomm;
#endif

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=== Simulateur radio Benshi (VR-N76) sur ESP32 ===");
    Serial.printf("[BUILD] firmware compile le %s %s\n", __DATE__, __TIME__);
    Serial.println("[INFO] Bluetooth Classic uniquement (voir config.h / README)");

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
    } else {
        Serial.println("[MODE] === UV-K1 : simulation + passerelle vers poste externe ===");
    }
#else
    Serial.println("[MODE] === UV-K1 : sonde SA818 desactivee (RF_MODULE_ENABLE=false) ===");
#endif

    if (!rfcomm.begin(rf)) {
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
