#include <Arduino.h>
#include <esp_mac.h>
#include "config.h"

#if USE_DUAL_RFCOMM_SERVERS
    #include "DualRfcommServers.h"
    DualRfcommServers rfcomm;
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
    if (!rfcomm.begin()) {
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
#if !USE_DUAL_RFCOMM_SERVERS
    rfcomm.loop(); // DualRfcommServers est piloté par callback, rien à boucler ici
#endif
    delay(2);
}
