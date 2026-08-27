#pragma once
// ============================================================================
// FICHIER NON UTILISÉ / NON COMPILÉ PAR DÉFAUT — conservé pour référence.
//
// Le dump `bluetoothctl` de ta VR-N76 réelle montre qu'elle n'utilise QUE
// du Bluetooth Classic (BR/EDR), pas de service BLE. Ce fichier n'est donc
// inclus par aucun .cpp du projet (voir main.cpp) et ne sera pas compilé.
//
// De plus, NimBLE (utilisé ici) et Bluedroid Classic (utilisé par
// DualRfcommServers.h / AudioRfcomm.h) ne peuvent PAS être actifs
// simultanément dans un même firmware ESP32 : ce sont deux piles hôte
// Bluetooth différentes et incompatibles pour un même contrôleur radio.
// Utiliser ce fichier suppose donc de retirer les modules Classic et de
// réintégrer la lib NimBLE-Arduino dans platformio.ini.
//
// Il référence des constantes (BLE_DEVICE_NAME, BLE_SERVICE_UUID, ...) qui
// ont été retirées de config.h lors du passage tout-Classic : à rétablir
// si tu réactives un jour ce transport pour un autre modèle Benshi qui,
// lui, exposerait vraiment un service BLE.
// ============================================================================
#include <NimBLEDevice.h>
#include "BenshiProtocol.h"
#include "BenshiCommandHandler.h"
#include "config.h"

// ============================================================================
// Serveur BLE simulant le service de commande Benshi.
// - Le client (HTCommander) écrit un BenshiMessage sur la caractéristique
//   WRITE.
// - On construit la réponse via BenshiCommandHandler et on la pousse via une
//   INDICATION sur la caractéristique INDICATE.
// ============================================================================

class BenshiBleServer : public NimBLECharacteristicCallbacks, public NimBLEServerCallbacks {
public:
    void begin() {
        NimBLEDevice::init(BLE_DEVICE_NAME);
        NimBLEDevice::setMTU(247); // MTU large pour laisser passer les RfCh/Settings

        server_ = NimBLEDevice::createServer();
        server_->setCallbacks(this);

        NimBLEService* svc = server_->createService(BLE_SERVICE_UUID);

        writeChar_ = svc->createCharacteristic(
            BLE_CHAR_WRITE_UUID,
            NIMBLE_PROPERTY::WRITE // "write with response" côté client
        );
        writeChar_->setCallbacks(this);

        indicateChar_ = svc->createCharacteristic(
            BLE_CHAR_INDICATE_UUID,
            NIMBLE_PROPERTY::INDICATE
        );

        // NimBLE-Arduino 2.x : plus besoin d'appeler svc->start(), les
        // services démarrent automatiquement avec server_->start().
        server_->start();

        NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
        adv->addServiceUUID(BLE_SERVICE_UUID);
        adv->setName(BLE_DEVICE_NAME);
        // Pas d'appel setScanResponse*() ici : la signature de cette méthode
        // a changé plusieurs fois entre versions mineures de NimBLE-Arduino
        // 2.x (bool -> NimBLEAdvertisementData). Par défaut, NimBLE inclut
        // déjà le nom et l'UUID de service dans les paquets d'advertising
        // standards, ce qui est suffisant pour être détecté par un scan BLE
        // (nRF Connect, HTCommander/benlink). Si tu veux forcer des données
        // de scan-response custom, regarde NimBLEAdvertising::setScanResponseData()
        // dans la version exacte de la lib installée chez toi.
        adv->start();

        Serial.printf("[BLE] Service Benshi annoncé sous le nom '%s'\n", BLE_DEVICE_NAME);
    }

    // --- NimBLEServerCallbacks ---------------------------------------------
    void onConnect(NimBLEServer* server, NimBLEConnInfo& info) override {
        Serial.println("[BLE] Client connecté");
    }
    void onDisconnect(NimBLEServer* server, NimBLEConnInfo& info, int reason) override {
        Serial.println("[BLE] Client déconnecté, relance de l'advertising");
        NimBLEDevice::startAdvertising();
    }

    // --- NimBLECharacteristicCallbacks --------------------------------------
    void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo& info) override {
        std::string raw = chr->getValue();
        BenshiMessage in;
        if (!decodeMessage((const uint8_t*)raw.data(), raw.size(), in)) {
            Serial.println("[BLE] Message trop court, ignoré");
            return;
        }
        BenshiMessage out;
        if (handler_.process(in, out)) {
            std::vector<uint8_t> encoded = encodeMessage(out);
            indicateChar_->setValue(encoded.data(), encoded.size());
            indicateChar_->indicate();
        }
    }

private:
    NimBLEServer* server_ = nullptr;
    NimBLECharacteristic* writeChar_ = nullptr;
    NimBLECharacteristic* indicateChar_ = nullptr;
    BenshiCommandHandler handler_;
};
