#pragma once
#include <Arduino.h>
#include <string.h>

// ============================================================================
// Publication du service SDP vendor "BS AOC" de la VR-N76 :
//     39144315-32fa-40db-85ed-fbfeba2d86e6
//
// POURQUOI C'EST INDISPENSABLE
// HTCommander n'identifie une "radio compatible" QUE par la presence de ce
// service dans les enregistrements SDP de l'appareil appaire (verifie dans
// son code : src/windows/runner/bluetooth_classic_plugin.cpp -> kBsAocUuid,
// src/linux/runner/bluetooth_classic_plugin.cc -> kRadioServiceUuid). Ni la
// MAC, ni le nom, ni la Class of Device ne sont regardes. Sans ce service,
// l'ESP32 n'apparait jamais dans la liste des radios.
//
// COMMENT
// esp_spp_start_srv() n'enregistre que des records SDP "SerialPort" (0x1101).
// L'API publique esp_sdp_api.h (esp_sdp_create_record / ESP_SDP_TYPE_RAW)
// permettrait de publier un UUID arbitraire, mais elle n'est compilee dans
// AUCUNE lib Arduino-ESP32 precompilee (2.x comme 3.x : CONFIG_BT_SDP off).
//
// En revanche, les primitives bas niveau de la base de donnees SDP de
// Bluedroid (SDP_CreateRecord / SDP_AddAttribute / SDP_AddProtocolList /
// SDP_AddUuidSequence) SONT presentes dans libbt.a - c'est SPP lui-meme qui
// s'en sert pour publier son 0x1101. On les appelle donc directement pour
// creer un second record portant l'UUID 128 bits vendor + le canal RFCOMM
// audio. C'est exactement ce que fait la fonction add_raw_sdp() de
// components/bt/.../btc/profile/std/sdp/btc_sdp.c dans l'IDF recent, reproduit
// ici sans dependre du header interne (non livre par le framework Arduino).
//
// Ces prototypes / constantes proviennent de :
//   components/bt/host/bluedroid/stack/include/stack/sdp_api.h
//   components/bt/host/bluedroid/stack/include/stack/sdpdefs.h
// API stable de Bluedroid depuis des annees (memes signatures IDF 4.x / 5.x).
// ============================================================================

extern "C" {
    // --- stack/sdp_api.h -----------------------------------------------------
    #ifndef SDP_MAX_PROTOCOL_PARAMS
    #define SDP_MAX_PROTOCOL_PARAMS 2   // valeur par defaut du stack Bluedroid
    #endif

    typedef struct {
        uint16_t protocol_uuid;
        uint16_t num_params;
        uint16_t params[SDP_MAX_PROTOCOL_PARAMS];
    } tSDP_PROTOCOL_ELEM;

    uint32_t SDP_CreateRecord(void);
    uint8_t  SDP_DeleteRecord(uint32_t handle);
    uint8_t  SDP_AddAttribute(uint32_t handle, uint16_t attr_id,
                              uint8_t attr_type, uint32_t attr_len,
                              uint8_t* p_val);
    uint8_t  SDP_AddProtocolList(uint32_t handle, uint16_t num_elem,
                                 tSDP_PROTOCOL_ELEM* p_elem_list);
    uint8_t  SDP_AddUuidSequence(uint32_t handle, uint16_t attr_id,
                                 uint16_t num_uuids, uint16_t* p_uuids);
}

namespace VendorSdp {

    // --- stack/sdpdefs.h ------------------------------------------------------
    static const uint16_t ATTR_ID_SERVICE_CLASS_ID_LIST = 0x0001;
    static const uint16_t ATTR_ID_BROWSE_GROUP_LIST     = 0x0005;
    static const uint16_t ATTR_ID_SERVICE_NAME          = 0x0100; // LANGUAGE_BASE_ID + 0

    static const uint16_t UUID_PROTOCOL_RFCOMM          = 0x0003;
    static const uint16_t UUID_PROTOCOL_L2CAP           = 0x0100;
    static const uint16_t UUID_SERVCLASS_PUBLIC_BROWSE  = 0x1002;

    static const uint8_t  UINT_DESC_TYPE                = 1;
    static const uint8_t  UUID_DESC_TYPE                = 3;
    static const uint8_t  TEXT_STR_DESC_TYPE            = 4;
    static const uint8_t  DATA_ELE_SEQ_DESC_TYPE        = 6;
    static const uint8_t  SIZE_SIXTEEN_BYTES            = 4; // index de taille (16 octets)

    // UUID vendor "BS AOC", en BIG-ENDIAN (ordre de lecture) : c'est la forme
    // attendue dans un enregistrement SDP et par la recherche SDP entrante.
    static const uint8_t BS_AOC_UUID128_BE[16] = {
        0x39, 0x14, 0x43, 0x15, 0x32, 0xFA, 0x40, 0xDB,
        0x85, 0xED, 0xFB, 0xFE, 0xBA, 0x2D, 0x86, 0xE6
    };

    // Cree le record SDP vendor pointant sur le canal RFCOMM `audioScn`.
    // Retourne le handle SDP (0 = echec). A appeler APRES esp_spp_init() et le
    // demarrage des serveurs SPP (la base SDP est alors initialisee).
    inline uint32_t publishBsAoc(uint8_t audioScn, const char* serviceName = "BS Audio") {
        uint32_t h = SDP_CreateRecord();
        if (h == 0) {
            Serial.println("[SDP] SDP_CreateRecord() a echoue");
            return 0;
        }

        bool ok = true;

        // 1) ServiceClassIDList = { UUID 128 bits vendor }
        uint8_t scList[1 + 16];
        scList[0] = (uint8_t)((UUID_DESC_TYPE << 3) | SIZE_SIXTEEN_BYTES); // 0x1C
        memcpy(&scList[1], BS_AOC_UUID128_BE, 16);
        ok &= SDP_AddAttribute(h, ATTR_ID_SERVICE_CLASS_ID_LIST,
                               DATA_ELE_SEQ_DESC_TYPE, sizeof(scList), scList);

        // 2) ProtocolDescriptorList = L2CAP, RFCOMM(channel = audioScn)
        tSDP_PROTOCOL_ELEM proto[2];
        memset(proto, 0, sizeof(proto));
        proto[0].protocol_uuid = UUID_PROTOCOL_L2CAP;
        proto[0].num_params    = 0;
        proto[1].protocol_uuid = UUID_PROTOCOL_RFCOMM;
        proto[1].num_params    = 1;
        proto[1].params[0]     = audioScn;
        ok &= SDP_AddProtocolList(h, 2, proto);

        // 3) BrowseGroupList = PublicBrowseGroup (rend le service decouvrable)
        uint16_t browse = UUID_SERVCLASS_PUBLIC_BROWSE;
        ok &= SDP_AddUuidSequence(h, ATTR_ID_BROWSE_GROUP_LIST, 1, &browse);

        // 4) Nom de service (cosmetique, mais present sur la vraie radio)
        ok &= SDP_AddAttribute(h, ATTR_ID_SERVICE_NAME, TEXT_STR_DESC_TYPE,
                               (uint32_t)(strlen(serviceName) + 1),
                               (uint8_t*)serviceName);

        if (!ok) {
            Serial.println("[SDP] Un SDP_Add* a echoue, suppression du record");
            SDP_DeleteRecord(h);
            return 0;
        }

        Serial.printf("[SDP] Service vendor 39144315-... publie sur RFCOMM scn=%u "
                      "(handle=%lu)\n", audioScn, (unsigned long)h);
        return h;
    }

} // namespace VendorSdp
