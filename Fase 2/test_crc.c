#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* ============================================================
 *  CRC-16 del IEEE 802.15.7  (TFG VLC sobre RP2040)
 *
 *  Sirve tanto para el HCS (cabecera) como para el FCS (datos):
 *  el estandar define el MISMO CRC-16 para ambos.
 *
 *  Parametros tomados del Anexo C del IEEE 802.15.7-2018:
 *    - Polinomio generador : x^16 + x^12 + x^5 + 1  = 0x1021
 *    - Registro inicial     : todo unos             = 0xFFFF
 *    - Orden de bits        : LSB-first (reflejado)
 *    - XOR final            : ninguno               = 0x0000
 *
 *  Equivale al CRC-16/MCRF4XX. Vector de referencia:
 *    CRC("123456789") = 0x6F91   (validado, ver main()).
 * ============================================================ */

#define CRC_POLY      0x1021u
#define CRC_INIT      0xFFFFu
#define CRC_REFLECT   1
#define CRC_XOROUT    0x0000u

static uint8_t reflect8(uint8_t b) {
    uint8_t r = 0;
    for (int i = 0; i < 8; i++) if (b & (1u << i)) r |= (1u << (7 - i));
    return r;
}
static uint16_t reflect16(uint16_t w) {
    uint16_t r = 0;
    for (int i = 0; i < 16; i++) if (w & (1u << i)) r |= (1u << (15 - i));
    return r;
}

/* CRC-16 del 802.15.7 sobre 'len' bytes. Usar para HCS y para FCS. */
uint16_t crc16_802157(const uint8_t *data, size_t len) {
    uint16_t crc = CRC_INIT;
    for (size_t i = 0; i < len; i++) {
        uint8_t b = CRC_REFLECT ? reflect8(data[i]) : data[i];
        crc ^= (uint16_t)b << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000u) crc = (uint16_t)((crc << 1) ^ CRC_POLY);
            else               crc = (uint16_t)(crc << 1);
        }
    }
    if (CRC_REFLECT) crc = reflect16(crc);
    return crc ^ CRC_XOROUT;
}

int main(void) {
    /* --- Autotest contra el vector de referencia --- */
    uint16_t r = crc16_802157((const uint8_t *)"123456789", 9);
    printf("CRC(\"123456789\") = 0x%04X  (referencia: 0x6F91)\n", r);
    printf(r == 0x6F91 ? "VALIDACION OK\n\n" : "ERROR: no coincide\n\n");

    /* --- Ejemplo de uso: FCS de una trama --- */
    uint8_t trama[] = { 0xA5, 0x01, 0x02, 0x03, 0x04 };
    uint16_t fcs = crc16_802157(trama, sizeof(trama));
    printf("Trama A5 01 02 03 04 -> FCS = 0x%04X (hi=0x%02X lo=0x%02X)\n",
           fcs, (fcs >> 8) & 0xFF, fcs & 0xFF);
    return 0;
}