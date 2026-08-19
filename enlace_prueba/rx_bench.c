#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>

/* ============================================================
 *  BANCO DE PRUEBAS DEL RECEPTOR (chip-a-chip, canal ideal)
 *  TFG VLC sobre RP2040 - Pablo - UC3M
 *
 *  Reproduce en el PC la cadena completa TX -> canal -> RX:
 *
 *    build_frame -> manchester_encode -> [canal ideal] ->
 *      -> manchester_decode -> deserializar+CRC -> comparar
 *
 *  El "canal" es un array en memoria (canal ideal, sin ruido).
 *  Se valida que la trama construida por el transmisor se
 *  recupera identica en el receptor y con CRC correcto.
 *
 *  Esta version es chip-a-chip (1 valor por chip). El sobre-
 *  muestreo (4 muestras/chip) se anadira sobre el mismo canal
 *  como capa posterior, sin tocar la logica de decodificacion.
 * ============================================================ */

/* ---------- CRC-16/MCRF4XX (validado: "123456789" -> 0x6F91) ---------- */
static uint8_t refl8(uint8_t b){uint8_t r=0;for(int i=0;i<8;i++)if(b&(1u<<i))r|=1u<<(7-i);return r;}
static uint16_t refl16(uint16_t w){uint16_t r=0;for(int i=0;i<16;i++)if(w&(1u<<i))r|=1u<<(15-i);return r;}
static uint16_t crc16_802157(const uint8_t *d, size_t n){
    uint16_t c=0xFFFF;
    for(size_t i=0;i<n;i++){uint8_t b=refl8(d[i]);c^=(uint16_t)b<<8;
        for(int k=0;k<8;k++) c=(c&0x8000u)?(uint16_t)((c<<1)^0x1021):(uint16_t)(c<<1);}
    return refl16(c);
}

/* ---------- Buffer de bits (8192: la trama Manchester dobla) ---------- */
typedef struct { uint8_t bit[8192]; size_t n; } bits_t;
static void put_bit(bits_t *b, int v){ b->bit[b->n++] = v ? 1 : 0; }
static void put_bits(bits_t *b, uint32_t value, int nbits){
    for(int i=nbits-1;i>=0;i--) put_bit(b, (value>>i)&1);
}
static void put_byte_bits(bits_t *b, uint8_t v){ put_bits(b, v, 8); }

/* ---------- Parametros del estandar ---------- */
#define FLP_LEN_BITS   64
static const char *TDP_P2 = "001011101111110";   /* peer-to-peer, 15 bits */
#define PREAMBLE_BITS  (FLP_LEN_BITS + 4*15)      /* 64 + 60 = 124 */
#define PHR_BITS       32
#define HCS_BITS       16
#define PHR_BURST_MODE   0
#define PHR_CHANNEL      0
#define PHR_MCS_ID       0x04
#define PHR_DIMMED_OOK   0
#define PHR_RESERVED     0
#define FC_FRAME_VERSION 0
#define FC_FRAME_TYPE    0x1

/* ============================================================
 *  TRANSMISOR (identico al del enlace, para generar estimulo)
 * ============================================================ */
static void build_preamble(bits_t *b){
    for(int i=0;i<FLP_LEN_BITS;i++) put_bit(b, (i%2==0) ? 1 : 0);
    for(int rep=0; rep<4; rep++){
        int invert = (rep%2==1);
        for(int i=0;i<15;i++){
            int bit = (TDP_P2[i]=='1') ? 1 : 0;
            if(invert) bit ^= 1;
            put_bit(b, bit);
        }
    }
}
static void build_phr_hcs(bits_t *b, uint16_t psdu_len){
    bits_t phr = {0};
    put_bits(&phr, PHR_BURST_MODE, 1);
    put_bits(&phr, PHR_CHANNEL,    3);
    put_bits(&phr, PHR_MCS_ID,     6);
    put_bits(&phr, psdu_len,       16);
    put_bits(&phr, PHR_DIMMED_OOK, 1);
    put_bits(&phr, PHR_RESERVED,   5);
    uint8_t phr_bytes[4] = {0,0,0,0};
    for(int i=0;i<32;i++) if(phr.bit[i]) phr_bytes[i/8] |= (1u << (7-(i%8)));
    for(int i=0;i<32;i++) put_bit(b, phr.bit[i]);
    uint16_t hcs = crc16_802157(phr_bytes, 4);
    put_byte_bits(b, (hcs>>8)&0xFF);
    put_byte_bits(b,  hcs&0xFF);
}
static uint16_t build_psdu(bits_t *b, const uint8_t *payload, uint8_t plen, uint8_t seq){
    uint16_t fc = 0;
    fc |= (FC_FRAME_VERSION & 0x3) << 0;
    fc |= (FC_FRAME_TYPE    & 0x7) << 6;
    uint8_t mhr[3];
    mhr[0] = (fc >> 8) & 0xFF; mhr[1] = fc & 0xFF; mhr[2] = seq;
    uint8_t macbuf[260]; size_t mn=0;
    for(int i=0;i<3;i++)    macbuf[mn++]=mhr[i];
    for(int i=0;i<plen;i++) macbuf[mn++]=payload[i];
    uint16_t fcs = crc16_802157(macbuf, mn);
    for(size_t i=0;i<mn;i++) put_byte_bits(b, macbuf[i]);
    put_byte_bits(b, (fcs>>8)&0xFF);
    put_byte_bits(b,  fcs&0xFF);
    return (uint16_t)(mn + 2);
}
static void build_frame(bits_t *b, const uint8_t *payload, uint8_t plen, uint8_t seq){
    b->n = 0;
    build_preamble(b);
    uint16_t psdu_len = 3 + plen + 2;
    build_phr_hcs(b, psdu_len);
    build_psdu(b, payload, plen, seq);
}
/* Manchester: bit 0 -> chips "01", bit 1 -> chips "10" (Tabla 118) */
static void manchester_encode(const bits_t *in, bits_t *out){
    out->n = 0;
    for(size_t i=0;i<in->n;i++){
        if(in->bit[i]==0){ out->bit[out->n++]=0; out->bit[out->n++]=1; }
        else             { out->bit[out->n++]=1; out->bit[out->n++]=0; }
    }
}

/* ============================================================
 *  CANAL  (ideal, chip-a-chip: copia el flujo de chips)
 *  Este es el punto donde luego se insertara el sobremuestreo.
 * ============================================================ */
static void channel_ideal(const bits_t *tx_chips, bits_t *rx_chips){
    *rx_chips = *tx_chips;
}

/* ============================================================
 *  RECEPTOR - bloque 1: decodificacion Manchester
 *  Lee pares de chips: "01"->0, "10"->1.
 *  "00" o "11" son ilegales (sin transicion central) -> error.
 *  Devuelve 0 si OK, o el numero de par ilegal (1-based) si falla.
 * ============================================================ */
static int manchester_decode(const bits_t *chips, bits_t *bits){
    bits->n = 0;
    if(chips->n % 2 != 0) return -1;            /* longitud impar: imposible */
    for(size_t i=0;i<chips->n;i+=2){
        int c0 = chips->bit[i], c1 = chips->bit[i+1];
        if(c0==0 && c1==1)      put_bit(bits, 0);   /* 01 -> 0 */
        else if(c0==1 && c1==0) put_bit(bits, 1);   /* 10 -> 1 */
        else return (int)(i/2)+1;                    /* 00 u 11: par ilegal */
    }
    return 0;
}

/* ---------- utilidad: leer n bits MSB-first desde un offset ---------- */
static uint32_t read_bits_msb(const bits_t *b, size_t off, int n){
    uint32_t v = 0;
    for(int i=0;i<n;i++) v = (v<<1) | (b->bit[off+i] & 1u);
    return v;
}

/* ============================================================
 *  RECEPTOR - bloque 2: deserializar trama y verificar CRC
 *  (chip-a-chip: se asume alineado, se salta el preambulo por
 *   longitud fija; la sincronizacion real sera un bloque aparte)
 *
 *  Rellena payload_out/len_out con la carga util recuperada.
 *  Devuelve un codigo de estado (0 = OK).
 * ============================================================ */
enum { RX_OK=0, RX_ERR_SHORT, RX_ERR_HCS, RX_ERR_FCS };

static int deserialize_and_check(const bits_t *raw,
                                 uint8_t *payload_out, size_t *len_out,
                                 uint16_t *hcs_rx_out, uint16_t *hcs_calc_out,
                                 uint16_t *fcs_rx_out, uint16_t *fcs_calc_out){
    size_t off = PREAMBLE_BITS;                 /* saltar preambulo (alineado) */

    if(raw->n < off + PHR_BITS + HCS_BITS) return RX_ERR_SHORT;

    /* --- PHR: reconstruir 4 octetos y verificar HCS --- */
    uint8_t phr_bytes[4] = {0,0,0,0};
    for(int i=0;i<32;i++) if(raw->bit[off+i]) phr_bytes[i/8] |= (1u << (7-(i%8)));
    uint16_t hcs_calc = crc16_802157(phr_bytes, 4);
    uint16_t hcs_rx   = (uint16_t)read_bits_msb(raw, off+PHR_BITS, 16);
    if(hcs_rx_out)   *hcs_rx_out   = hcs_rx;
    if(hcs_calc_out) *hcs_calc_out = hcs_calc;
    if(hcs_rx != hcs_calc) return RX_ERR_HCS;

    /* --- longitud del PSDU (bits 10-25 del PHR) --- */
    uint16_t psdu_len = (uint16_t)read_bits_msb(raw, off+10, 16);
    size_t psdu_off = off + PHR_BITS + HCS_BITS;

    if(raw->n < psdu_off + (size_t)psdu_len*8) return RX_ERR_SHORT;

    /* --- extraer los octetos del PSDU --- */
    uint8_t psdu[260];
    for(uint16_t i=0;i<psdu_len;i++)
        psdu[i] = (uint8_t)read_bits_msb(raw, psdu_off + i*8, 8);

    /* --- verificar FCS sobre (MHR + payload) --- */
    uint16_t fcs_calc = crc16_802157(psdu, psdu_len - 2);
    uint16_t fcs_rx   = ((uint16_t)psdu[psdu_len-2] << 8) | psdu[psdu_len-1];
    if(fcs_rx_out)   *fcs_rx_out   = fcs_rx;
    if(fcs_calc_out) *fcs_calc_out = fcs_calc;
    if(fcs_rx != fcs_calc) return RX_ERR_FCS;

    /* --- carga util = PSDU sin MHR(3) y sin FCS(2) --- */
    size_t plen = psdu_len - 3 - 2;
    for(size_t i=0;i<plen;i++) payload_out[i] = psdu[3+i];
    *len_out = plen;
    return RX_OK;
}

/* ============================================================
 *  ARNES DE PRUEBA
 * ============================================================ */
static int run_case(const char *name, const uint8_t *payload, uint8_t plen, uint8_t seq){
    bits_t raw, manch, rx_chips, rx_bits;
    build_frame(&raw, payload, plen, seq);
    manchester_encode(&raw, &manch);
    channel_ideal(&manch, &rx_chips);

    int md = manchester_decode(&rx_chips, &rx_bits);
    printf("  [%s] payload=%u bytes | crudo=%zu bits | chips=%zu | decode=%s\n",
           name, plen, raw.n, manch.n, md==0 ? "OK" : "PAR ILEGAL");
    if(md != 0){ printf("    -> FALLO en decodificacion Manchester (par %d)\n", md); return 1; }

    /* comparar bits decodificados con los crudos originales (round-trip codificacion) */
    if(rx_bits.n != raw.n || memcmp(rx_bits.bit, raw.bit, raw.n)!=0){
        printf("    -> FALLO: los bits decodificados no coinciden con la trama cruda\n");
        return 1;
    }

    uint8_t rxpl[260]; size_t rxlen=0;
    uint16_t hcs_rx=0,hcs_c=0,fcs_rx=0,fcs_c=0;
    int st = deserialize_and_check(&rx_bits, rxpl, &rxlen, &hcs_rx,&hcs_c,&fcs_rx,&fcs_c);
    printf("    HCS rx=0x%04X calc=0x%04X | FCS rx=0x%04X calc=0x%04X | estado=%d\n",
           hcs_rx,hcs_c,fcs_rx,fcs_c,st);
    if(st != RX_OK){ printf("    -> FALLO en verificacion (%d)\n", st); return 1; }

    if(rxlen != plen || memcmp(rxpl, payload, plen)!=0){
        printf("    -> FALLO: payload recuperado != enviado\n"); return 1;
    }
    printf("    -> OK: trama recuperada, CRC correcto, payload identico\n");
    return 0;
}

int main(void){
    printf("== Banco RX chip-a-chip (canal ideal) ==\n\n");
    int fails = 0;

    uint8_t p_aa[] = {0xAA,0xAA,0xAA,0xAA};
    fails += run_case("0xAA x4", p_aa, sizeof(p_aa), 0x01);

    uint8_t p_hola[] = {'H','o','l','a'};
    fails += run_case("Hola",    p_hola, sizeof(p_hola), 0x01);

    uint8_t p_00[] = {0x00,0x00};
    fails += run_case("0x00 x2", p_00, sizeof(p_00), 0x05);

    uint8_t p_ff[] = {0xFF,0xFF,0xFF};
    fails += run_case("0xFF x3", p_ff, sizeof(p_ff), 0x2A);

    uint8_t p_mix[] = {0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF};
    fails += run_case("mix 8B",  p_mix, sizeof(p_mix), 0x7F);

    printf("\n== Resultado: %s ==\n", fails==0 ? "TODOS OK" : "HAY FALLOS");
    return fails ? 1 : 0;
}