#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* ============================================================
 *  CONSTRUCTOR DE TRAMA PPDU — IEEE 802.15.7, PHY I, OOK, 100 kbps
 *  TFG VLC sobre RP2040 · Pablo · UC3M
 *
 *  Estructura (Figura 135 del estandar), version minima conforme:
 *
 *  PREAMBULO (SHR):  FLP (64 bits) + [P2, ~P2, P2, ~P2] (60 bits)
 *  PHR (32 bits):    burst|channel|MCS|len|dimmed|reserved
 *  HCS (16 bits):    CRC-16 sobre el PHR
 *  PSDU:             MHR(FrameControl+SeqNum) + MAC payload + FCS(16)
 *
 *  Decisiones de diseno (100 kbps, sin dimming, sin FEC):
 *   - Dimmed OOK ext = 0  -> sin campos opcionales
 *   - Sin tail bits (solo se anaden con FEC a 11,67/24,44/48,89 kbps)
 *   - MHR minimo: Frame Control (2 oct) + Sequence Number (1 oct)
 * ============================================================ */

/* ---------- CRC-16 del 802.15.7 (validado: "123456789" -> 0x6F91) ---------- */
static uint8_t refl8(uint8_t b){uint8_t r=0;for(int i=0;i<8;i++)if(b&(1u<<i))r|=1u<<(7-i);return r;}
static uint16_t refl16(uint16_t w){uint16_t r=0;for(int i=0;i<16;i++)if(w&(1u<<i))r|=1u<<(15-i);return r;}
uint16_t crc16_802157(const uint8_t *d, size_t n){
    uint16_t c=0xFFFF;
    for(size_t i=0;i<n;i++){uint8_t b=refl8(d[i]);c^=(uint16_t)b<<8;
        for(int k=0;k<8;k++) c=(c&0x8000u)?(uint16_t)((c<<1)^0x1021):(uint16_t)(c<<1);}
    return refl16(c);
}

/* ============================================================
 *  BUFFER DE BITS (el preambulo se define en bits, no en octetos)
 * ============================================================ */
typedef struct { uint8_t bit[4096]; size_t n; } bits_t;

static void put_bit(bits_t *b, int v){ b->bit[b->n++] = v ? 1 : 0; }

/* Anade los 'nbits' de 'value' (MSB primero) */
static void put_bits(bits_t *b, uint32_t value, int nbits){
    for(int i=nbits-1;i>=0;i--) put_bit(b, (value>>i)&1);
}
/* Anade un octeto (MSB primero) */
static void put_byte_bits(bits_t *b, uint8_t v){ put_bits(b, v, 8); }

/* ============================================================
 *  PARAMETROS DEL ESTANDAR (confirmados)
 * ============================================================ */
#define FLP_LEN_BITS   64          /* minimo del estandar */
static const char *TDP_P2 = "001011101111110";   /* peer-to-peer, 15 bits */

/* PHR (confirmar Channel; MCS y demas ya fijados) */
#define PHR_BURST_MODE   0         /* 1 bit  */
#define PHR_CHANNEL      0         /* 3 bits  (confirmar banda) */
#define PHR_MCS_ID       0x04      /* 6 bits  = 000100 (100 kbps OOK Manchester) */
#define PHR_DIMMED_OOK   0         /* 1 bit  -> sin campos opcionales */
#define PHR_RESERVED     0         /* 5 bits */

/* Frame Control (16 bits) - version minima */
#define FC_FRAME_VERSION 0         /* bits 0-1 */
#define FC_FRAME_TYPE    0x1       /* bits 6-8 = 001 = Data (Tabla 7 del estandar) */
/* resto de flags y modos de direccion = 0 */

/* ============================================================
 *  1) PREAMBULO:  FLP + [P2, ~P2, P2, ~P2]
 * ============================================================ */
static void build_preamble(bits_t *b){
    /* FLP: 1010...0, 64 bits, empieza en 1 y acaba en 0 */
    for(int i=0;i<FLP_LEN_BITS;i++) put_bit(b, (i%2==0) ? 1 : 0);

    /* 4 repeticiones del TDP, invirtiendo las alternas (balance DC) */
    for(int rep=0; rep<4; rep++){
        int invert = (rep%2==1);              /* rep 1 y 3 invertidas */
        for(int i=0;i<15;i++){
            int bit = (TDP_P2[i]=='1') ? 1 : 0;
            if(invert) bit ^= 1;
            put_bit(b, bit);
        }
    }
}

/* ============================================================
 *  2) PHR (32 bits) + 3) HCS (16 bits)
 *     El PSDU length se pasa como parametro.
 * ============================================================ */
static void build_phr_hcs(bits_t *b, uint16_t psdu_len){
    /* Construimos el PHR en 4 octetos para poder calcular el HCS sobre ellos */
    bits_t phr = {0};
    put_bits(&phr, PHR_BURST_MODE, 1);
    put_bits(&phr, PHR_CHANNEL,    3);
    put_bits(&phr, PHR_MCS_ID,     6);
    put_bits(&phr, psdu_len,       16);
    put_bits(&phr, PHR_DIMMED_OOK, 1);
    put_bits(&phr, PHR_RESERVED,   5);
    /* -> 32 bits exactos */

    /* Empaquetar los 32 bits del PHR en 4 octetos (MSB primero) */
    uint8_t phr_bytes[4] = {0,0,0,0};
    for(int i=0;i<32;i++) if(phr.bit[i]) phr_bytes[i/8] |= (1u << (7-(i%8)));

    /* Volcar el PHR al buffer de bits */
    for(int i=0;i<32;i++) put_bit(b, phr.bit[i]);

    /* HCS = CRC-16 sobre los 4 octetos del PHR */
    uint16_t hcs = crc16_802157(phr_bytes, 4);
    put_byte_bits(b, (hcs>>8)&0xFF);
    put_byte_bits(b,  hcs&0xFF);
}

/* ============================================================
 *  4) PSDU: MHR (FrameControl + SeqNum) + payload + FCS
 * ============================================================ */
static uint16_t build_psdu(bits_t *b, const uint8_t *payload, uint8_t plen, uint8_t seq){
    /* --- Frame Control (16 bits) --- */
    uint16_t fc = 0;
    fc |= (FC_FRAME_VERSION & 0x3) << 0;   /* bits 0-1 */
    fc |= (FC_FRAME_TYPE    & 0x7) << 6;   /* bits 6-8 */
    /* resto a 0 */

    /* MHR en octetos, para el FCS */
    uint8_t mhr[3];
    mhr[0] = (fc >> 8) & 0xFF;   /* FC hi */
    mhr[1] =  fc       & 0xFF;   /* FC lo */
    mhr[2] =  seq;               /* Sequence Number */

    /* Montar MHR + payload en un buffer para el FCS */
    uint8_t macbuf[260]; size_t mn=0;
    for(int i=0;i<3;i++)   macbuf[mn++]=mhr[i];
    for(int i=0;i<plen;i++) macbuf[mn++]=payload[i];

    /* FCS = CRC-16 sobre (MHR + MAC payload) */
    uint16_t fcs = crc16_802157(macbuf, mn);

    /* Volcar todo al buffer de bits: MHR + payload + FCS */
    for(size_t i=0;i<mn;i++) put_byte_bits(b, macbuf[i]);
    put_byte_bits(b, (fcs>>8)&0xFF);
    put_byte_bits(b,  fcs&0xFF);
    /* sin tail bits (100 kbps sin FEC) */

    /* longitud del PSDU en octetos = MHR + payload + FCS */
    return (uint16_t)(mn + 2);
}

/* ============================================================
 *  TRAMA COMPLETA
 * ============================================================ */
void build_frame(bits_t *b, const uint8_t *payload, uint8_t plen, uint8_t seq){
    b->n = 0;
    build_preamble(b);

    /* longitud del PSDU (MHR 3 + payload + FCS 2) */
    uint16_t psdu_len = 3 + plen + 2;

    build_phr_hcs(b, psdu_len);
    build_psdu(b, payload, plen, seq);
}

/* ---------- Utilidades de impresion ---------- */
static void print_section(const char *name, bits_t *b, size_t from, size_t to){
    printf("  %-14s [%3zu bits] ", name, to-from);
    for(size_t i=from;i<to;i++){ putchar(b->bit[i]?'1':'0'); if((i-from)%8==7) putchar(' '); }
    printf("\n");
}

int main(void){
    uint8_t payload[] = {'H','o','l','a'};
    bits_t b;
    build_frame(&b, payload, sizeof(payload), 0x01);

    printf("TRAMA PPDU 802.15.7 (PHY I, 100 kbps) - total %zu bits (%zu octetos + preambulo)\n\n", b.n, (b.n-124)/8);

    size_t p=0;
    print_section("FLP", &b, p, p+64); p+=64;
    print_section("TDP x4", &b, p, p+60); p+=60;
    print_section("PHR", &b, p, p+32); p+=32;
    print_section("HCS", &b, p, p+16); p+=16;
    printf("  PSDU (resto): %zu bits (MHR + payload 'Hola' + FCS)\n", b.n-p);

    /* Comprobacion CRC visible */
    uint8_t phr_bytes[4]={0}; for(int i=0;i<32;i++) if(b.bit[124+i]) phr_bytes[i/8]|=(1u<<(7-(i%8)));
    printf("\n  HCS calculado sobre PHR %02X %02X %02X %02X = 0x%04X\n",
           phr_bytes[0],phr_bytes[1],phr_bytes[2],phr_bytes[3], crc16_802157(phr_bytes,4));
    return 0;
}