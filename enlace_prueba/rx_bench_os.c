#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>

/* ============================================================
 *  BANCO DE PRUEBAS DEL RECEPTOR - CON SOBREMUESTREO
 *  TFG VLC sobre RP2040 - Pablo - UC3M
 *
 *  Extiende el banco chip-a-chip anadiendo la capa que emplea
 *  el receptor sobre hardware: el pin se muestrea OSR veces por
 *  chip (aqui OSR=4). A partir de ese flujo de muestras, el
 *  receptor debe RECUPERAR la fase de chip (donde empieza cada
 *  chip) y reconstruir la secuencia, sin conocer de antemano el
 *  desfase de muestreo. Cadena:
 *
 *    build_frame -> manchester_encode ->
 *      [canal: sobremuestreo OSR + desfase] ->
 *      [recuperacion de fase de chip] ->
 *      manchester_decode -> deserializar + CRC
 *
 *  Se prueba primero alineado (desfase 0) y despues con desfase
 *  arbitrario (1..OSR-1), que es la condicion realista: el
 *  arranque del muestreo del receptor no esta sincronizado con
 *  el transmisor.
 * ============================================================ */

#define OSR 4    /* muestras por chip */

/* ---------- CRC-16/MCRF4XX ---------- */
static uint8_t refl8(uint8_t b){uint8_t r=0;for(int i=0;i<8;i++)if(b&(1u<<i))r|=1u<<(7-i);return r;}
static uint16_t refl16(uint16_t w){uint16_t r=0;for(int i=0;i<16;i++)if(w&(1u<<i))r|=1u<<(15-i);return r;}
static uint16_t crc16_802157(const uint8_t *d, size_t n){
    uint16_t c=0xFFFF;
    for(size_t i=0;i<n;i++){uint8_t b=refl8(d[i]);c^=(uint16_t)b<<8;
        for(int k=0;k<8;k++) c=(c&0x8000u)?(uint16_t)((c<<1)^0x1021):(uint16_t)(c<<1);}
    return refl16(c);
}

/* ---------- Buffer (16384: el sobremuestreo multiplica x OSR) ---------- */
typedef struct { uint8_t bit[16384]; size_t n; } bits_t;
static void put_bit(bits_t *b, int v){ b->bit[b->n++] = v ? 1 : 0; }
static void put_bits(bits_t *b, uint32_t value, int nbits){
    for(int i=nbits-1;i>=0;i--) put_bit(b, (value>>i)&1);
}
static void put_byte_bits(bits_t *b, uint8_t v){ put_bits(b, v, 8); }

/* ---------- Parametros del estandar ---------- */
#define FLP_LEN_BITS   64
static const char *TDP_P2 = "001011101111110";
#define PREAMBLE_BITS  (FLP_LEN_BITS + 4*15)
#define PHR_BITS       32
#define HCS_BITS       16
#define PHR_MCS_ID       0x04
#define FC_FRAME_TYPE    0x1

/* ---------- Transmisor (estimulo) ---------- */
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
    put_bits(&phr, 0, 1); put_bits(&phr, 0, 3); put_bits(&phr, PHR_MCS_ID, 6);
    put_bits(&phr, psdu_len, 16); put_bits(&phr, 0, 1); put_bits(&phr, 0, 5);
    uint8_t phr_bytes[4] = {0,0,0,0};
    for(int i=0;i<32;i++) if(phr.bit[i]) phr_bytes[i/8] |= (1u << (7-(i%8)));
    for(int i=0;i<32;i++) put_bit(b, phr.bit[i]);
    uint16_t hcs = crc16_802157(phr_bytes, 4);
    put_byte_bits(b, (hcs>>8)&0xFF); put_byte_bits(b, hcs&0xFF);
}
static uint16_t build_psdu(bits_t *b, const uint8_t *payload, uint8_t plen, uint8_t seq){
    uint16_t fc = (FC_FRAME_TYPE & 0x7) << 6;
    uint8_t mhr[3] = {(uint8_t)((fc>>8)&0xFF),(uint8_t)(fc&0xFF),seq};
    uint8_t macbuf[260]; size_t mn=0;
    for(int i=0;i<3;i++) macbuf[mn++]=mhr[i];
    for(int i=0;i<plen;i++) macbuf[mn++]=payload[i];
    uint16_t fcs = crc16_802157(macbuf, mn);
    for(size_t i=0;i<mn;i++) put_byte_bits(b, macbuf[i]);
    put_byte_bits(b, (fcs>>8)&0xFF); put_byte_bits(b, fcs&0xFF);
    return (uint16_t)(mn + 2);
}
static void build_frame(bits_t *b, const uint8_t *payload, uint8_t plen, uint8_t seq){
    b->n = 0; build_preamble(b);
    build_phr_hcs(b, 3 + plen + 2);
    build_psdu(b, payload, plen, seq);
}
static void manchester_encode(const bits_t *in, bits_t *out){
    out->n = 0;
    for(size_t i=0;i<in->n;i++){
        if(in->bit[i]==0){ out->bit[out->n++]=0; out->bit[out->n++]=1; }
        else             { out->bit[out->n++]=1; out->bit[out->n++]=0; }
    }
}

/* ============================================================
 *  CANAL CON SOBREMUESTREO
 *  Cada chip se expande a OSR muestras. El parametro 'phase'
 *  (0..OSR-1) simula el desfase de muestreo del receptor:
 *  se descartan las primeras 'phase' muestras, como si el
 *  muestreo hubiera arrancado a mitad del primer chip.
 * ============================================================ */
static void channel_oversample(const bits_t *chips, bits_t *samples, int osr, int phase){
    samples->n = 0;
    for(size_t i=0;i<chips->n;i++)
        for(int k=0;k<osr;k++){
            int gidx = (int)(i*osr + k);
            if(gidx < phase) continue;            /* desfase: saltar muestras iniciales */
            samples->bit[samples->n++] = chips->bit[i];
        }
}

/* ============================================================
 *  RECUPERACION DE FASE DE CHIP
 *  1) detecta flancos en el flujo de muestras
 *  2) los flancos caen en las fronteras de chip -> estima la
 *     fase como la moda de (posicion_flanco mod OSR)
 *  3) reconstruye cada chip por voto de mayoria de sus muestras
 *  Devuelve la fase estimada (rho).
 * ============================================================ */
static int recover_chips(const bits_t *samples, int osr, bits_t *chips_out){
    int hist[64] = {0};
    for(size_t r=1;r<samples->n;r++)
        if(samples->bit[r] != samples->bit[r-1]) hist[r % osr]++;
    int m = 0;
    for(int i=1;i<osr;i++) if(hist[i] > hist[m]) m = i;
    int rho = (m == 0) ? osr : m;               /* tamano del primer chip (parcial) */

    chips_out->n = 0;
    size_t ws = 0, we = (size_t)rho;
    while(ws < samples->n){
        if(we > samples->n) we = samples->n;
        int ones=0; size_t cnt=0;
        for(size_t r=ws;r<we;r++){ ones += samples->bit[r]; cnt++; }
        if(cnt>0) chips_out->bit[chips_out->n++] = (ones*2 >= (int)cnt) ? 1 : 0;
        ws = we; we = ws + (size_t)osr;
    }
    return rho;
}

/* ---------- Receptor: decode + deserializar (igual que chip-a-chip) ---------- */
static int manchester_decode(const bits_t *chips, bits_t *bits){
    bits->n = 0;
    if(chips->n % 2 != 0) return -1;
    for(size_t i=0;i<chips->n;i+=2){
        int c0=chips->bit[i], c1=chips->bit[i+1];
        if(c0==0&&c1==1) put_bit(bits,0);
        else if(c0==1&&c1==0) put_bit(bits,1);
        else return (int)(i/2)+1;
    }
    return 0;
}
static uint32_t read_bits_msb(const bits_t *b, size_t off, int n){
    uint32_t v=0; for(int i=0;i<n;i++) v=(v<<1)|(b->bit[off+i]&1u); return v;
}
enum { RX_OK=0, RX_ERR_SHORT, RX_ERR_HCS, RX_ERR_FCS };
static int deserialize_and_check(const bits_t *raw, uint8_t *pl_out, size_t *len_out){
    size_t off = PREAMBLE_BITS;
    if(raw->n < off + PHR_BITS + HCS_BITS) return RX_ERR_SHORT;
    uint8_t phr_bytes[4]={0,0,0,0};
    for(int i=0;i<32;i++) if(raw->bit[off+i]) phr_bytes[i/8]|=(1u<<(7-(i%8)));
    uint16_t hcs_calc=crc16_802157(phr_bytes,4);
    uint16_t hcs_rx=(uint16_t)read_bits_msb(raw,off+PHR_BITS,16);
    if(hcs_rx!=hcs_calc) return RX_ERR_HCS;
    uint16_t psdu_len=(uint16_t)read_bits_msb(raw,off+10,16);
    size_t psdu_off=off+PHR_BITS+HCS_BITS;
    if(raw->n < psdu_off + (size_t)psdu_len*8) return RX_ERR_SHORT;
    uint8_t psdu[260];
    for(uint16_t i=0;i<psdu_len;i++) psdu[i]=(uint8_t)read_bits_msb(raw,psdu_off+i*8,8);
    uint16_t fcs_calc=crc16_802157(psdu,psdu_len-2);
    uint16_t fcs_rx=((uint16_t)psdu[psdu_len-2]<<8)|psdu[psdu_len-1];
    if(fcs_rx!=fcs_calc) return RX_ERR_FCS;
    size_t plen=psdu_len-5;
    for(size_t i=0;i<plen;i++) pl_out[i]=psdu[3+i];
    *len_out=plen;
    return RX_OK;
}

/* ============================================================
 *  ARNES
 * ============================================================ */
static int run_case(const char *name, const uint8_t *payload, uint8_t plen, uint8_t seq, int phase){
    bits_t raw, manch, samples, rec_chips, rx_bits;
    build_frame(&raw, payload, plen, seq);
    manchester_encode(&raw, &manch);
    channel_oversample(&manch, &samples, OSR, phase);

    int rho = recover_chips(&samples, OSR, &rec_chips);

    /* los chips recuperados deben coincidir con los emitidos */
    int chips_ok = (rec_chips.n == manch.n) && (memcmp(rec_chips.bit, manch.bit, manch.n)==0);

    int md = manchester_decode(&rec_chips, &rx_bits);
    uint8_t rxpl[260]; size_t rxlen=0;
    int st = (md==0) ? deserialize_and_check(&rx_bits, rxpl, &rxlen) : -1;
    int pl_ok = (st==RX_OK) && (rxlen==plen) && (memcmp(rxpl,payload,plen)==0);

    int ok = chips_ok && (md==0) && (st==RX_OK) && pl_ok;
    printf("  [%-8s desfase=%d] muestras=%4zu | fase_est(rho)=%d | chips=%s | decode=%s | CRC=%s | payload=%s -> %s\n",
           name, phase, samples.n, rho,
           chips_ok?"OK":"MAL", md==0?"OK":"ILEGAL",
           (st==RX_OK)?"OK":"MAL", pl_ok?"OK":"MAL",
           ok?"OK":"FALLO");
    return ok ? 0 : 1;
}

int main(void){
    printf("== Banco RX con sobremuestreo (OSR=%d) ==\n\n", OSR);
    int fails = 0;

    uint8_t p_aa[]  = {0xAA,0xAA,0xAA,0xAA};
    uint8_t p_hola[]= {'H','o','l','a'};
    uint8_t p_mix[] = {0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF};

    printf("-- Etapa 1: alineado (desfase 0) --\n");
    fails += run_case("0xAA x4", p_aa,  sizeof(p_aa),  0x01, 0);
    fails += run_case("Hola",    p_hola,sizeof(p_hola),0x01, 0);
    fails += run_case("mix 8B",  p_mix, sizeof(p_mix), 0x7F, 0);

    printf("\n-- Etapa 2: con desfase de muestreo (1..OSR-1) --\n");
    for(int ph=1; ph<OSR; ph++){
        fails += run_case("0xAA x4", p_aa,  sizeof(p_aa),  0x01, ph);
        fails += run_case("Hola",    p_hola,sizeof(p_hola),0x01, ph);
        fails += run_case("mix 8B",  p_mix, sizeof(p_mix), 0x7F, ph);
    }

    printf("\n== Resultado: %s ==\n", fails==0 ? "TODOS OK" : "HAY FALLOS");
    return fails ? 1 : 0;
}