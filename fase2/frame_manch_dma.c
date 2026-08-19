#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "frame_tx.pio.h" //generado por pico_generate_pio_header


/* ============================================================
 *  TRANSMISOR PPDU MANCHESTER — IEEE 802.15.7, PHY I, OOK, 100 kbps
 *  TFG VLC sobre RP2040 · Pablo · UC3M
 *
 *  Cadena:  build_frame  ->  manchester_encode  ->  pack_frame  ->  DMA  ->  PIO
 *
 *  - build_frame        construye la PPDU cruda (bits OOK), como en el paso 2.3
 *  - manchester_encode  expande cada bit a 2 chips (Tabla 118): 0->01, 1->10
 *  - pack_frame         empaqueta los chips MSB-first en uint32_t
 *  - el PIO NO cambia: sigue sacando 1 chip por instruccion a 5 us (200 kHz)
 *
 *  El PIO "no sabe" que es Manchester: solo emite el flujo de chips. La
 *  codificacion es un precalculo en memoria (sin restriccion temporal); el
 *  timing lo pone el PIO (hardware determinista). Esta separacion es lo que
 *  evita el fallo del TFG previo (timing por software).
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
 *  BUFFER DE BITS  (8192: la trama Manchester DOBLA el tamano)
 * ============================================================ */
typedef struct { uint8_t bit[8192]; size_t n; } bits_t;

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

/* ============================================================
 *  1) PREAMBULO:  FLP + [P2, ~P2, P2, ~P2]
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

/* ============================================================
 *  2) PHR (32 bits) + 3) HCS (16 bits)
 * ============================================================ */
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

/* ============================================================
 *  4) PSDU: MHR (FrameControl + SeqNum) + payload + FCS
 * ============================================================ */
static uint16_t build_psdu(bits_t *b, const uint8_t *payload, uint8_t plen, uint8_t seq){
    uint16_t fc = 0;
    fc |= (FC_FRAME_VERSION & 0x3) << 0;
    fc |= (FC_FRAME_TYPE    & 0x7) << 6;

    uint8_t mhr[3];
    mhr[0] = (fc >> 8) & 0xFF;
    mhr[1] =  fc       & 0xFF;
    mhr[2] =  seq;

    uint8_t macbuf[260]; size_t mn=0;
    for(int i=0;i<3;i++)   macbuf[mn++]=mhr[i];
    for(int i=0;i<plen;i++) macbuf[mn++]=payload[i];

    uint16_t fcs = crc16_802157(macbuf, mn);

    for(size_t i=0;i<mn;i++) put_byte_bits(b, macbuf[i]);
    put_byte_bits(b, (fcs>>8)&0xFF);
    put_byte_bits(b,  fcs&0xFF);

    return (uint16_t)(mn + 2);
}

/* ============================================================
 *  TRAMA COMPLETA (cruda, bits OOK)
 * ============================================================ */
void build_frame(bits_t *b, const uint8_t *payload, uint8_t plen, uint8_t seq){
    b->n = 0;
    build_preamble(b);
    uint16_t psdu_len = 3 + plen + 2;
    build_phr_hcs(b, psdu_len);
    build_psdu(b, payload, plen, seq);
}

/* ============================================================
 *  CODIFICACION MANCHESTER (Tabla 118, clausula 10.5.2)
 *     bit 0 -> chips "01"   (bajo, alto)
 *     bit 1 -> chips "10"   (alto, bajo)
 *  Cada bit de entrada produce 2 chips. Duplica la longitud.
 * ============================================================ */
void manchester_encode(const bits_t *in, bits_t *out){
    out->n = 0;
    for(size_t i=0;i<in->n;i++){
        if(in->bit[i]==0){ out->bit[out->n++]=0; out->bit[out->n++]=1; }  /* 0 -> 01 */
        else             { out->bit[out->n++]=1; out->bit[out->n++]=0; }  /* 1 -> 10 */
    }
}

/* ============================================================
 *  EMPAQUETADO MSB-first a uint32_t
 *  (bit/chip 0 de aire -> bit 31 de word0; shift LEFT en el PIO)
 * ============================================================ */
size_t pack_frame(const bits_t *b, uint32_t *w, size_t maxw){
    size_t nw = (b->n + 31)/32;
    if(nw > maxw) return 0;
    for(size_t i=0;i<nw;i++) w[i]=0;
    for(size_t j=0;j<b->n;j++)
        if(b->bit[j]) w[j/32] |= (1u << (31-(j%32)));
    return nw;
}

#define DATA_PIN   2           // GPIO2 / pin 4 -> gate del MOSFET (o LED en prueba directa)
#define SYNC_PIN   3           // GPIO3 / pin 5 -> disparo del osciloscopio
#define CHIP_US    5           // 5 us por CHIP (200 kHz, divisor 625)

int main(void){
    stdio_init_all();

    gpio_init(SYNC_PIN); gpio_set_dir(SYNC_PIN, GPIO_OUT); gpio_put(SYNC_PIN, 0);

    // --- construir, CODIFICAR MANCHESTER y empaquetar (una vez) ---
    static uint32_t words[32];                 // 32 palabras: la trama Manchester dobla
    uint8_t payload[] = {0xAA,0xAA,0xAA,0xAA};  // patron facil de leer (10101010)
    static bits_t raw, manch;                  // static: no saturar la pila
    build_frame(&raw, payload, sizeof(payload), 0x01);
    manchester_encode(&raw, &manch);           // <-- paso nuevo respecto al 2.3
    size_t nwords = pack_frame(&manch, words, 32);
    uint32_t frame_us = (uint32_t)(nwords*32*CHIP_US);   // duracion total incl. relleno

    // --- PIO (identico al 2.3) ---
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &frame_tx_program);
    pio_sm_config c = frame_tx_program_get_default_config(offset);
    sm_config_set_out_pins(&c, DATA_PIN, 1);
    sm_config_set_out_shift(&c, false, true, 32);   // shift_left, autopull ON, umbral 32
    sm_config_set_clkdiv_int_frac(&c, 625, 0);      // 125MHz/625 = 200 kHz -> 5 us/chip
    pio_gpio_init(pio, DATA_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, DATA_PIN, 1, true);
    pio_sm_init(pio, sm, offset, &c);

    // --- DMA: memoria -> FIFO TX del PIO, ritmado por DREQ (identico al 2.3) ---
    int dma = dma_claim_unused_channel(true);
    dma_channel_config dc = dma_channel_get_default_config(dma);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, true);
    channel_config_set_write_increment(&dc, false);
    channel_config_set_dreq(&dc, pio_get_dreq(pio, sm, true));  // true = TX
    dma_channel_configure(dma, &dc, &pio->txf[sm], words, nwords, false);

    while(true){
        // reset limpio: cada trama arranca en word0, OSR vacio, linea en reposo
        pio_sm_set_enabled(pio, sm, false);
        pio_sm_clear_fifos(pio, sm);
        pio_sm_restart(pio, sm);

        gpio_put(SYNC_PIN, 1);                 // sube el sync
        pio_sm_set_enabled(pio, sm, true);     // arranca la emision
        dma_channel_set_read_addr(dma, words, false);
        dma_channel_set_trans_count(dma, nwords, true);  // dispara DMA

        sleep_us(frame_us + 20);               // espera a que el PIO drene la FIFO
        gpio_put(SYNC_PIN, 0);                 // baja el sync
        sleep_ms(5);                           // hueco entre tramas para re-disparo
    }
}