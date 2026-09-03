/* ============================================================
 *  rx_firmware.c  -  Firmware del receptor VLC (enlace cableado)
 *  TFG VLC sobre RP2040 - Pablo - UC3M
 *
 *  Integra en un unico programa del Pico receptor:
 *    1) captura de la senal por PIO + DMA (muestreo a 1 MHz, OSR=5)
 *    2) la cadena de proceso ya validada en simulacion:
 *         recuperar fase de chip -> sincronizar sobre preambulo ->
 *         decodificar Manchester -> deserializar -> verificar CRC
 *    3) salida por USB serie con el resultado de cada trama
 *
 *  El transmisor emite tramas en bucle; el receptor captura
 *  bloques mas largos que un periodo de trama, de modo que cada
 *  bloque contiene al menos una trama completa, y la localiza.
 *
 *  Toda la LOGICA es identica a la de los bancos gcc (misma
 *  funcion de CRC, mismo recover_chips, find_preamble, decode y
 *  deserialize). Lo unico nuevo respecto a la simulacion es la
 *  captura fisica y la salida serie.
 * ============================================================ */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "rx_capture.pio.h"     // generado por pico_generate_pio_header

/* ===================== Parametros ===================== */
#define RX_PIN     6            // GPIO6: entrada de datos (cable desde TX)
#define SAMP_DIV   125          // 125 MHz / 125 = 1 MHz -> 5 muestras/chip
#define OSR        5            // muestras por chip (coherente con la simulacion)
#define CAP_WORDS  512          // 512 x 32 = 16384 muestras = 16.4 ms
                                // (> 2 periodos de trama: garantiza 1 trama entera)

#define MAXBITS    20000        // holgura para muestras/chips/bits desempaquetados

/* ===================== CRC-16/MCRF4XX ===================== */
static uint8_t refl8(uint8_t b){uint8_t r=0;for(int i=0;i<8;i++)if(b&(1u<<i))r|=1u<<(7-i);return r;}
static uint16_t refl16(uint16_t w){uint16_t r=0;for(int i=0;i<16;i++)if(w&(1u<<i))r|=1u<<(15-i);return r;}
static uint16_t crc16_802157(const uint8_t *d, size_t n){
    uint16_t c=0xFFFF;
    for(size_t i=0;i<n;i++){uint8_t b=refl8(d[i]);c^=(uint16_t)b<<8;
        for(int k=0;k<8;k++) c=(c&0x8000u)?(uint16_t)((c<<1)^0x1021):(uint16_t)(c<<1);}
    return refl16(c);
}

/* ===================== Buffer de bits ===================== */
typedef struct { uint8_t bit[MAXBITS]; size_t n; } bits_t;
static void put_bit(bits_t *b, int v){ b->bit[b->n++] = v ? 1 : 0; }
static void put_bits(bits_t *b, uint32_t value, int nbits){
    for(int i=nbits-1;i>=0;i--) put_bit(b, (value>>i)&1);
}

/* ===================== Parametros de trama ===================== */
#define FLP_LEN_BITS   64
static const char *TDP_P2 = "001011101111110";
#define PREAMBLE_BITS  (FLP_LEN_BITS + 4*15)     /* 124 */
#define PHR_BITS       32
#define HCS_BITS       16
#define HEADER_BITS    (PREAMBLE_BITS + PHR_BITS + HCS_BITS)  /* 172 */

/* Reconstruye el patron de preambulo (FLP+TDP) para correlar.
 * Necesita build_preamble + manchester_encode (solo el preambulo). */
static void build_preamble_bits(bits_t *b){
    b->n=0;
    for(int i=0;i<FLP_LEN_BITS;i++) put_bit(b,(i%2==0)?1:0);
    for(int rep=0;rep<4;rep++){
        int inv=(rep%2==1);
        for(int i=0;i<15;i++){
            int bit=(TDP_P2[i]=='1')?1:0;
            if(inv) bit^=1;
            put_bit(b,bit);
        }
    }
}
static void manchester_encode(const bits_t *in, bits_t *out){
    out->n=0;
    for(size_t i=0;i<in->n;i++){
        if(in->bit[i]==0){out->bit[out->n++]=0;out->bit[out->n++]=1;}
        else            {out->bit[out->n++]=1;out->bit[out->n++]=0;}
    }
}

/* ===================== Recuperacion de fase de chip ===================== */
static int recover_chips(const bits_t *samples, int osr, bits_t *chips_out){
    int hist[64]={0};
    for(size_t r=1;r<samples->n;r++)
        if(samples->bit[r]!=samples->bit[r-1]) hist[r%osr]++;
    int m=0; for(int i=1;i<osr;i++) if(hist[i]>hist[m]) m=i;
    int rho=(m==0)?osr:m;
    chips_out->n=0;
    size_t ws=0, we=(size_t)rho;
    while(ws<samples->n){
        if(we>samples->n) we=samples->n;
        int ones=0; size_t cnt=0;
        for(size_t r=ws;r<we;r++){ones+=samples->bit[r];cnt++;}
        if(cnt>0) chips_out->bit[chips_out->n++]=(ones*2>=(int)cnt)?1:0;
        ws=we; we=ws+(size_t)osr;
    }
    return rho;
}

/* ===================== Sincronizacion ===================== */
static int find_preamble(const bits_t *chips, const bits_t *pre){
    if(pre->n > chips->n) return -1;
    for(size_t s=0; s+pre->n <= chips->n; s++){
        size_t i=0;
        for(; i<pre->n; i++) if(chips->bit[s+i]!=pre->bit[i]) break;
        if(i==pre->n) return (int)s;
    }
    return -1;
}

/* ===================== Decodificacion + deserializacion ===================== */
static int manchester_decode_range(const bits_t *chips, size_t start, size_t nchips, bits_t *bits){
    bits->n=0;
    if(nchips%2!=0) return -1;
    if(start+nchips > chips->n) return -2;
    for(size_t i=0;i<nchips;i+=2){
        int c0=chips->bit[start+i], c1=chips->bit[start+i+1];
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
    size_t off=PREAMBLE_BITS;
    if(raw->n < off+PHR_BITS+HCS_BITS) return RX_ERR_SHORT;
    uint8_t pb[4]={0,0,0,0};
    for(int i=0;i<32;i++) if(raw->bit[off+i]) pb[i/8]|=(1u<<(7-(i%8)));
    uint16_t hcs_c=crc16_802157(pb,4);
    uint16_t hcs_r=(uint16_t)read_bits_msb(raw,off+PHR_BITS,16);
    if(hcs_r!=hcs_c) return RX_ERR_HCS;
    uint16_t psdu_len=(uint16_t)read_bits_msb(raw,off+10,16);
    size_t psdu_off=off+PHR_BITS+HCS_BITS;
    if(raw->n < psdu_off+(size_t)psdu_len*8) return RX_ERR_SHORT;
    if(psdu_len < 5 || psdu_len > 260) return RX_ERR_SHORT;
    uint8_t psdu[260];
    for(uint16_t i=0;i<psdu_len;i++) psdu[i]=(uint8_t)read_bits_msb(raw,psdu_off+i*8,8);
    uint16_t fcs_c=crc16_802157(psdu,psdu_len-2);
    uint16_t fcs_r=((uint16_t)psdu[psdu_len-2]<<8)|psdu[psdu_len-1];
    if(fcs_r!=fcs_c) return RX_ERR_FCS;
    size_t plen=psdu_len-5;
    for(size_t i=0;i<plen;i++) pl_out[i]=psdu[3+i];
    *len_out=plen;
    return RX_OK;
}

/* ===================== Captura (PIO + DMA) ===================== */
static uint32_t capture_buf[CAP_WORDS];
static int  rx_dma;
static PIO  rx_pio = pio0;
static uint rx_sm;

static void capture_init(void){
    rx_sm = pio_claim_unused_sm(rx_pio, true);
    uint offset = pio_add_program(rx_pio, &rx_capture_program);
    pio_sm_config c = rx_capture_program_get_default_config(offset);
    sm_config_set_in_pins(&c, RX_PIN);
    sm_config_set_in_shift(&c, false, true, 32);   // shift_left, autopush, umbral 32
    sm_config_set_clkdiv_int_frac(&c, SAMP_DIV, 0);
    pio_gpio_init(rx_pio, RX_PIN);
    pio_sm_set_consecutive_pindirs(rx_pio, rx_sm, RX_PIN, 1, false); // entrada
    pio_sm_init(rx_pio, rx_sm, offset, &c);

    rx_dma = dma_claim_unused_channel(true);
    dma_channel_config dc = dma_channel_get_default_config(rx_dma);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_dreq(&dc, pio_get_dreq(rx_pio, rx_sm, false)); // RX
    dma_channel_configure(rx_dma, &dc, capture_buf, &rx_pio->rxf[rx_sm], CAP_WORDS, false);
}

static void capture_block(void){
    pio_sm_set_enabled(rx_pio, rx_sm, false);
    pio_sm_clear_fifos(rx_pio, rx_sm);
    pio_sm_restart(rx_pio, rx_sm);
    dma_channel_set_write_addr(rx_dma, capture_buf, false);
    dma_channel_set_trans_count(rx_dma, CAP_WORDS, true);
    pio_sm_set_enabled(rx_pio, rx_sm, true);
    dma_channel_wait_for_finish_blocking(rx_dma);
    pio_sm_set_enabled(rx_pio, rx_sm, false);
}

static void unpack_samples(bits_t *samples){
    samples->n=0;
    for(size_t w=0; w<CAP_WORDS; w++)
        for(int k=0;k<32;k++)
            samples->bit[samples->n++] = (capture_buf[w] >> (31-k)) & 1u;
}

/* ===================== Buffers globales (evitar la pila) ===================== */
static bits_t g_pre;       // patron de preambulo (referencia)
static bits_t g_samples;   // muestras desempaquetadas
static bits_t g_chips;     // chips recuperados
static bits_t g_hdr;       // cabecera decodificada
static bits_t g_frame;     // trama decodificada

int main(void){
    stdio_init_all();
    sleep_ms(2000);                 // margen para que el monitor serie conecte

    /* patron de preambulo para correlar */
    bits_t pre_bits;
    build_preamble_bits(&pre_bits);
    manchester_encode(&pre_bits, &g_pre);   // 124 bits -> 248 chips

    capture_init();

    printf("\n=== Receptor VLC 802.15.7 (OSR=%d, pin GPIO%d) ===\n", OSR, RX_PIN);
    printf("Patron de preambulo: %u chips. Esperando tramas...\n\n", (unsigned)g_pre.n);

    uint32_t total=0, ok=0;

    while(true){
        capture_block();
        unpack_samples(&g_samples);
        recover_chips(&g_samples, OSR, &g_chips);

        int p = find_preamble(&g_chips, &g_pre);
        if(p < 0){
            printf("[sin trama en el bloque]\n");
            continue;
        }

        total++;

        /* 1) leer la longitud del PSDU de la cabecera */
        int r = manchester_decode_range(&g_chips, (size_t)p, HEADER_BITS*2, &g_hdr);
        if(r != 0){ printf("Trama #%lu: preambulo@%d pero cabecera ilegible (par %d)\n",
                           (unsigned long)total, p, r); continue; }
        uint16_t psdu_len = (uint16_t)read_bits_msb(&g_hdr, PREAMBLE_BITS+10, 16);

        /* 2) decodificar la trama completa */
        size_t frame_bits = HEADER_BITS + (size_t)psdu_len*8;
        r = manchester_decode_range(&g_chips, (size_t)p, frame_bits*2, &g_frame);
        if(r != 0){ printf("Trama #%lu: decode incompleto (par %d)\n",
                           (unsigned long)total, r); continue; }

        /* 3) deserializar y verificar CRC */
        uint8_t pl[260]; size_t plen=0;
        int st = deserialize_and_check(&g_frame, pl, &plen);
        if(st == RX_OK){
            ok++;
            printf("Trama #%lu OK | preambulo@chip %d | payload (%u B): ",
                   (unsigned long)total, p, (unsigned)plen);
            for(size_t i=0;i<plen;i++) printf("%02X ", pl[i]);
            printf("| texto: ");
            for(size_t i=0;i<plen;i++) printf("%c", (pl[i]>=32&&pl[i]<127)?pl[i]:'.');
            printf("\n");
        } else {
            const char *m = (st==RX_ERR_HCS)?"HCS":(st==RX_ERR_FCS)?"FCS":"corta";
            printf("Trama #%lu: preambulo@%d pero CRC %s incorrecto\n",
                   (unsigned long)total, p, m);
        }
        printf("   -> aciertos: %lu / %lu\n", (unsigned long)ok, (unsigned long)total);
    }
}