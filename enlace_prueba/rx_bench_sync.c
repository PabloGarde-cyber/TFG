#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

/* ============================================================
 *  BANCO DE PRUEBAS DEL RECEPTOR - SINCRONIZACION DE TRAMA
 *  TFG VLC sobre RP2040 - Pablo - UC3M
 *
 *  Anade el bloque de sincronizacion: el receptor ve un flujo
 *  continuo que contiene ruido ANTES de la trama (como si
 *  hubiera empezado a escuchar antes de que llegara). Debe
 *  localizar el preambulo dentro del flujo y alinearse.
 *
 *  Cadena:
 *   build_frame -> manchester_encode -> [chips de trama]
 *   [ruido previo (chips aleatorios)] + [chips de trama]
 *        -> canal sobremuestreo (OSR=4, desfase) ->
 *        -> recuperacion de fase de chip ->
 *        -> SINCRONIZACION: buscar patron de preambulo en chips ->
 *        -> decodificar Manchester (ya alineado) ->
 *        -> deserializar + CRC
 *
 *  NOTA DE DISENO: la sincronizacion se hace sobre la secuencia
 *  de CHIPS (correlando el patron conocido del preambulo), no
 *  sobre los bits ya decodificados. El motivo es que la
 *  decodificacion Manchester necesita saber donde empieza cada
 *  bit (que chip abre el par); esa alineacion de par la resuelve
 *  precisamente la deteccion del preambulo. Localizar el
 *  preambulo en los chips fija a la vez la posicion de trama y
 *  la alineacion de par. La logica posterior (deserializar, CRC)
 *  sigue en el dominio de bytes.
 * ============================================================ */

#define OSR 4

/* ---------- CRC-16/MCRF4XX ---------- */
static uint8_t refl8(uint8_t b){uint8_t r=0;for(int i=0;i<8;i++)if(b&(1u<<i))r|=1u<<(7-i);return r;}
static uint16_t refl16(uint16_t w){uint16_t r=0;for(int i=0;i<16;i++)if(w&(1u<<i))r|=1u<<(15-i);return r;}
static uint16_t crc16_802157(const uint8_t *d, size_t n){
    uint16_t c=0xFFFF;
    for(size_t i=0;i<n;i++){uint8_t b=refl8(d[i]);c^=(uint16_t)b<<8;
        for(int k=0;k<8;k++) c=(c&0x8000u)?(uint16_t)((c<<1)^0x1021):(uint16_t)(c<<1);}
    return refl16(c);
}

typedef struct { uint8_t bit[16384]; size_t n; } bits_t;
static void put_bit(bits_t *b, int v){ b->bit[b->n++] = v ? 1 : 0; }
static void put_bits(bits_t *b, uint32_t value, int nbits){
    for(int i=nbits-1;i>=0;i--) put_bit(b, (value>>i)&1);
}
static void put_byte_bits(bits_t *b, uint8_t v){ put_bits(b, v, 8); }

#define FLP_LEN_BITS   64
static const char *TDP_P2 = "001011101111110";
#define PREAMBLE_BITS  (FLP_LEN_BITS + 4*15)
#define PHR_BITS       32
#define HCS_BITS       16
#define HEADER_BITS    (PREAMBLE_BITS + PHR_BITS + HCS_BITS)   /* 172 */
#define PHR_MCS_ID       0x04
#define FC_FRAME_TYPE    0x1

/* ---------- Transmisor ---------- */
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
    put_bits(&phr,0,1);put_bits(&phr,0,3);put_bits(&phr,PHR_MCS_ID,6);
    put_bits(&phr,psdu_len,16);put_bits(&phr,0,1);put_bits(&phr,0,5);
    uint8_t pb[4]={0,0,0,0};
    for(int i=0;i<32;i++) if(phr.bit[i]) pb[i/8]|=(1u<<(7-(i%8)));
    for(int i=0;i<32;i++) put_bit(b,phr.bit[i]);
    uint16_t hcs=crc16_802157(pb,4);
    put_byte_bits(b,(hcs>>8)&0xFF);put_byte_bits(b,hcs&0xFF);
}
static uint16_t build_psdu(bits_t *b, const uint8_t *pl, uint8_t plen, uint8_t seq){
    uint16_t fc=(FC_FRAME_TYPE&0x7)<<6;
    uint8_t mhr[3]={(uint8_t)((fc>>8)&0xFF),(uint8_t)(fc&0xFF),seq};
    uint8_t mac[260]; size_t mn=0;
    for(int i=0;i<3;i++) mac[mn++]=mhr[i];
    for(int i=0;i<plen;i++) mac[mn++]=pl[i];
    uint16_t fcs=crc16_802157(mac,mn);
    for(size_t i=0;i<mn;i++) put_byte_bits(b,mac[i]);
    put_byte_bits(b,(fcs>>8)&0xFF);put_byte_bits(b,fcs&0xFF);
    return (uint16_t)(mn+2);
}
static void build_frame(bits_t *b, const uint8_t *pl, uint8_t plen, uint8_t seq){
    b->n=0; build_preamble(b); build_phr_hcs(b,3+plen+2); build_psdu(b,pl,plen,seq);
}
static void manchester_encode(const bits_t *in, bits_t *out){
    out->n=0;
    for(size_t i=0;i<in->n;i++){
        if(in->bit[i]==0){out->bit[out->n++]=0;out->bit[out->n++]=1;}
        else            {out->bit[out->n++]=1;out->bit[out->n++]=0;}
    }
}

/* ---------- patron de chips del preambulo (para correlar) ---------- */
static void build_preamble_chips(bits_t *pre_chips){
    bits_t pre_bits; pre_bits.n=0;
    build_preamble(&pre_bits);
    manchester_encode(&pre_bits, pre_chips);   /* 124 bits -> 248 chips */
}

/* ---------- canal: [ruido] + [trama], sobremuestreo + desfase ---------- */
static void channel_with_noise(const bits_t *frame_chips, bits_t *samples,
                               int osr, int phase, int pre_noise_chips, unsigned seed){
    bits_t stream; stream.n=0;
    srand(seed);
    for(int i=0;i<pre_noise_chips;i++) put_bit(&stream, rand()&1);   /* ruido previo */
    for(size_t i=0;i<frame_chips->n;i++) put_bit(&stream, frame_chips->bit[i]);
    /* sobremuestreo con desfase */
    samples->n=0;
    for(size_t i=0;i<stream.n;i++)
        for(int k=0;k<osr;k++){
            int g=(int)(i*osr+k);
            if(g<phase) continue;
            samples->bit[samples->n++]=stream.bit[i];
        }
}

/* ---------- recuperacion de fase de chip (igual que antes) ---------- */
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

/* ============================================================
 *  SINCRONIZACION: localizar el preambulo en la secuencia de chips
 *  Correla el patron conocido del preambulo; devuelve el indice
 *  de chip donde empieza, o -1 si no lo encuentra.
 * ============================================================ */
static int find_preamble(const bits_t *chips, const bits_t *pre_chips){
    if(pre_chips->n > chips->n) return -1;
    for(size_t s=0; s+pre_chips->n <= chips->n; s++){
        size_t i=0;
        for(; i<pre_chips->n; i++)
            if(chips->bit[s+i] != pre_chips->bit[i]) break;
        if(i==pre_chips->n) return (int)s;
    }
    return -1;
}

/* ---------- decodificar un rango de chips ---------- */
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

/* ============================================================
 *  RECEPTOR COMPLETO (sincroniza + decodifica + verifica)
 * ============================================================ */
static int rx_full(const bits_t *rec_chips, const bits_t *pre_chips,
                   uint8_t *pl_out, size_t *len_out, int *found_at){
    int p = find_preamble(rec_chips, pre_chips);
    *found_at = p;
    if(p < 0) return -100;                          /* no se encontro preambulo */

    /* 1) decodificar cabecera para leer la longitud del PSDU */
    bits_t hdr;
    int r = manchester_decode_range(rec_chips, (size_t)p, HEADER_BITS*2, &hdr);
    if(r != 0) return -101;
    uint16_t psdu_len = (uint16_t)read_bits_msb(&hdr, PREAMBLE_BITS+10, 16);

    /* 2) decodificar la trama completa segun esa longitud */
    size_t frame_bits = HEADER_BITS + (size_t)psdu_len*8;
    bits_t frame;
    r = manchester_decode_range(rec_chips, (size_t)p, frame_bits*2, &frame);
    if(r != 0) return -102;

    /* 3) deserializar y verificar CRC */
    return deserialize_and_check(&frame, pl_out, len_out);
}

/* ============================================================
 *  ARNES
 * ============================================================ */
static bits_t g_pre_chips;

static int run_case(const char *name, const uint8_t *pl, uint8_t plen, uint8_t seq,
                    int phase, int pre_noise, unsigned seed){
    bits_t raw, manch, samples, rec;
    build_frame(&raw, pl, plen, seq);
    manchester_encode(&raw, &manch);
    channel_with_noise(&manch, &samples, OSR, phase, pre_noise, seed);
    recover_chips(&samples, OSR, &rec);

    uint8_t rxpl[260]; size_t rxlen=0; int found=-1;
    int st = rx_full(&rec, &g_pre_chips, rxpl, &rxlen, &found);

    int sync_ok = (found == pre_noise);             /* deberia hallarlo tras el ruido */
    int pl_ok   = (st==RX_OK) && (rxlen==plen) && (memcmp(rxpl,pl,plen)==0);
    int ok = sync_ok && pl_ok;

    printf("  [%-8s ruido=%3d desfase=%d] preambulo_hallado_en=%-4d (esperado %-4d) | sync=%s | CRC=%s -> %s\n",
           name, pre_noise, phase, found, pre_noise,
           sync_ok?"OK":"MAL", (st==RX_OK)?"OK":"MAL", ok?"OK":"FALLO");
    return ok?0:1;
}

int main(void){
    build_preamble_chips(&g_pre_chips);
    printf("== Banco RX sincronizacion de trama (OSR=%d, preambulo=%zu chips) ==\n\n",
           OSR, g_pre_chips.n);
    int fails=0;

    uint8_t p_aa[]  ={0xAA,0xAA,0xAA,0xAA};
    uint8_t p_hola[]={'H','o','l','a'};
    uint8_t p_mix[] ={0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF};

    printf("-- Sin ruido previo (preambulo al inicio) --\n");
    fails += run_case("Hola", p_hola, sizeof(p_hola), 0x01, 0, 0, 1);

    printf("\n-- Con ruido previo de longitud variable --\n");
    int noises[] = {1, 7, 23, 50, 137};
    for(size_t i=0;i<sizeof(noises)/sizeof(noises[0]);i++)
        fails += run_case("Hola", p_hola, sizeof(p_hola), 0x01, 0, noises[i], 100+i);

    printf("\n-- Combinando ruido previo y desfase de muestreo --\n");
    for(int ph=0; ph<OSR; ph++)
        fails += run_case("mix 8B", p_mix, sizeof(p_mix), 0x7F, ph, 40+ph, 200+ph);

    printf("\n-- Otras cargas utiles con ruido y desfase --\n");
    fails += run_case("0xAA x4", p_aa, sizeof(p_aa), 0x01, 2, 63, 300);
    fails += run_case("mix 8B",  p_mix, sizeof(p_mix), 0x55, 3, 90, 301);

    printf("\n== Resultado: %s ==\n", fails==0?"TODOS OK":"HAY FALLOS");
    return fails?1:0;
}