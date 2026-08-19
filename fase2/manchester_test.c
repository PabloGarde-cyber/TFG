#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "frame_tx.pio.h"      // el MISMO .pio del 2.3, sin cambios

#define DATA_PIN   2           // GPIO2 / pin 4 (DATA), como en el 2.3
#define BIT_US     5           // 5 us por CHIP (200 kHz, divisor 625)

int main(void){
    stdio_init_all();

    // --- Codificar 0x00 en Manchester, a mano (es trivial y fijo) ---
    // 0x00 = 00000000 -> cada bit 0 = chips "01" -> 16 chips: 0101...01
    // Empaquetado MSB-first en una palabra de 32 bits:
    //   chip 0 (=0) al bit 31, chip 1 (=1) al bit 30, etc.
    // Patron 0101... en 32 bits = 0x55555555
    // (usamos 32 chips = 2 bytes 0x00 seguidos, para llenar una palabra entera)
    static uint32_t chips[] = { 0x55555555 };   // 32 chips "01" repetidos
    size_t nwords = 1;

    // --- PIO (identico al 2.3) ---
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &frame_tx_program);
    pio_sm_config c = frame_tx_program_get_default_config(offset);
    sm_config_set_out_pins(&c, DATA_PIN, 1);
    sm_config_set_out_shift(&c, false, true, 32);   // shift_left, autopull, umbral 32
    sm_config_set_clkdiv_int_frac(&c, 625, 0);      // 200 kHz -> 5 us/chip
    pio_gpio_init(pio, DATA_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, DATA_PIN, 1, true);
    pio_sm_init(pio, sm, offset, &c);

    // --- DMA en bucle continuo (reinicia solo, emision infinita) ---
    int dma = dma_claim_unused_channel(true);
    dma_channel_config dc = dma_channel_get_default_config(dma);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, true);
    channel_config_set_write_increment(&dc, false);
    channel_config_set_dreq(&dc, pio_get_dreq(pio, sm, true));

    pio_sm_set_enabled(pio, sm, true);

    while(true){
        dma_channel_configure(dma, &dc, &pio->txf[sm], chips, nwords, true);
        dma_channel_wait_for_finish_blocking(dma);
        // se reinicia inmediatamente -> flujo continuo de "0101..." = onda cuadrada
    }
}