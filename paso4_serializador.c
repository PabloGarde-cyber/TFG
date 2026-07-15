#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "ook_tx.pio.h"

#define OUT_PIN   15        // GPIO15 -> pin fisico 20 (el mismo de siempre)

// T_bit = 2 ciclos / (125 MHz / div)
// div = 2500 -> f_SM = 50 kHz -> T_bit = 40 us -> 25 kbps
#define CLK_DIV   2500.0f

int main() {
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &ook_tx_program);

    ook_tx_program_init(pio, sm, offset, OUT_PIN, CLK_DIV);

    while (true) {
        // OJO: MSB primero => hay que alinear el byte a la izquierda (<< 24)
        pio_sm_put_blocking(pio, sm, ((uint32_t)0xA5) << 24);  // el byte: 1010 0101
        pio_sm_put_blocking(pio, sm, 0u);                      // hueco: 8 bits a 0
    }
}