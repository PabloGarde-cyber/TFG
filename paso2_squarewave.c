#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "squarewave.pio.h"

#define OUT_PIN 15   // GPIO15 = pin fisico 20

int main() {
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &squarewave_program);
    squarewave_program_init(pio, sm, offset, OUT_PIN);

    pio_sm_set_clkdiv(pio, sm, 1250.0f);   // <-- el divisor a barrer
    pio_sm_set_enabled(pio, sm, true);

    while (true) { tight_loop_contents(); }
}