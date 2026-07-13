#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "squarewave.pio.h"   // lo genera CMake a partir del .pio

#define OUT_PIN 15            // GPIO15 = pin fisico 20

int main() {
    stdio_init_all();

    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &squarewave_program);

    squarewave_program_init(pio, sm, offset, OUT_PIN);

    // ---- EL MANDO: el divisor de reloj ----
    float div = 1250.0f;                 // <-- cambia esto y observa
    pio_sm_set_clkdiv(pio, sm, div);

    pio_sm_set_enabled(pio, sm, true);   // arranca la maquina de estados

    while (true) {
        tight_loop_contents();           // el PIO trabaja solo; la CPU no hace nada
    }
}