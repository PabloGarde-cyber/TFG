#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "squarewave.pio.h"

#define PIN_PIO  15   // pin 20 : PIO
#define PIN_SW   14   // pin 19 : software

static volatile uint32_t basura = 0;
static void trabajo_extra(uint32_t n) {
    for (uint32_t i = 0; i < n; i++) basura += i;
}

int main() {
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &squarewave_program);
    squarewave_program_init(pio, sm, offset, PIN_PIO);
    pio_sm_set_clkdiv(pio, sm, 1250.0f);
    pio_sm_set_enabled(pio, sm, true);

    gpio_init(PIN_SW);
    gpio_set_dir(PIN_SW, GPIO_OUT);

    uint32_t contador = 0;
    while (true) {
        gpio_put(PIN_SW, 1); sleep_us(20);
        gpio_put(PIN_SW, 0); sleep_us(20);
        if (++contador % 10 == 0) {
            trabajo_extra(50);   // <-- barrer: 50 / 200 / 500 / 1000
        }
    }
}