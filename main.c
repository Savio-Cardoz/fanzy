#include <stdio.h>
#include "pico/stdlib.h"
#include "board.h"
#include <bsp/board_api.h>
#include <tusb.h>
#include "usb_cdc.h"
#include "host_communication.h"

bool led_state = false;

bool systick_callback(repeating_timer_t *rt);

int main()
{
    tusb_init();

    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    host_comm_init();

    // Setup systick timer
    repeating_timer_t timer;
    // negative timeout means exact delay (rather than delay between callbacks)
    if (!add_repeating_timer_us(-500000, systick_callback, NULL, &timer))
    {
        gpio_put(LED_PIN, 1);
        return 1;
    }
    while (true)
    {
        usb_cdc_tick();

        host_comm_tick();
    }
}

bool systick_callback(repeating_timer_t *rt)
{
    gpio_put(LED_PIN, led_state);
    led_state = !led_state;
    return true; // keep repeating
}
