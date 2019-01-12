#include <sys/types.h>

#include <autoconf.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <misc/printk.h>

#include <board.h>
#include <device.h>
#include <led_strip.h>
#include <gpio.h>

#include <nrfx_timer.h>

#include "pov.h"

#if CONFIG_NUM_METAIRQ_PRIORITIES < 1
    #error Need atleast one MetaIRQ priority
#endif

#define POV_TREAD_STACK_SIZE 500
#define POV_TREAD_PRIORITY K_HIGHEST_THREAD_PRIO
#define POV_PIXEL_COLUMNS 36
#define POV_PIXEL_ROWS 8

// Use Timer2 since Timer1 appears to be used by the bluetooth stack
static const nrfx_timer_t kPovTimer = NRFX_TIMER_INSTANCE(2);
// TODO: the rate at which the PoV pixels are rendered. Needs to be dynamicly set based on RPM
static u32_t pov_timer_us = 1000; //Time (in microseconds) between consecutive compare events.

static u32_t pov_timer_ticks;

// A running counter. Will be deprecated
static int count = 0;
// Semaphore used to wake up  the POV_THEAD
static struct k_sem pov_update_sem;

// Our pixel buffer
static struct led_rgb pixels[POV_PIXEL_COLUMNS][POV_PIXEL_ROWS];

static struct device *led_strip = NULL,
                     *leds = NULL;

static void pov_thread(void *, void *, void *);

K_THREAD_STACK_DEFINE(pov_stack_area, POV_TREAD_STACK_SIZE);
struct k_thread pov_thread_data;
k_tid_t pov_thread_id;

void pov_timer_event_handler(nrf_timer_event_t event_type, void* p_context);

/// Wrapper to invoke the nRF5 SDK's timer IRS
ISR_DIRECT_DECLARE(timer2_isr)
{
    nrfx_timer_2_irq_handler();

    ISR_DIRECT_PM();
    return 1;
}

void pov_init()
{
    led_strip = device_get_binding(CONFIG_APA102_STRIP_NAME);
    leds = device_get_binding(LED1_GPIO_PORT);

    memset(pixels, 0, sizeof(pixels));

    k_sem_init(&pov_update_sem, 0, 1);

    // TODO: Remove this. is used for testing purposes only and not intended for final use.
    for(int col = 0; col < POV_PIXEL_COLUMNS; col++) {
        for(int row = 0; row < POV_PIXEL_ROWS; row++) {
            struct led_rgb *pixel = &pixels[col][row];
            pixel->b = col * 7;
            pixel->r = col * 7;
            pixel->g = 0;
        }
    }

    if(leds != NULL)
    {
        gpio_pin_configure(leds, LED1_GPIO_PIN, GPIO_DIR_OUT);
    }

    if(led_strip == NULL)
    {
        printk("Failed to get LED strip driver %s\n", CONFIG_APA102_STRIP_NAME);
    } else {
        led_strip_update_rgb(led_strip, pixels[0], 8);
    }

    nrfx_timer_config_t timer_config = {
        .frequency          = NRF_TIMER_FREQ_8MHz,
        .mode               = NRF_TIMER_MODE_TIMER,
        .bit_width          = NRF_TIMER_BIT_WIDTH_32,
        .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
        .p_context          = NULL
    };

    // Wire up the nordic's timer handler with Zephyr's IRQ vector table through the direct wrapper above.
    IRQ_DIRECT_CONNECT(NRF5_IRQ_TIMER2_IRQn, NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY, timer2_isr, 0);
    // TODO: not sure if this is required.
    irq_enable(NRF5_IRQ_TIMER2_IRQn);

    // Typical Timer setup
    nrfx_timer_init(&kPovTimer, &timer_config, pov_timer_event_handler);
    pov_timer_ticks = nrfx_timer_us_to_ticks(&kPovTimer, pov_timer_us);

    // Use a compare value to trigger the interrupt
    nrfx_timer_extended_compare(&kPovTimer,
            NRF_TIMER_CC_CHANNEL0,
            pov_timer_ticks,
            NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
            true);

    // Create our meta-IRQ thread for handling SPI driver stuff
    pov_thread_id = k_thread_create(
        &pov_thread_data, pov_stack_area,
        K_THREAD_STACK_SIZEOF(pov_stack_area),
        pov_thread,
        NULL, NULL, NULL,
        POV_TREAD_PRIORITY, K_ESSENTIAL, K_NO_WAIT);

    nrfx_timer_enable(&kPovTimer);
}

void pov_set_message(const char *message, ssize_t length)
{
    // TODO: Render the text to the pixel buffer.
}

void pov_set_rpm(u32_t rpm)
{
    // TODO: Divide the RPM into pixel columns and update the timer2 compare value to match

    // pov_timer_ticks = nrf_drv_timer_ms_to_ticks(&kPovTimer, pov_timer_ms);

    // nrf_drv_timer_extended_compare(
    //      &kPovTimer, NRF_TIMER_CC_CHANNEL0, pov_timer_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
}

void pov_timer_event_handler(nrf_timer_event_t event_type, void* p_context) {
    ARG_UNUSED(p_context);

    if (event_type == NRF_TIMER_EVENT_COMPARE0)
        k_sem_give(&pov_update_sem);
}

static void pov_thread(void *_p1, void *_p2, void *_p3)
{
    ARG_UNUSED(_p1); ARG_UNUSED(_p2); ARG_UNUSED(_p3);

    while(1)
    {
        // Wait for the timer interrupt
        k_sem_take(&pov_update_sem, K_FOREVER);

        // TODO: remove thie
        // Flash a LED for debug purposes only
        if(leds != NULL)
            gpio_pin_write(leds, LED1_GPIO_PIN, count++ & 0x01);

        // TODO: don't rely on counter MOD columns, visually glitches when counter overflows.
        // Push out a column of the pixel buffer to the LED strip
        if(led_strip != NULL)
            led_strip_update_rgb(led_strip, pixels[count % POV_PIXEL_COLUMNS], POV_PIXEL_ROWS);
    }
}
