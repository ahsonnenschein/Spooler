#include "stepper.h"
#include "stepper.pio.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

// Use PIO0 for both state machines
#define STEPPER_PIO         pio0
#define FEED_SM             0
#define RECOVERY_SM         1

// Loaded PIO program offset (shared between both state machines)
static uint g_pio_offset;

// Convert steps/sec to the half-period value written to the PIO TX FIFO.
// half_period = PIO_CLK / (2 * steps_per_sec)
// Returns 0 if steps_per_sec <= 0 (stop condition).
static uint32_t steps_per_sec_to_half_period(float steps_per_sec)
{
    if (steps_per_sec <= 0.0f) {
        return 0;
    }
    float clk = (float)clock_get_hz(clk_sys);
    float half = clk / (2.0f * steps_per_sec);
    if (half < (float)(PULSE_HIGH_CYCLES + 2)) {
        // Clamp to maximum achievable frequency
        half = (float)(PULSE_HIGH_CYCLES + 2);
    }
    return (uint32_t)half;
}

void stepper_init(void)
{
    // Load PIO program into PIO0
    g_pio_offset = pio_add_program(STEPPER_PIO, &stepper_pio_program);

    // Initialize feed stepper state machine
    stepper_pio_program_init(STEPPER_PIO, FEED_SM, g_pio_offset, FEED_STEP_PIN);

    // Initialize recovery stepper state machine
    stepper_pio_program_init(STEPPER_PIO, RECOVERY_SM, g_pio_offset, RECOVERY_STEP_PIN);

    // Initialize direction pins as GPIO outputs, default forward direction
    gpio_init(FEED_DIR_PIN);
    gpio_set_dir(FEED_DIR_PIN, GPIO_OUT);
    gpio_put(FEED_DIR_PIN, 1);

    gpio_init(RECOVERY_DIR_PIN);
    gpio_set_dir(RECOVERY_DIR_PIN, GPIO_OUT);
    gpio_put(RECOVERY_DIR_PIN, 1);

    // Initialize manual control pins with pull-ups (active low buttons)
    gpio_init(MANUAL_FEED_PIN);
    gpio_set_dir(MANUAL_FEED_PIN, GPIO_IN);
    gpio_pull_up(MANUAL_FEED_PIN);

    gpio_init(MANUAL_RECOVER_PIN);
    gpio_set_dir(MANUAL_RECOVER_PIN, GPIO_IN);
    gpio_pull_up(MANUAL_RECOVER_PIN);

    // Auto/manual switch: high = auto, low = manual
    gpio_init(AUTO_MANUAL_PIN);
    gpio_set_dir(AUTO_MANUAL_PIN, GPIO_IN);
    gpio_pull_up(AUTO_MANUAL_PIN);

    // Start both state machines stopped (send half_period = 0)
    stepper_stop_all();
}

void stepper_set_feed_rate(float steps_per_sec)
{
    uint32_t half_period = steps_per_sec_to_half_period(steps_per_sec);
    pio_sm_put(STEPPER_PIO, FEED_SM, half_period);
}

void stepper_set_recovery_rate(float steps_per_sec)
{
    uint32_t half_period = steps_per_sec_to_half_period(steps_per_sec);
    pio_sm_put(STEPPER_PIO, RECOVERY_SM, half_period);
}

void stepper_stop_all(void)
{
    // Writing 0 causes the PIO to enter the stopped branch
    pio_sm_put(STEPPER_PIO, FEED_SM, 0);
    pio_sm_put(STEPPER_PIO, RECOVERY_SM, 0);
}

bool stepper_manual_feed_pressed(void)
{
    return !gpio_get(MANUAL_FEED_PIN);  // Active low
}

bool stepper_manual_recover_pressed(void)
{
    return !gpio_get(MANUAL_RECOVER_PIN);  // Active low
}

bool stepper_in_auto_mode(void)
{
    return gpio_get(AUTO_MANUAL_PIN);  // High = auto
}
