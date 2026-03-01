#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include <stdbool.h>

// GPIO pin assignments (from spec GPIO table)
#define FEED_STEP_PIN       10  // GP10 - Feed STEP
#define FEED_DIR_PIN        11  // GP11 - Feed DIR
#define RECOVERY_STEP_PIN   15  // GP15 - Recovery STEP
#define RECOVERY_DIR_PIN    14  // GP14 - Recovery DIR

// Manual button and mode pins
#define MANUAL_RECOVER_PIN  9   // GP9  - Manual Recover button (active low)
#define MANUAL_FEED_PIN     13  // GP13 - Manual Feed button (active low)
#define AUTO_MANUAL_PIN     12  // GP12 - Auto/Manual switch (high=auto, low=manual)

// Motor mechanical parameters
#define FULL_STEPS_PER_REV      200     // 1.8 degree stepper
#define GEARBOX_RATIO           27      // 27:1 gearbox
#define BELT_RATIO              4       // 80T/20T = 4:1 belt stage
#define MICROSTEP               8       // TMC2209 microstepping setting
#define STEPS_PER_SPOOL_REV     (FULL_STEPS_PER_REV * GEARBOX_RATIO * BELT_RATIO * MICROSTEP)
// = 200 * 27 * 4 * 8 = 172,800 steps/rev

// PIO clock (125 MHz default)
#define PIO_CLOCK_HZ    125000000

// Minimum step pulse high time in PIO cycles (matches PULSE_HIGH_CYCLES in .pio)
// 10 cycles at 125MHz = 80ns; TMC2209 requires min 100ns, use 10 cycles (80ns) + overhead
#define PULSE_HIGH_CYCLES   10

// Manual speed: fixed step rate when buttons pressed (steps/sec)
#define MANUAL_STEP_RATE    1000.0f

// Initialize both stepper PIO state machines and direction GPIO pins.
void stepper_init(void);

// Set feed spool step rate in steps/sec. Pass 0.0f to stop.
void stepper_set_feed_rate(float steps_per_sec);

// Set recovery spool step rate in steps/sec. Pass 0.0f to stop.
void stepper_set_recovery_rate(float steps_per_sec);

// Stop both motors immediately.
void stepper_stop_all(void);

// Read manual button states (debounced).
// Returns true if the manual feed button is currently pressed.
bool stepper_manual_feed_pressed(void);

// Returns true if the manual recover button is currently pressed.
bool stepper_manual_recover_pressed(void);

// Returns true if the auto/manual switch is in auto position.
bool stepper_in_auto_mode(void);

#endif // STEPPER_H
