#include "tension_ctrl.h"
#include "stepper.h"
#include "pid.h"

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#include <string.h>
#include <stdio.h>

// ============================================================
// Shared register bank definitions
// ============================================================
volatile uint16_t g_holding_regs[NUM_HOLDING_REGS];
volatile uint16_t g_input_regs[NUM_INPUT_REGS];
volatile uint8_t  g_coils          = 0;
volatile uint8_t  g_discrete_inputs = 0;

// ============================================================
// Private state
// ============================================================
static pid_ctrl_t g_pid;
static float g_dancer_filtered = 0.0f;  // Low-pass filtered dancer position (cm x1000)
static bool  g_fault_latched   = false;

#define PID_DT          0.01f   // 100Hz = 10ms
#define IIR_ALPHA       0.1f    // Low-pass filter coefficient (0=slow, 1=no filter)

// Maximum recovery step rate offset allowed from PID output (steps/sec)
// Prevents runaway; set conservatively at 2x typical feed rate
#define MAX_PID_OUTPUT_STEPS    50000.0f

// ============================================================
// Private helpers
// ============================================================

static uint16_t adc_read_channel(uint8_t channel)
{
    adc_select_input(channel);
    return adc_read();
}

// Convert raw 12-bit ADC to dancer position in cm x1000
static uint16_t adc_to_dancer_cm_x1000(uint16_t adc_raw)
{
    // KTR2-R-75: 75mm stroke, linear 0-3.3V, 12-bit ADC
    return (uint16_t)(((uint32_t)adc_raw * DANCER_MAX_MM * 100) / ADC_MAX);
}

// Apply PID gains from holding registers (stored as gain x1000)
static void apply_pid_gains(void)
{
    float kp = (float)g_holding_regs[REG_PID_KP] / 1000.0f;
    float ki = (float)g_holding_regs[REG_PID_KI] / 1000.0f;
    float kd = (float)g_holding_regs[REG_PID_KD] / 1000.0f;
    pid_set_gains(&g_pid, kp, ki, kd);
}

// ============================================================
// Public API
// ============================================================

void tension_ctrl_init(void)
{
    // Default holding register values
    memset((void*)g_holding_regs, 0, sizeof(g_holding_regs));
    g_holding_regs[REG_WIRE_FEED_RATE]  = 100;   // 1.00 rev/min
    g_holding_regs[REG_DANCER_SETPOINT] = 3750;  // 37.5mm (midpoint of 75mm stroke)
    g_holding_regs[REG_MIN_DANCER_POS]  = 500;   // 5mm
    g_holding_regs[REG_MAX_DANCER_POS]  = 7000;  // 70mm
    g_holding_regs[REG_PID_KP]          = 1000;  // 1.0
    g_holding_regs[REG_PID_KI]          = 0;
    g_holding_regs[REG_PID_KD]          = 0;

    memset((void*)g_input_regs, 0, sizeof(g_input_regs));
    g_coils           = 0;
    g_discrete_inputs = 0;

    // Initialize ADC
    adc_init();
    adc_gpio_init(26);  // GP26 = ADC0 = TDS sensor
    adc_gpio_init(27);  // GP27 = ADC1 = dancer pot

    // Initialize PID
    pid_init(&g_pid);
    apply_pid_gains();
    pid_set_imax(&g_pid, MAX_PID_OUTPUT_STEPS);

    // Initialize stepper GPIO and PIO
    stepper_init();

    printf("Tension controller initialized.\n");
    printf("  Steps/spool rev: %d\n", STEPS_PER_SPOOL_REV);
    printf("  Default setpoint: %.1f mm\n",
           (float)g_holding_regs[REG_DANCER_SETPOINT] / 100.0f);
}

bool tension_ctrl_timer_cb(struct repeating_timer* t)
{
    (void)t;

    // --------------------------------------------------------
    // 1. Read and filter dancer ADC
    // --------------------------------------------------------
    uint16_t dancer_raw = adc_read_channel(ADC_DANCER_CHANNEL);
    uint16_t dancer_cm_x1000 = adc_to_dancer_cm_x1000(dancer_raw);

    // IIR low-pass filter
    g_dancer_filtered = (1.0f - IIR_ALPHA) * g_dancer_filtered
                        + IIR_ALPHA * (float)dancer_cm_x1000;
    uint16_t dancer_pos = (uint16_t)g_dancer_filtered;
    g_input_regs[INREG_DANCER_POSITION] = dancer_pos;

    // --------------------------------------------------------
    // 2. Read TDS sensor
    // --------------------------------------------------------
    g_input_regs[INREG_WATER_TDS] = adc_read_channel(ADC_TDS_CHANNEL);

    // --------------------------------------------------------
    // 2b. Read raw button states (visible over Modbus for debugging)
    // --------------------------------------------------------
    g_input_regs[INREG_FEED_BTN]    = stepper_manual_feed_pressed()    ? 1 : 0;
    g_input_regs[INREG_RECOVER_BTN] = stepper_manual_recover_pressed() ? 1 : 0;

    // --------------------------------------------------------
    // 3. Detect auto/manual mode from switch
    // --------------------------------------------------------
    bool auto_mode = stepper_in_auto_mode();
    SET_DINPUT(DINPUT_AUTO_MODE, auto_mode);

    // --------------------------------------------------------
    // 4. Handle FAULT_CLEAR register (written by Modbus)
    // --------------------------------------------------------
    if (g_holding_regs[REG_FAULT_CLEAR]) {
        g_holding_regs[REG_FAULT_CLEAR] = 0;
        g_fault_latched = false;
        g_input_regs[INREG_FAULT_CODE] = FAULT_NONE;
        SET_DINPUT(DINPUT_WIRE_FAULT, 0);
        pid_reset(&g_pid);
        printf("Fault cleared.\n");
    }

    // --------------------------------------------------------
    // 5. Apply current PID gains (allows live tuning via Modbus)
    // --------------------------------------------------------
    apply_pid_gains();

    // --------------------------------------------------------
    // 6. Fault detection (independent of mode)
    // --------------------------------------------------------
    if (!g_fault_latched) {
        uint16_t min_pos = g_holding_regs[REG_MIN_DANCER_POS];
        uint16_t max_pos = g_holding_regs[REG_MAX_DANCER_POS];

        if (dancer_pos < min_pos) {
            g_fault_latched = true;
            g_input_regs[INREG_FAULT_CODE] = FAULT_WIRE_BREAK;
            SET_DINPUT(DINPUT_WIRE_FAULT, 1);
            SET_COIL(COIL_WIRE_FEED, 0);
            stepper_stop_all();
            printf("FAULT: Wire break detected (dancer pos %u < min %u).\n",
                   dancer_pos, min_pos);
        } else if (dancer_pos > max_pos) {
            g_fault_latched = true;
            g_input_regs[INREG_FAULT_CODE] = FAULT_OVER_TRAVEL;
            SET_DINPUT(DINPUT_WIRE_FAULT, 1);
            SET_COIL(COIL_WIRE_FEED, 0);
            stepper_stop_all();
            printf("FAULT: Over-travel detected (dancer pos %u > max %u).\n",
                   dancer_pos, max_pos);
        }
    }

    // --------------------------------------------------------
    // 7. Motor control
    // --------------------------------------------------------
    if (!auto_mode) {
        // -- Manual mode -- fault does not block jogging; PID disabled
        pid_reset(&g_pid);

        float feed_rate     = stepper_manual_feed_pressed()    ? MANUAL_STEP_RATE : 0.0f;
        float recovery_rate = stepper_manual_recover_pressed() ? MANUAL_STEP_RATE : 0.0f;

        stepper_set_feed_rate(feed_rate);
        stepper_set_recovery_rate(recovery_rate);
        return true;
    }

    // Auto mode: fault stops all motors
    if (g_fault_latched) {
        stepper_stop_all();
        return true;
    }

    if (auto_mode) {
        // -- Automatic mode --
        if (GET_COIL(COIL_WIRE_FEED)) {
            // Feed rate: WIRE_FEED_RATE is in rev/min x100
            // f_feed = (rate_x100 / 100) rev/min * STEPS_PER_SPOOL_REV / 60 steps/sec
            float feed_rate_rpm = (float)g_holding_regs[REG_WIRE_FEED_RATE] / 100.0f;
            float f_feed = feed_rate_rpm * (float)STEPS_PER_SPOOL_REV / 60.0f;

            // PID: setpoint and measurement in cm x1000 (same units, so no scaling needed)
            float setpoint  = (float)g_holding_regs[REG_DANCER_SETPOINT];
            float u = pid_update(&g_pid, setpoint, g_dancer_filtered, PID_DT);

            // Recovery = feed + PID correction
            float f_recovery = f_feed + u;
            if (f_recovery < 0.0f) f_recovery = 0.0f;

            stepper_set_feed_rate(f_feed);
            stepper_set_recovery_rate(f_recovery);
        } else {
            // WIRE_FEED coil off — stop both motors
            stepper_stop_all();
            pid_reset(&g_pid);
        }
    }

    return true;
}
