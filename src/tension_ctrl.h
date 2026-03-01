#ifndef TENSION_CTRL_H
#define TENSION_CTRL_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/time.h"

// ============================================================
// Modbus register indices (0-based)
// ============================================================

// Holding registers (FC03 read, FC06/FC16 write)
#define REG_WIRE_FEED_RATE      0   // rev/min x100  (e.g. 100 = 1.00 rev/min)
#define REG_DANCER_SETPOINT     1   // cm x1000      (e.g. 3750 = 37.5mm)
#define REG_MIN_DANCER_POS      2   // cm x1000
#define REG_MAX_DANCER_POS      3   // cm x1000
#define REG_PID_KP              4   // gain x1000    (e.g. 1000 = 1.0)
#define REG_PID_KI              5   // gain x1000
#define REG_PID_KD              6   // gain x1000
#define REG_FAULT_CLEAR         7   // write 1 to clear fault latch
#define REG_SPOOL_RESET         8   // write 1 to reset (future use)
#define NUM_HOLDING_REGS        9

// Input registers (FC04 read only)
#define INREG_DANCER_POSITION   0   // cm x1000 (filtered)
#define INREG_WATER_TDS         1   // raw ADC 0-4095
#define INREG_FAULT_CODE        2   // 0=none, 1=wire break (min), 2=over-travel (max)
#define NUM_INPUT_REGS          3

// Coil indices (FC01 read, FC05 write)
#define COIL_WIRE_FEED          0   // 0=off, 1=running
#define NUM_COILS               1

// Discrete input indices (FC02 read)
#define DINPUT_WIRE_FAULT       0
#define DINPUT_AUTO_MODE        1
#define NUM_DISCRETE_INPUTS     2

// Fault codes
#define FAULT_NONE              0
#define FAULT_WIRE_BREAK        1   // Dancer at minimum (wire break)
#define FAULT_OVER_TRAVEL       2   // Dancer at maximum (wire slack/overrun)

// ============================================================
// Shared register bank (accessed by both tension_ctrl and modbus_server)
// ============================================================
extern volatile uint16_t g_holding_regs[NUM_HOLDING_REGS];
extern volatile uint16_t g_input_regs[NUM_INPUT_REGS];
extern volatile uint8_t  g_coils;          // Bit field, 1 bit per coil
extern volatile uint8_t  g_discrete_inputs; // Bit field, 1 bit per discrete input

// Accessor macros
#define GET_COIL(n)         (((g_coils) >> (n)) & 1)
#define SET_COIL(n, v)      do { if (v) g_coils |= (1u << (n)); \
                                 else   g_coils &= ~(1u << (n)); } while(0)
#define GET_DINPUT(n)       (((g_discrete_inputs) >> (n)) & 1)
#define SET_DINPUT(n, v)    do { if (v) g_discrete_inputs |= (1u << (n)); \
                                 else   g_discrete_inputs &= ~(1u << (n)); } while(0)

// ADC channels
#define ADC_TDS_CHANNEL     0   // GP26 (ADC0) - TDS water sensor
#define ADC_DANCER_CHANNEL  1   // GP27 (ADC1) - Dancer position pot

// Dancer pot full-scale: KTR2-R-75 = 75mm stroke, 3.3V full scale, 12-bit ADC
// Conversion: dancer_cm_x1000 = (adc_raw * 7500) / 4095
#define DANCER_MAX_MM       75
#define ADC_MAX             4095

// ============================================================
// Public API
// ============================================================

// Initialize tension controller: registers, ADC, GPIO, PID, timer.
void tension_ctrl_init(void);

// Timer callback — called at 100Hz by main.c.
// Handles: ADC read, PID, motor commands, fault detection, manual buttons.
bool tension_ctrl_timer_cb(struct repeating_timer* t);

#endif // TENSION_CTRL_H
