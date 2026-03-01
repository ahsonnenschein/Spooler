#ifndef MODBUS_SERVER_H
#define MODBUS_SERVER_H

#include <stdint.h>
#include <stdbool.h>

// Modbus TCP default port
#define MODBUS_TCP_PORT  502

// Modbus function codes
#define MODBUS_FC_READ_COILS              0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS    0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS  0x03
#define MODBUS_FC_READ_INPUT_REGISTERS    0x04
#define MODBUS_FC_WRITE_SINGLE_COIL       0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER   0x06
#define MODBUS_FC_WRITE_MULTIPLE_REGS     0x10

// Modbus exception codes
#define MODBUS_EX_ILLEGAL_FUNCTION        0x01
#define MODBUS_EX_ILLEGAL_DATA_ADDRESS    0x02
#define MODBUS_EX_ILLEGAL_DATA_VALUE      0x03
#define MODBUS_EX_SLAVE_DEVICE_FAILURE    0x04

// Modbus TCP MBAP header size
#define MODBUS_MBAP_HEADER_SIZE  7

// Maximum Modbus PDU size (253 bytes max per spec)
#define MODBUS_MAX_PDU_SIZE  253

// Buffer size for Modbus TCP frame
#define MODBUS_TCP_BUFFER_SIZE  (MODBUS_MBAP_HEADER_SIZE + MODBUS_MAX_PDU_SIZE)

// Socket number for Modbus server
#define MODBUS_SOCKET  0

// Initialize Modbus TCP server
void modbus_server_init(void);

// Run Modbus TCP server (call in main loop)
// Returns true if a request was processed
bool modbus_server_run(void);

#endif // MODBUS_SERVER_H
