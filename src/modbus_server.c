#include "modbus_server.h"
#include "tension_ctrl.h"

#include "socket.h"
#include "wizchip_conf.h"

#include <string.h>
#include <stdio.h>

// Modbus TCP buffers
static uint8_t g_rx_buffer[MODBUS_TCP_BUFFER_SIZE];
static uint8_t g_tx_buffer[MODBUS_TCP_BUFFER_SIZE];

// Forward declarations
static int16_t modbus_process_request(uint8_t* request, uint16_t request_len, uint8_t* response);
static int16_t modbus_read_coils(uint16_t start_addr, uint16_t quantity, uint8_t* resp_pdu);
static int16_t modbus_read_discrete_inputs(uint16_t start_addr, uint16_t quantity, uint8_t* resp_pdu);
static int16_t modbus_read_holding_registers(uint16_t start_addr, uint16_t quantity, uint8_t* resp_pdu);
static int16_t modbus_read_input_registers(uint16_t start_addr, uint16_t quantity, uint8_t* resp_pdu);
static int16_t modbus_write_single_coil(uint16_t addr, uint16_t value, uint8_t* resp_pdu);
static int16_t modbus_write_single_register(uint16_t addr, uint16_t value, uint8_t* resp_pdu);
static int16_t modbus_write_multiple_registers(uint16_t start_addr, uint16_t quantity,
                                               uint8_t byte_count, uint8_t* data,
                                               uint8_t* resp_pdu);
static void modbus_build_exception(uint8_t function_code, uint8_t exception_code, uint8_t* resp_pdu);

// ============================================================
// Public API
// ============================================================

void modbus_server_init(void)
{
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_tx_buffer, 0, sizeof(g_tx_buffer));
}

bool modbus_server_run(void)
{
    uint8_t  sock_status;
    int16_t  recv_len;
    int16_t  resp_len;

    sock_status = getSn_SR(MODBUS_SOCKET);

    switch (sock_status) {
        case SOCK_ESTABLISHED:
            recv_len = getSn_RX_RSR(MODBUS_SOCKET);
            if (recv_len > 0) {
                if (recv_len > MODBUS_TCP_BUFFER_SIZE) {
                    recv_len = MODBUS_TCP_BUFFER_SIZE;
                }
                recv_len = recv(MODBUS_SOCKET, g_rx_buffer, recv_len);
                if (recv_len > 0) {
                    resp_len = modbus_process_request(g_rx_buffer, recv_len, g_tx_buffer);
                    if (resp_len > 0) {
                        send(MODBUS_SOCKET, g_tx_buffer, resp_len);
                    }
                    return true;
                }
            }
            break;

        case SOCK_CLOSE_WAIT:
            disconnect(MODBUS_SOCKET);
            break;

        case SOCK_CLOSED:
            if (socket(MODBUS_SOCKET, Sn_MR_TCP, MODBUS_TCP_PORT, 0x00) == MODBUS_SOCKET) {
                listen(MODBUS_SOCKET);
            }
            break;

        case SOCK_INIT:
            listen(MODBUS_SOCKET);
            break;

        case SOCK_LISTEN:
        default:
            break;
    }

    return false;
}

// ============================================================
// Internal: request dispatcher
// ============================================================

static int16_t modbus_process_request(uint8_t* request, uint16_t request_len, uint8_t* response)
{
    if (request_len < MODBUS_MBAP_HEADER_SIZE + 1) {
        return -1;
    }

    // Parse MBAP header
    uint16_t transaction_id = ((uint16_t)request[0] << 8) | request[1];
    uint16_t protocol_id    = ((uint16_t)request[2] << 8) | request[3];

    if (protocol_id != 0x0000) {
        return -1;  // Not Modbus TCP
    }

    uint8_t* pdu     = &request[MODBUS_MBAP_HEADER_SIZE];
    uint8_t* resp_pdu = &response[MODBUS_MBAP_HEADER_SIZE];
    uint8_t  fc       = pdu[0];

    int16_t resp_pdu_len = -1;

    switch (fc) {
        case MODBUS_FC_READ_COILS: {
            if (request_len < MODBUS_MBAP_HEADER_SIZE + 5) break;
            uint16_t start_addr = ((uint16_t)pdu[1] << 8) | pdu[2];
            uint16_t quantity   = ((uint16_t)pdu[3] << 8) | pdu[4];
            resp_pdu_len = modbus_read_coils(start_addr, quantity, resp_pdu);
            break;
        }

        case MODBUS_FC_READ_DISCRETE_INPUTS: {
            if (request_len < MODBUS_MBAP_HEADER_SIZE + 5) break;
            uint16_t start_addr = ((uint16_t)pdu[1] << 8) | pdu[2];
            uint16_t quantity   = ((uint16_t)pdu[3] << 8) | pdu[4];
            resp_pdu_len = modbus_read_discrete_inputs(start_addr, quantity, resp_pdu);
            break;
        }

        case MODBUS_FC_READ_HOLDING_REGISTERS: {
            if (request_len < MODBUS_MBAP_HEADER_SIZE + 5) break;
            uint16_t start_addr = ((uint16_t)pdu[1] << 8) | pdu[2];
            uint16_t quantity   = ((uint16_t)pdu[3] << 8) | pdu[4];
            resp_pdu_len = modbus_read_holding_registers(start_addr, quantity, resp_pdu);
            break;
        }

        case MODBUS_FC_READ_INPUT_REGISTERS: {
            if (request_len < MODBUS_MBAP_HEADER_SIZE + 5) break;
            uint16_t start_addr = ((uint16_t)pdu[1] << 8) | pdu[2];
            uint16_t quantity   = ((uint16_t)pdu[3] << 8) | pdu[4];
            resp_pdu_len = modbus_read_input_registers(start_addr, quantity, resp_pdu);
            break;
        }

        case MODBUS_FC_WRITE_SINGLE_COIL: {
            if (request_len < MODBUS_MBAP_HEADER_SIZE + 5) break;
            uint16_t addr  = ((uint16_t)pdu[1] << 8) | pdu[2];
            uint16_t value = ((uint16_t)pdu[3] << 8) | pdu[4];
            resp_pdu_len = modbus_write_single_coil(addr, value, resp_pdu);
            break;
        }

        case MODBUS_FC_WRITE_SINGLE_REGISTER: {
            if (request_len < MODBUS_MBAP_HEADER_SIZE + 5) break;
            uint16_t addr  = ((uint16_t)pdu[1] << 8) | pdu[2];
            uint16_t value = ((uint16_t)pdu[3] << 8) | pdu[4];
            resp_pdu_len = modbus_write_single_register(addr, value, resp_pdu);
            break;
        }

        case MODBUS_FC_WRITE_MULTIPLE_REGS: {
            if (request_len < MODBUS_MBAP_HEADER_SIZE + 6) break;
            uint16_t start_addr = ((uint16_t)pdu[1] << 8) | pdu[2];
            uint16_t quantity   = ((uint16_t)pdu[3] << 8) | pdu[4];
            uint8_t  byte_count = pdu[5];
            if (request_len < (uint16_t)(MODBUS_MBAP_HEADER_SIZE + 6 + byte_count)) break;
            resp_pdu_len = modbus_write_multiple_registers(start_addr, quantity,
                                                           byte_count, &pdu[6], resp_pdu);
            break;
        }

        default:
            modbus_build_exception(fc, MODBUS_EX_ILLEGAL_FUNCTION, resp_pdu);
            resp_pdu_len = 2;
            break;
    }

    if (resp_pdu_len < 0) {
        return -1;
    }

    // Build MBAP response header
    response[0] = (transaction_id >> 8) & 0xFF;
    response[1] =  transaction_id       & 0xFF;
    response[2] = 0x00;  // Protocol ID high
    response[3] = 0x00;  // Protocol ID low
    uint16_t length = 1 + resp_pdu_len;  // Unit ID byte + PDU
    response[4] = (length >> 8) & 0xFF;
    response[5] =  length       & 0xFF;
    response[6] = 0x01;  // Unit ID (slave address)

    return MODBUS_MBAP_HEADER_SIZE + resp_pdu_len;
}

// ============================================================
// FC 0x01 — Read Coils
// ============================================================
static int16_t modbus_read_coils(uint16_t start_addr, uint16_t quantity, uint8_t* resp_pdu)
{
    if (quantity == 0 || quantity > 2000 || start_addr + quantity > NUM_COILS) {
        modbus_build_exception(MODBUS_FC_READ_COILS, MODBUS_EX_ILLEGAL_DATA_ADDRESS, resp_pdu);
        return 2;
    }

    uint8_t byte_count = (uint8_t)((quantity + 7) / 8);
    resp_pdu[0] = MODBUS_FC_READ_COILS;
    resp_pdu[1] = byte_count;
    memset(&resp_pdu[2], 0, byte_count);

    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t coil_idx = start_addr + i;
        if (GET_COIL(coil_idx)) {
            resp_pdu[2 + i / 8] |= (1 << (i % 8));
        }
    }

    return 2 + byte_count;
}

// ============================================================
// FC 0x02 — Read Discrete Inputs
// ============================================================
static int16_t modbus_read_discrete_inputs(uint16_t start_addr, uint16_t quantity, uint8_t* resp_pdu)
{
    if (quantity == 0 || quantity > 2000 || start_addr + quantity > NUM_DISCRETE_INPUTS) {
        modbus_build_exception(MODBUS_FC_READ_DISCRETE_INPUTS, MODBUS_EX_ILLEGAL_DATA_ADDRESS, resp_pdu);
        return 2;
    }

    uint8_t byte_count = (uint8_t)((quantity + 7) / 8);
    resp_pdu[0] = MODBUS_FC_READ_DISCRETE_INPUTS;
    resp_pdu[1] = byte_count;
    memset(&resp_pdu[2], 0, byte_count);

    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t input_idx = start_addr + i;
        if (GET_DINPUT(input_idx)) {
            resp_pdu[2 + i / 8] |= (1 << (i % 8));
        }
    }

    return 2 + byte_count;
}

// ============================================================
// FC 0x03 — Read Holding Registers
// ============================================================
static int16_t modbus_read_holding_registers(uint16_t start_addr, uint16_t quantity, uint8_t* resp_pdu)
{
    if (quantity == 0 || quantity > 125 || start_addr + quantity > NUM_HOLDING_REGS) {
        modbus_build_exception(MODBUS_FC_READ_HOLDING_REGISTERS, MODBUS_EX_ILLEGAL_DATA_ADDRESS, resp_pdu);
        return 2;
    }

    uint8_t byte_count = (uint8_t)(quantity * 2);
    resp_pdu[0] = MODBUS_FC_READ_HOLDING_REGISTERS;
    resp_pdu[1] = byte_count;

    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t val = g_holding_regs[start_addr + i];
        resp_pdu[2 + i * 2]     = (val >> 8) & 0xFF;
        resp_pdu[2 + i * 2 + 1] =  val       & 0xFF;
    }

    return 2 + byte_count;
}

// ============================================================
// FC 0x04 — Read Input Registers
// ============================================================
static int16_t modbus_read_input_registers(uint16_t start_addr, uint16_t quantity, uint8_t* resp_pdu)
{
    if (quantity == 0 || quantity > 125 || start_addr + quantity > NUM_INPUT_REGS) {
        modbus_build_exception(MODBUS_FC_READ_INPUT_REGISTERS, MODBUS_EX_ILLEGAL_DATA_ADDRESS, resp_pdu);
        return 2;
    }

    uint8_t byte_count = (uint8_t)(quantity * 2);
    resp_pdu[0] = MODBUS_FC_READ_INPUT_REGISTERS;
    resp_pdu[1] = byte_count;

    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t val = g_input_regs[start_addr + i];
        resp_pdu[2 + i * 2]     = (val >> 8) & 0xFF;
        resp_pdu[2 + i * 2 + 1] =  val       & 0xFF;
    }

    return 2 + byte_count;
}

// ============================================================
// FC 0x05 — Write Single Coil
// ============================================================
static int16_t modbus_write_single_coil(uint16_t addr, uint16_t value, uint8_t* resp_pdu)
{
    if (addr >= NUM_COILS) {
        modbus_build_exception(MODBUS_FC_WRITE_SINGLE_COIL, MODBUS_EX_ILLEGAL_DATA_ADDRESS, resp_pdu);
        return 2;
    }
    if (value != 0x0000 && value != 0xFF00) {
        modbus_build_exception(MODBUS_FC_WRITE_SINGLE_COIL, MODBUS_EX_ILLEGAL_DATA_VALUE, resp_pdu);
        return 2;
    }

    SET_COIL(addr, value == 0xFF00 ? 1 : 0);

    // Echo request
    resp_pdu[0] = MODBUS_FC_WRITE_SINGLE_COIL;
    resp_pdu[1] = (addr  >> 8) & 0xFF;
    resp_pdu[2] =  addr        & 0xFF;
    resp_pdu[3] = (value >> 8) & 0xFF;
    resp_pdu[4] =  value       & 0xFF;
    return 5;
}

// ============================================================
// FC 0x06 — Write Single Register
// ============================================================
static int16_t modbus_write_single_register(uint16_t addr, uint16_t value, uint8_t* resp_pdu)
{
    if (addr >= NUM_HOLDING_REGS) {
        modbus_build_exception(MODBUS_FC_WRITE_SINGLE_REGISTER, MODBUS_EX_ILLEGAL_DATA_ADDRESS, resp_pdu);
        return 2;
    }

    g_holding_regs[addr] = value;

    // Echo request
    resp_pdu[0] = MODBUS_FC_WRITE_SINGLE_REGISTER;
    resp_pdu[1] = (addr  >> 8) & 0xFF;
    resp_pdu[2] =  addr        & 0xFF;
    resp_pdu[3] = (value >> 8) & 0xFF;
    resp_pdu[4] =  value       & 0xFF;
    return 5;
}

// ============================================================
// FC 0x10 — Write Multiple Registers
// ============================================================
static int16_t modbus_write_multiple_registers(uint16_t start_addr, uint16_t quantity,
                                               uint8_t byte_count, uint8_t* data,
                                               uint8_t* resp_pdu)
{
    if (quantity == 0 || quantity > 123 ||
        start_addr + quantity > NUM_HOLDING_REGS ||
        byte_count != quantity * 2) {
        modbus_build_exception(MODBUS_FC_WRITE_MULTIPLE_REGS, MODBUS_EX_ILLEGAL_DATA_ADDRESS, resp_pdu);
        return 2;
    }

    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t val = ((uint16_t)data[i * 2] << 8) | data[i * 2 + 1];
        g_holding_regs[start_addr + i] = val;
    }

    resp_pdu[0] = MODBUS_FC_WRITE_MULTIPLE_REGS;
    resp_pdu[1] = (start_addr >> 8) & 0xFF;
    resp_pdu[2] =  start_addr       & 0xFF;
    resp_pdu[3] = (quantity   >> 8) & 0xFF;
    resp_pdu[4] =  quantity         & 0xFF;
    return 5;
}

// ============================================================
// Exception response builder
// ============================================================
static void modbus_build_exception(uint8_t function_code, uint8_t exception_code, uint8_t* resp_pdu)
{
    resp_pdu[0] = function_code | 0x80;
    resp_pdu[1] = exception_code;
}
