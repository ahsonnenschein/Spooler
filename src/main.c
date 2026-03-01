#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/timer.h"

#include "w6100_spi.h"
#include "wizchip_conf.h"
#include "w6100.h"
#include "socket.h"
#include "dhcp.h"

#include "tension_ctrl.h"
#include "modbus_server.h"

// Network configuration
#define DHCP_SOCKET         7       // Socket 7 reserved for DHCP
#define DHCP_BUFFER_SIZE    1024
#define DHCP_TIMER_INTERVAL_US  1000000  // 1 second

// Tension control loop interval: 100Hz = 10,000 us
#define TENSION_CTRL_INTERVAL_US  (-10000)  // Negative = best-effort scheduling

// Buffers and state
static uint8_t g_dhcp_buffer[DHCP_BUFFER_SIZE];

static uint8_t g_mac_addr[6];
static uint8_t g_ip_addr[4]  = {0, 0, 0, 0};
static uint8_t g_subnet[4]   = {255, 255, 255, 0};
static uint8_t g_gateway[4]  = {0, 0, 0, 0};

static volatile bool g_dhcp_got_ip = false;
static struct repeating_timer g_dhcp_timer;
static struct repeating_timer g_tension_timer;

// Forward declarations
static void dhcp_ip_assign(void);
static void dhcp_ip_update(void);
static void dhcp_ip_conflict(void);
static bool dhcp_timer_callback(struct repeating_timer* t);
static bool network_init(void);
static void generate_mac_address(uint8_t* mac);
static void print_network_info(void);

int main(void)
{
    stdio_init_all();

    // Wait for USB serial
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(500);

    printf("\n");
    printf("========================================\n");
    printf("SPOOLER Wire Tension Controller\n");
    printf("W6100-EVB-Pico2 (RP2350)\n");
    printf("========================================\n");

    // Initialize tension controller (ADC, GPIO, PIO steppers, PID)
    tension_ctrl_init();

    // Start 100Hz tension control timer
    add_repeating_timer_us(TENSION_CTRL_INTERVAL_US, tension_ctrl_timer_cb, NULL, &g_tension_timer);

    // Initialize W6100 SPI and Ethernet
    w6100_spi_init();

    if (!network_init()) {
        printf("ERROR: Failed to initialize network!\n");
        while (1) {
            sleep_ms(1000);
        }
    }

    // Initialize Modbus server buffers
    modbus_server_init();

    printf("Waiting for DHCP...\n");

    // Main loop
    while (1) {
        uint8_t dhcp_status = DHCP_run();

        if (dhcp_status == DHCP_IP_LEASED || g_dhcp_got_ip) {
            modbus_server_run();
        }

        sleep_us(100);
    }

    return 0;
}

static bool network_init(void)
{
    uint8_t tmp;
    uint8_t mem_size[16] = {2, 2, 2, 2, 2, 2, 2, 2,
                            2, 2, 2, 2, 2, 2, 2, 2};

    generate_mac_address(g_mac_addr);

    if (ctlwizchip(CW_GET_PHYLINK, &tmp) != 0) {
        printf("ERROR: W6100 not responding!\n");
        return false;
    }

    if (ctlwizchip(CW_INIT_WIZCHIP, mem_size) != 0) {
        printf("ERROR: Failed to initialize W6100 memory!\n");
        return false;
    }

    NETUNLOCK();
    setSHAR(g_mac_addr);
    NETLOCK();

    setNET4MR(0x00);

    reg_dhcp_cbfunc(dhcp_ip_assign, dhcp_ip_update, dhcp_ip_conflict);
    DHCP_init(DHCP_SOCKET, g_dhcp_buffer);

    add_repeating_timer_us(DHCP_TIMER_INTERVAL_US, dhcp_timer_callback, NULL, &g_dhcp_timer);

    return true;
}

static void generate_mac_address(uint8_t* mac)
{
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);

    mac[0] = 0x00;  // WIZnet OUI
    mac[1] = 0x08;
    mac[2] = 0xDC;
    mac[3] = board_id.id[5];
    mac[4] = board_id.id[6];
    mac[5] = board_id.id[7];
}

static void dhcp_ip_assign(void)
{
    getIPfromDHCP(g_ip_addr);
    getGWfromDHCP(g_gateway);
    getSNfromDHCP(g_subnet);

    NETUNLOCK();
    setSIPR(g_ip_addr);
    setGAR(g_gateway);
    setSUBR(g_subnet);
    NETLOCK();

    g_dhcp_got_ip = true;

    printf("\n========================================\n");
    printf("DHCP IP Assigned!\n");
    print_network_info();
    printf("Modbus TCP Server ready on port %d\n", MODBUS_TCP_PORT);
    printf("========================================\n");
}

static void dhcp_ip_update(void)
{
    getIPfromDHCP(g_ip_addr);
    getGWfromDHCP(g_gateway);
    getSNfromDHCP(g_subnet);

    NETUNLOCK();
    setSIPR(g_ip_addr);
    setGAR(g_gateway);
    setSUBR(g_subnet);
    NETLOCK();

    printf("\nDHCP IP Updated!\n");
    print_network_info();
}

static void dhcp_ip_conflict(void)
{
    printf("ERROR: DHCP IP conflict detected!\n");
    g_dhcp_got_ip = false;
}

static bool dhcp_timer_callback(struct repeating_timer* t)
{
    (void)t;
    DHCP_time_handler();
    return true;
}

static void print_network_info(void)
{
    printf("IP Address: %d.%d.%d.%d\n",
           g_ip_addr[0], g_ip_addr[1], g_ip_addr[2], g_ip_addr[3]);
    printf("Subnet:     %d.%d.%d.%d\n",
           g_subnet[0], g_subnet[1], g_subnet[2], g_subnet[3]);
    printf("Gateway:    %d.%d.%d.%d\n",
           g_gateway[0], g_gateway[1], g_gateway[2], g_gateway[3]);
}
