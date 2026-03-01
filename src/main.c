#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/timer.h"

#include "w6100_spi.h"
#include "wizchip_conf.h"
#include "w6100.h"
#include "socket.h"

#include "tension_ctrl.h"
#include "modbus_server.h"

// ============================================================
// Static network configuration
// Change these to match your machine network.
// PC ethernet card should be set to 192.168.100.1/24.
// ============================================================
#define STATIC_IP       {192, 168, 100, 2}
#define STATIC_SUBNET   {255, 255, 255, 0}
#define STATIC_GATEWAY  {192, 168, 100, 1}

// Tension control loop interval: 100Hz = 10,000 us
#define TENSION_CTRL_INTERVAL_US  (-10000)  // Negative = best-effort scheduling

static uint8_t g_mac_addr[6];
static uint8_t g_ip_addr[4]  = STATIC_IP;
static uint8_t g_subnet[4]   = STATIC_SUBNET;
static uint8_t g_gateway[4]  = STATIC_GATEWAY;

static struct repeating_timer g_tension_timer;

// Forward declarations
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

    printf("\n========================================\n");
    print_network_info();
    printf("Modbus TCP Server ready on port %d\n", MODBUS_TCP_PORT);
    printf("========================================\n");

    // Main loop
    while (1) {
        modbus_server_run();
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

    // Wait for PHY link to come up before configuring the network stack.
    // Autonegotiation can take a few seconds; without link the W6100
    // silently discards all frames including ARP.
    printf("Waiting for PHY link");
    for (int i = 0; i < 100; i++) {
        ctlwizchip(CW_GET_PHYLINK, &tmp);
        if (tmp == PHY_LINK_ON) {
            printf(" OK\n");
            break;
        }
        printf(".");
        sleep_ms(100);
        if (i == 99) {
            printf(" TIMEOUT (no cable?)\n");
        }
    }

    // Apply static network configuration
    NETUNLOCK();
    setSHAR(g_mac_addr);
    setSIPR(g_ip_addr);
    setSUBR(g_subnet);
    setGAR(g_gateway);
    setNET4MR(0x00);
    NETLOCK();

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

static void print_network_info(void)
{
    printf("IP Address: %d.%d.%d.%d\n",
           g_ip_addr[0], g_ip_addr[1], g_ip_addr[2], g_ip_addr[3]);
    printf("Subnet:     %d.%d.%d.%d\n",
           g_subnet[0], g_subnet[1], g_subnet[2], g_subnet[3]);
    printf("Gateway:    %d.%d.%d.%d\n",
           g_gateway[0], g_gateway[1], g_gateway[2], g_gateway[3]);

    // Read back from W6100 registers to confirm the chip accepted the config
    uint8_t rb_ip[4], rb_mac[6];
    getSIPR(rb_ip);
    getSHAR(rb_mac);
    printf("W6100 IP (readback):  %d.%d.%d.%d\n",
           rb_ip[0], rb_ip[1], rb_ip[2], rb_ip[3]);
    printf("W6100 MAC (readback): %02X:%02X:%02X:%02X:%02X:%02X\n",
           rb_mac[0], rb_mac[1], rb_mac[2], rb_mac[3], rb_mac[4], rb_mac[5]);
}
