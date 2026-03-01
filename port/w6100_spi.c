#include "w6100_spi.h"
#include "wizchip_conf.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/critical_section.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

static critical_section_t g_spi_crit_sec;

void w6100_spi_init(void)
{
    // Initialize critical section
    critical_section_init(&g_spi_crit_sec);

    // Initialize SPI
    spi_init(W6100_SPI_PORT, W6100_SPI_CLOCK_HZ);

    // Set SPI format: 8 bits, CPOL=0, CPHA=0
    spi_set_format(W6100_SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Initialize SPI pins
    gpio_set_function(W6100_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(W6100_SPI_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(W6100_SPI_MISO_PIN, GPIO_FUNC_SPI);

    // Initialize CS pin as GPIO output (active low)
    gpio_init(W6100_SPI_CS_PIN);
    gpio_set_dir(W6100_SPI_CS_PIN, GPIO_OUT);
    gpio_put(W6100_SPI_CS_PIN, 1); // Deselect

    // Initialize reset pin
    gpio_init(W6100_RST_PIN);
    gpio_set_dir(W6100_RST_PIN, GPIO_OUT);
    gpio_put(W6100_RST_PIN, 1); // Not in reset

    // Initialize interrupt pin as input (optional, for interrupt-driven operation)
    gpio_init(W6100_INT_PIN);
    gpio_set_dir(W6100_INT_PIN, GPIO_IN);

    // Perform hardware reset
    w6100_reset();

    // Register callback functions with ioLibrary
    reg_wizchip_cris_cbfunc(w6100_cris_enter, w6100_cris_exit);
    reg_wizchip_cs_cbfunc(w6100_cs_select, w6100_cs_deselect);
    reg_wizchip_spi_cbfunc(w6100_spi_read_byte, w6100_spi_write_byte,
                          w6100_spi_read_burst, w6100_spi_write_burst);
}

void w6100_reset(void)
{
    gpio_put(W6100_RST_PIN, 0);
    sleep_ms(1);
    gpio_put(W6100_RST_PIN, 1);
    sleep_ms(50); // Wait for W6100 to initialize
}

void w6100_cs_select(void)
{
    gpio_put(W6100_SPI_CS_PIN, 0);
}

void w6100_cs_deselect(void)
{
    gpio_put(W6100_SPI_CS_PIN, 1);
}

uint8_t w6100_spi_read_byte(void)
{
    uint8_t data;
    spi_read_blocking(W6100_SPI_PORT, 0x00, &data, 1);
    return data;
}

void w6100_spi_write_byte(uint8_t data)
{
    spi_write_blocking(W6100_SPI_PORT, &data, 1);
}

void w6100_spi_read_burst(uint8_t* buf, uint16_t len)
{
    spi_read_blocking(W6100_SPI_PORT, 0x00, buf, len);
}

void w6100_spi_write_burst(uint8_t* buf, uint16_t len)
{
    spi_write_blocking(W6100_SPI_PORT, buf, len);
}

void w6100_cris_enter(void)
{
    // Intentionally empty. The tension control timer callback does not access
    // the W6100 SPI bus, so there is no concurrent SPI access to protect.
    // Using a real critical section here disables interrupts during every SPI
    // transaction; the disconnect() spin-loop in the ioLibrary then runs so
    // fast that the 100Hz motor control timer never gets a chance to fire,
    // causing the motors to pause.
}

void w6100_cris_exit(void)
{
    // Intentionally empty — see w6100_cris_enter.
}
