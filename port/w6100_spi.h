#ifndef W6100_SPI_H
#define W6100_SPI_H

#include <stdint.h>

// W6100-EVB-Pico2 SPI pin definitions
#define W6100_SPI_PORT      spi0
#define W6100_SPI_SCK_PIN   18
#define W6100_SPI_MOSI_PIN  19
#define W6100_SPI_MISO_PIN  16
#define W6100_SPI_CS_PIN    17
#define W6100_RST_PIN       20
#define W6100_INT_PIN       21

// SPI clock frequency (33MHz max for W6100)
#define W6100_SPI_CLOCK_HZ  33000000

// Initialize W6100 SPI interface
void w6100_spi_init(void);

// Hardware reset W6100
void w6100_reset(void);

// Chip select control
void w6100_cs_select(void);
void w6100_cs_deselect(void);

// SPI read/write functions (registered with ioLibrary)
uint8_t w6100_spi_read_byte(void);
void w6100_spi_write_byte(uint8_t data);
void w6100_spi_read_burst(uint8_t* buf, uint16_t len);
void w6100_spi_write_burst(uint8_t* buf, uint16_t len);

// Critical section for SPI transactions
void w6100_cris_enter(void);
void w6100_cris_exit(void);

#endif // W6100_SPI_H
