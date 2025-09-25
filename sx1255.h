#ifndef SX1255_H
#define SX1255_H

#include <stdint.h>
#include <stdbool.h>

// SX1255 sample rate options
typedef enum
{
    SX1255_RATE_125K,
    SX1255_RATE_250K,
    SX1255_RATE_500K
} sx1255_rate_t;

// Library initialization and cleanup
int sx1255_init(const char* spi_device, const char* gpio_chip_path, int reset_pin_offset);
void sx1255_cleanup(void);

// Reset functionality
int sx1255_reset(void);

// Low-level register access
void sx1255_write_reg(uint8_t addr, uint8_t val);
uint8_t sx1255_read_reg(uint8_t addr);

// Frequency control
void sx1255_set_rx_freq(uint32_t freq);
void sx1255_set_tx_freq(uint32_t freq);

// Sample rate configuration
int8_t sx1255_set_rate(sx1255_rate_t rate);

// Gain controls
void sx1255_set_lna_gain(uint8_t gain_db);
void sx1255_set_pga_gain(uint8_t gain_db);
void sx1255_set_dac_gain(int8_t gain_db);
void sx1255_set_mixer_gain(float gain_db);

// PLL bandwidth configuration
void sx1255_set_rx_pll_bw(uint16_t bw_khz);
void sx1255_set_tx_pll_bw(uint16_t bw_khz);

// Path enable/disable
void sx1255_enable_tx(bool enable);
void sx1255_enable_rx(bool enable);

// Status functions
void sx1255_get_pll_status(bool* tx_locked, bool* rx_locked);
uint8_t sx1255_get_chip_version(void);

#endif // SX1255_H