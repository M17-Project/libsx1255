# libsx1255

A C library for controlling the SX1255 radio transceiver chip via SPI interface with GPIO reset functionality.

## Building

```bash
# Build static and shared libraries
make

# Build example program
make example

# Clean build artifacts
make clean
```

## Usage

```c
#include "sx1255.h"

// Initialize library
sx1255_init("/dev/spidev0.0", "/dev/gpiochip0", 22);

// Reset device
sx1255_reset();

// Configure frequencies
sx1255_set_rx_freq(433475000);  // 433.475 MHz
sx1255_set_tx_freq(438000000);  // 438 MHz

// Set sample rate
sx1255_set_rate(SX1255_RATE_500K);

// Configure gains
sx1255_set_lna_gain(48);
sx1255_set_pga_gain(30);

// Enable paths
sx1255_enable_rx(true);
sx1255_enable_tx(true);

// Cleanup
sx1255_cleanup();
```

## API Functions

### Initialization
- `sx1255_init()` - Initialize SPI and GPIO
- `sx1255_cleanup()` - Release resources
- `sx1255_reset()` - Hardware reset

### Frequency Control
- `sx1255_set_rx_freq()` - Set RX frequency
- `sx1255_set_tx_freq()` - Set TX frequency

### Configuration
- `sx1255_set_rate()` - Set sample rate (125K/250K/500K)
- `sx1255_set_lna_gain()` - Set LNA gain (0-48 dB)
- `sx1255_set_pga_gain()` - Set PGA gain (0-30 dB)
- `sx1255_set_dac_gain()` - Set DAC gain (-9/-6/-3/0 dB)
- `sx1255_set_mixer_gain()` - Set mixer gain (-37.5 to -7.5 dB)

### PLL Control
- `sx1255_set_rx_pll_bw()` - Set RX PLL bandwidth
- `sx1255_set_tx_pll_bw()` - Set TX PLL bandwidth

### Path Control
- `sx1255_enable_rx()` - Enable/disable RX path
- `sx1255_enable_tx()` - Enable/disable TX path

### Low-level Access
- `sx1255_read_reg()` - Read register
- `sx1255_write_reg()` - Write register

### Status
- `sx1255_get_pll_status()` - Get PLL lock status
- `sx1255_get_chip_version()` - Get chip version

## Installation

```bash
make install    # Install to /usr/
make uninstall  # Remove from /usr/
```
