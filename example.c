#include "sx1255.h"
#include <stdio.h>
#include <unistd.h>

int main(void)
{
    // Initialize the SX1255 library
    if (sx1255_init("/dev/spidev0.0", "/dev/gpiochip0", 22) != 0)
    {
        printf("Failed to initialize SX1255\n");
        return -1;
    }

    // Reset the device
    printf("Resetting SX1255...\n");
    if (sx1255_reset() != 0)
    {
        printf("Reset failed\n");
        sx1255_cleanup();
        return -1;
    }

    // Check chip version
    uint8_t version = sx1255_get_chip_version();
    printf("SX1255 chip version: V%d%c\n", (version >> 4) & 0xF, 'A' + (version & 0xF) - 1);

    // Configure the device
    printf("Configuring SX1255...\n");
    
    // Set frequencies
    sx1255_set_rx_freq(433475000);  // 433.475 MHz
    sx1255_set_tx_freq(438000000);  // 438 MHz
    
    // Set sample rate
    if (sx1255_set_rate(SX1255_RATE_500K) == 0)
    {
        printf("Sample rate set to 500 kSa/s - OK\n");
    }
    else
    {
        printf("Sample rate setup error\n");
    }
    
    // Set gains
    sx1255_set_lna_gain(48);        // 48 dB LNA gain
    sx1255_set_pga_gain(30);        // 30 dB PGA gain
    sx1255_set_dac_gain(-3);        // -3 dB DAC gain
    sx1255_set_mixer_gain(-9.5);    // -9.5 dB mixer gain
    
    // Set PLL bandwidths
    sx1255_set_rx_pll_bw(75);       // 75 kHz RX PLL BW
    sx1255_set_tx_pll_bw(75);       // 75 kHz TX PLL BW
    
    // Enable RX and TX paths
    sx1255_enable_rx(true);
    sx1255_enable_tx(true);
    
    // Check PLL lock status
    bool tx_locked, rx_locked;
    sx1255_get_pll_status(&tx_locked, &rx_locked);
    printf("TX PLL: %s\n", tx_locked ? "locked" : "unlocked");
    printf("RX PLL: %s\n", rx_locked ? "locked" : "unlocked");

    printf("SX1255 configuration complete\n");

    // Cleanup
    sx1255_cleanup();
    return 0;
}