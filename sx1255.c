#include "sx1255.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <math.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>

// Configuration constants
#define CLK_FREQ (32.0e6f)

// SPI configuration
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint8_t mode = SPI_MODE_0;

// Device paths and configuration
static const char *spi_device = NULL;
static const char *gpio_chip_path = NULL;
static int rst_pin_offset = 22;

// Global libgpiod handles
static struct gpiod_chip *chip_v2 = NULL;
static struct gpiod_line_request *rst_line_request = NULL;

// Internal SPI initialization
static int spi_init(const char *dev)
{
    int fd;
    int ret;

    fd = open(dev, O_RDWR);
    if (fd < 0)
    {
        perror("Can't open SPI device");
        return fd;
    }

    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1)
    {
        perror("Can't set SPI write mode");
        close(fd);
        return -1;
    }
    ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
    if (ret == -1)
    {
        perror("Can't set SPI read mode");
        close(fd);
        return -1;
    }

    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1)
    {
        perror("Can't set SPI write bits per word");
        close(fd);
        return -1;
    }
    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1)
    {
        perror("Can't set SPI read bits per word");
        close(fd);
        return -1;
    }

    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1)
    {
        perror("Can't set SPI write max speed");
        close(fd);
        return -1;
    }
    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1)
    {
        perror("Can't set SPI read max speed");
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

void sx1255_write_reg(uint8_t addr, uint8_t val)
{
    int fd = open(spi_device, O_RDWR);
    if (fd < 0)
    {
        perror("sx1255_write_reg: open");
        return;
    }
    uint8_t tx[2] = {addr | (1 << 7), val};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = 0,
        .len = 2,
    };
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    close(fd);
    usleep(10000U);
}

uint8_t sx1255_read_reg(uint8_t addr)
{
    int fd = open(spi_device, O_RDWR);
    if (fd < 0)
    {
        perror("sx1255_read_reg: open");
        return 0;
    }
    uint8_t tx[2] = {addr & ~(1 << 7), 0};
    uint8_t rx[2] = {0, 0};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
    };
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    close(fd);
    usleep(10000U);
    return rx[1];
}

void sx1255_set_rx_freq(uint32_t freq)
{
    uint32_t val = lround((float)freq * 1048576.0f / CLK_FREQ);
    sx1255_write_reg(0x01, (val >> 16) & 0xFF);
    sx1255_write_reg(0x02, (val >> 8) & 0xFF);
    sx1255_write_reg(0x03, val & 0xFF);
}

void sx1255_set_tx_freq(uint32_t freq)
{
    uint32_t val = lround((float)freq * 1048576.0f / CLK_FREQ);
    sx1255_write_reg(0x04, (val >> 16) & 0xFF);
    sx1255_write_reg(0x05, (val >> 8) & 0xFF);
    sx1255_write_reg(0x06, val & 0xFF);
}

int8_t sx1255_set_rate(sx1255_rate_t rate)
{
    uint8_t n, div;

    switch (rate)
    {
    case SX1255_RATE_125K:
        n = 5;
        div = 2;
        break;

    case SX1255_RATE_250K:
        n = 4;
        div = 1;
        break;

    case SX1255_RATE_500K:
        n = 3;
        div = 0;
        break;

    default: // 125k
        n = 5;
        div = 2;
        break;
    }

    // interpolation/decimation factor = mant * 3^m * 2^n
    sx1255_write_reg(0x13, (0 << 7) | (0 << 6) | (n << 3) | (1 << 2));

    // mode B2, XTAL/CLK_OUT division factor=2^div
    sx1255_write_reg(0x12, (2 << 4) | div);

    // return i2s status flag
    return (sx1255_read_reg(0x13) & (1 << 1)) >> 1;
}

// Internal GPIO initialization
static int gpio_init(void)
{
    // Open the GPIO chip
    chip_v2 = gpiod_chip_open(gpio_chip_path);
    if (!chip_v2)
    {
        fprintf(stderr, "Error opening GPIO chip '%s': %s\n", gpio_chip_path, strerror(errno));
        return -1;
    }

    // Create line settings object to configure individual lines
    struct gpiod_line_settings *settings = gpiod_line_settings_new();
    if (!settings)
    {
        perror("Error creating line settings");
        gpiod_chip_close(chip_v2);
        return -1;
    }
    // Configure the settings for our output pin
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(settings, 0); // Initial value is LOW

    // Create a line config object to hold the settings for lines
    struct gpiod_line_config *line_cfg = gpiod_line_config_new();
    if (!line_cfg)
    {
        perror("Error creating line config");
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip_v2);
        return -1;
    }
    // Add the settings for our specific line (by offset) to the config
    unsigned int offsets[] = {rst_pin_offset};
    if (gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings) != 0)
    {
        fprintf(stderr, "Error adding line settings to config for pin %d: %s\n",
                rst_pin_offset, strerror(errno));
        gpiod_line_config_free(line_cfg);
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip_v2);
        return -1;
    }

    // Create a request config object for the overall request
    struct gpiod_request_config *req_cfg = gpiod_request_config_new();
    if (!req_cfg)
    {
        perror("Error creating request config");
        gpiod_line_config_free(line_cfg);
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip_v2);
        return -1;
    }
    gpiod_request_config_set_consumer(req_cfg, "sx1255-reset");

    // Request the lines using the 3-argument function signature
    rst_line_request = gpiod_chip_request_lines(chip_v2, req_cfg, line_cfg);

    // Free the config objects as they are no longer needed
    gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(settings);

    if (!rst_line_request)
    {
        fprintf(stderr, "Error requesting GPIO lines for pin %d: %s\n",
                rst_pin_offset, strerror(errno));
        gpiod_chip_close(chip_v2);
        return -1;
    }
    return 0;
}

// Internal reset control
static int rst(uint8_t state)
{
    // Set value using gpiod V2 API
    enum gpiod_line_value values[] = {state ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE};
    if (gpiod_line_request_set_values(rst_line_request, values) < 0)
    {
        fprintf(stderr, "Error setting GPIO line value for pin %d to state %d: %s\n",
                rst_pin_offset, state, strerror(errno));
        return 1;
    }
    return 0;
}

void sx1255_cleanup(void)
{
    if (rst_line_request)
    {
        gpiod_line_request_release(rst_line_request);
        rst_line_request = NULL;
    }
    if (chip_v2)
    {
        gpiod_chip_close(chip_v2);
        chip_v2 = NULL;
    }
}

// Public API implementation

int sx1255_init(const char* spi_dev, const char* gpio_chip, int reset_pin)
{
    spi_device = spi_dev;
    gpio_chip_path = gpio_chip;
    rst_pin_offset = reset_pin;

    if (spi_init(spi_device) != 0)
    {
        return -1;
    }

    if (gpio_init() != 0)
    {
        fprintf(stderr, "Can not initialize RST pin\n");
        return -1;
    }

    sx1255_write_reg(0x0D, (0x01 << 5) | (0x05 << 2) | 0x03);
    sx1255_write_reg(0x0B, 5);

    return 0;
}

int sx1255_reset(void)
{
    usleep(100000U);
    if (rst(1) != 0)
        return -1;
    usleep(100000U);
    if (rst(0) != 0)
        return -1;
    usleep(250000U);
    return 0;
}

uint8_t sx1255_get_chip_version(void)
{
    return sx1255_read_reg(0x07);
}

void sx1255_set_lna_gain(uint8_t gain_db)
{
    uint8_t lna_gain;
    uint8_t val;

    if (gain_db > 45)
    {
        lna_gain = 1;
    }
    else if (gain_db > 39)
    {
        lna_gain = 2;
    }
    else if (gain_db > 30)
    {
        lna_gain = 3;
    }
    else if (gain_db > 18)
    {
        lna_gain = 4;
    }
    else if (gain_db > 6)
    {
        lna_gain = 5;
    }
    else
    {
        lna_gain = 6;
    }

    val = sx1255_read_reg(0x0C);
    val &= (uint8_t)~(0xE0);
    val |= lna_gain << 5;
    sx1255_write_reg(0x0C, val);
}

void sx1255_set_pga_gain(uint8_t gain_db)
{
    uint8_t pga_gain;
    uint8_t val;

    if (gain_db <= 30)
    {
        pga_gain = gain_db / 2;
    }
    else
    {
        pga_gain = 15;
    }

    val = sx1255_read_reg(0x0C);
    val &= (uint8_t)~(0x1E);
    val |= pga_gain << 1;
    sx1255_write_reg(0x0C, val);
}

void sx1255_set_dac_gain(int8_t gain_db)
{
    uint8_t dac_gain;
    uint8_t val;

    if (gain_db == 0)
    {
        dac_gain = 3;
    }
    else if (gain_db == -3)
    {
        dac_gain = 2;
    }
    else if (gain_db == -6)
    {
        dac_gain = 1;
    }
    else if (gain_db == -9)
    {
        dac_gain = 0;
    }
    else
    {
        dac_gain = 2; // default -3 dB
    }

    val = sx1255_read_reg(0x08);
    val &= (uint8_t)0x0F;
    val |= (uint8_t)dac_gain << 4;
    sx1255_write_reg(0x08, val);
}

void sx1255_set_mixer_gain(float gain_db)
{
    uint8_t mix_gain;
    uint8_t val;

    if (gain_db >= -37.5 && gain_db <= -7.5)
    {
        float temp = gain_db + 37.5f;
        mix_gain = (uint8_t)roundf(temp / 2.0f);
    }
    else
    {
        mix_gain = 0x0E; // default -9.5 dB
    }

    val = sx1255_read_reg(0x08);
    val &= (uint8_t)0xF0;
    val |= (uint8_t)mix_gain;
    sx1255_write_reg(0x08, val);
}

void sx1255_set_rx_pll_bw(uint16_t bw_khz)
{
    uint8_t pll_bw;
    uint8_t val;

    if (bw_khz >= 75 && bw_khz <= 300)
    {
        pll_bw = (bw_khz - 75) / 75;
    }
    else
    {
        pll_bw = 0; // default 75 kHz
    }

    val = sx1255_read_reg(0x0E);
    val &= (uint8_t)~(0x06);
    val |= (uint8_t)pll_bw << 1;
    sx1255_write_reg(0x0E, val);
}

void sx1255_set_tx_pll_bw(uint16_t bw_khz)
{
    uint8_t pll_bw;
    uint8_t val;

    if (bw_khz >= 75 && bw_khz <= 300)
    {
        pll_bw = (bw_khz - 75) / 75;
    }
    else
    {
        pll_bw = 0; // default 75 kHz
    }

    val = sx1255_read_reg(0x0A);
    val &= (uint8_t)~(0x60);
    val |= (uint8_t)pll_bw << 5;
    sx1255_write_reg(0x0A, val);
}

void sx1255_enable_tx(bool enable)
{
    uint8_t val = sx1255_read_reg(0x00);
    
    if (enable)
    {
        val |= (uint8_t)(1 << 2) | (1 << 3);
    }
    else
    {
        val &= (uint8_t)~((1 << 2) | (1 << 3));
    }
    
    sx1255_write_reg(0x00, val);
}

void sx1255_enable_rx(bool enable)
{
    uint8_t val = sx1255_read_reg(0x00);
    
    if (enable)
    {
        val |= (uint8_t)(1 << 1);
    }
    else
    {
        val &= (uint8_t)~(1 << 1);
    }
    
    sx1255_write_reg(0x00, val);
}

void sx1255_get_pll_status(bool* tx_locked, bool* rx_locked)
{
    uint8_t val = sx1255_read_reg(0x11);
    if (tx_locked) *tx_locked = (val & (1 << 0)) != 0;
    if (rx_locked) *rx_locked = (val & (1 << 1)) != 0;
}