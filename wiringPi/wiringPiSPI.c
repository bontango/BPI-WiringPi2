/*
 * wiringPiSPI.c:
 *	DMA SPI access routines
 *	Copyright (c) 2022 BogDan Vatra <bogdan@kdab.com>,
 *    Sponsored by Glueck Elektronik <https://www.glueck-elektronik.de/>
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <asm/ioctl.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "wiringPi.h"

#include "wiringPiSPI.h"

#define BLOCK_SIZE (4 * 1024)

#define BIT(nr) ((uint32_t)(1) << (nr))

#define SUN8I_FIFO_DEPTH 64

#define SUN6I_GBL_CTL_REG 0x04
#define SUN6I_GBL_CTL_BUS_ENABLE BIT(0)
#define SUN6I_GBL_CTL_MASTER BIT(1)
#define SUN6I_GBL_CTL_TP BIT(7)
#define SUN6I_GBL_CTL_RST BIT(31)

#define SUN6I_TFR_CTL_REG 0x08
#define SUN6I_TFR_CTL_CPHA BIT(0)
#define SUN6I_TFR_CTL_CPOL BIT(1)
#define SUN6I_TFR_CTL_SPOL BIT(2)
#define SUN6I_TFR_CTL_CS_MANUAL BIT(6)
#define SUN6I_TFR_CTL_DHB BIT(8)
#define SUN6I_TFR_CTL_XCH BIT(31)

#define SUN6I_INT_CTL_REG 0x10

#define SUN6I_INT_STA_REG 0x14

#define SUN6I_FIFO_CTL_REG 0x18
#define SUN6I_FIFO_CTL_RF_DRQ_EN BIT(8)
#define SUN6I_FIFO_CTL_RF_RDY_TRIG_LEVEL_MASK 0xff
#define SUN6I_FIFO_CTL_RF_RDY_TRIG_LEVEL_BITS 0
#define SUN6I_FIFO_CTL_RF_RST BIT(15)
#define SUN6I_FIFO_CTL_TF_DRQ_EN BIT(24)
#define SUN6I_FIFO_CTL_TF_RST BIT(31)

#define SUN6I_FIFO_STA_REG 0x1c

#define SUN6I_CLK_CTL_REG 0x24
#define SUN6I_CLK_CTL_CDR2_MASK 0xff
#define SUN6I_CLK_CTL_CDR2(div) (((div)&SUN6I_CLK_CTL_CDR2_MASK) << 0)
#define SUN6I_CLK_CTL_CDR1_MASK 0xf
#define SUN6I_CLK_CTL_CDR1(div) (((div)&SUN6I_CLK_CTL_CDR1_MASK) << 8)
#define SUN6I_CLK_CTL_DRS BIT(12)

#define SUN6I_BURST_CNT_REG 0x30

#define SUN6I_XMIT_CNT_REG 0x34

#define SUN6I_BURST_CTL_CNT_REG 0x38

#define SUN6I_TXDATA_REG 0x200

#define SUN6I_RXDATA_REG 0x300

#define SUN6I_BUS_SOFT_RST_REG0 0x2C0
#define AHB_RESET_SPI_SHIFT 20
#define SPI_CLK_REG 0xa0

extern int mem_fd;
extern volatile uint32_t *gpio_map;
extern void sunxi_set_pin_mode(int pin, int mode);
#define CCM_AHB_GATING0 (0x60)
#define AHB_GATE_OFFSET_SPI 20

static const uint64_t spi_addresses[2] = {0x01c68000, 0x01c69000};

typedef struct {
  volatile uint8_t *base;
  volatile uint32_t *clk_ptr;
  uint32_t speed;
  uint32_t mode;
  uint8_t pins[4];
} SpiData;

static SpiData spis[2] = {
    {.base = NULL,
     .clk_ptr = NULL,
     .speed = 0x1001,
     .mode = 0,
     .pins = {64 /*SPI0-MOSI*/, 65 /*SPI0-MISO*/, 66 /*SPI0-CLK*/, 67 /*SPI0-CS*/}
    },
    {.base = NULL,
     .clk_ptr = NULL,
     .speed = 0x1001,
     .mode = 0,
     .pins = {15 /*SPI1-MOSI*/, 16 /*SPI1-MISO*/, 14 /*SPI1-CLK*/, 13 /*SPI1-CS*/}
    },
};

#define spi_write8(REG, VAL) *(spi->base + REG) = (uint8_t)VAL
#define spi_read8(REG) *(spi->base + REG)
#define spi_write32(REG, VAL)                                                  \
  *(volatile uint32_t *)(spi->base + REG) = (uint32_t)VAL
#define spi_read32(REG) *(volatile uint32_t *)(spi->base + REG)

#define spi_clrbits(REG, VAL)                                                  \
  (*((volatile uint32_t *)(spi->base + REG)) &= ~VAL)
#define spi_clrsetbits(REG, CLR, SET)                                          \
  (*((volatile uint32_t *)(spi->base + REG)) =                                 \
       (*((volatile uint32_t *)(spi->base + REG)) & ~CLR) | SET)
#define spi_setbits(REG, VAL) (*((volatile uint32_t *)(spi->base + REG)) |= VAL)

#define gpio_clrbits(REG, VAL) (*(gpio_map + (REG >> 2)) &= ~VAL)
#define gpio_clrsetbits(REG, CLR, SET)                                         \
  (*(gpio_map + (REG >> 2)) = (*(gpio_map + (REG >> 2)) & ~VAL) | SET)
#define gpio_setbits(REG, VAL) (*(gpio_map + (REG >> 2)) |= VAL)

#define min(X, Y) ((X) < (Y) ? (X) : (Y))

/*
 * wiringPiSPIDataRW:
 *	Write and Read a block of data over the SPI bus.
 *	Note the data ia being read into the transmit buffer, so will
 *	overwrite it!
 *	This is also a full-duplex operation.
 *********************************************************************************
 */
int wiringPiSPIDataRW(int channel, unsigned char *data, int len) {
  channel &= 1;
  SpiData *spi = &spis[channel];
  if (!spi->base)
    return -1;

  /* Reset FIFO */
  spi_write32(SUN6I_FIFO_CTL_REG, SUN6I_FIFO_CTL_RF_RST | SUN6I_FIFO_CTL_TF_RST);

  // Disbale interrupts
  spi_write32(SUN6I_INT_CTL_REG, 0);

  // Enable DMA
  spi_setbits(SUN6I_FIFO_CTL_REG, SUN6I_FIFO_CTL_RF_DRQ_EN | SUN6I_FIFO_CTL_TF_DRQ_EN);

  int reg = 0;
  switch (spi->mode) {
  case 1:
    reg |= SUN6I_TFR_CTL_CPHA;
    break;
  case 2:
    reg |= SUN6I_TFR_CTL_CPOL;
    break;
  case 3:
    reg |= (SUN6I_TFR_CTL_CPHA | SUN6I_TFR_CTL_CPOL);
    break;
  default:
    break;
  }
  reg &= ~SUN6I_TFR_CTL_DHB;
  /* We want to control the chip select manually */
  reg |= SUN6I_TFR_CTL_CS_MANUAL;
  spi_write32(SUN6I_TFR_CTL_REG, reg);

  // Setup the spi clk speed
  spi_write32(SUN6I_CLK_CTL_REG, spi->speed);

  /* Finally enable the bus - doing so before might raise SCK to HIGH */
  spi_setbits(SUN6I_GBL_CTL_REG, SUN6I_GBL_CTL_BUS_ENABLE | SUN6I_GBL_CTL_MASTER | SUN6I_GBL_CTL_TP | SUN6I_GBL_CTL_RST);

  while (spi_read32(SUN6I_GBL_CTL_REG) & SUN6I_GBL_CTL_RST) {
    // Wait for completion
  }

  /* Setup the transfer now... */
  const int count = len;
  while (len) {
    uint32_t cnt = SUN8I_FIFO_DEPTH - ((spi_read32(SUN6I_FIFO_STA_REG) >> 16) & 0xff);
    cnt = min(cnt, (uint32_t)len);

    spi_write32(SUN6I_BURST_CNT_REG, cnt);
    spi_write32(SUN6I_XMIT_CNT_REG, cnt);
    spi_write32(SUN6I_BURST_CTL_CNT_REG, cnt);

    for (uint8_t i = 0; i < cnt; i++)
        spi_write8(SUN6I_TXDATA_REG, data[i]);

    spi_setbits(SUN6I_TFR_CTL_REG, SUN6I_TFR_CTL_XCH);

    while ((spi_read32(SUN6I_FIFO_STA_REG) & 0xff) < cnt) {
        // wait until transfer is complete
    }

    for (uint8_t i = 0; i < cnt; i++)
        data[i] = spi_read8(SUN6I_RXDATA_REG);

    len -= cnt;
    data += cnt;
  }

  // disable the bus
  spi_clrbits(SUN6I_GBL_CTL_REG, SUN6I_GBL_CTL_BUS_ENABLE | SUN6I_GBL_CTL_MASTER);
  return count;
}

/*
 * wiringPiSPISetupMode:
 *	Open the SPI device, and set it up, with the mode, etc.
 *********************************************************************************
 */
int wiringPiSPISetupMode(int channel, int speed, int mode) {
  mode &= 3;    // Mode is 0, 1, 2 or 3
  channel &= 1; // Channel is 0 or 1
  SpiData *spi = &spis[channel];
  if (!spi->base) {

    if (mem_fd == -1)
      return -1;

    spi->base = mmap(NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
                     mem_fd, spi_addresses[channel]);
    if (!spi->base)
      return -1;

    for (int i = 0; i < 4; ++i)
      sunxi_set_pin_mode(spi->pins[i], SPI_PIN);

    /* Deassert SPI reset */
    gpio_setbits(SUN6I_BUS_SOFT_RST_REG0, (1 << (AHB_RESET_SPI_SHIFT + channel)));

    /* Open the SPI gate */
    gpio_setbits(CCM_AHB_GATING0, (1 << (AHB_GATE_OFFSET_SPI + channel)));

    spi->clk_ptr = gpio_map + channel + (SPI_CLK_REG >> 2);
    *spi->clk_ptr = ((uint32_t)1 << 24) /*PLL_PERIPH0*/ | ((uint32_t)1 << 31); // Clock is on.
  }

  /*
     * Setup clock divider.
     *
     * We have two choices there. Either we can use the clock
     * divide rate 1, which is calculated thanks to this formula:
     * SPI_CLK = MOD_CLK / (2 ^ cdr)
     * Or we can use CDR2, which is calculated with the formula:
     * SPI_CLK = MOD_CLK / (2 * (cdr + 1))
     * Wether we use the former or the latter is set through the
     * DRS bit.
     *
     * First try CDR2, and if we can't reach the expected
     * frequency, fall back to CDR1.
     */
  const uint32_t mclk_rate = 200000000; // ahb0 clock is set at 200Mhz
#define DIV_ROUND_UP(n, d) (((n) + (d)-1) / (d))
  uint32_t div_cdr1 = DIV_ROUND_UP(mclk_rate, speed);
  uint32_t div_cdr2 = DIV_ROUND_UP(div_cdr1, 2);

  if (div_cdr2 <= (SUN6I_CLK_CTL_CDR2_MASK + 1)) {
    spi->speed = SUN6I_CLK_CTL_CDR2(div_cdr2 - 1) | SUN6I_CLK_CTL_DRS;
  } else {
    const uint32_t div = min(SUN6I_CLK_CTL_CDR1_MASK, (uint32_t)(log2((div_cdr1)-1) + 1));
    spi->speed = SUN6I_CLK_CTL_CDR1(div);
  }
  spi->mode = (uint32_t)mode;
  return channel;
}

/*
 * wiringPiSPISetup:
 *	Open the SPI device, and set it up, etc. in the default MODE 0
 *********************************************************************************
 */
int wiringPiSPISetup(int channel, int speed) {
  return wiringPiSPISetupMode(channel, speed, 0);
}
