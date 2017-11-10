/* msp430_spi.c
 * Library for performing SPI I/O on a wide range of MSP430 chips.
 *
 * Serial interfaces supported:
 * 1. USI - developed on MSP430G2231
 * 2. USCI_A - developed on MSP430G2553
 * 3. USCI_B - developed on MSP430G2553
 * 4. USCI_A F5xxx - developed on MSP430F5172, added F5529
 * 5. USCI_B F5xxx - developed on MSP430F5172, added F5529
 *
 * Copyright (c) 2013, Eric Brundick <spirilis@linux.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any purpose
 * with or without fee is hereby granted, provided that the above copyright notice
 * and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT,
 * OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
 * ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "stm32f4xx_hal.h"
#include "msp430_spi.h"

#define SPI_GET_FLAG(__FLAG__) (((SPI1->SR) & (__FLAG__)) == (__FLAG__)) 

void spi_init()
{
   SPI1->CR1 |= 0x40;
  //hspi1.Instance = SPI1;
  //hspi1.Init.Mode = SPI_MODE_MASTER;
  //hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  //hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  //hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  //hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  //hspi1.Init.NSS = SPI_NSS_SOFT;
  //hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  //hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  //hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  //hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  //hspi1.Init.CRCPolynomial = 10;
  //if (HAL_SPI_Init(&hspi1) != HAL_OK)
  //{
  //  _Error_Handler(__FILE__, __LINE__);
  //}
}

uint8_t spi_transfer(uint8_t inb) 
{
	uint8_t tmp = 0x00;
        uint8_t tmp1 = 0x00;
        tmp = SPI1->DR; //dummy read!
	SPI1->DR = inb; // Send byte to SPI (TXE cleared)
	tmp1 = (uint8_t)SPI1->SR;
        while (tmp1 & 0x80)
        {
         tmp1 = (uint8_t)SPI1->SR;
        } // Wait until the transmission is complete
	tmp = SPI1->DR;
        while (!SPI_GET_FLAG(SPI_SR_TXE)); // Wait until TX buffer is empty
        while (SPI_GET_FLAG(SPI_SR_BSY));
        osDelay(1);
	return tmp;
}

/* What wonderful toys TI gives us!  A 16-bit SPI function. */
uint16_t spi_transfer16(uint16_t inw)
{
	uint16_t tmp = 0x00;
	
	tmp = 0xFF & spi_transfer(inw & 0xFF);
	tmp = tmp | (uint16_t)((spi_transfer((inw >> 8) & 0xFF) << 8) & 0xFF00);
	
	return tmp;
}
