/****************************************************************

Siano Mobile Silicon, Inc.
MDTV receiver kernel modules.
Copyright (C) 2006-2008, Uri Shkolnik

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

 This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

****************************************************************/

#ifndef __SMS_SPI_PHY_H__
#define __SMS_SPI_PHY_H__

struct _spi_device_st;

struct smsspi_platform_data {
	int rst_gpio;
	int pow1_gpio;
	int pow2_gpio;
	int irq_gpio;
	int spi_bus_num;
	int spi_cs_num;
	int spi_max_speed;
	int spi_download_speed;
	char const *vdd1;
	char const *vdd2;
};

void smsspibus_xfer(void *context, unsigned char *txbuf,
		    unsigned long txbuf_phy_addr, unsigned char *rxbuf,
		    unsigned long rxbuf_phy_addr, int len);
void *smsspiphy_register(void *context, void (*smsspi_interruptHandler) (void *),
		     void *intr_context);
void smsspiphy_unregister(void *context, void *platform_data);
int smsspiphy_init(void *context);
void smsspiphy_deinit(void *context);
int smsspiphy_is_device_exists(void *context);
void smschipreset(void *context);
int smsspiphy_set_chip_reset(void *context, int value);
void WriteFWtoStellar(void *pSpiPhy, unsigned char *pFW, unsigned long Len);
void prepareForFWDnl(void *pSpiPhy);
void fwDnlComplete(void *context, int App);

#endif /* __SMS_SPI_PHY_H__ */
