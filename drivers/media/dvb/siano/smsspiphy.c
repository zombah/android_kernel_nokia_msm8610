#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include "smscoreapi.h"

#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include "smsspiphy.h"

/************************************************************/
/*Platform specific defaults - can be changes by parameters */
/*or in compilation at this section                         */
/************************************************************/

/*Default SMS device type connected to SPI bus.*/
#define DEFAULT_SMS_DEVICE_TYPE		 SMS_RIO	

/*************************************/
/*End of platform specific parameters*/ 
/*************************************/


#define SPI_PACKET_SIZE 		256
#define SPI_PACKET_SIZE_THRESHOLD       48	

int host_spi_bus;
int host_spi_cs;
int spi_max_speed;
int spi_download_speed;
int spi_default_type =DEFAULT_SMS_DEVICE_TYPE;

struct sms_spi {
	struct spi_device	*spi_dev;
	char			*zero_txbuf;
	dma_addr_t 		zero_txbuf_phy_addr;
	int 			bus_speed;
	void (*interruptHandler) (void *);
	void			*intr_context;
};

/*!
invert the endianness of a single 32it integer

\param[in]		u: word to invert

\return		the inverted word
*/
static inline u32 invert_bo(u32 u)
{
	return ((u & 0xff) << 24) | ((u & 0xff00) << 8) | ((u & 0xff0000) >> 8)
		| ((u & 0xff000000) >> 24);
}

/*!
invert the endianness of a data buffer

\param[in]		buf: buffer to invert
\param[in]		len: buffer length

\return		the inverted word
*/

static int invert_endianness(char *buf, int len)
{
	int i;
	u32 *ptr = (u32 *) buf;

	len = (len + 3) / 4;
	for (i = 0; i < len; i++, ptr++)
	{
		*ptr = invert_bo(*ptr);
	}
	
	return 4 * ((len + 3) & (~3));
}


static irqreturn_t spibus_interrupt(int irq, void *context)
{
	struct sms_spi *sms_spi = (struct sms_spi *)context;
	sms_debug ("SPI interrupt recieved.");
	if (sms_spi->interruptHandler)
		sms_spi->interruptHandler(sms_spi->intr_context);
	return IRQ_HANDLED;

}

static struct regulator *vdd1_dtv_regulator = NULL;
static struct regulator *vdd2_dtv_regulator = NULL;

static int smsspiphy_config_vreg(void *context)
{
        int rc;
	struct smsspi_platform_data *pdata = (struct smsspi_platform_data *)context;

	vdd1_dtv_regulator = regulator_get(NULL,
				pdata->vdd1);

	rc = PTR_RET(vdd1_dtv_regulator);

	if (rc) {
		sms_err("Could not get regulator.\n");
		vdd1_dtv_regulator = NULL;
		goto vreg1_config_fail;
	}

	rc = regulator_set_voltage(
			vdd1_dtv_regulator,
			1200000,
			1200000);
	if (rc < 0) {
		sms_err("Could not set regulator1.\n");
		goto vreg1_config_fail;
	}

	vdd2_dtv_regulator = regulator_get(NULL,
				pdata->vdd2);

	rc = PTR_RET(vdd2_dtv_regulator);

	if (rc) {
		sms_err("Could not get regulator.\n");
		vdd2_dtv_regulator = NULL;
		goto vreg1_config_fail;
	}

	rc = regulator_set_voltage(
			vdd2_dtv_regulator,
			1800000,
			1800000);
	if (rc < 0) {
		sms_err("Could not set regulator2.\n");
		goto vreg2_config_fail;
	}

	return 0;
vreg2_config_fail:
	regulator_put(vdd2_dtv_regulator);
	vdd2_dtv_regulator = NULL;
vreg1_config_fail:
	regulator_put(vdd1_dtv_regulator);
	vdd1_dtv_regulator = NULL;
	return rc;
}

static int smsspiphy_control_vreg1(bool on)
{
	int rc;

	if (vdd1_dtv_regulator == NULL)
	    return -1;

	if (on)
		rc = regulator_enable(vdd1_dtv_regulator);
	else
	        rc = regulator_disable(vdd1_dtv_regulator);

	if (rc < 0){
		sms_err("Could not enable regulator.\n");
		goto vreg_control_fail;
	}
	if (on == 0)
	{
	       regulator_put(vdd1_dtv_regulator);
	       vdd1_dtv_regulator = NULL;
	}
	return 0;

vreg_control_fail:
	regulator_put(vdd1_dtv_regulator);
	vdd1_dtv_regulator = NULL;
	return rc;
}

static int smsspiphy_control_vreg2(bool on)
{
	int rc;

	if (vdd2_dtv_regulator == NULL)
	    return -1;

	if (on)
		rc = regulator_enable(vdd2_dtv_regulator);
	else
	        rc = regulator_disable(vdd2_dtv_regulator);

	if (rc < 0){
		sms_err("Could not enable regulator.\n");
		goto vreg_control_fail;
	}
	if (on == 0)
	{
	       regulator_put(vdd2_dtv_regulator);
	       vdd2_dtv_regulator = NULL;
	}
	return 0;

vreg_control_fail:
	regulator_put(vdd2_dtv_regulator);
	vdd2_dtv_regulator = NULL;
	return rc;
}

static void smsspiphy_poweron(void *context)
{
    struct smsspi_platform_data *pdata = (struct smsspi_platform_data *)context;
    smsspiphy_config_vreg(context);
    smsspiphy_control_vreg1(1);

    //note:1.2v should be at 0ms< t <=15ms earlier than 1.8v power on
    msleep(1);
    smsspiphy_control_vreg2(1);
    msleep(10);

    if (0 > gpio_request(pdata->rst_gpio, "SMSRESET"))
    {
        sms_err("Could not get GPIO for SMS RESET. Device will work but reset is not supported.\n");
        gpio_free(pdata->rst_gpio);

        goto SMSCHAR_POWERON_ERROR;;
    }
    else
    {

        if (0 > gpio_direction_output(pdata->rst_gpio, 0))
        {
            sms_err("Could not set GPIO direction for SMS RESET. Device will work but reset is not supported.\n");
            gpio_free(pdata->rst_gpio);

            goto SMSCHAR_POWERON_ERROR;
        }
        else
        {
            gpio_set_value(pdata->rst_gpio, 0);
            sms_info("rst_gpio power on");
        }
    }
    msleep(10);//10ms

    //do a HW reset
    //gpio_set_value(pdata->rst_gpio, 0);
    msleep(10);
    gpio_set_value(pdata->rst_gpio, 1);
    msleep(200);
return;

SMSCHAR_POWERON_ERROR:
    return;
}

static void smsspiphy_poweroff(void *context)
{
    struct smsspi_platform_data *pdata = (struct smsspi_platform_data *)context;
    gpio_set_value(pdata->rst_gpio, 0);
    //note:1.2v should be at 0ms< t <=15ms later than 1.8v power off
    msleep(1);
    smsspiphy_control_vreg2(0);
    msleep(1);
    smsspiphy_control_vreg1(0);
    msleep(1);

    gpio_free(pdata->rst_gpio);
}

void prepareForFWDnl(void *context)
{
	/*Reduce clock rate for FW download*/
	struct sms_spi *sms_spi = (struct sms_spi *)context;
	sms_spi->bus_speed = spi_download_speed;
	sms_spi->spi_dev->max_speed_hz = sms_spi->bus_speed;
	if (spi_setup(sms_spi->spi_dev))
	{
		sms_err("SMS device setup failed");
	}

	sms_debug ("Start FW download.");
	msleep(100);
	sms_debug ("done sleeping.");
}

void fwDnlComplete(void *context, int App)
{
	/*Set clock rate for working mode*/
	struct sms_spi *sms_spi = (struct sms_spi *)context;
	sms_spi->bus_speed = spi_max_speed;
	sms_spi->spi_dev->max_speed_hz = sms_spi->bus_speed;
	if (spi_setup(sms_spi->spi_dev))
	{
		sms_err("SMS device setup failed");
	}
	sms_debug ("FW download complete.");
	msleep(100);
}


void smsspibus_xfer(void *context, unsigned char *txbuf,
		    unsigned long txbuf_phy_addr, unsigned char *rxbuf,
		    unsigned long rxbuf_phy_addr, int len)
{
	struct sms_spi *sms_spi = (struct sms_spi *)context;
	struct spi_message msg;
	//unsigned char *temp = txbuf;
	//int i;
	struct spi_transfer xfer = {
		.tx_buf = txbuf,
		.rx_buf = rxbuf,
		.len = len,
		.tx_dma = txbuf_phy_addr,
		.rx_dma = rxbuf_phy_addr,
		.cs_change = 0,
	};
	if (txbuf)
	{
 		if (SPI_PACKET_SIZE_THRESHOLD <= len)
		{
			invert_endianness(txbuf, len);
		}
	}

	if (!txbuf)
	{
		xfer.tx_buf = sms_spi->zero_txbuf;
		xfer.tx_dma = sms_spi->zero_txbuf_phy_addr;
		
	}
	spi_message_init(&msg);
	msg.is_dma_mapped = 1;
	spi_message_add_tail(&xfer, &msg);
	spi_sync (sms_spi->spi_dev, &msg);
	invert_endianness(rxbuf, len);

}

int smsspiphy_is_device_exists(void *context)
{
	int i = 0;
	struct smsspi_platform_data *pdata = (struct smsspi_platform_data *)context;

	/* Check 3 times if the interrupt pin is 0. if it never goes down - 
	there is not SPI device,*/
	for (i = 0; i < 3; i++)
	{
		if (gpio_get_value(pdata->irq_gpio) == 0)
		{
			return 1;
		}
		msleep(1);
	}
	return 0;
}


int smsspiphy_init(void *context)
{
	struct sms_spi *sms_spi = (struct sms_spi *)context;
	int ret;
	struct spi_device *sms_device;

	struct spi_master *master = spi_busnum_to_master(host_spi_bus);
	struct spi_board_info sms_chip = {
		.modalias = "SmsSPI",
		.platform_data 	= NULL,
		.controller_data = NULL,
		.irq		= 0, 
		.max_speed_hz	= spi_download_speed,
		.bus_num	= host_spi_bus,
		.chip_select 	= host_spi_cs,
		.mode		= SPI_MODE_0,
	};
	if (!master)
	{
		sms_err("Could not find SPI master device.");
		ret = -ENODEV;
		goto no_spi_master;
	}
	

	sms_device = spi_new_device(master, &sms_chip);	
	if (!sms_device)
	{
		sms_err("Failed on allocating new SPI device for SMS");
		ret = -ENODEV;
		goto no_spi_master;
	}

	sms_device->bits_per_word = 32;
	if (spi_setup(sms_device))
	{
		sms_err("SMS device setup failed");
		ret = -ENODEV;
		goto spi_setup_failed;
	}


	sms_spi->spi_dev = sms_device;
	sms_spi->bus_speed = spi_download_speed;
	sms_debug("after init sms_spi=0x%x, spi_dev = 0x%x", (int)sms_spi, (int)sms_spi->spi_dev);

	return 0;

spi_setup_failed:
	spi_unregister_device(sms_device);
no_spi_master:
	return ret;
}

void smsspiphy_deinit(void *context)
{
	struct sms_spi *sms_spi = (struct sms_spi *)context;
	sms_debug("smsspiphy_deinit\n");
	/*Release the SPI device*/
	spi_unregister_device(sms_spi->spi_dev);
	sms_spi->spi_dev = NULL;

}


void *smsspiphy_register(void *context, void (*smsspi_interruptHandler) (void *), 
		     void *intr_context)
{
	int ret;
	struct sms_spi *sms_spi;
	struct smsspi_platform_data *pdata = (struct smsspi_platform_data *)context;

	host_spi_bus = pdata->spi_bus_num;
	host_spi_cs = pdata->spi_cs_num;
	spi_max_speed = pdata->spi_max_speed;
	spi_download_speed = pdata->spi_download_speed;
	sms_spi = kzalloc(sizeof(struct sms_spi), GFP_KERNEL);
	if (!sms_spi)
	{
		sms_err("SMS device mempory allocating");
		goto memory_allocate_failed;

	}

	sms_spi->zero_txbuf =  dma_alloc_coherent(NULL, SPI_PACKET_SIZE,
			       &sms_spi->zero_txbuf_phy_addr,
			       GFP_KERNEL | GFP_DMA);
	if (!sms_spi->zero_txbuf) {
		sms_err ("dma_alloc_coherent(...) failed\n");
		goto dma_allocate_failed;
	}
	memset (sms_spi->zero_txbuf, 0, SPI_PACKET_SIZE);
	sms_spi->interruptHandler = smsspi_interruptHandler;
	sms_spi->intr_context = intr_context;

	smsspiphy_poweron(pdata);

	if (gpio_request(pdata->irq_gpio, "SMSSPI"))
	{
		sms_err("Could not get GPIO for SMS device intr.\n");
		goto request_gpio_failed;
	}
	gpio_direction_input(pdata->irq_gpio);
	gpio_export(pdata->irq_gpio, 1);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	irq_set_irq_type(gpio_to_irq(pdata->irq_gpio), IRQ_TYPE_EDGE_FALLING);
#else
	set_irq_type(gpio_to_irq(pdata->irq_gpio), IRQ_TYPE_EDGE_FALLING);
#endif
	ret = request_irq(gpio_to_irq(pdata->irq_gpio), spibus_interrupt, IRQF_TRIGGER_FALLING, "SMSSPI", sms_spi);
	if (ret) {
		sms_err("Could not get interrupt for SMS device. status =%d\n", ret);
		goto request_irq_failed;
	}
	return sms_spi;

request_irq_failed:
	gpio_free(pdata->irq_gpio);
request_gpio_failed:
	dma_free_coherent(NULL, SPI_PACKET_SIZE, sms_spi->zero_txbuf, sms_spi->zero_txbuf_phy_addr);
dma_allocate_failed:
	kfree(sms_spi);
memory_allocate_failed:
	return NULL;
}

void smsspiphy_unregister(void *context, void *platform_data)
{
	struct sms_spi *sms_spi = (struct sms_spi *)context;
	struct smsspi_platform_data *pdata = (struct smsspi_platform_data *)platform_data;
	/*Release the IRQ line*/
	free_irq(gpio_to_irq(pdata->irq_gpio), sms_spi);
	/*Release the GPIO lines*/
	gpio_free(pdata->irq_gpio);
	/*Release the DMA buffer*/
	dma_free_coherent(NULL, SPI_PACKET_SIZE, sms_spi->zero_txbuf, sms_spi->zero_txbuf_phy_addr);
	/*Release memory*/
	kfree(sms_spi);

	smsspiphy_poweroff(pdata);
}

void smschipreset(void *context)
{
	msleep(100);
}


