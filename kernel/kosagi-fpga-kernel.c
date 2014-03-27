#define DEBUG
#include <net/genetlink.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/firmware.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <linux/platform_data/dma-imx.h>

#define BITSTREAM_FILENAME "novena_fpga.bit"

#define IMX6_EIM_CS0_BASE_ADDR 0x08040000
#define IMX6_EIM_CS1_BASE_ADDR 0x0c040000
#define DATA_FIFO_ADDR (IMX6_EIM_CS1_BASE_ADDR + 0xf000)

#define FPGA_W_TEST0       0
#define FPGA_W_TEST1       1

#define FPGA_W_DDR3_P2_CMD  16
#define FPGA_W_DDR3_P2_LADR 17
#define FPGA_W_DDR3_P2_HADR 18
#define FPGA_W_DDR3_P2_WEN  19
#define FPGA_W_DDR3_P2_LDAT 20
#define FPGA_W_DDR3_P2_HDAT 21

#define FPGA_W_DDR3_P3_CMD  24
#define FPGA_W_DDR3_P3_LADR 25
#define FPGA_W_DDR3_P3_HADR 26
#define FPGA_W_DDR3_P3_REN  27

#define FPGA_W_ADC_CTL       128
#define FPGA_W_ADC_SAMPLEN_L 129
#define FPGA_W_ADC_SAMPLEN_H 130

#define FPGA_W_RBK_CTL       136
#define  RBK_CTL_CLEAR_ERROR 0x4
#define  RBK_CTL_INIT        0x2
#define  RBK_CTL_ENABLE      0x1
#define FPGA_W_RBK_PAGE_L    137
#define FPGA_W_RBK_PAGE_H    138
#define FPGA_W_BURSTLEN      139


#define FPGA_R_TEST0        2048
#define FPGA_R_TEST1        2049
#define FPGA_R_DDR3_CAL     2050

#define FPGA_R_DDR3_P2_STAT 2064
#define FPGA_R_DDR3_P3_STAT 2072
#define FPGA_R_DDR3_P3_LDAT 2073
#define FPGA_R_DDR3_P3_HDAT 2074

#define FPGA_R_ADC_STAT     2176
#define FPGA_R_RBK_STAT     2177
#define FPGA_R_MEAS_BURST   2178
          
#define FPGA_R_DDR3_V_MINOR 4094
#define FPGA_R_DDR3_V_MAJOR 4095


/* list of valid commands */
enum kosagi_fpga_commands {
	KOSAGI_CMD_UNSPEC,
	KOSAGI_CMD_SEND,
	KOSAGI_CMD_READ,
	__KOSAGI_CMD_MAX,
};
#define KOSAGI_CMD_MAX (__KOSAGI_CMD_MAX - 1)

/* list of valid command attributes */
enum kosagi_fpga_attributes {
	KOSAGI_ATTR_NONE,
	KOSAGI_ATTR_FPGA_DATA,
	KOSAGI_ATTR_MESSAGE,
	__KOSAGI_ATTR_MAX,
};
#define KOSAGI_ATTR_MAX (__KOSAGI_ATTR_MAX - 1)

struct kosagi_fpga {
	struct spi_device *spi;
	struct clk *lvds_clk, *lvds_switch_clk, *lvds_parent_clk, *eim_slow_clk;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_eim;
	struct pinctrl_state *pins_gpio;
	int power_gpio;

	__iomem void *byte_area;
	__iomem u16 *fpga_ctrl;
	unsigned int byte_size;
	u8 dma_buffer[4096];
};
struct kosagi_fpga *g_fpga;

/* mapping of attributes to type */
static struct nla_policy kosagi_fpga_genl_policy[KOSAGI_ATTR_MAX + 1] = {
	[KOSAGI_ATTR_FPGA_DATA] = { .type = NLA_BINARY, .len = 4096 },
	[KOSAGI_ATTR_MESSAGE]   = { .type = NLA_NUL_STRING },
};

/* actual family definition */
static struct genl_family kosagi_fpga_family = {
	/* genetlink should generate an id */
	.id		= GENL_ID_GENERATE,
	.hdrsize	= 0,

	/* The name of this family, used by userspace application */
	.name		= "kosagi-fpga",
	.version	= 1,
	.maxattr	= KOSAGI_ATTR_MAX,
};


/* receives a message, prints it and sends another message back */
static int kosagi_fpga_send(struct sk_buff *skb_2, struct genl_info *info)
{
#if 0
	struct nlattr *na;
	char *mydata;

	if (info == NULL)
		goto out;

	/*
	 * for each attribute there is an index in info->attrs which points
	 * to a nlattr structure in this structure the data is given
	 */
	na = info->attrs[KOSAGI_ATTR_MESSAGE];
	if (na) {
		mydata = (char *)nla_data(na);
		if (mydata == NULL)
			printk("error while receiving data\n");
		else
			printk("received: %s\n", mydata);
		}
	else
		printk("no info->attrs %i\n", KOSAGI_ATTR_MESSAGE);

	/* Send a DMA message */
	if (fpga->byte_area) {
		int i;
		for (i = 0; i < sizeof(fpga->dma_buffer); i++)
			fpga->dma_buffer[i] = i;
		memcpy(fpga->byte_area, fpga->dma_buffer, sizeof(fpga->dma_buffer));
	}
out:
#endif
	return 0;
}

static void kosagi_trigger_sample(struct kosagi_fpga *fpga)
{
	fpga->fpga_ctrl[FPGA_W_ADC_SAMPLEN_L] = 0x0;
	fpga->fpga_ctrl[FPGA_W_ADC_SAMPLEN_H] = 0x1; // capture 64k for now

	fpga->fpga_ctrl[FPGA_W_ADC_CTL] = 0x0;
	fpga->fpga_ctrl[FPGA_W_ADC_CTL] = 0x2;
	fpga->fpga_ctrl[FPGA_W_ADC_CTL] = 0x0;
	//udelay(1000);
	fpga->fpga_ctrl[FPGA_W_ADC_CTL] = 0x1; // start sampling run, fill buffer
}

static void kosagi_trigger_burst(struct kosagi_fpga *fpga)
{
	/* Set burst mode priority */
	fpga->fpga_ctrl[FPGA_W_DDR3_P3_CMD] |= 0x8000;

	pr_info("last run's burst length was: %d\n",
			fpga->fpga_ctrl[FPGA_R_MEAS_BURST]);

	fpga->fpga_ctrl[FPGA_W_RBK_PAGE_L] = 0x0;
	fpga->fpga_ctrl[FPGA_W_RBK_PAGE_H] = 0x0; // read back starting at page 0

	fpga->fpga_ctrl[FPGA_W_BURSTLEN] = 0x10; // 16-beat bursts

	fpga->fpga_ctrl[FPGA_W_RBK_CTL] = RBK_CTL_CLEAR_ERROR | RBK_CTL_INIT;
	fpga->fpga_ctrl[FPGA_W_RBK_CTL] = 0;
	fpga->fpga_ctrl[FPGA_W_RBK_CTL] = RBK_CTL_ENABLE; // enable readback machine
}

static int kosagi_fpga_read(struct sk_buff *skb_2, struct genl_info *info)
{
	struct sk_buff *buf;
	void *hdr = NULL;
	void *data;
	int ret = 0;
	struct kosagi_fpga *fpga = g_fpga;

	kosagi_trigger_sample(fpga);
	kosagi_trigger_burst(fpga);

	if (info == NULL) {
		pr_err("Unable to read: info was NULL\n");
		return -EINVAL;
	}

	buf = genlmsg_new(fpga->byte_size, GFP_KERNEL);
	if (!buf) {
		pr_err("Unable to create new nlmsg\n");
		return -ENOMEM;
	}

	hdr = genlmsg_put(buf, info->snd_portid, info->snd_seq,
			&kosagi_fpga_family, 0, KOSAGI_CMD_READ);
	if (!hdr) {
		pr_err("Unable to put info into genlmsg\n");
		ret = -EMSGSIZE;
		goto err;
	}

	data = nla_reserve_nohdr(buf, fpga->byte_size);
	memcpy(data, fpga->byte_area, fpga->byte_size);

	genlmsg_end(buf, hdr);

#ifdef DEBUG_PRINT_HEADER
	{
		int i;
		u8 *d;
		d = (u8 *)data;
		for (i = 0; i < 64; i += 16)
			printk("Scope %d: "
				"%02x %02x %02x %02x %02x %02x %02x %02x  "
				"%02x %02x %02x %02x %02x %02x %02x %02x\n",
				i,
				d[i+0], d[i+1], d[i+2], d[i+3],
				d[i+4], d[i+5], d[i+6], d[i+7],
				d[i+8], d[i+9], d[i+10], d[i+11],
				d[i+12], d[i+13], d[i+14], d[i+15]
			);
	}
#endif

	ret = genlmsg_unicast(genl_info_net(info), buf, info->snd_portid);
	return ret;

err:
	if (buf)
		nlmsg_free(buf);
	return ret;
}

/* map commands to their corresponding function call */
struct genl_ops kosagi_genl_ops[] = {
	{
		.cmd	= KOSAGI_CMD_SEND,
		.flags	= 0,
		.policy	= kosagi_fpga_genl_policy,
		.doit	= kosagi_fpga_send,
		.dumpit	= NULL,
	},
	{
		.cmd	= KOSAGI_CMD_READ,
		.flags	= 0,
		.policy	= kosagi_fpga_genl_policy,
		.doit	= kosagi_fpga_read,
		.dumpit	= NULL,
	},
};

static int load_firmware(struct spi_device *spi)
{
	const struct firmware *fw;
	int ret;
	int size;
	const u8 *data;
	ret = request_firmware(&fw, BITSTREAM_FILENAME, &spi->dev);
	if (ret) {
		dev_err(&spi->dev, "unable to request firmwrae: %d\n", ret);
		goto out;
	}

	size = fw->size;
	data = fw->data;
	while (size > 0 && !ret) {
		struct spi_transfer t = {
			.tx_buf         = data,
			.len            = size > 128 ? 128 : size,
		};
		struct spi_message      m;

		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		ret = spi_sync(spi, &m);

		data += 128;
		size -= 128;
	}

out:
	if (fw)
		release_firmware(fw);
	return ret;
}

static int kosagi_fpga_probe(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	struct kosagi_fpga *fpga;
	int ret;

	fpga = kzalloc(sizeof(*fpga), GFP_KERNEL);
	if (!fpga) {
		ret = -ENOMEM;
		goto fail;
	}

	ret = genl_register_family_with_ops(&kosagi_fpga_family,
					    kosagi_genl_ops);
	if (ret != 0)
		goto fail;

	/* Pin control comes first */
	fpga->pinctrl = devm_pinctrl_get(&spi->dev);
	fpga->pins_eim = pinctrl_lookup_state(fpga->pinctrl, "state-eim");
	fpga->pins_gpio = pinctrl_lookup_state(fpga->pinctrl, "state-gpio");
	ret = pinctrl_select_state(fpga->pinctrl, fpga->pins_eim);
	if (ret < 0) {
		dev_err(&spi->dev, "unable to select pinctrl: %d\n", ret);
		goto fail;
	}

	/* Set up clocks */
	fpga->lvds_clk = devm_clk_get(&spi->dev, "lvds");
	fpga->lvds_switch_clk = devm_clk_get(&spi->dev, "lvds-switch");
	fpga->lvds_parent_clk = devm_clk_get(&spi->dev, "lvds-parent");
	fpga->eim_slow_clk = devm_clk_get(&spi->dev, "eim-slow");
	if (!fpga->lvds_clk || !fpga->lvds_switch_clk
	|| !fpga->lvds_parent_clk || !fpga->eim_slow_clk) {
		ret = -EINVAL;
		dev_err(&spi->dev, "missing clocks\n");
		goto fail;
	}

	ret = clk_set_parent(fpga->lvds_clk, fpga->lvds_parent_clk);
	if (ret) {
		dev_err(&spi->dev, "unable to set parent clock\n");
		goto fail;
	}

	/* Power it up */
	fpga->power_gpio = of_get_named_gpio(np, "power-switch", 0);
	if (!gpio_is_valid(fpga->power_gpio)) {
		dev_err(&spi->dev, "unable to find power-switch gpio\n");
		goto fail;
	}

	ret = devm_gpio_request_one(&spi->dev, fpga->power_gpio,
			GPIOF_OUT_INIT_LOW, "fpga power");
	if (ret) {
		dev_err(&spi->dev, "unable to get fpga power switch: %d\n",
				ret);
		goto fail;
	}
	gpio_set_value(fpga->power_gpio, 1);

	/* Load firmware */
	ret = load_firmware(spi);
	if (ret)
		goto fail;

	/* Now that firmware is loaded, start the FPGA */
	ret = clk_prepare_enable(fpga->lvds_switch_clk);
	if (ret) {
		dev_err(&spi->dev, "unable to enable lvds clock\n");
		goto fail;
	}

	/* Map areas */
	fpga->fpga_ctrl = devm_ioremap(&spi->dev, IMX6_EIM_CS0_BASE_ADDR, 8192);
	dev_dbg(&spi->dev, "remapped data 0x%08x to 0x%p\n",
			IMX6_EIM_CS0_BASE_ADDR, fpga->fpga_ctrl);

	fpga->byte_area = ioremap_wc(DATA_FIFO_ADDR, 4096);
	dev_dbg(&spi->dev, "remapped data 0x%08x to 0x%p\n",
			DATA_FIFO_ADDR, fpga->byte_area);
	fpga->byte_size = 4096;

	dev_info(&spi->dev, "FPGA version %d.%d\n",
			fpga->fpga_ctrl[FPGA_R_DDR3_V_MAJOR],
			fpga->fpga_ctrl[FPGA_R_DDR3_V_MINOR]);

	spi_set_drvdata(spi, fpga);
	g_fpga = fpga;

	return 0;
	
fail:
	if (fpga)
		kfree(fpga);
	return ret;
}

static int kosagi_fpga_remove(struct spi_device *spi)
{
	int ret;
	struct kosagi_fpga *fpga;

	fpga = spi_get_drvdata(spi);

	if (fpga->byte_area) {
		dev_dbg(&spi->dev, "unmapped 0x%p from 0x%08x\n",
				fpga->byte_area, DATA_FIFO_ADDR);
		iounmap(fpga->byte_area);
	}

	ret = genl_unregister_family(&kosagi_fpga_family);
	if (ret !=0) {
		dev_err(&spi->dev, "error while unregistering family: %d\n",
				ret);
	}

	kfree(fpga);
	return 0;
}

static const struct of_device_id kosagi_fpga_dt_ids[] = {
	{ .compatible = "kosagi,novena-fpga" },
	{},
};

static struct spi_driver kosagi_fpga_driver = {
	.driver = {
		.name =		"kosagi-fpga",
		.owner =	THIS_MODULE,
		.of_match_table	= of_match_ptr(kosagi_fpga_dt_ids),
	},
	.probe =        kosagi_fpga_probe,
	.remove =       kosagi_fpga_remove,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};


static int __init kosagi_fpga_init(void)
{
	int ret;
	ret = spi_register_driver(&kosagi_fpga_driver);
	if (ret < 0) {
		pr_err("unable to load kosagi fpga driver: %d\n", ret);
	}
	return ret;
}

static void __exit kosagi_fpga_exit(void)
{
	spi_unregister_driver(&kosagi_fpga_driver);
}

module_init(kosagi_fpga_init);
module_exit(kosagi_fpga_exit);
MODULE_LICENSE("GPL");
