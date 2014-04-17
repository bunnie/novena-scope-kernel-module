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
#include <linux/regmap.h>

#include <linux/platform_data/dma-imx.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/mfd/syscon.h>

#include "fpga.h"

#define BITSTREAM_FILENAME "novena_fpga.bit"

/* list of valid commands */
enum kosagi_fpga_commands {
	KOSAGI_CMD_UNSPEC,
	KOSAGI_CMD_SEND,
	KOSAGI_CMD_READ,
	KOSAGI_CMD_POWER_OFF,
	KOSAGI_CMD_POWER_ON,
	KOSAGI_CMD_FPGA_ASSERT_RESET,
	KOSAGI_CMD_FPGA_DEASSERT_RESET,
	KOSAGI_CMD_TRIGGER_SAMPLE,
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

/* Here because I can't figure out how to attach context to network calls */
static struct kosagi_fpga *g_fpga;

static int load_firmware(struct spi_device *spi);
static int kosagi_fpga_trigger_sample(struct sk_buff *skb_2,
				      struct genl_info *info);

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

static void kosagi_initiate_transfer(struct kosagi_fpga *fpga)
{
	pr_err("Initiating transfer\n");
	/* Set burst mode priority */
	fpga->fpga_ctrl[FPGA_W_DDR3_P3_CMD] |= 0x8000;

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

	kosagi_fpga_trigger_sample(skb_2, info);
	kosagi_initiate_transfer(fpga);

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

static int kosagi_fpga_power_off(struct sk_buff *skb_2, struct genl_info *info)
{
	struct kosagi_fpga *fpga = g_fpga;
	gpio_set_value(fpga->power_gpio, 0);
	return 0;
}

static int kosagi_fpga_power_on(struct sk_buff *skb_2, struct genl_info *info)
{
	struct kosagi_fpga *fpga = g_fpga;
	gpio_set_value(fpga->power_gpio, 1);
	return 0;
}

static int kosagi_fpga_assert_reset(struct sk_buff *skb_2,
					  struct genl_info *info)
{
	struct kosagi_fpga *fpga = g_fpga;
	gpio_set_value(fpga->reset_gpio, 0);
	return 0;
}

static int kosagi_fpga_deassert_reset(struct sk_buff *skb_2,
					    struct genl_info *info)
{
	struct kosagi_fpga *fpga = g_fpga;
	int ret;

	gpio_set_value(fpga->reset_gpio, 1);
	ret = load_firmware(fpga->spi);
	if (ret)
		return ret;
	return 0;
}

static int kosagi_fpga_trigger_sample(struct sk_buff *skb_2,
				       struct genl_info *info)
{
	struct kosagi_fpga *fpga = g_fpga;
	fpga->fpga_ctrl[FPGA_W_ADC_SAMPLEN_L] = 0x0;
	fpga->fpga_ctrl[FPGA_W_ADC_SAMPLEN_H] = 0x1; // Capture 64k of data

	fpga->fpga_ctrl[FPGA_W_ADC_CTL] = 0x0;
	fpga->fpga_ctrl[FPGA_W_ADC_CTL] = 0x2;
	fpga->fpga_ctrl[FPGA_W_ADC_CTL] = 0x0;
	udelay(1000);
	fpga->fpga_ctrl[FPGA_W_ADC_CTL] = 0x1; // start sampling run, fill buffer
	udelay(1000);
	return 0;
}

/* map commands to their corresponding function call */
struct genl_ops kosagi_genl_ops[] = {
	{
		.cmd	= KOSAGI_CMD_SEND,
		.flags	= 0,
		.policy	= kosagi_fpga_genl_policy,
		.doit	= kosagi_fpga_send,
	},
	{
		.cmd	= KOSAGI_CMD_READ,
		.flags	= 0,
		.policy	= kosagi_fpga_genl_policy,
		.doit	= kosagi_fpga_read,
	},
	{
		.cmd	= KOSAGI_CMD_POWER_OFF,
		.flags	= 0,
		.policy	= kosagi_fpga_genl_policy,
		.doit	= kosagi_fpga_power_off,
	},
	{
		.cmd	= KOSAGI_CMD_POWER_ON,
		.flags	= 0,
		.policy	= kosagi_fpga_genl_policy,
		.doit	= kosagi_fpga_power_on,
	},
	{
		.cmd	= KOSAGI_CMD_FPGA_ASSERT_RESET,
		.flags	= 0,
		.policy	= kosagi_fpga_genl_policy,
		.doit	= kosagi_fpga_assert_reset,
	},
	{
		.cmd	= KOSAGI_CMD_FPGA_DEASSERT_RESET,
		.flags	= 0,
		.policy	= kosagi_fpga_genl_policy,
		.doit	= kosagi_fpga_deassert_reset,
	},
	{
		.cmd	= KOSAGI_CMD_TRIGGER_SAMPLE,
		.flags	= 0,
		.policy	= kosagi_fpga_genl_policy,
		.doit	= kosagi_fpga_trigger_sample,
	},
};

static int load_firmware(struct spi_device *spi)
{
	const struct firmware *fw;
	int ret;
	int size;
	const u8 *data;
	int chunk_size = 65536;

	ret = request_firmware(&fw, BITSTREAM_FILENAME, &spi->dev);
	if (ret) {
		dev_err(&spi->dev, "unable to request firmware: %d\n", ret);
		goto out;
	}

	size = fw->size;
	data = fw->data;
	dev_dbg(&spi->dev, "loading %d bytes of firmware (in %d-byte chunks)\n",
			fw->size, chunk_size);
	while (size > 0 && !ret) {
		struct spi_transfer t = {
			.tx_buf         = data,
			.len            = size > chunk_size ? chunk_size : size,
		};
		struct spi_message      m;

		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		ret = spi_sync(spi, &m);

		data += chunk_size;
		size -= chunk_size;
	}

out:
	if (fw)
		release_firmware(fw);
	return ret;
}

static int fpga_timing_setup(struct spi_device *spi, struct kosagi_fpga *fpga)
{
	struct device_node *np = spi->dev.of_node;
	u32 values[6];
	int ret;

	ret = of_property_read_u32_array(np, "kosagi,weim-cs0-timing",
			values, 6);
	if (!ret) {
		int i;
		dev_dbg(&spi->dev, "setting up cs0 timing\n");
		for (i = 0; i < 6; i++)
			writel(values[i], fpga->eim_area + (i * 4)
				+ IMX6_EIM_CS0_BASE);
	}

	ret = of_property_read_u32_array(np, "kosagi,weim-cs1-timing",
			values, 6);
	if (!ret) {
		int i;
		dev_dbg(&spi->dev, "setting up cs1 timing\n");
		for (i = 0; i < 6; i++)
			writel(values[i], fpga->eim_area + (i * 4)
				+ IMX6_EIM_CS1_BASE);
	}

	ret = of_property_read_u32_array(np, "kosagi,weim-cs2-timing",
			values, 6);
	if (!ret) {
		int i;
		dev_dbg(&spi->dev, "setting up cs2 timing\n");
		for (i = 0; i < 6; i++)
			writel(values[i], fpga->eim_area + (i * 4)
				+ IMX6_EIM_CS2_BASE);
	}

	ret = of_property_read_u32_array(np, "kosagi,weim-cs3-timing",
			values, 6);
	if (!ret) {
		int i;
		dev_dbg(&spi->dev, "setting up cs3 timing\n");
		for (i = 0; i < 6; i++)
			writel(values[i], fpga->eim_area + (i * 4)
				+ IMX6_EIM_CS3_BASE);
	}

	/*
	 * EIM_WCR
	 * BCM = 1   free-run BCLK
	 * GBCD = 0  divide the burst clock by 1
	 * add timeout watchdog after 1024 bclk cycles
	 */
	writel(0x701, fpga->eim_area + IMX6_EIM_WCR);

	/*
	 * EIM_WIAR 
	 * ACLK_EN = 1
	 */
	writel(0x10, fpga->eim_area + IMX6_EIM_WIAR);

#define CS0_OFFSET 0
#define CS1_OFFSET 3
#define CS2_OFFSET 6
#define CS3_OFFSET 9
#define CSn_128M ((1 << 0) | (2 << 1))
#define CSn_64M ((1 << 0) | (1 << 1))
#define CSn_32M ((1 << 0) | (0 << 1))
	/* Map CS0 and CS1 (and CS2, due to chip bug) */
	/* CS0 gets 64MB, CS1 gets 32MB, and CS2 gets 32MB */
	regmap_update_bits(fpga->iomuxc_gpr, IOMUXC_GPR1, 0x7ff,
		  (CSn_64M << CS0_OFFSET)
		| (CSn_32M << CS1_OFFSET)
		| (CSn_32M << CS2_OFFSET));

	return 0;
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

	fpga->spi = spi;

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

	/* Bring it out of reset */
	fpga->reset_gpio = of_get_named_gpio(np, "reset-switch", 0);
	if (!gpio_is_valid(fpga->reset_gpio)) {
		dev_err(&spi->dev, "unable to find reset-switch gpio\n");
		goto fail;
	}
	ret = devm_gpio_request_one(&spi->dev, fpga->reset_gpio,
			GPIOF_OUT_INIT_LOW, "fpga reset");
	if (ret) {
		dev_err(&spi->dev, "unable to get fpga reset switch: %d\n",
				ret);
		goto fail;
	}

	gpio_set_value(fpga->power_gpio, 1);
	gpio_set_value(fpga->reset_gpio, 1);

	ret = load_firmware(spi);
	if (ret)
		goto fail;

	/* Now that firmware is loaded, start the FPGA */
	ret = clk_prepare_enable(fpga->lvds_switch_clk);
	if (ret) {
		dev_err(&spi->dev, "unable to enable lvds clock\n");
		goto fail;
	}

	/* Set up EIM regions */
	ret = clk_prepare_enable(fpga->eim_slow_clk);
	if (ret) {
		dev_err(&spi->dev, "unable to enable eim clock\n");
		goto fail;
	}

	/* Map areas */
	fpga->fpga_ctrl = devm_ioremap(&spi->dev, IMX6_EIM_CS0_BASE_ADDR, 8192);
	dev_dbg(&spi->dev, "remapped fpga ctrl 0x%08x to 0x%p\n",
			IMX6_EIM_CS0_BASE_ADDR, fpga->fpga_ctrl);

	fpga->eim_area = devm_ioremap(&spi->dev, IMX6_EIM_BASE_ADDR, 4096);
	dev_dbg(&spi->dev, "remapped eim 0x%08x to 0x%p\n",
			IMX6_EIM_BASE_ADDR, fpga->eim_area);

	fpga->iomuxc_gpr =
		syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (IS_ERR(fpga->iomuxc_gpr)) {
		dev_err(&spi->dev, "unable to find iomuxc registers\n");
		ret = PTR_ERR(fpga->iomuxc_gpr);
		goto fail;
	}

	fpga_timing_setup(spi, fpga);

	fpga->byte_area = ioremap_wc(DATA_FIFO_ADDR, 4096);
	dev_dbg(&spi->dev, "remapped fifo area 0x%08x to 0x%p\n",
			DATA_FIFO_ADDR, fpga->byte_area);
	fpga->byte_size = 4096;

	dev_info(&spi->dev, "FPGA version %d.%d\n",
			fpga->fpga_ctrl[FPGA_R_DDR3_V_MAJOR],
			fpga->fpga_ctrl[FPGA_R_DDR3_V_MINOR]);
	dev_info(&spi->dev, "FPGA version (again) %d.%d\n",
			fpga->fpga_ctrl[FPGA_R_DDR3_V_MAJOR],
			fpga->fpga_ctrl[FPGA_R_DDR3_V_MINOR]);

	spi_set_drvdata(spi, fpga);
	g_fpga = fpga;

	return 0;
	
fail:
	genl_unregister_family(&kosagi_fpga_family);

	if (fpga) {
		if (fpga->byte_area)
			iounmap(fpga->byte_area);
		kfree(fpga);
	}
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
