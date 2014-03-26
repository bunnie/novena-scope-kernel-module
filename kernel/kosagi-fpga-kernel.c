#define DEBUG
#include <net/genetlink.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dmaengine.h>

#include <linux/platform_data/dma-imx.h>

#define IMX6_EIM_CS1_BASE_ADDR 0x0c040000
#define DATA_FIFO_ADDR (IMX6_EIM_CS1_BASE_ADDR + 0xf000)

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
	unsigned int seq_num;
	void *byte_area;
	unsigned int byte_size;
	u8 dma_buffer[4096];
};
static struct kosagi_fpga *fpga;

/* mapping of attributes to type */
static struct nla_policy kosagi_fpga_genl_policy[KOSAGI_ATTR_MAX + 1] = {
	[KOSAGI_ATTR_FPGA_DATA] = { .type = NLA_BINARY, .len = 4096 },
	[KOSAGI_ATTR_MESSAGE] = { .type = NLA_NUL_STRING },
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

static int kosagi_fpga_read(struct sk_buff *skb_2, struct genl_info *info)
{
	struct sk_buff *buf;
	void *hdr = NULL;
	void *data;
	int ret = 0;

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


static int __init kosagi_fpga_init(void)
{
	int ret;

	ret = genl_register_family_with_ops(&kosagi_fpga_family,
					    kosagi_genl_ops);
	if (ret != 0)
		goto fail;

	fpga = kzalloc(sizeof(*fpga), GFP_KERNEL);
	if (!fpga)
		goto fail;

	fpga->byte_area = ioremap_wc(DATA_FIFO_ADDR, 4096);
	pr_err("Scope: Remapped 0x%08x to 0x%p\n",
			DATA_FIFO_ADDR, fpga->byte_area);
	fpga->byte_size = 4096;

	return 0;
	
fail:
	pr_err("Unable to load Kosagi FPGA module: %d\n", ret);
	return -1;
}

static void __exit kosagi_fpga_exit(void)
{
	int ret;

	if (fpga->byte_area) {
		pr_err("Scope: Unmapped 0x%p from 0x%08x\n",
				fpga->byte_area, DATA_FIFO_ADDR);
		iounmap(fpga->byte_area);
	}

	ret = genl_unregister_family(&kosagi_fpga_family);
	if (ret !=0) {
		pr_err("Error while unregistering family: %d\n", ret);
	}

	kfree(fpga);
}


module_init(kosagi_fpga_init);
module_exit(kosagi_fpga_exit);
MODULE_LICENSE("GPL");
