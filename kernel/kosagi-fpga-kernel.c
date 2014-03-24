#define DEBUG
#include <net/genetlink.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dmaengine.h>

#include <linux/platform_data/dma-imx.h>

#define IMX6_EIM_CS1_BASE_ADDR 0x0C040000

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
	u32 snd_portid;
	struct net *net;
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
	struct nlattr *na;
	char *mydata;

	if (info == NULL)
		goto out;

	fpga->snd_portid = info->snd_portid,
	fpga->net = genl_info_net(info);

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

	//kosagi_receive_message("Hello from the kernel");
	//kosagi_receive_message("Hello again from the kernel");

out:
	return 0;
}

#if 0
static int kosagi_receive_message(const char *msg)
{
	struct sk_buff *skb;
	void *msg_head;
	int err;

	/*
	 * Send a message back.
	 * Allocate some memory, but since the size is not yet known
	 * use NLMSG_GOODSIZE
	 */
	skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (skb == NULL) {
		err = -ENOMEM;
		goto out;
	}

	/* create the message headers */
	/* arguments of genlmsg_put:
	   struct sk_buff *,
	   int (sending) pid,
	   int sequence number,
	   struct genl_family *,
	   int flags,
	   u8 command index (why do we need this?)
	*/
	msg_head = genlmsg_put(skb, 0, fpga->seq_num++,
			&kosagi_fpga_family, 0, KOSAGI_CMD_SEND);
	if (msg_head == NULL) {
		err = -ENOMEM;
		goto out;
	}

	/* add a KOSAGI_ATTR_MESSAGE attribute (actual value to be sent) */
	err = nla_put_string(skb, KOSAGI_ATTR_MESSAGE, msg);
	if (err != 0)
		goto out;

	/* finalize the message */
	genlmsg_end(skb, msg_head);

	/* send the message back */
	err = genlmsg_unicast(fpga->net, skb, fpga->snd_portid);

	if (err != 0)
		goto out;

	return 0;

out:
	printk("an error occured in kosagi_receive_message: %d\n", err);
	return 0;
}
#endif

static int kosagi_fpga_read(struct sk_buff *skb_2, struct genl_info *info)
{
	struct sk_buff *msg;
	struct nlattr *nla;
	void *hdr;
	int ret;

	if (info == NULL) {
		pr_err("Unable to read: info was NULL\n");
		return -EINVAL;
	}

	msg = genlmsg_new(fpga->byte_size, GFP_KERNEL);
	if (!msg) {
		pr_err("Unable to create new nlmsg\n");
		return -ENOMEM;
	}

	hdr = genlmsg_put(msg, info->snd_portid, info->snd_seq,
			&kosagi_fpga_family, 0, KOSAGI_CMD_READ);
	if (!hdr) {
		pr_err("Unable to put info into genlmsg\n");
		ret = -EMSGSIZE;
		goto err;
	}

	nla = __nla_reserve(msg, KOSAGI_ATTR_FPGA_DATA, fpga->byte_size);
	memcpy(nla_data(nla), fpga->byte_area, fpga->byte_size);

	genlmsg_end(msg, hdr);

	u8 *data = nla_data(nla);
	pr_err("Unicasting message\n");
	int i;
	for (i = 0; i < 3; i++) {
		pr_err("0x%02x 0x%02x 0x%02x 0x%02x "
			"0x%02x 0x%02x 0x%02x 0x%02x  "
			"0x%02x 0x%02x 0x%02x 0x%02x "
			"0x%02x 0x%02x 0x%02x 0x%02x\n",
			data[0], data[1], data[2], data[3],
			data[4], data[5], data[6], data[7],
			data[8], data[9], data[10], data[11],
			data[12], data[13], data[14], data[15]);
		data += 16;
	}
	return genlmsg_unicast(genl_info_net(info), msg, info->snd_portid);

err:
	if (msg)
		nlmsg_free(msg);
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

	fpga->byte_area = ioremap_wc(IMX6_EIM_CS1_BASE_ADDR, 4096);
	fpga->byte_size = 2048;

	return 0;
	
fail:
	pr_err("Unable to load Kosagi FPGA module: %d\n", ret);
	return -1;
}

static void __exit kosagi_fpga_exit(void)
{
	int ret;

	/*unregister the family*/
	ret = genl_unregister_family(&kosagi_fpga_family);
	if (ret !=0) {
		pr_err("Error while unregistering family: %d\n", ret);
	}

	kfree(fpga);
}


module_init(kosagi_fpga_init);
module_exit(kosagi_fpga_exit);
MODULE_LICENSE("GPL");
