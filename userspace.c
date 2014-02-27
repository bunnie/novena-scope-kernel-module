#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <signal.h>

#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>  
#include <netlink/msg.h>
#include <netlink/attr.h>

#define FAMILY_NAME "kosagi-fpga"
#define DATA_SIZE 4096

/* list of valid commands */
enum kosagi_fpga_commands {
	KOSAGI_C_UNSPEC,
	KOSAGI_C_ECHO,
	KOSAGI_C_READ_4K,
	KOSAGI_C_READ_4K_RESPONSE,
	KOSAGI_C_WRITE_4K,
	KOSAGI_C_WRITE_4K_WITH_ADDR,
	__KOSAGI_C_MAX,
};

/* list of valid command attributes */
enum kosagi_fpga_attributes {
	KOSAGI_A_NONE,
	KOSAGI_A_MESSAGE,
	KOSAGI_A_4K_DATA,
        KOSAGI_A_4K_ADDRESS,
	__KOSAGI_A_MAX,
};


struct fpga_connection {
	struct nl_sock *handle;
	struct nl_cache *cache;
	struct genl_family *id;
};


int print_hex_offset(uint8_t *block, int count, int offset)
{
	int byte;
	count += offset;
	block -= offset;
	for ( ; offset < count; offset += 16) {
		printf("%08x ", offset);

		for (byte=0; byte<16; byte++) {
			if (byte == 8)
				printf(" ");
			if (offset + byte < count)
				printf(" %02x", block[offset + byte] & 0xff);
			else
				printf("   ");
		}

		printf("  |");
		for (byte = 0; byte < 16 && byte + offset < count; byte++)
			printf("%c", isprint(block[offset + byte]) ?
					block[offset + byte] : '.');
		printf("|\n");
	}
	return 0;
}

int print_hex(uint8_t *block, int count)
{
	return print_hex_offset(block, count, 0);
}

/*
 * Get a particular attribute (and length) from the NetLink message.
 * XXX Note: Currently only returns the first attribute XXX
 */
static int get_attr(struct nlmsghdr *hdr, int attr, void **data, int *datalen)
{
	struct genlmsghdr *genlhdr;
	struct nlattr *nlattr;

	genlhdr = genlmsg_hdr(hdr);

	nlattr = genlmsg_attrdata(genlhdr, 0);
	if (!nlattr)
		return -1;

	*data = nla_data(nlattr);
	*datalen = nla_len(nlattr);
	return 0;
}


static struct nl_msg *fpga_alloc_msg(struct fpga_connection *conn, int cmd)
{
	struct nl_msg *msg;
	void *header;

	msg = nlmsg_alloc_size(2 * DATA_SIZE);
	if (!msg) {
		fprintf(stderr, "Unable to alloc nlmsg\n");
		return NULL;
	}

	header = genlmsg_put(msg, NL_AUTO_PORT, NL_AUTO_SEQ,
			genl_family_get_id(conn->id),
			0, NLM_F_REQUEST, cmd, 1);
	if (!header) {
		fprintf(stderr, "Unable to call genlmsg_put()\n");
		nlmsg_free(msg);
		return NULL;
	}

	return msg;
}


struct fpga_connection *fpga_open_connection(char *str)
{
	struct fpga_connection *conn;
	int ret;

	conn = malloc(sizeof(struct fpga_connection));
	if (!conn)
		goto err;

	conn->handle = nl_socket_alloc();
	if (!conn->handle) {
		fprintf(stderr, "Failed to allocate netlink handle\n");
		goto err;
	}

	ret = genl_connect(conn->handle);
	if (ret) {
		fprintf(stderr, "Failed to connect to generic netlink: %s\n",
				nl_geterror(ret));
		goto err;
	}

	ret = genl_ctrl_alloc_cache(conn->handle, &conn->cache);
	if (ret) {
		fprintf(stderr, "Failed to allocate generic netlink cache\n");
		goto err;
	}

	conn->id = genl_ctrl_search_by_name(conn->cache, str);
	if (!conn->id) {
		fprintf(stderr, "Family %s not found\n", str);
		goto err;
	}

	ret = nl_socket_set_msg_buf_size(conn->handle, 2 * DATA_SIZE);
	if (ret < 0) {
		fprintf(stderr, "Failed to set buffer size: %s\n",
				nl_geterror(ret));
		goto err;
	}

	return conn;

err:
	return NULL;
}

static int print_result(struct nl_msg *msg, void *arg)
{
	struct nlmsghdr *hdr = nlmsg_hdr(msg);
	struct nlattr *attr = nlmsg_attrdata(hdr, KOSAGI_A_MESSAGE);
	char *str = nla_get_string(attr);
	printf("Received message: %s\n", str);
	return NL_OK;
}

static int fpga_receive_echo(struct fpga_connection *conn)
{
	int ret;
	struct nl_cb *cb = NULL;
	cb = nl_cb_alloc(NL_CB_CUSTOM);
	if (!cb) {
		fprintf(stderr, "Unable to allocate callback\n");
		exit(5);
	}

	printf("Receiving...\n");
	nl_cb_set(cb, NL_CB_MSG_IN, NL_CB_CUSTOM, print_result, NULL);
	ret = nl_recvmsgs(conn->handle, cb);
	ret = nl_recvmsgs(conn->handle, cb);
	nl_wait_for_ack(conn->handle);
	return ret;
}

static int fpga_send_echo(struct fpga_connection *conn, char *message)
{
	int ret;
	struct nl_msg *msg;

	msg = fpga_alloc_msg(conn, KOSAGI_C_ECHO);
	if (!msg)
		return -1;

	ret = nla_put_string(msg, KOSAGI_A_MESSAGE, message);
	if (ret < 0) {
		fprintf(stderr, "Unable to add message string: %s\n",
				nl_geterror(ret));
		goto out;
	}

	printf("Sending...\n");
	ret = nl_send_auto(conn->handle, msg);
	if (ret < 0) {
		fprintf(stderr, "Unable to send msg: %s\n", nl_geterror(ret));
		goto out;
	}

out:
	nlmsg_free(msg);
	return ret;
}

int fpga_read_4k(struct fpga_connection *conn)
{
	struct nl_msg *msg;
	struct nlmsghdr *hdr;
	struct sockaddr_nl nla;
	int ret;
	void *data;
	int datalen;

	msg = fpga_alloc_msg(conn, KOSAGI_C_READ_4K);
	if (!msg)
		return -1;

	ret = nl_send_auto(conn->handle, msg);
	if (ret < 0) {
		fprintf(stderr, "Unable to send msg: %s\n", nl_geterror(ret));
		nlmsg_free(msg);
		return ret;
	}
	nlmsg_free(msg);


	ret = nl_recv(conn->handle, &nla, (unsigned char **)&hdr, NULL);
	if (ret < 0) {
		fprintf(stderr, "Unable to receive data: %s\n", nl_geterror(ret));
		return ret;
	}
	fprintf(stderr, "Received %d bytes\n", ret);

	ret = get_attr(hdr, KOSAGI_A_4K_DATA, &data, &datalen);
	print_hex(data, datalen);
	fprintf(stderr, "Data length: %d\n", datalen);

	free(hdr);
	return ret;
}

static int fpga_write_4k(struct fpga_connection *conn, void *data)
{
	int ret;
	struct nl_msg *msg;

	msg = fpga_alloc_msg(conn, KOSAGI_C_WRITE_4K);
	if (!msg)
		return -1;

	ret = nla_put(msg, KOSAGI_A_4K_DATA, sizeof(data), data);
	if (ret < 0) {
		fprintf(stderr, "Unable to add data: %s\n", nl_geterror(ret));
		goto out;
	}

	printf("Sending...\n");
	ret = nl_send_auto(conn->handle, msg);
	if (ret < 0) {
		fprintf(stderr, "Unable to send msg: %s\n", nl_geterror(ret));
		goto out;
	}

out:
	nlmsg_free(msg);
	return ret;
}

static int fpga_write_4k_addr(struct fpga_connection *conn,
		void *data, int addr)
{
	int ret;
	struct nl_msg *msg;

	msg = fpga_alloc_msg(conn, KOSAGI_C_WRITE_4K_WITH_ADDR);
	if (!msg)
		return -1;

	ret = nla_put_u32(msg, KOSAGI_A_4K_ADDRESS, addr);
	if (ret < 0) {
		fprintf(stderr, "Unable to add address: %s\n", nl_geterror(ret));
		goto out;
	}

	ret = nla_put(msg, KOSAGI_A_4K_DATA, sizeof(data), data);
	if (ret < 0) {
		fprintf(stderr, "Unable to add data: %s\n", nl_geterror(ret));
		goto out;
	}

	printf("Sending...\n");
	ret = nl_send_auto(conn->handle, msg);
	if (ret < 0) {
		fprintf(stderr, "Unable to send msg: %s\n", nl_geterror(ret));
		goto out;
	}

out:
	nlmsg_free(msg);
	return ret;
}

#if 0
int main(int argc, char **argv)
{
	struct fpga_connection *conn;
	char *message = "Hi there";
	uint8_t data[DATA_SIZE];
	int i;

	conn = fpga_open_connection(FAMILY_NAME);
	if (!conn)
		return -1;

	fpga_send_echo(conn, message);
	fpga_receive_echo(conn);

	fprintf(stderr, "Going to receive data...\n");
	fpga_read_4k(conn);

	fprintf(stderr, "Going to send data...\n");
	for (i = 0; i < sizeof(data); i++)
		data[i] = i;
	fpga_write_4k(conn, data);

	fprintf(stderr, "Going to send data to a specific address...\n");
	for (i = 0; i < sizeof(data); i++)
		data[i] = i;
	fpga_write_4k_addr(conn, data, 0x0C04F000);

	return 0;
}
#endif
