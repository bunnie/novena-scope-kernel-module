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
        KOSAGI_CMD_UNSPEC,
	KOSAGI_CMD_SEND,
	KOSAGI_CMD_READ,
	__KOSAGI_CMD_MAX,
};

/* list of valid command attributes */
enum kosagi_fpga_attributes {
	KOSAGI_ATTR_NONE,
	KOSAGI_ATTR_FPGA_DATA,
	KOSAGI_ATTR_MESSAGE,
	__KOSAGI_ATTR_MAX,
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

	nl_socket_disable_auto_ack(conn->handle);

	return conn;

err:
	return NULL;
}

static int fpga_send_read_request(struct fpga_connection *conn)
{
	struct nl_msg *msg;
	int ret;

	msg = fpga_alloc_msg(conn, KOSAGI_CMD_READ);
	if (!msg)
		return -1;

	ret = nl_send_auto(conn->handle, msg);
	if (ret < 0) {
		fprintf(stderr, "Unable to send msg: %s\n", nl_geterror(ret));
		nlmsg_free(msg);
		return ret;
	}
	nlmsg_free(msg);
	return 0;
}

static int fpga_do_read_request(struct fpga_connection *conn, uint8_t data[4096])
{
	void *nhdr;
	struct genlmsghdr *ghdr;
	struct sockaddr_nl nla;
	void *d;
	int ret;

	ret = nl_recv(conn->handle, &nla, (unsigned char **)&nhdr, NULL);
	if (ret < 0) {
		fprintf(stderr, "Unable to receive data: %s\n", nl_geterror(ret));
		return ret;
	}

	ghdr = nlmsg_data(nhdr);
	d = genlmsg_user_data(ghdr, 0);

	memcpy(data, d, 4096);

	free(nhdr);

	return 0;
}

int fpga_read_4k(struct fpga_connection *conn, uint8_t data[4096])
{
	int ret;

	ret = fpga_send_read_request(conn);
	if (ret < 0)
		return ret;

	ret = fpga_do_read_request(conn, data);
	if (ret < 0)
		return ret;

	return ret;
}
