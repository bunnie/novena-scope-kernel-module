#ifndef __USERSPACE_H__
#define __USERSPACE_H__
#define FAMILY_NAME "kosagi-fpga"
#include <stdint.h>

struct fpga_connection;

struct fpga_connection *fpga_open_connection(char *str);
int fpga_read_4k(struct fpga_connection *conn, uint8_t data[4096]);
#endif /* __USERSPACE_H__ */
