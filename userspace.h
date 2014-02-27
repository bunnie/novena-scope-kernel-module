#define FAMILY_NAME "kosagi-fpga"

struct fpga_connection;

struct fpga_connection *fpga_open_connection(char *str);
int fpga_read_4k(struct fpga_connection *conn);
