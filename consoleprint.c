#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "userspace.h"

int main(int argc, char **argv)
{
	struct fpga_connection *c = fpga_open_connection(FAMILY_NAME);
	int i;
	
	for (i = 0; i < 4; i++) {
		uint8_t data[4096];
		fpga_read_4k(c, data);
	}
}
