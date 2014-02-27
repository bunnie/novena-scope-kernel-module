#define DDR3_SIZE (1024 * 1024 * 1)  // in words (4 bytes per word)
#define DDR3_FIFODEPTH 64

#define PULSE_GATE_MASK  0x1000

void ddr3_test();
void ddr3_test_opt();
void dump_ddr3(unsigned int address, unsigned int len);
void ddr3load(int ifd, int verify);
void ddr3_burst_read();
