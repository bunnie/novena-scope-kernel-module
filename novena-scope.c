//#define _GNU_SOURCE // for O_DIRECT

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
//#include <sys/types.h>
//#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include "gpio.h"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "ad9520.h"
#include "dac101c085.h"
#include "adc08d1020.h"
#include "lmh6518.h"
#include "ddr3.h"
#include "novena-scope.h"
#include "userspace.h"

static int fd = 0;
static int   *mem_32 = 0;
static short *mem_16 = 0;
static char  *mem_8  = 0;
static int *prev_mem_range = 0;

int read_kernel_memory(long offset, int virtualized, int size) {
  int result;

  int *mem_range = (int *)(offset & ~0xFFFF);
  if( mem_range != prev_mem_range ) {
    //        fprintf(stderr, "New range detected.  Reopening at memory range %p\n", mem_range);
    prev_mem_range = mem_range;

    if(mem_32)
      munmap(mem_32, 0xFFFF);
    if(fd)
      close(fd);

    if(virtualized) {
      fd = open("/dev/kmem", O_RDWR);
      if( fd < 0 ) {
	perror("Unable to open /dev/kmem");
	fd = 0;
	return -1;
      }
    }
    else {
      fd = open("/dev/mem", O_RDWR);
      if( fd < 0 ) {
	perror("Unable to open /dev/mem");
	fd = 0;
	return -1;
      }
    }

    mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset&~0xFFFF);
    if( -1 == (int)mem_32 ) {
      perror("Unable to mmap file");

      if( -1 == close(fd) )
	perror("Also couldn't close file");

      fd=0;
      return -1;
    }
    mem_16 = (short *)mem_32;
    mem_8  = (char  *)mem_32;
  }

  int scaled_offset = (offset-(offset&~0xFFFF));
  //    fprintf(stderr, "Returning offset 0x%08x\n", scaled_offset);
  if(size==1)
    result = mem_8[scaled_offset/sizeof(char)];
  else if(size==2)
    result = mem_16[scaled_offset/sizeof(short)];
  else
    result = mem_32[scaled_offset/sizeof(long)];

  return result;
}

int write_kernel_memory(long offset, long value, int virtualized, int size) {
  int old_value = read_kernel_memory(offset, virtualized, size);
  int scaled_offset = (offset-(offset&~0xFFFF));
  if(size==1)
    mem_8[scaled_offset/sizeof(char)]   = value;
  else if(size==2)
    mem_16[scaled_offset/sizeof(short)] = value;
  else
    mem_32[scaled_offset/sizeof(long)]  = value;
  return old_value;
}

void print_usage(char *progname) {
  printf("Usage:\n"
        "%s [-h]\n"
        "\t-h                           This help message\n"
        "\t-v                           Version code (0xMinor.0xMajor)\n"
        "\t-ddr3dump <address> <count>  dump DDR3 memory\n"
        "\t-ddr3load <file>             load DDR3 from file to offset 0x0\n"
        "\t-ddr3qload <file>            load just the top 64k to 0x0\n"
        "\t-ddr3verify <file>           compare file vs DDR3 memory contents\n"
        "\t-dt                          run DDR3 stress test\n"
        "\t-dt2                         run optimized DDR3 stress test\n"
        "\t-adc_reset                   reset the ADC subsystem\n"
        "\t-adc_off                     power down expansion card\n"
        "\t-adc_pll                     configure the ADC PLL\n"
        "\t-adc_cal                     initiate ADC calibration\n"
        "\t-adc_calstat                 calibration completion status\n"
        "\t-adc_tp                      set ADC to generate a test pattern\n"
        "\t-adc_default                 set ADC to defaults\n"
        "\t-adc_dev                     dev use only\n"
	"\t-burstread                   do a burst read test\n"
	"\t-afe_offset 0x<code>         set offset to digital hex <code> value\n"
	"\t-afe_set <filt> <atten>      set AFE preamp, filt 20-750MHz (0=inf), atten 0-20, 20 most attenuative\n"
	 "", progname);
}


//static inline int swab(int arg) {
//  return ((arg&0xff)<<24) | ((arg&0xff00)<<8) | ((arg&0xff0000)>>8) | ((arg&0xff000000)>>24);
//}

void setup_fpga() {
  int i;
  printf( "setting up EIM CS0 (register interface) pads and configuring timing\n" );
  // set up pads to be mapped to EIM
  for( i = 0; i < 16; i++ ) {
    write_kernel_memory( 0x20e0114 + i*4, 0x0, 0, 4 );  // mux mapping
    write_kernel_memory( 0x20e0428 + i*4, 0xb0b1, 0, 4 ); // pad strength config'd for a 100MHz rate 
  }

  // mux mapping
  write_kernel_memory( 0x20e046c - 0x314, 0x0, 0, 4 ); // BCLK
  write_kernel_memory( 0x20e040c - 0x314, 0x0, 0, 4 ); // CS0
  write_kernel_memory( 0x20e0410 - 0x314, 0x0, 0, 4 ); // CS1
  write_kernel_memory( 0x20e0414 - 0x314, 0x0, 0, 4 ); // OE
  write_kernel_memory( 0x20e0418 - 0x314, 0x0, 0, 4 ); // RW
  write_kernel_memory( 0x20e041c - 0x314, 0x0, 0, 4 ); // LBA
  write_kernel_memory( 0x20e0468 - 0x314, 0x0, 0, 4 ); // WAIT
  write_kernel_memory( 0x20e0408 - 0x314, 0x0, 0, 4 ); // A16
  write_kernel_memory( 0x20e0404 - 0x314, 0x0, 0, 4 ); // A17
  write_kernel_memory( 0x20e0400 - 0x314, 0x0, 0, 4 ); // A18

  // pad strength
  write_kernel_memory( 0x20e046c, 0xb0b1, 0, 4 ); // BCLK
  write_kernel_memory( 0x20e040c, 0xb0b1, 0, 4 ); // CS0
  write_kernel_memory( 0x20e0410, 0xb0b1, 0, 4 ); // CS1
  write_kernel_memory( 0x20e0414, 0xb0b1, 0, 4 ); // OE
  write_kernel_memory( 0x20e0418, 0xb0b1, 0, 4 ); // RW
  write_kernel_memory( 0x20e041c, 0xb0b1, 0, 4 ); // LBA
  write_kernel_memory( 0x20e0468, 0xb0b1, 0, 4 ); // WAIT
  write_kernel_memory( 0x20e0408, 0xb0b1, 0, 4 ); // A16
  write_kernel_memory( 0x20e0404, 0xb0b1, 0, 4 ); // A17
  write_kernel_memory( 0x20e0400, 0xb0b1, 0, 4 ); // A18

  write_kernel_memory( 0x020c4080, 0xcf3, 0, 4 ); // ungate eim slow clocks

  // rework timing for sync use
  // 0011 0  001 1   001    0   001 00  00  1  011  1    0   1   1   1   1   1   1
  // PSZ  WP GBC AUS CSREC  SP  DSZ BCS BCD WC BL   CREP CRE RFL WFL MUM SRD SWR CSEN
  //
  // PSZ = 0011  64 words page size
  // WP = 0      (not protected)
  // GBC = 001   min 1 cycles between chip select changes
  // AUS = 0     address shifted according to port size
  // CSREC = 001 min 1 cycles between CS, OE, WE signals
  // SP = 0      no supervisor protect (user mode access allowed)
  // DSZ = 001   16-bit port resides on DATA[15:0]
  // BCS = 00    0 clock delay for burst generation
  // BCD = 00    divide EIM clock by 0 for burst clock
  // WC = 1      write accesses are continuous burst length
  // BL = 011    32 word memory wrap length
  // CREP = 1    non-PSRAM, set to 1
  // CRE = 0     CRE is disabled
  // RFL = 1     fixed latency reads
  // WFL = 1     fixed latency writes
  // MUM = 1     multiplexed mode enabled
  // SRD = 1     synch reads
  // SWR = 1     synch writes
  // CSEN = 1    chip select is enabled

  //  write_kernel_memory( 0x21b8000, 0x5191C0B9, 0, 4 );
  write_kernel_memory( 0x21b8000, 0x31910BBF, 0, 4 );

  // EIM_CS0GCR2   
  //  MUX16_BYP_GRANT = 1
  //  ADH = 1 (1 cycles)
  //  0x1001
  write_kernel_memory( 0x21b8004, 0x1000, 0, 4 );


  // EIM_CS0RCR1   
  // 00 000101 0 000   0   000   0 000 0 000 0 000 0 000
  //    RWSC     RADVA RAL RADVN   OEA   OEN   RCSA  RCSN
  // RWSC 000101    5 cycles for reads to happen
  //
  // 0000 0111 0000   0011   0000 0000 0000 0000
  //  0    7     0     3      0  0    0    0
  // 0000 0101 0000   0000   0 000 0 000 0 000 0 000
//  write_kernel_memory( 0x21b8008, 0x05000000, 0, 4 );
//  write_kernel_memory( 0x21b8008, 0x0A024000, 0, 4 );
  write_kernel_memory( 0x21b8008, 0x09014000, 0, 4 );
  // EIM_CS0RCR2  
  // 0000 0000 0   000 00 00 0 010  0 001 
  //           APR PAT    RL   RBEA   RBEN
  // APR = 0   mandatory because MUM = 1
  // PAT = XXX because APR = 0
  // RL = 00   because async mode
  // RBEA = 000  these match RCSA/RCSN from previous field
  // RBEN = 000
  // 0000 0000 0000 0000 0000  0000
  write_kernel_memory( 0x21b800c, 0x00000000, 0, 4 );

  // EIM_CS0WCR1
  // 0   0    000100 000   000   000  000  010 000 000  000
  // WAL WBED WWSC   WADVA WADVN WBEA WBEN WEA WEN WCSA WCSN
  // WAL = 0       use WADVN
  // WBED = 0      allow BE during write
  // WWSC = 000100 4 write wait states
  // WADVA = 000   same as RADVA
  // WADVN = 000   this sets WE length to 1 (this value +1)
  // WBEA = 000    same as RBEA
  // WBEN = 000    same as RBEN
  // WEA = 010     2 cycles between beginning of access and WE assertion
  // WEN = 000     1 cycles to end of WE assertion
  // WCSA = 000    cycles to CS assertion
  // WCSN = 000    cycles to CS negation
  // 1000 0111 1110 0001 0001  0100 0101 0001
  // 8     7    E    1    1     4    5    1
  // 0000 0111 0000 0100 0000  1000 0000 0000
  // 0      7    0   4    0     8    0     0
  // 0000 0100 0000 0000 0000  0100 0000 0000
  //  0    4    0    0     0    4     0    0

  write_kernel_memory( 0x21b8010, 0x09080800, 0, 4 );
  //  write_kernel_memory( 0x21b8010, 0x02040400, 0, 4 );

  // EIM_WCR
  // BCM = 1   free-run BCLK
  // GBCD = 0  don't divide the burst clock
  write_kernel_memory( 0x21b8090, 0x701, 0, 4 );

  // EIM_WIAR 
  // ACLK_EN = 1
  write_kernel_memory( 0x21b8094, 0x10, 0, 4 );

  printf( "done.\n" );
}

void setup_fpga_cs1() { 
  int i;
  printf( "setting up EIM CS1 (burst interface) pads and configuring timing\n" );
  // ASSUME: setup_fpga() is already called to configure gpio mux setting.
  // this just gets the pads set to high-speed mode

  // set up pads to be mapped to EIM
  for( i = 0; i < 16; i++ ) {
    write_kernel_memory( 0x20e0428 + i*4, 0xb0f1, 0, 4 ); // pad strength config'd for a 200MHz rate 
  }

  // pad strength
  write_kernel_memory( 0x20e046c, 0xb0f1, 0, 4 ); // BCLK
  //  write_kernel_memory( 0x20e040c, 0xb0b1, 0, 4 ); // CS0
  write_kernel_memory( 0x20e0410, 0xb0f1, 0, 4 ); // CS1
  write_kernel_memory( 0x20e0414, 0xb0f1, 0, 4 ); // OE
  write_kernel_memory( 0x20e0418, 0xb0f1, 0, 4 ); // RW
  write_kernel_memory( 0x20e041c, 0xb0f1, 0, 4 ); // LBA
  write_kernel_memory( 0x20e0468, 0xb0f1, 0, 4 ); // WAIT
  write_kernel_memory( 0x20e0408, 0xb0f1, 0, 4 ); // A16
  write_kernel_memory( 0x20e0404, 0xb0f1, 0, 4 ); // A17
  write_kernel_memory( 0x20e0400, 0xb0f1, 0, 4 ); // A18

  //////////// NOTE EIM max operating frequency derated to 104MHz in latest version of datasheet

  // EIM_CS1GCR1   
  // 0011 0  001 1   001    0   001 00  00  1  011  1    0   1   1   1   1   1   1
  // PSZ  WP GBC AUS CSREC  SP  DSZ BCS BCD WC BL   CREP CRE RFL WFL MUM SRD SWR CSEN
  //
  // PSZ = 1000  2k words page size
  // WP = 0      (not protected)
  // GBC = 001   min 1 cycles between chip select changes
  // AUS = 0     address shifted according to port size
  // CSREC = 001 min 1 cycles between CS, OE, WE signals
  // SP = 0      no supervisor protect (user mode access allowed)
  // DSZ = 001   16-bit port resides on DATA[15:0]
  // BCS = 00    0 clock delay for burst generation
  // BCD = 00    divide EIM clock by 0 for burst clock
  // WC = 1      write accesses are continuous burst length
  // BL = 100    32 word memory wrap length
  // CREP = 1    non-PSRAM, set to 1
  // CRE = 0     CRE is disabled
  // RFL = 1     fixed latency reads
  // WFL = 1     fixed latency writes
  // MUM = 1     multiplexed mode enabled
  // SRD = 1     synch reads
  // SWR = 1     synch writes
  // CSEN = 1    chip select is enabled

  // 0101 0111 1111    0001 1100  0000  1011   1   0   0   1
  // 0x5  7    F        1   C     0     B    9

  // 0101 0001 1001    0001 1100  0000  1011   1001
  // 5     1    9       1    c     0     B      9

  // 0011 0001 1001    0001 0000  1011  1011   1111

  //  write_kernel_memory( 0x21b8000 + 0x18, 0x31910BBF, 0, 4 ); // pre-tweak setting
  write_kernel_memory( 0x21b8000 + 0x18, 0x81910CBF, 0, 4 );  // from xobs board

  // EIM_CS1GCR2   
  //  MUX16_BYP_GRANT = 1
  //  ADH = 0 (0 cycles)
  //  0x1000
  write_kernel_memory( 0x21b8004 + 0x18, 0x1000, 0, 4 );


  // 9 cycles is total length of read
  // 2 cycles for address
  // +4 more cycles for first data to show up

  // EIM_CS1RCR1   
  // 00 000100 0 000   0   001   0 010 0 000 0 000 0 000
  //    RWSC     RADVA RAL RADVN   OEA   OEN   RCSA  RCSN
  //
  // 00 001001 0 000   0   001   0 110 0 000 0 000 0 000
  //    RWSC     RADVA RAL RADVN   OEA   OEN   RCSA  RCSN
  //
  // 0000 0111 0000   0011   0000 0000 0000 0000
  //  0    7     0     3      0  0    0    0
  // 0000 0101 0000   0000   0 000 0 000 0 000 0 000
//  write_kernel_memory( 0x21b8008, 0x05000000, 0, 4 );
  // 0000 0011 0000   0001   0001 0000 0000 0000

  // 0000 1001 0000   0001   0110 0000 0000 0000
  // 
  //  write_kernel_memory( 0x21b8008 + 0x18, 0x09014000, 0, 4 );  // pre-tweak setting
  write_kernel_memory( 0x21b8008 + 0x18, 0x07001000, 0, 4 ); // tweak to optimize

  // EIM_CS1RCR2  
  // 0000 0000 0   000 00 00 0 010  0 001 
  //           APR PAT    RL   RBEA   RBEN
  // APR = 0   mandatory because MUM = 1
  // PAT = XXX because APR = 0
  // RL = 00   because async mode
  // RBEA = 000  these match RCSA/RCSN from previous field
  // RBEN = 000
  // 0000 0000 0000 0000 0000  0000
  write_kernel_memory( 0x21b800c + 0x18, 0x00000200, 0, 4 );

  // EIM_CS1WCR1
  // 0   0    000010 000   001   000  000  010 000 000  000
  // WAL WBED WWSC   WADVA WADVN WBEA WBEN WEA WEN WCSA WCSN
  // WAL = 0       use WADVN
  // WBED = 0      allow BE during write
  // WWSC = 000100 4 write wait states
  // WADVA = 000   same as RADVA
  // WADVN = 000   this sets WE length to 1 (this value +1)
  // WBEA = 000    same as RBEA
  // WBEN = 000    same as RBEN
  // WEA = 010     2 cycles between beginning of access and WE assertion
  // WEN = 000     1 cycles to end of WE assertion
  // WCSA = 000    cycles to CS assertion
  // WCSN = 000    cycles to CS negation
  // 1000 0111 1110 0001 0001  0100 0101 0001
  // 8     7    E    1    1     4    5    1
  // 0000 0111 0000 0100 0000  1000 0000 0000
  // 0      7    0   4    0     8    0     0
  // 0000 0100 0000 0000 0000  0100 0000 0000
  //  0    4    0    0     0    4     0    0

  // 0000 0010 0000 0000 0000  0010 0000 0000
  // 0000 0010 0000 0100 0000  0100 0000 0000

  write_kernel_memory( 0x21b8010 + 0x18, 0x02040400, 0, 4 );

  // EIM_WCR
  // BCM = 1   free-run BCLK
  // GBCD = 0  divide the burst clock by 1
  // add timeout watchdog after 1024 bclk cycles
  write_kernel_memory( 0x21b8090, 0x701, 0, 4 );

  // EIM_WIAR 
  // ACLK_EN = 1
  write_kernel_memory( 0x21b8094, 0x10, 0, 4 );

  printf( "resetting CS0 space to 32M and enabling 32M CS1, CS2, CS3 space.\n" );
  write_kernel_memory( 0x20e0004, 
		       (read_kernel_memory(0x20e0004, 0, 4) & 0xFFFFFFC0) |
		       0x249, 0, 4);

  printf( "done.\n" );
}



int analysis1() {
  int memfd;
  int i2cfd;
  char i2cbuf[256]; // meh too big but meh
  int i2c_read;
  int address = 0x1E; // device address of the FPGA on I2C bus 2 (hardware interface I2C3)
  int i;
  unsigned int record;
  unsigned int repeat;
  struct fpga_connection *conn;
  
  i2cfd = open("/dev/i2c-2", O_RDWR);
  if( i2cfd < 0 ) {
    perror("Unable to open /dev/i2c-2\n");
    i2cfd = 0;
    return 0;
  }
  if( ioctl( i2cfd, I2C_SLAVE, address) < 0 ) {
    perror("Unable to set I2C slave device 0x1E\n");
    return 0;
  }
  
  memfd = open("/dev/mem", O_RDWR );
  if( memfd < 0 ) {
    perror("Unable to open /dev/mem a second time\n");
    memfd = 0;
    return 0;
  }
  
  // run a quick test to make sure the interface is working
  printf( "interface test: (should see 0xaa the 0x55)\n" );
  i2cbuf[0] = 0x0; i2cbuf[1] = 0xAA;
  if( write(i2cfd, i2cbuf, 2) != 2 ) {
    perror("i2c write failed\n");
  }

  i2c_read = i2c_smbus_read_byte_data(i2cfd, 0x40);
  printf( "read back %02x\n", i2c_read & 0xFF );

  i2cbuf[0] = 0x0; i2cbuf[1] = 0x55;
  if( write(i2cfd, i2cbuf, 2) != 2 ) {
    perror("i2c write failed\n");
  }

  i2c_read = i2c_smbus_read_byte_data(i2cfd, 0x40);
  printf( "read back %02x\n", i2c_read & 0xFF );
  fflush(stdout); // make sure these tests are flushed to the screen

  /// now we are all ready. First, reset the log...
  i2cbuf[0] = 0x2; i2cbuf[1] = 0x2; // reset the log
  write(i2cfd, i2cbuf, 2);
  
  i2cbuf[0] = 0x2; i2cbuf[1] = 0x1; // set the log to run
  write(i2cfd, i2cbuf, 2);

  // kick off a test poke
  //  testWord = 0xfacebabebeefbad5LL;
  //  *cs1 = testWord;
  //  testWord = 0x5a5aa5a5ceceececLL;
  // *cs1 = testWord;
  //  *cs1 = 0xface;

  //  testWord = *cs1;
  //  testWord = *cs1;

  conn = fpga_open_connection(FAMILY_NAME);
  if (!conn)
    return -1;
  uint8_t data[4096];
  fpga_read_4k(conn, data);

  repeat = 0;
  // now read back from the log
  for( i = 0; i < 1024; i++ ) { // the log is 1024 entries long
    record = 0;
    i2c_read = i2c_smbus_read_byte_data(i2cfd,0x44);
    record = i2c_read & 0xFF;
    i2c_read = i2c_smbus_read_byte_data(i2cfd,0x45);
    record |= (i2c_read & 0xFF) << 8;
    i2c_read = i2c_smbus_read_byte_data(i2cfd,0x46);
    record |= (i2c_read & 0xFF) << 16;

    //    if( i > 480 && i < 600 ) {
    if( i > 240 && i < 1024 ) {
      if( 1  /*last != record*/ ) {
	//	if( repeat != 0 )
	//	  printf( "Previous value repeated %d times\n", repeat );
	printf( "%04d: %08x ", i, record );
	if( record & 0x400000 )
	  printf( "    " );
	else
	  printf( "  OE" );
	if( record & 0x200000 )
	  printf( " ADV" );
	else
	  printf( "    " );
	if( record & 0x100000 )
	  printf( " RD " );
	else
	  printf( " WR " );
	
	if( record & 0x080000 ) 
	  printf( "     " );
	else
	  printf( " CS1 " );
	
	printf( " %01x ", (record >> 16) & 0x7 );
	printf( " %04x\n", record & 0xFFFF );
	repeat = 0;
      } else {
	repeat++;
      }
    }
  }

  return 0;
}

int burstread() {
  volatile unsigned short *cs0;
  unsigned int *buf;
  struct fpga_connection *conn;
  int i2cfd;
  char i2cbuf[256]; // meh too big but meh
  int i2c_read;
  int address = 0x1E; // device address of the FPGA on I2C bus 2 (hardware interface I2C3)
  unsigned int record;
  unsigned int repeat;
  int i;

  i2cfd = open("/dev/i2c-2", O_RDWR);
  if( i2cfd < 0 ) {
    perror("Unable to open /dev/i2c-2\n");
    i2cfd = 0;
    return 0;
  }
  if( ioctl( i2cfd, I2C_SLAVE, address) < 0 ) {
    perror("Unable to set I2C slave device 0x1E\n");
    return 0;
  }

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return -1;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;

  cs0[F(FPGA_W_DDR3_P3_CMD)] |= 0x8000;  // set burst mode priority

  buf = calloc(256 * 1024 * 1024 / 4, 4);
  if( buf == NULL ) {
    printf( "Unable to allocate 256MB shadow area. Yell at bunnie for making crappy code.\n" );
    return -1;
  }

  printf( "last run's burst length was: %d\n", cs0[F(FPGA_R_MEAS_BURST)]);

  printf( "init readback state\n" );
  cs0[F(FPGA_W_RBK_PAGE_L)] = 0x0;
  cs0[F(FPGA_W_RBK_PAGE_H)] = 0x0; // read back starting at page 0

  cs0[F(FPGA_W_BURSTLEN)] = 0x10;  // 16-beat bursts
  
  cs0[F(FPGA_W_RBK_CTL)] = RBK_CTL_CLEAR_ERROR | RBK_CTL_INIT;
  cs0[F(FPGA_W_RBK_CTL)] = 0;
  cs0[F(FPGA_W_RBK_CTL)] = RBK_CTL_ENABLE; // enable readback machine

  /// now we are all ready. First, reset the log...
  i2cbuf[0] = 0x2; i2cbuf[1] = 0x2; // reset the log
  write(i2cfd, i2cbuf, 2);
  
  i2cbuf[0] = 0x2; i2cbuf[1] = 0x1; // set the log to run
  write(i2cfd, i2cbuf, 2);

  conn = fpga_open_connection(FAMILY_NAME);
  if (!conn)
    return -1;
  uint8_t data[8];
  fpga_read_4k(conn, data);

  repeat = 0;
  // now read back from the log
  for( i = 0; i < 1024; i++ ) { // the log is 1024 entries long
    record = 0;
    i2c_read = i2c_smbus_read_byte_data(i2cfd,0x44);
    record = i2c_read & 0xFF;
    i2c_read = i2c_smbus_read_byte_data(i2cfd,0x45);
    record |= (i2c_read & 0xFF) << 8;
    i2c_read = i2c_smbus_read_byte_data(i2cfd,0x46);
    record |= (i2c_read & 0xFF) << 16;

    //    if( i > 480 && i < 600 ) {
    if( i > 240 && i < 1024 ) {
      if( 1  /*last != record*/ ) {
	//	if( repeat != 0 )
	//	  printf( "Previous value repeated %d times\n", repeat );
	printf( "%04d: %08x ", i, record );
	if( record & 0x400000 )
	  printf( "    " );
	else
	  printf( "  OE" );
	if( record & 0x200000 )
	  printf( " ADV" );
	else
	  printf( "    " );
	if( record & 0x100000 )
	  printf( " RD " );
	else
	  printf( " WR " );
	
	if( record & 0x080000 ) 
	  printf( "     " );
	else
	  printf( " CS1 " );
	
	printf( " %01x ", (record >> 16) & 0x7 );
	printf( " %04x\n", record & 0xFFFF );
	repeat = 0;
      } else {
	repeat++;
      }
    }
  }

  return 0;

}

#define FPGA_EXP_ON  17
void power_cycle_expansion() {
  gpio_export(FPGA_EXP_ON);
  gpio_set_direction(FPGA_EXP_ON, GPIO_OUT);
  printf( "Turning expansion board off and waiting a few seconds...\n" );
  gpio_set_value(FPGA_EXP_ON, 0);
  sleep(3);
  printf( "Powering expansion board back on...\n" );
  gpio_set_value(FPGA_EXP_ON, 1);

}

void power_off_expansion() {
  gpio_export(FPGA_EXP_ON);
  gpio_set_direction(FPGA_EXP_ON, GPIO_OUT);
  printf( "Turning expansion board off and waiting a few seconds...\n" );
  gpio_set_value(FPGA_EXP_ON, 0);
  sleep(1);
}

int main(int argc, char **argv) {
  unsigned int a1, a2;
  int infile = -1; 
  int atten;

  char *prog = argv[0];
  argv++;
  argc--;

  if(!argc) {
    print_usage(prog);
    return 1;
  }

  // we're always going to need this, so make it a default call
  setup_fpga();
  setup_fpga_cs1();

  while(argc > 0) {
    if(!strcmp(*argv, "-h")) {
      argc--;
      argv++;
      print_usage(prog);
    } 
    else if(!strcmp(*argv, "-v")) {
      argc--;
      argv++;
      printf( "FPGA version code: %04hx.%04hx\n", 
	      read_kernel_memory(FPGA_R_DDR3_V_MINOR, 0, 2),
	      read_kernel_memory(FPGA_R_DDR3_V_MAJOR, 0, 2) );
    }
    else if(!strcmp(*argv, "-dt")) { // ddr3 stress test
      argc--;
      argv++;
      ddr3_test();
    }
    else if(!strcmp(*argv, "-dt2")) { // ddr3 stress test, optimized
      argc--;
      argv++;
      ddr3_test_opt();
    }
    else if(!strcmp(*argv, "-ddr3dump")) { // dump DDR3 to screen
      argc--;
      argv++;
      if( argc < 2 ) {
	printf( "usage -ddr3dump <address> <count> [file]\n" );
	return 1;
      }
      a1 = strtoul(*argv, NULL, 16);
      argc--;
      argv++;
      a2 = strtoul(*argv, NULL, 16);
      argc--;
      argv++;
      if( argc == 1 ) {
	infile = open(*argv, O_RDWR | O_CREAT, 0666 );
	if( infile == -1 ) {
	  printf("Unable to open %s\n", *argv );
	  return 1;
	}
	argc--;
	argv++;
	dump_ddr3(a1, a2, infile);
      } else {
	dump_ddr3(a1, a2, -1);
      }
    }
    else if(!strcmp(*argv, "-ddr3load")) { // load DDR3 with values from file (for romulation)
      argc--;
      argv++;
      if( argc != 1 ) {
	printf( "usage -ddr3load <file>\n" );
	return 1;
      }
      infile = open(*argv, O_RDONLY );
      if( infile == -1 ) {
	printf("Unable to open %s\n", *argv );
	return 1;
      }
      argc--;
      argv++;
      ddr3load(infile, 0);
    } else if(!strcmp(*argv, "-ddr3qload")) { // load DDR3 with values from file (just first 64k)
      argc--;
      argv++;
      if( argc != 1 ) {
	printf( "usage -ddr3qload <file>\n" );
	return 1;
      }
      infile = open(*argv, O_RDONLY );
      if( infile == -1 ) {
	printf("Unable to open %s\n", *argv );
	return 1;
      }
      argc--;
      argv++;
      ddr3load(infile, 2);
    } else if(!strcmp(*argv, "-ddr3verify")) { // verify romulator image integrity
      argc--;
      argv++;
      if( argc != 1 ) {
	printf( "usage -ddr3verify <file>\n" );
	return 1;
      }
      infile = open(*argv, O_RDONLY );
      if( infile == -1 ) {
	printf("Unable to open %s\n", *argv );
	return 1;
      }
      argc--;
      argv++;
      ddr3load(infile, 1);
    } else if(!strcmp(*argv, "-a1")) { // ever-so-helpful name for "analysis 1 -- trying to figure out if bursts to CS1 work or not"
      argc--;
      argv++;
      analysis1();
    } else if(!strcmp(*argv, "-adc_reset")) {  // reset the subsystem
      argc--;
      argv++;
      power_cycle_expansion();
    } else if(!strcmp(*argv, "-adc_off")) {  // turn off the ADC
      argc--;
      argv++;
      power_off_expansion();
    } else if(!strcmp(*argv, "-adc_pll")) {  // setup ADC PLL
      argc--;
      argv++;
      self_config_ad9520(ADC_500MHZ);
      default_adc08d1020();
    } else if(!strcmp(*argv, "-adc_cal")) {  // calibrate the ADC
      argc--;
      argv++;
      cal_adc08d1020();
    } else if(!strcmp(*argv, "-adc_calstat")) {  // setup ADC PLL
      argc--;
      argv++;
      adc08d1020_calrun_stat() ? printf( "cal ongoing\n" ) : printf( "cal done\n" );
    } else if(!strcmp(*argv, "-adc_tp")) {  // put ADC into test pattern mode
      argc--;
      argv++;
      testpattern_adc08d1020();
    } else if(!strcmp(*argv, "-adc_default")) {  // set ADC defaults
      argc--;
      argv++;
      default_adc08d1020();
    } else if(!strcmp(*argv, "-adc_dev")) {  // dev routine for ADC
      argc--;
      argv++;
      ddr3_burst_read();
      analysis1();
    } else if(!strcmp(*argv, "-burstread")) {  // simple burst readback routine
      argc--;
      argv++;
      burstread();
    } else if(!strcmp(*argv, "-afe_offset")) { 
      argc--;
      argv++;
      if( argc != 1 ) {
	printf( "usage -afe_offset 0x<code>\n" );
	return 1;
      }
      a1 = strtoul(*argv, NULL, 16);
      argc--;
      argv++;
      afe_offset(a1);
    } else if(!strcmp(*argv, "-afe_set")) {
      argc--;
      argv++;
      if( argc != 2 ) {
	printf( "usage -afe_set <filt> <atten>\n" );
	return 1;
      }
      a1 = strtoul(*argv, NULL, 10);
      argc--;
      argv++;
      a2 = strtoul(*argv, NULL, 10);
      argc--;
      argv++;
      atten = 0 - (int)a2;
      afe_setgain(1, a1, 0, atten);
    } else {
      print_usage(prog);
      return 1;
    }
  }

  return 0;
}
