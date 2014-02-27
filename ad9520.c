#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "ad9520.h"

#define AD9520_I2C_ADR  0x58

//#define DEBUG
//#define DEBUG_STANDALONE   // add a main routine for stand-alone debug

#ifdef DEBUG
# define DEBUG_PRINT(x) printf x
#else
# define DEBUG_PRINT(x) do {} while (0)
#endif

#ifdef DEBUG  // discipline to only use dump in debug mode
void dump(void *buf, int count) {
  int i = 0;
  for( i = 0; i < count; i++ ) {
    if( (i%8) == 0 ) 
      printf( "\n" );
    printf( "%02x ", ((char *) buf)[i] );
  }
}
#endif

int ad9520_write_byte( int address, unsigned char *data, int count ) {
  int i2cfd;
  char i2cbuf[4098]; 
  int slave_address = AD9520_I2C_ADR;

  struct i2c_msg msg[2];
		
  struct i2c_ioctl_rdwr_data {
    struct i2c_msg *msgs;  /* ptr to array of simple messages */              
    int nmsgs;             /* number of messages to exchange */ 
  } msgst;
  
  if( count > 4096 ) {
    printf( "ad9520_write_byte: count argument too large (%d out of 4096)\n", count );
    return 1;
  }
  
  i2cfd = open("/dev/i2c-1", O_RDWR);
  if( i2cfd < 0 ) {
    perror("Unable to open /dev/i2c-1\n");
    i2cfd = 0;
    return 1;
  }
  if( ioctl( i2cfd, I2C_SLAVE, slave_address) < 0 ) {
    perror("Unable to set I2C slave device\n" );
    printf( "Address: %02x\n", slave_address );
    return 1;
  }

#if 1
  i2cbuf[0] = ((address & 0xFF00) >> 8); i2cbuf[1] = (address & 0xFF);
  memcpy( &(i2cbuf[2]), data, count);
  // set address for read
  msg[0].addr = slave_address;
  msg[0].flags = 0; // no flag means do a write
  msg[0].len = 2 + count;
  msg[0].buf = i2cbuf;
  
#ifdef DEBUG
  dump(i2cbuf, 2+count);
#endif

  msgst.msgs = msg;	
  msgst.nmsgs = 1;

  if (ioctl(i2cfd, I2C_RDWR, &msgst) < 0){
    perror("Transaction failed\n" );
    return -1;
  }


#else  
  i2cbuf[0] = ((address & 0xFF00) >> 8); i2cbuf[1] = (address & 0xFF);
  DEBUG_PRINT(( "data: %02x %02x, count %d\n", data[0], data[1], count ));
  memcpy( &(i2cbuf[3]), data, count);
  
  DEBUG_PRINT(( "ad9520_write_byte: adr %02x %02x, dat %02x, count %d\n", 
		i2cbuf[0], i2cbuf[1], i2cbuf[3], count + 2 ));
  if( write(i2cfd, i2cbuf, count + 2) != (count + 2) ) {
    perror("i2c write failed\n");
    return 1;
  }
#endif

  close( i2cfd );
  
  return 0;
}


int ad9520_read_byte( int address, unsigned char *data, int count ) {
  int i2cfd;
  int slave_address = AD9520_I2C_ADR;
  unsigned char buff[2];

  struct i2c_msg msg[2];
		
  struct i2c_ioctl_rdwr_data {
    struct i2c_msg *msgs;  /* ptr to array of simple messages */              
    int nmsgs;             /* number of messages to exchange */ 
  } msgst;
  
  
  i2cfd = open("/dev/i2c-1", O_RDWR);
  if( i2cfd < 0 ) {
    perror("Unable to open /dev/i2c-1\n");
    i2cfd = 0;
    return -1;
  }
  if( ioctl( i2cfd, I2C_SLAVE, slave_address) < 0 ) {
    perror("Unable to set I2C slave device\n" );
    printf( "Address: %02x\n", slave_address );
    return -1;
  }

  buff[0] = ((address & 0xFF00) >> 8); buff[1] = (address & 0xFF);
  // set address for read
  msg[0].addr = slave_address;
  msg[0].flags = 0; // no flag means do a write
  msg[0].len = 2;
  msg[0].buf = (char *) buff;

  // set readback buffer
  msg[1].addr = slave_address;
  msg[1].flags = I2C_M_NOSTART | I2C_M_RD;
  //  msg[1].flags = I2C_M_RD;
  msg[1].len = count;
  msg[1].buf = (char *) data;

  msgst.msgs = msg;	
  msgst.nmsgs = 2;

  if (ioctl(i2cfd, I2C_RDWR, &msgst) < 0){
    perror("Transaction failed\n" );
    return -1;
  }

  close( i2cfd );
  
  return 0;
}

int decode_ad9520_id(unsigned char id, char *idbuf) {
  int retval = 0;

  switch(id) {
  case 0x20:
    sprintf( idbuf, "%s", "AD9520-0\n" );
    break;
  case 0x60:
    sprintf( idbuf, "%s", "AD9520-1\n" );
    break;
  case 0xA0:
    sprintf( idbuf, "%s", "AD9520-2\n" );
    break;
  case 0x61:
    sprintf( idbuf, "%s", "AD9520-3\n" );
    break;
  case 0xE1:
    sprintf( idbuf, "%s", "AD9520-4\n" );
    break;
  case 0xE0:
    sprintf( idbuf, "%s", "AD9520-5\n" );
    break;
  default:
    sprintf( idbuf, "%s", "unknown\n" );
    retval = -1;
  }

  return retval;
}

int load_ad9520_from_file(char *fname) {
  int fd;
  unsigned char cache[4096]; // only 2820 strictly needed
  int readcount;
  
  fd = open( fname, O_RDONLY );
  if( fd < 0 ) {
    perror("Can't open file ");
    printf( "%s\n", fname );
    return -1;
  }
  
  readcount = read( fd, cache, 2820 );
  if( readcount != 2820 ) {
    printf( "Error: read only %d out of 2820 expected bytes in AD9520 PLL config file\n", readcount );
    close( fd );
    return -1;
  }
  
  // blast out bytes in chunks based on active memory areas in AD9520
  // determine the relevant chunks by looking at address space locations in .h file

  // chunk 1: memory region bounded by PFD_CHARGE_PUMP and PLL_READBACK_READ_ONLY, e.g. 0x010-0x01f
  DEBUG_PRINT( ("program PFD & PLL\n" ));
  ad9520_write_byte(AD9520_PFD_CHARGE_PUMP, &(cache[AD9520_PFD_CHARGE_PUMP]),
		    AD9520_PLL_READBACK_READ_ONLY - AD9520_PFD_CHARGE_PUMP);

  DEBUG_PRINT( ("program output control\n" ));
  // next region is from 0x0f0-0x0fc, OUT0_CONTROL - ENABLE_OUTPUT_ON_CSDLD
  ad9520_write_byte(AD9520_OUT0_CONTROL, &(cache[AD9520_OUT0_CONTROL]),
		    AD9520_ENABLE_OUTPUT_ON_CSDLD_MSB - AD9520_OUT0_CONTROL + 1);

  DEBUG_PRINT( ("program output mode and dividers\n" ));
  // next region is LVPECL driver configs
  ad9520_write_byte(AD9520_DIVIDER_0_PECL, &(cache[AD9520_DIVIDER_0_PECL]),
		    AD9520_DIVIDER_3_BYPASS - AD9520_DIVIDER_0_PECL + 1);

  DEBUG_PRINT( ("program VCO divider\n" ));
  // next region is VCO divider
  ad9520_write_byte(AD9520_VCO_DIVIDER, &(cache[AD9520_VCO_DIVIDER]),
		    AD9520_INPUT_CLKS - AD9520_VCO_DIVIDER + 1);

  DEBUG_PRINT( ("program system power control\n" ));
  // next region is system power control (it's just one register)
  ad9520_write_byte(AD9520_POWER_DOWN_AND_SYNC, &(cache[AD9520_POWER_DOWN_AND_SYNC]),
		    AD9520_POWER_DOWN_AND_SYNC - AD9520_POWER_DOWN_AND_SYNC + 1);
  
  DEBUG_PRINT( ("commit data to PLL\n" ));
  // finally, commit data to PLL
  cache[AD9520_IO_UPDATE] = 1; // set to 1 to update, 0 just for testing (no commit)
  ad9520_write_byte(AD9520_IO_UPDATE, &(cache[AD9520_IO_UPDATE]), 1);

  DEBUG_PRINT( ("calibrate VCO\n" ));
  // do a VCO calibrate
  cache[AD9520_PLL_CTRL_3] &= 0xFE; // set bit 0 to 0 to start calibration
  ad9520_write_byte(AD9520_PLL_CTRL_3, &(cache[AD9520_PLL_CTRL_3]), 1);
  cache[AD9520_IO_UPDATE] = 1;  // send an update
  ad9520_write_byte(AD9520_IO_UPDATE, &(cache[AD9520_IO_UPDATE]), 1);
  cache[AD9520_PLL_CTRL_3] |= 0x01; // now set to 1 to force the cal immediately
  ad9520_write_byte(AD9520_PLL_CTRL_3, &(cache[AD9520_PLL_CTRL_3]), 1);
  cache[AD9520_IO_UPDATE] = 1;  // send an update
  ad9520_write_byte(AD9520_IO_UPDATE, &(cache[AD9520_IO_UPDATE]), 1);

  close( fd );
  return 0;
}

//// we do this, instead of reading in a binary blob generated by the ADI tools
//// because there are bugs in the ADI tools I can't fix because it's closed source
//// and ADI offers no support :-(
int self_config_ad9520(int speed) {
  // if speed = 0, set to 500 MHz ADC clock; if speed = 1, set to 1GHz clock
  unsigned char cache[4096]; // only 2820 strictly needed
  int i;
  unsigned char data;
  unsigned char idbuf[64];
  
  ad9520_read_byte( AD9520_PART_ID, &data, 1 );
  printf( "AD9520 part ID: %02x = ", data );

  decode_ad9520_id(data, (char *) idbuf);
  printf( "%s", idbuf );

  for( i = 0; i < 4096; i++ ) {
    cache[i] = 0;
  }

  cache[AD9520_PFD_CHARGE_PUMP] = 0x3c;
  // 0    PFD polarity = positive
  // 011  CP current = 2.4mA
  // 11   CP mode normal
  // 00   PLL power down is normal operation

  cache[AD9520_R_COUNTER_LSB] = 0x01;
  cache[AD9520_R_COUNTER_MSB] = 0x00;
  // R counter = 1

  cache[AD9520_A_COUNTER] = 16;
  // A counter = 16 (counter is lower 6 bit only)
  
  cache[AD9520_B_COUNTER_LSB] = 23;
  cache[AD9520_B_COUNTER_MSB] = 0x00;
  // B counter = 23 (counter is 14 bits)

  cache[AD9520_PLL_CTRL_1] = 0x04;
  // 0    CP pin set to normal operation
  // 0    R counter is not held in reset
  // 0    A and B counters are not held in reset
  // 0    R, A, and B counters are not held in reset
  // 0    B counter is not bypassed
  // 100  Prescaler is in dual-modulus mode, div 8/9
  
  cache[AD9520_PLL_CTRL_2] = 0x04;
  // .0001 01  PFD up pulse
  // .0001 00  prescaler output
  // .0000 10  status pin is R divider output, after delay
  // .0000 01  status pin is N divider output, after delay
  // 00       anti-backlash pulse width is 2.9ns

  cache[AD9520_PLL_CTRL_3] = 0x86;
  // 1   enable DC offset on CMOS ref input
  // 00  5 PFD cycles to determine lock
  // 0   digital lock window detect is high range, 3.5ns
  // 0   normal lock detect operation
  // 11  VCO clock cal divider 16
  // 0   VCO calibrate now

  cache[AD9520_PLL_CTRL_4] = 0x00;
  // 00   do nothing with SYNC_N
  // 000  R path delay = 0ns
  // 000  N path delay = 0ns
  
  cache[AD9520_PLL_CTRL_5] = 0x00;
  // 0       divde-by-4 disabled on STATUS
  // 0       frequency valid if freq above 1.02MHz
  // 000000  LD pin in normal operation

  cache[AD9520_PLL_CTRL_6] = 0x00;
  // 0     VCO freq mon off
  // 0     ref2 freq mon off
  // 0     ref1 freq mon off
  // 00000 REFMON is set to GND
  
  cache[AD9520_PLL_CTRL_7] = 0x02;
  // 0   enable switchover deglitch
  // 0   select REF1
  // 0   use this register to select reference clock (as opposed to pin strap)
  // 0   manual reference switchover
  // 0   return to ref1 when ref1 status is good again
  // 0   ref2 power off
  // 1   ref1 power on
  // 0   single-ended reference mode

  cache[AD9520_PLL_CTRL_8] = 0x00;
  // 0   status pin is controlled by STATUS register (not EEPROM)
  // 0   crystal oscillator is disabled
  // 0   refclk doubler disabled
  // 0   PLL status register enabled
  // 0   disable LD pin comparator
  // -
  // 0   automatic holdover
  // 0   holdover disabled

  cache[AD9520_PLL_CTRL_9] = 0x00;
  // ---  unused
  // 00   select channel divider 0 for zero-delay path
  // 0    enables internal delay if next bit is 1
  // 0    disable zero delay function
  // -

  cache[AD9520_OUT0_CONTROL] = 0x65;
  // 0   LVPECL
  // 11  OUTnA on, OUTnB on
  // 00  LVPECL -> OUTnA non-inverting, OUTnB inverting
  // 10  LVPECL level is 0.78V
  // 1   channel in safe-powerdown
  cache[AD9520_OUT1_CONTROL] = 0x65;
  cache[AD9520_OUT2_CONTROL] = 0x65;

  cache[AD9520_OUT3_CONTROL] = 0x64;
  // 0   LVPECL
  // 11  OUTnA on, OUTnB on
  // 00  LVPECL -> OUTnA non-inverting, OUTnB inverting
  // 10  LVPECL level is 0.78V
  // 0   channel in normal operation
  cache[AD9520_OUT4_CONTROL] = 0x65;
  cache[AD9520_OUT5_CONTROL] = 0x65;

  cache[AD9520_OUT6_CONTROL] = 0x65;
  cache[AD9520_OUT7_CONTROL] = 0x65;
  cache[AD9520_OUT8_CONTROL] = 0x65;

  cache[AD9520_OUT9_CONTROL] = 0x65;
  cache[AD9520_OUT10_CONTROL] = 0x65;
  cache[AD9520_OUT11_CONTROL] = 0x65;

  cache[AD9520_ENABLE_OUTPUT_ON_CSDLD_LSB] = 0x00;  // not affected by CSDLD
  cache[AD9520_ENABLE_OUTPUT_ON_CSDLD_MSB] = 0x00;
  
  cache[AD9520_DIVIDER_0_PECL] = 0x11;  // fpga clock channel
  // 0001  divider 0 is low for 2 cycles 
  // 0001  divider 0 is high for 2 cycles
  // total: divide-by-2 off of 1GHz ref = 500MHz
  
  cache[AD9520_DIVIDER_0_BYPASS] = 0x00;
  // 0     use divider
  // 0     obey chip-level SYNC
  // 0     divider output forced to low
  // 0     divider starts low
  // 0000  phase offset

  cache[AD9520_DIVIDER_0_POWER] = 0x04;
  // ----- unused
  // 1     powered down -- FPGA gets clock from ADC mirror
  // 0     OUT0, OUT1, OUT2 connected to divider 0
  // 0     duty-cycle correction enabled


  cache[AD9520_DIVIDER_1_PECL] = 0x00;  // adc clock channel
  // 0000  divider 0 is low for 1 cycles 
  // 0000  divider 0 is high for 1 cycles
  cache[AD9520_DIVIDER_1_BYPASS] = 0x80;
  // 1     bypass divider
  // 0     obey chip-level SYNC
  // 0     divider output forced to low
  // 0     divider starts low
  // 0000  phase offset
  cache[AD9520_DIVIDER_1_POWER] = 0x01;
  // ----- unused
  // 0     normal operation
  // 0/1   OUT3, OUT4, OUT5 connected to 1 = VCO output / 0 = divider 1
  // 1     duty-cycle correction disabled


  cache[AD9520_DIVIDER_2_PECL] = 0x11;
  cache[AD9520_DIVIDER_2_BYPASS] = 0x00;
  cache[AD9520_DIVIDER_2_POWER] = 0x04;
  // ----- unused
  // 1     powered down
  // 0     OUT0, OUT1, OUT2 connected to divider 2
  // 0     duty-cycle correction enabled

  cache[AD9520_DIVIDER_3_PECL] = 0x00;
  cache[AD9520_DIVIDER_3_BYPASS] = 0x00;
  cache[AD9520_DIVIDER_3_POWER] = 0x04;

  if( speed )
    cache[AD9520_VCO_DIVIDER] = 0x00; 
  else
    cache[AD9520_VCO_DIVIDER] = 0x02; 
  // 000   divide by 2 
  // 001   divide by 3
  // 010   divide by 4
  // 110   bypass (divide by 1)

  cache[AD9520_INPUT_CLKS] = 0x22;
  // ---   unused - but tool generates 001
  // 0     clock input section is in normal operation
  // 0     VCO clock interface normal operation
  // 0     VCO and clock input not powered down
  // 1     selects VCO as input to VCO divider
  // 0     use VCO divider

  cache[AD9520_POWER_DOWN_AND_SYNC] = 0x00;
  // ----  unused
  // 0     enable antiruntpulse circuitry
  // 0     normal operation of sync function
  // 0     normal operation of reference distribution
  // 0     soft sync not asserted
  
  cache[AD9520_IO_UPDATE] = 0;  // just commit, then update later

  DEBUG_PRINT( ("reset AD9520\n" ));
  cache[AD9520_SERIAL_PORT_CONFIG] = 0x22;
  ad9520_write_byte(AD9520_SERIAL_PORT_CONFIG, &(cache[AD9520_SERIAL_PORT_CONFIG]), 1);
  cache[AD9520_SERIAL_PORT_CONFIG] = 0x00;
  usleep(10000); // give it 10 ms to recover from reset
  
  // blast out bytes in chunks based on active memory areas in AD9520
  // determine the relevant chunks by looking at address space locations in .h file

  // chunk 1: memory region bounded by PFD_CHARGE_PUMP and PLL_READBACK_READ_ONLY, e.g. 0x010-0x01f
  DEBUG_PRINT( ("program PFD & PLL\n" ));
  ad9520_write_byte(AD9520_PFD_CHARGE_PUMP, &(cache[AD9520_PFD_CHARGE_PUMP]),
		    AD9520_PLL_READBACK_READ_ONLY - AD9520_PFD_CHARGE_PUMP);

  DEBUG_PRINT( ("program output control\n" ));
  // next region is from 0x0f0-0x0fc, OUT0_CONTROL - ENABLE_OUTPUT_ON_CSDLD
  ad9520_write_byte(AD9520_OUT0_CONTROL, &(cache[AD9520_OUT0_CONTROL]),
		    AD9520_ENABLE_OUTPUT_ON_CSDLD_MSB - AD9520_OUT0_CONTROL + 1);

  DEBUG_PRINT( ("program output mode and dividers\n" ));
  // next region is LVPECL driver configs
  ad9520_write_byte(AD9520_DIVIDER_0_PECL, &(cache[AD9520_DIVIDER_0_PECL]),
		    AD9520_DIVIDER_3_POWER - AD9520_DIVIDER_0_PECL + 1);

  DEBUG_PRINT( ("program VCO divider\n" ));
  // next region is VCO divider
  ad9520_write_byte(AD9520_VCO_DIVIDER, &(cache[AD9520_VCO_DIVIDER]),
		    AD9520_INPUT_CLKS - AD9520_VCO_DIVIDER + 1);

  DEBUG_PRINT( ("program system power control\n" ));
  // next region is system power control (it's just one register)
  ad9520_write_byte(AD9520_POWER_DOWN_AND_SYNC, &(cache[AD9520_POWER_DOWN_AND_SYNC]),
		    AD9520_POWER_DOWN_AND_SYNC - AD9520_POWER_DOWN_AND_SYNC + 1);
  
  DEBUG_PRINT( ("commit data to PLL\n" ));
  // finally, commit data to PLL
  cache[AD9520_IO_UPDATE] = 1; // set to 1 to update, 0 just for testing (no commit)
  ad9520_write_byte(AD9520_IO_UPDATE, &(cache[AD9520_IO_UPDATE]), 1);

  DEBUG_PRINT( ("calibrate VCO\n" ));
  // do a VCO calibrate
  cache[AD9520_PLL_CTRL_3] &= 0xFE; // set bit 0 to 0 to start calibration
  ad9520_write_byte(AD9520_PLL_CTRL_3, &(cache[AD9520_PLL_CTRL_3]), 1);
  cache[AD9520_IO_UPDATE] = 1;  // send an update
  ad9520_write_byte(AD9520_IO_UPDATE, &(cache[AD9520_IO_UPDATE]), 1);
  cache[AD9520_PLL_CTRL_3] |= 0x01; // now set to 1 to force the cal immediately
  ad9520_write_byte(AD9520_PLL_CTRL_3, &(cache[AD9520_PLL_CTRL_3]), 1);
  cache[AD9520_IO_UPDATE] = 1;  // send an update
  ad9520_write_byte(AD9520_IO_UPDATE, &(cache[AD9520_IO_UPDATE]), 1);

  // wait for cal to finish
  data = 0;
  i = 0;
  do {
    ad9520_read_byte( AD9520_PLL_READBACK_READ_ONLY, &data, 1 );
    i++;
  } while( ((data & 0x40) == 0) && (i < 1000) );
  if( i >= 1000 ) {
    printf( "Calibration timed out!\n" );
    return 1;
  }

  return 0;
}

void decode_ad9520_status(unsigned char status) {
  printf( "\nAD9520 status register decode value %02x:\n", status );
  if( status & 0x40 ) printf( " VCO cal finished\n" ); else printf( " VCO cal unfinished\n" );
  if( status & 0x20 ) printf( " holdover active\n" ); else printf( " holdover not active\n" );
  if( status & 0x10 ) printf( " REF2 selected\n" ); else printf( " REF1 selected\n" );
  if( status & 0x08 ) printf( " VCO freq > thresh\n" ); else printf( " VCO freq < thrtesh\n" );
  if( status & 0x04 ) printf( " REF2 freq > thresh\n" ); else printf( " REF2 freq < thrtesh\n" );
  if( status & 0x02 ) printf( " REF1 freq > thresh\n" ); else printf( " REF1 freq < thrtesh\n" );
  if( status & 0x01 ) printf( " PLL locked\n" ); else printf( " PLL unlocked\n" );
}

#if DEBUG_STANDALONE
int main() {
  unsigned char data[4096];
  int i;
  int readcount;
  unsigned char idbuf[32];
  unsigned char status;

  printf( "Hello! Some initial tests first...\n" );

  for( i = 0; i < 32; i++ )
    data[i] = 0;

  ad9520_read_byte( AD9520_PART_ID, data, 1 );
  printf( "Part ID: %02x = ", data[0] );

  decode_ad9520_id(data[0], idbuf);
  printf( "%s", idbuf );

  printf( "\nconfiguring AD9520 for 500 MHz\n" );
  self_config_ad9520(ADC_500MHZ);

  printf( "verification of cached data\n" );
  data[0] = 1;
  ad9520_write_byte(AD9520_READBACK_CONTROL, data, 1 ); // read back only active registers
  
  readcount = ad9520_read_byte( 0x00, data, 32 );
  for( i = 0; i < 32; i++ ) {
    if( (i % 8) == 0 )
      printf( "\n%03x: ", i );
    printf( "%02x ", data[i] );
  }

  decode_ad9520_status(data[31]);

  readcount = ad9520_read_byte( 0xF0, data, 32 );
  for( i = 0xf0; i < (32 + 0xf0); i++ ) {
    if( (i % 8) == 0 )
      printf( "\n%03x: ", i );
    printf( "%02x ", data[i - 0xf0] );
  }
  
  readcount = ad9520_read_byte( 0x190, data, 32 );
  for( i = 0x190; i < (32 + 0x190); i++ ) {
    if( (i % 8) == 0 )
      printf( "\n%03x: ", i );
    printf( "%02x ", data[i - 0x190] );
  }
  
  readcount = ad9520_read_byte( 0x1E0, data, 8 );
  for( i = 0x1E0; i < (8 + 0x1E0); i++ ) {
    if( (i % 8) == 0 )
      printf( "\n%03x: ", i );
    printf( "%02x ", data[i - 0x1e0] );
  }

  readcount = ad9520_read_byte( 0x230, data, 8 );
  for( i = 0x230; i < (8 + 0x230); i++ ) {
    if( (i % 8) == 0 )
      printf( "\n%03x: ", i );
    printf( "%02x ", data[i - 0x230] );
  }
  printf( "\n" );

  readcount = ad9520_read_byte( AD9520_PLL_READBACK_READ_ONLY, data, 1 );
  decode_ad9520_status(data[0]);

  data[0] = 0;

  ad9520_write_byte(AD9520_READBACK_CONTROL, data, 1 ); // set back to buffered readbcak

}
#endif
