#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "adc08d1020.h"

#define ADC08D1020_I2C_ADR  0x1E  // same as FPGA address

//#define DEBUG

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

// returns 1 if cal is running
int adc08d1020_calrun_stat() {
  int i2cfd;
  char i2cbuf[4]; 
  char data;
  int slave_address = ADC08D1020_I2C_ADR;

  struct i2c_msg msg[2];
		
  struct i2c_ioctl_rdwr_data {
    struct i2c_msg *msgs;  /* ptr to array of simple messages */              
    int nmsgs;             /* number of messages to exchange */ 
  } msgst;
  
  i2cfd = open("/dev/i2c-2", O_RDWR);
  if( i2cfd < 0 ) {
    perror("Unable to open /dev/i2c-2\n");
    i2cfd = 0;
    return 1;
  }
  if( ioctl( i2cfd, I2C_SLAVE, slave_address) < 0 ) {
    perror("Unable to set I2C slave device\n" );
    printf( "Address: %02x\n", slave_address );
    return 1;
  }

  //////// read back the status register
  i2cbuf[0] = FPGA_ADC_STAT;

  msg[0].addr = slave_address;
  msg[0].flags = 0; // no flag means do a write
  msg[0].len = 1;
  msg[0].buf = i2cbuf;

  // set readback buffer
  msg[1].addr = slave_address;
  msg[1].flags = I2C_M_NOSTART | I2C_M_RD;
  msg[1].len = 1;
  msg[1].buf = &data;

  msgst.msgs = msg;	
  msgst.nmsgs = 2;
  
  if (ioctl(i2cfd, I2C_RDWR, &msgst) < 0){
    perror("Transaction failed\n" );
    return -1;
  }
  
  return ((data & 0x2) ? 0 : 1);  // note inversion, since level converter in hw is inverting

}

int adc08d1020_write_reg( int address, unsigned short data ) {
  int i2cfd;
  char i2cbuf[4]; 
  int slave_address = ADC08D1020_I2C_ADR;

  struct i2c_msg msg[2];
		
  struct i2c_ioctl_rdwr_data {
    struct i2c_msg *msgs;  /* ptr to array of simple messages */              
    int nmsgs;             /* number of messages to exchange */ 
  } msgst;
  
  i2cfd = open("/dev/i2c-2", O_RDWR);
  if( i2cfd < 0 ) {
    perror("Unable to open /dev/i2c-2\n");
    i2cfd = 0;
    return 1;
  }
  if( ioctl( i2cfd, I2C_SLAVE, slave_address) < 0 ) {
    perror("Unable to set I2C slave device\n" );
    printf( "Address: %02x\n", slave_address );
    return 1;
  }

  //////// write data, address, and commit using multi-byte write
  i2cbuf[0] = FPGA_ADC_WDATA_LSB;
  i2cbuf[1] = data & 0xFF;
  i2cbuf[2] = (data >> 8) & 0xFF;
  i2cbuf[3] = (address & 0xF) | 0x10;

  msg[0].addr = slave_address;
  msg[0].flags = 0; // no flag means do a write
  msg[0].len = 4;
  msg[0].buf = i2cbuf;
  
#ifdef DEBUG
  dump(i2cbuf, 4);
#endif

  msgst.msgs = msg;	
  msgst.nmsgs = 1;

  if (ioctl(i2cfd, I2C_RDWR, &msgst) < 0){
    perror("Transaction failed\n" );
    return -1;
  }

  //////// now clear the commit, it's not self-clearing
  i2cbuf[0] = FPGA_ADC_COMMIT_ADR;
  i2cbuf[1] = address & 0xF;

  msg[0].addr = slave_address;
  msg[0].flags = 0; // no flag means do a write
  msg[0].len = 2;
  msg[0].buf = i2cbuf;

  msgst.msgs = msg;	
  msgst.nmsgs = 1;

  if (ioctl(i2cfd, I2C_RDWR, &msgst) < 0){
    perror("Transaction failed\n" );
    return -1;
  }

  close( i2cfd );
  
  return 0;
}

void default_adc08d1020() {
  // set config
  adc08d1020_write_reg( ADC08D1020_CONFIG, 0xB3FF );
  // 1
  // 0
  // 1 nSD -- set for DCLK + OR output
  // 1 DCS -- duty cycle stabilizer on
  // 0 DCP -- DDR data changes on edges, set to 1 to change mid-phase
  // 0 nDE -- DDR enable, write 0 to enable DDR mode
  // 1 OV -- 1=output voltage set to 720mVp-p; 0=510mVp-p
  // 1 OED -- demux control - set non-demux mode, so the delay outputs are not used
  // 1111_1111
  
  // set extended config
  adc08d1020_write_reg( ADC08D1020_EXTCONFIG, 0x3FF );
  // 0  - no test pattern, set 1 for TP
  // 0  - keep resistor trim on
  // 0  - normal dual channel mode (set 1 for DES)
  // 0  - input select -- both on I channel if in DES
  // 0  
  // 0  - DLF -- set only if ADC running < 900MHz
  // 11_1111_1111

}

void cal_adc08d1020() {

  // initiate a calibration
  adc08d1020_write_reg( ADC08D1020_EXTCONFIG, 0x43FF ); // RTD has to be high for cal to run
  adc08d1020_write_reg( ADC08D1020_CAL, 0xFFFF ); // set the cal bit
  // clear the bit, it's not self clearing
  adc08d1020_write_reg( ADC08D1020_CAL, 0x7FFF ); // clear the cal bit
  adc08d1020_write_reg( ADC08D1020_EXTCONFIG, 0x03FF ); // reset RTD
  
  while( adc08d1020_calrun_stat() )  // wait for cal to finish
    ;

}

void testpattern_adc08d1020() {
  // set extended config
  adc08d1020_write_reg( ADC08D1020_EXTCONFIG, 0x83FF );
  // 1  - no test pattern, set 1 for TP
  // 0  - keep resistor trim on
  // 0  - normal dual channel mode (set 1 for DES)
  // 0  - input select -- both on I channel if in DES
  // 0  
  // 0  - DLF -- set only if ADC running < 900MHz
  // 11_1111_1111
}


#if DEBUG
int main() {
  unsigned char data[4096];
  int i;
  int readcount;
  unsigned char idbuf[32];
  unsigned char status;

  printf( "Hello! Some initial tests first...\n" );


}
#endif
