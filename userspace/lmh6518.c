#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "lmh6518.h"

#define LMH6518_I2C_ADR  0x28

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

int lmh6518_write_byte( unsigned short data ) {
  int i2cfd;
  char i2cbuf[2]; 
  int slave_address = -1;

  struct i2c_msg msg[2];
		
  struct i2c_ioctl_rdwr_data {
    struct i2c_msg *msgs;  /* ptr to array of simple messages */              
    int nmsgs;             /* number of messages to exchange */ 
  } msgst;
  
  slave_address = LMH6518_I2C_ADR;
  
  i2cfd = open("/dev/i2c-1", O_RDWR);
  if( i2cfd < 0 ) {
    perror("Unable to open /dev/i2c-1\n");
    i2cfd = 0;
    return 1;
  }
  if( ioctl( i2cfd, I2C_SLAVE, slave_address) < 0 ) {
    perror("Unable to set I2C slave device\n" );
    printf( "Address: %02x\n", slave_address );
    close( i2cfd );
    return 1;
  }

  i2cbuf[0] = ((data & 0xFF00) >> 8); i2cbuf[1] = (data & 0xFF);
  // set address for read
  msg[0].addr = slave_address;
  msg[0].flags = 0; // no flag means do a write
  msg[0].len = 2;
  msg[0].buf = i2cbuf;
  
#ifdef DEBUG
  dump(i2cbuf, 2);
#endif

  msgst.msgs = msg;	
  msgst.nmsgs = 1;

  if (ioctl(i2cfd, I2C_RDWR, &msgst) < 0){
    perror("Transaction failed\n" );
    close( i2cfd );
    return -1;
  }

  close( i2cfd );
  
  return 0;
}

/*
  auxpwr - turn the aux channel (for trigger) on or off. 1 = on, 0 = off
  filter - set filter bandwidth, in MHz; 0 = infinity
  premap - preamp in low or high gain. 0 = low gain, 1 = high gain
  attenuation - set attenuation, in dB (minus values expected, from 0 to minus 20 dB)
 */

void afe_setgain(unsigned int auxpwr, unsigned int filter, unsigned int preamp, int attenuation ) {
  unsigned short data = 0;

  if( auxpwr )
    data |= (LMH6518_AUXPWR_ON << LMH6518_AUXPWR_BIT);
  else
    data |= (LMH6518_AUXPWR_OFF << LMH6518_AUXPWR_BIT);

  if( filter == 0 || filter > 750 )
    data |= (LMH6518_FILT_FULL << LMH6518_FILT_BIT);
  else if( filter <= 20 )
    data |= (LMH6518_FILT_20MHZ << LMH6518_FILT_BIT);
  else if( filter <= 100 )
    data |= (LMH6518_FILT_100MHZ << LMH6518_FILT_BIT);
  else if( filter <= 200 )
    data |= (LMH6518_FILT_200MHZ << LMH6518_FILT_BIT);
  else if( filter <= 350 )
    data |= (LMH6518_FILT_350MHZ << LMH6518_FILT_BIT);
  else if( filter <= 650 )
    data |= (LMH6518_FILT_650MHZ << LMH6518_FILT_BIT);
  else if( filter <= 750 )
    data |= (LMH6518_FILT_750MHZ << LMH6518_FILT_BIT);
  else
    data |= (LMH6518_FILT_FULL << LMH6518_FILT_BIT);

  if( preamp )
    data |= (LMH6518_PREAMP_HG << LMH6518_PREAMP_BIT);
  else
    data |= (LMH6518_PREAMP_LG << LMH6518_PREAMP_BIT);

  if( attenuation >= 0 )
    data |= (LMH6518_ATTEN_0DB << LMH6518_ATTEN_BIT);
  else if( attenuation >= -2 )
    data |= (LMH6518_ATTEN_2DB << LMH6518_ATTEN_BIT);
  else if( attenuation >= -4 )
    data |= (LMH6518_ATTEN_4DB << LMH6518_ATTEN_BIT);
  else if( attenuation >= -6 )
    data |= (LMH6518_ATTEN_6DB << LMH6518_ATTEN_BIT);
  else if( attenuation >= -8 )
    data |= (LMH6518_ATTEN_8DB << LMH6518_ATTEN_BIT);
  else if( attenuation >= -10 )
    data |= (LMH6518_ATTEN_10DB << LMH6518_ATTEN_BIT);
  else if( attenuation >= -12 )
    data |= (LMH6518_ATTEN_12DB << LMH6518_ATTEN_BIT);
  else if( attenuation >= -14 )
    data |= (LMH6518_ATTEN_14DB << LMH6518_ATTEN_BIT);
  else if( attenuation >= -16 )
    data |= (LMH6518_ATTEN_16DB << LMH6518_ATTEN_BIT);
  else if( attenuation >= -18 )
    data |= (LMH6518_ATTEN_18DB << LMH6518_ATTEN_BIT);
  else
    data |= (LMH6518_ATTEN_20DB << LMH6518_ATTEN_BIT);

  printf ("writing byte %04x to LMH6518\n", data );
  lmh6518_write_byte( data );
  
}


#ifdef DEBUG_STANDALONE
int main() {
  unsigned short data;
  int i;
  int readcount;
  unsigned char status;

  lmh6518_write_byte( 0x0009 );

}
#endif
