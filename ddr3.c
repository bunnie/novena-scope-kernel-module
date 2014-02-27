#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>

#include "ddr3.h"
#include "novena-scope.h"

static int fd = 0;
static int   *mem_32 = 0;


void ddr3_test() {
  unsigned int *testdat;
  unsigned int readback[DDR3_FIFODEPTH];
  int i;
  int burstaddr = 0;
  unsigned int data;
  int iters = 0;
  int offset;
  unsigned int rv;
  unsigned int arg = 0;

  srand(time(NULL)); // seed the random generator

  testdat = calloc(DDR3_SIZE, sizeof(unsigned int));
  if( testdat == NULL ) {
    printf( "Can't allocate test array.\n" );
    return;
  }

  while(1) {
    // dummy writes to clear any previous data in the queue -- caution, writes to "wherever"!
    while( !(read_kernel_memory(FPGA_R_DDR3_P2_STAT, 0, 2) & 4) ) {
      write_kernel_memory( FPGA_W_DDR3_P2_CMD, 0x008 | PULSE_GATE_MASK, 0, 2 );
      write_kernel_memory( FPGA_W_DDR3_P2_CMD, 0x000 | PULSE_GATE_MASK, 0, 2 );
    }
    // dummy reads to clear any previous data in the queue
    while( !(read_kernel_memory(FPGA_R_DDR3_P3_STAT, 0, 2) & 4) ) {
      write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x010, 0, 2 );
      write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x000, 0, 2 );
    }

    putchar('+'); fflush(stdout);
    for( i = 0; i < DDR3_SIZE; i++ ) {
      testdat[i] = (unsigned int) rand();
    }
    
    offset = 0;
    burstaddr = 0;
    write_kernel_memory( FPGA_W_DDR3_P2_LADR + offset, ((burstaddr * 4) & 0xFFFF), 0, 2 );
    write_kernel_memory( FPGA_W_DDR3_P2_HADR + offset, ((burstaddr * 4) >> 16) & 0xFFFF, 0, 2 );

    putchar('!'); fflush(stdout);
    while( burstaddr < DDR3_SIZE ) {
      while( !(read_kernel_memory(FPGA_R_DDR3_P2_STAT, 0, 2) & 4) ) {
	putchar('-'); fflush(stdout);  // wait for write queue to be empty
      }
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	write_kernel_memory( FPGA_W_DDR3_P2_LDAT, (testdat[burstaddr + i] & 0xFFFF), 0, 2 );
	write_kernel_memory( FPGA_W_DDR3_P2_HDAT, (testdat[burstaddr + i] >> 16) & 0xFFFF, 0, 2 );
	//	write_kernel_memory( FPGA_W_DDR3_P2_WEN, 0x10, 0, 2 );
	//	write_kernel_memory( FPGA_W_DDR3_P2_WEN, 0x00, 0, 2 );
      }
      if( (read_kernel_memory(FPGA_R_DDR3_P2_STAT, 0, 2) >> 8) != DDR3_FIFODEPTH ) {
	printf( "z%d\n", (read_kernel_memory(FPGA_R_DDR3_P2_STAT, 0, 2) >> 8) );
	putchar('z'); fflush(stdout);
      }
      arg = ((DDR3_FIFODEPTH - 1) << 4);
      write_kernel_memory( FPGA_W_DDR3_P2_CMD, arg | PULSE_GATE_MASK, 0, 2 );
      arg |= 8;
      write_kernel_memory( FPGA_W_DDR3_P2_CMD, arg | PULSE_GATE_MASK, 0, 2 );
      write_kernel_memory( FPGA_W_DDR3_P2_CMD, 0x000 | PULSE_GATE_MASK, 0, 2 );
      burstaddr += DDR3_FIFODEPTH;
      write_kernel_memory( FPGA_W_DDR3_P2_LADR + offset, ((burstaddr * 4) & 0xFFFF), 0, 2 );
      write_kernel_memory( FPGA_W_DDR3_P2_HADR + offset, ((burstaddr * 4) >> 16) & 0xFFFF, 0, 2 );
    }
    
    offset = 0x10; // accessing port 3 (read port)
    burstaddr = 0;
    write_kernel_memory( FPGA_W_DDR3_P2_LADR + offset, ((burstaddr * 4) & 0xFFFF), 0, 2 );
    write_kernel_memory( FPGA_W_DDR3_P2_HADR + offset, ((burstaddr * 4) >> 16) & 0xFFFF, 0, 2 );

    putchar('.'); fflush(stdout);

    while( burstaddr < DDR3_SIZE ) {
#if 1
      arg = ((DDR3_FIFODEPTH - 1) << 4) | 1;
      write_kernel_memory( FPGA_W_DDR3_P3_CMD, arg | PULSE_GATE_MASK, 0, 2 );
      arg |= 0x8;
      write_kernel_memory( FPGA_W_DDR3_P3_CMD, arg | PULSE_GATE_MASK, 0, 2 );
      arg &= ~0x8;
      write_kernel_memory( FPGA_W_DDR3_P3_CMD, arg | PULSE_GATE_MASK, 0, 2 );
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	while( (read_kernel_memory(FPGA_R_DDR3_P3_STAT, 0, 2) & 4) ) {
	  putchar('i'); fflush(stdout);// wait for queue to become full before reading
	}
	rv = read_kernel_memory(FPGA_R_DDR3_P3_LDAT, 0, 2);
	data = ((unsigned int) rv) & 0xFFFF;
	rv = read_kernel_memory(FPGA_R_DDR3_P3_HDAT, 0, 2);
	data |= (rv << 16);
	readback[i] = data;
	//	write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x10, 0, 2 );
	//	write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x00, 0, 2 );
      }
#else
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) { 
	write_kernel_memory( FPGA_W_DDR3_P2_LADR + offset, 
			     (((burstaddr + i) * 4) & 0xFFFF), 0, 2 );
	write_kernel_memory( FPGA_W_DDR3_P2_HADR + offset, 
			     (((burstaddr + i) * 4) >> 16) & 0xFFFF, 0, 2 );
	write_kernel_memory( FPGA_W_DDR3_P3_CMD, 1 | PULSE_GATE_MASK, 0, 2 ); // single beat reads
	write_kernel_memory( FPGA_W_DDR3_P3_CMD, 9 | PULSE_GATE_MASK, 0, 2 );
	write_kernel_memory( FPGA_W_DDR3_P3_CMD, 1 | PULSE_GATE_MASK, 0, 2 );
	while( ((read_kernel_memory(FPGA_R_DDR3_P3_STAT, 0, 2) >> 8) == 0) ) {
	  putchar('i'); fflush(stdout);// wait for queue to become full before reading
	}
	rv = read_kernel_memory(FPGA_R_DDR3_P3_LDAT, 0, 2);
	data = ((unsigned int) rv) & 0xFFFF;
	rv = read_kernel_memory(FPGA_R_DDR3_P3_HDAT, 0, 2);
	data |= (rv << 16);
	readback[i] = data;
	//	write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x10, 0, 2 );
	//	write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x00, 0, 2 );
      }
#endif
      while( !(read_kernel_memory(FPGA_R_DDR3_P3_STAT, 0, 2) & 0x4) ) {
	putchar('x'); fflush(stdout); // error, should be empty now
	write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x10, 0, 2 );
	write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x00, 0, 2 );
      }
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	if( testdat[burstaddr + i] != readback[i] ) {
	  printf( "\n%08x: %08x(w) %08x(r)", burstaddr + i, testdat[burstaddr + i], readback[i] );
	}
      }
      burstaddr += DDR3_FIFODEPTH;
      write_kernel_memory( FPGA_W_DDR3_P2_LADR + offset, ((burstaddr * 4) & 0xFFFF), 0, 2 );
      write_kernel_memory( FPGA_W_DDR3_P2_HADR + offset, ((burstaddr * 4) >> 16) & 0xFFFF, 0, 2 );
    }

    if( !(iters % 16) ) {
      printf( "\n%d iterations\n", iters );
    }
    iters++;
  }
}

void ddr3_test_opt() {
  unsigned int *testdat;
  unsigned int readback[DDR3_FIFODEPTH];
  int i;
  int burstaddr = 0;
  unsigned int data;
  int iters = 0;
  int offset;
  unsigned int rv;
  unsigned int arg = 0;
  volatile unsigned short *cs0;

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;

  srand(time(NULL)); // seed the random generator

  testdat = calloc(DDR3_SIZE, sizeof(unsigned int));
  if( testdat == NULL ) {
    printf( "Can't allocate test array.\n" );
    return;
  }

  while(1) {
    // dummy writes to clear any previous data in the queue -- caution, writes to "wherever"!
    while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] & 4) ) {
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  0x008 | PULSE_GATE_MASK;
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  0x000 | PULSE_GATE_MASK;
    }
    // dummy reads to clear any previous data in the queue
    while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 4) ) {
      cs0[FPGA_MAP(FPGA_W_DDR3_P3_REN)] =  0x010;
      cs0[FPGA_MAP(FPGA_W_DDR3_P3_REN)] =  0x000;
    }

    putchar('+'); fflush(stdout);
    for( i = 0; i < DDR3_SIZE; i++ ) {
      testdat[i] = (unsigned int) rand();
    }
    
    offset = 0;
    burstaddr = 0;
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;

    putchar('!'); fflush(stdout);
    while( burstaddr < DDR3_SIZE ) {
      while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] & 4) ) {
	putchar('-'); fflush(stdout);  // wait for write queue to be empty
      }
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	cs0[FPGA_MAP(FPGA_W_DDR3_P2_LDAT)] = (testdat[burstaddr + i] & 0xFFFF);
	cs0[FPGA_MAP(FPGA_W_DDR3_P2_HDAT)] = (testdat[burstaddr + i] >> 16) & 0xFFFF;
      }
      if( (cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] >> 8) != DDR3_FIFODEPTH ) {
	printf( "z%d\n", cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] >> 8 );
	putchar('z'); fflush(stdout);
      }
      arg = ((DDR3_FIFODEPTH - 1) << 4);
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  arg | PULSE_GATE_MASK;
      arg |= 8;
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  arg | PULSE_GATE_MASK;
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] = 0x000 | PULSE_GATE_MASK;
      burstaddr += DDR3_FIFODEPTH;
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;
    }
    
    offset = 0x10; // accessing port 3 (read port)
    burstaddr = 0;
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;

    putchar('.'); fflush(stdout);

    while( burstaddr < DDR3_SIZE ) {
      arg = ((DDR3_FIFODEPTH - 1) << 4) | 1;
      cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] =  arg | PULSE_GATE_MASK;
      arg |= 0x8;
      cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] =  arg | PULSE_GATE_MASK;
      arg &= ~0x8;
      cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] = arg | PULSE_GATE_MASK;
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	while( (cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 4) ) {
	  putchar('i'); fflush(stdout);// wait for queue to become full before reading
	}
	rv = cs0[FPGA_MAP(FPGA_R_DDR3_P3_LDAT)];
	data = ((unsigned int) rv) & 0xFFFF;
	rv = cs0[FPGA_MAP(FPGA_R_DDR3_P3_HDAT)];
	data |= (rv << 16);
	readback[i] = data;
      }
      while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 0x4) ) {
	putchar('x'); fflush(stdout); // error, should be empty now
	cs0[FPGA_MAP( FPGA_W_DDR3_P3_REN )] = 0x10;
	cs0[FPGA_MAP( FPGA_W_DDR3_P3_REN )] = 0x00;
      }
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	if( testdat[burstaddr + i] != readback[i] ) {
	  printf( "\n%08x: %08x(w) %08x(r)", burstaddr + i, testdat[burstaddr + i], readback[i] );
	}
      }
      burstaddr += DDR3_FIFODEPTH;
      cs0[FPGA_MAP( FPGA_W_DDR3_P2_LADR + offset )] = ((burstaddr * 4) & 0xFFFF);
      cs0[FPGA_MAP( FPGA_W_DDR3_P2_HADR + offset )] = ((burstaddr * 4) >> 16) & 0xFFFF;
    }

    if( !(iters % 16) ) {
      printf( "\n%d iterations\n", iters );
    }
    iters++;
  }
}


void dump_ddr3(unsigned int address, unsigned int len) {
  unsigned int readback[DDR3_FIFODEPTH];
  int i;
  int burstaddr = 0;
  unsigned int data;
  int offset;
  unsigned int rv;
  unsigned int arg = 0;
  volatile unsigned short *cs0;

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;

  cs0[F(FPGA_W_DDR3_P3_CMD)] &= 0x7FFF;  // clear burst mode

  offset = 0x10; // accessing port 3 (read port)
  burstaddr = address / 4;
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;

  while( burstaddr < (address + len) / 4 ) {
    arg = ((DDR3_FIFODEPTH - 1) << 4) | 1;
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] =  arg | PULSE_GATE_MASK;
    arg |= 0x8;
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] =  arg | PULSE_GATE_MASK;
    arg &= ~0x8;
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] = arg | PULSE_GATE_MASK;
    for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
      while( (cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 4) ) {
	putchar('i'); fflush(stdout);// wait for queue to become full before reading
      }
      rv = cs0[FPGA_MAP(FPGA_R_DDR3_P3_LDAT)];
      data = ((unsigned int) rv) & 0xFFFF;
      rv = cs0[FPGA_MAP(FPGA_R_DDR3_P3_HDAT)];
      data |= (rv << 16);
      readback[i] = data;
    }
    while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 0x4) ) {
      putchar('x'); fflush(stdout); // error, should be empty now
      cs0[FPGA_MAP( FPGA_W_DDR3_P3_REN )] = 0x10;
      cs0[FPGA_MAP( FPGA_W_DDR3_P3_REN )] = 0x00;
    }
    for( i = 0; i < DDR3_FIFODEPTH; i += 2 ) {
      if( (i % 8) == 0 )
	printf( "\n%08x: ", (burstaddr + i) * 4 );
      //      printf( "%08x ", readback[i] );
      printf( "%02x%02x%02x%02x%02x%02x%02x%02x ", 
	      readback[i] & 0xFF, (readback[i] >> 8) & 0xFF, (readback[i] >> 16) & 0xFF, (readback[i] >> 24) & 0xFF,
	      readback[i+1] & 0xFF, (readback[i+1] >> 8) & 0xFF, (readback[i+1] >> 16) & 0xFF, (readback[i+1] >> 24) & 0xFF );
    }
    burstaddr += DDR3_FIFODEPTH;
    cs0[FPGA_MAP( FPGA_W_DDR3_P2_LADR + offset )] = ((burstaddr * 4) & 0xFFFF);
    cs0[FPGA_MAP( FPGA_W_DDR3_P2_HADR + offset )] = ((burstaddr * 4) >> 16) & 0xFFFF;
  }
  printf( "\n" );

}


void ddr3load(int ifd, int verify) {
  volatile unsigned short *cs0;
  unsigned int *buf;
  unsigned int actual;
  int burstaddr = 0;
  int i;
  unsigned int arg = 0;
  int offset;
  unsigned int rv;
  unsigned int data;
  unsigned int readback[DDR3_FIFODEPTH];

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;

  cs0[F(FPGA_W_DDR3_P3_CMD)] &= 0x7FFF;  // clear burst mode

  buf = calloc(256 * 1024 * 1024 / 4, 4);
  if( buf == NULL ) {
    printf( "Unable to allocate 256MB shadow area. Yell at bunnie for making crappy code.\n" );
    return;
  }
  actual = read(ifd, buf, 256 * 1024 * 1024);
  actual = actual / 4;
  if( verify == 2 ) {
    // "quick load" flag set
    verify = 0;
    actual = 1024 * 1024 * 2; // just load 2 meg of data
  }
  printf( "Writing %x 32-bit words\n", actual );
  if( (actual % 64) != 0 )
    printf( "Warning: number of bytes being written is not divisible by 64\n" );

  if( !verify ) {
  // dummy writes to clear any previous data in the queue -- caution, writes to "wherever"!
  while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] & 4) ) {
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  0x008 | PULSE_GATE_MASK;
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  0x000 | PULSE_GATE_MASK;
  }
  // dummy reads to clear any previous data in the queue
  while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 4) ) {
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_REN)] =  0x010;
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_REN)] =  0x000;
  }

  putchar('+'); fflush(stdout);
    
  offset = 0;
  burstaddr = 0;
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;

  ////////////////////  // dummy loop to clear bad config state
  while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] & 4) ) {
    putchar('-'); fflush(stdout);  // wait for write queue to be empty
  }
  for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_LDAT)] = (buf[burstaddr + i] & 0xFFFF);
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_HDAT)] = (buf[burstaddr + i] >> 16) & 0xFFFF;
  }
  if( (cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] >> 8) != DDR3_FIFODEPTH ) {
  }
  arg = ((DDR3_FIFODEPTH - 1) << 4);
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  arg | PULSE_GATE_MASK;
  arg |= 8;
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  arg | PULSE_GATE_MASK;
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] = 0x000 | PULSE_GATE_MASK;
  burstaddr += DDR3_FIFODEPTH;
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;

  // dummy writes to clear any previous data in the queue -- caution, writes to "wherever"!
  while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] & 4) ) {
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  0x008 | PULSE_GATE_MASK;
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  0x000 | PULSE_GATE_MASK;
  }
  // dummy reads to clear any previous data in the queue
  while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 4) ) {
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_REN)] =  0x010;
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_REN)] =  0x000;
  }

  offset = 0;
  burstaddr = 0;
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;
  /////////////////////////// end dummy code

  putchar('!'); fflush(stdout);
  while( burstaddr < actual ) {
    if( (burstaddr % (1024 * 1024)) == 0 ) {
      printf( "." );
      fflush(stdout);
    }
    while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] & 4) ) {
      putchar('-'); fflush(stdout);  // wait for write queue to be empty
    }
    for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_LDAT)] = (buf[burstaddr + i] & 0xFFFF);
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_HDAT)] = (buf[burstaddr + i] >> 16) & 0xFFFF;
    }
    if( (cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] >> 8) != DDR3_FIFODEPTH ) {
      printf( "z%d\n", cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] >> 8 );
      putchar('z'); fflush(stdout);
    }
    arg = ((DDR3_FIFODEPTH - 1) << 4);
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  arg | PULSE_GATE_MASK;
    arg |= 8;
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  arg | PULSE_GATE_MASK;
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] = 0x000 | PULSE_GATE_MASK;
    burstaddr += DDR3_FIFODEPTH;
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;
  }
  } else {
    printf( "verifying...\n" );
    offset = 0x10; // accessing port 3 (read port)
    burstaddr = 0;
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;
    
    while( burstaddr < actual ) {
      if( (burstaddr % (1024 * 1024)) == 0 ) {
	printf( "." );
	fflush(stdout);
      }
      arg = ((DDR3_FIFODEPTH - 1) << 4) | 1;
      cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] =  arg | PULSE_GATE_MASK;
      arg |= 0x8;
      cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] =  arg | PULSE_GATE_MASK;
      arg &= ~0x8;
      cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] = arg | PULSE_GATE_MASK;
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	while( (cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 4) ) {
	  putchar('i'); fflush(stdout);// wait for queue to become full before reading
	}
	rv = cs0[FPGA_MAP(FPGA_R_DDR3_P3_LDAT)];
	data = ((unsigned int) rv) & 0xFFFF;
	rv = cs0[FPGA_MAP(FPGA_R_DDR3_P3_HDAT)];
	data |= (rv << 16);
	readback[i] = data;
      }
      while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 0x4) ) {
	putchar('x'); fflush(stdout); // error, should be empty now
	cs0[FPGA_MAP( FPGA_W_DDR3_P3_REN )] = 0x10;
	cs0[FPGA_MAP( FPGA_W_DDR3_P3_REN )] = 0x00;
      }
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	if(buf[burstaddr + i] != readback[i]) {
	  printf( "\n%08x: ", (burstaddr + i) * 4);
	  printf( "+%08x -%08x", buf[burstaddr + i], readback[i] );
	}
      }
      burstaddr += DDR3_FIFODEPTH;
      cs0[FPGA_MAP( FPGA_W_DDR3_P2_LADR + offset )] = ((burstaddr * 4) & 0xFFFF);
      cs0[FPGA_MAP( FPGA_W_DDR3_P2_HADR + offset )] = ((burstaddr * 4) >> 16) & 0xFFFF;
    }

  }
  printf( "\n" );

}

void ddr3_burst_read() {
  volatile unsigned short *cs0;
  unsigned int *buf;

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;

  cs0[F(FPGA_W_DDR3_P3_CMD)] |= 0x8000;  // set burst mode priority

  buf = calloc(256 * 1024 * 1024 / 4, 4);
  if( buf == NULL ) {
    printf( "Unable to allocate 256MB shadow area. Yell at bunnie for making crappy code.\n" );
    return;
  }

  printf( "last run's burst length was: %d\n", cs0[F(FPGA_R_MEAS_BURST)]);

#if 1
  printf( "run 64k capture\n" );
  cs0[F(FPGA_W_ADC_SAMPLEN_L)] = 0x0;
  cs0[F(FPGA_W_ADC_SAMPLEN_H)] = 0x1; // capture 64k for now
  
  cs0[F(FPGA_W_ADC_CTL)] = 0x0;
  cs0[F(FPGA_W_ADC_CTL)] = 0x2;
  cs0[F(FPGA_W_ADC_CTL)] = 0x0;
  usleep(1000); 
  cs0[F(FPGA_W_ADC_CTL)] = 0x1; // start sampling run, fill buffer

  usleep(1000);  // it's done in a flash
#endif

  printf( "init readback state\n" );
  cs0[F(FPGA_W_RBK_PAGE_L)] = 0x0;
  cs0[F(FPGA_W_RBK_PAGE_H)] = 0x0; // read back starting at page 0

  cs0[F(FPGA_W_BURSTLEN)] = 0x10;  // 16-beat bursts
  
  cs0[F(FPGA_W_RBK_CTL)] = RBK_CTL_CLEAR_ERROR | RBK_CTL_INIT;
  cs0[F(FPGA_W_RBK_CTL)] = 0;
  cs0[F(FPGA_W_RBK_CTL)] = RBK_CTL_ENABLE; // enable readback machine

}
