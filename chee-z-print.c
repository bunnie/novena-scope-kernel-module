#include <stdio.h>
#include <fcntl.h>

int main(int argc, char **argv) {
  int infile;
  char *prog = argv[0];
  unsigned char buf[16];
  int retval;
  int i, j;

  argv++;
  argc--;

  if(!argc) {
    printf( "Please specify file to chee-z-print.\n" );
    return 1;
  }
  
  infile = open(*argv, O_RDONLY );
  if( infile == -1 ) {
    printf("Unable to open %s\n", *argv );
    return 1;
  }
  
  retval = 16;
  do {
    retval = read(infile, buf, 16);
    if( retval != 16 )
      break; // this is the loop escape
    
    for( i = 8; i < 16; i++ ) {
      for( j = 0; j < (buf[i] / 4); j++ ) {
	putchar(' ');
      }
      putchar('*');
      putchar('\n');
    }
  } while(1);
  
  
}
