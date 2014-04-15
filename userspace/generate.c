#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int main() {
  int ofd;
  int i;
  char c;

  ofd = open( "count.bin", O_WRONLY | O_CREAT );
  
  c = 0;
  for( i = 0; i < 16384; i++ ) {
    write( ofd, &c, 1 );
    c++;
  }
  close(ofd);

}
