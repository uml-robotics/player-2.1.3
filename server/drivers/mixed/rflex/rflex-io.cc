#define IO_DEBUG

#if HAVE_CONFIG_H
  #include <config.h>
#endif
#if HAVE_SYS_FILIO_H
  #include <sys/filio.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <float.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "rflex-info.h"
#include "rflex-io.h"

#ifndef CRTSCTS
#ifdef IHFLOW
#ifdef OHFLOW
#define CRTSCTS ((IHFLOW) | (OHFLOW))
#endif
#endif
#endif

int
iParity( enum PARITY_TYPE par )
{
  if (par==N)
    return(IGNPAR);
  else
    return(INPCK);
}

int
iSoftControl( int flowcontrol )
{
  if (flowcontrol)
    return(IXON);
  else
    return(IXOFF);
}

int
cDataSize( int numbits )
{
  switch(numbits) {
  case 5:
    return(CS5);
    break;
  case 6:
    return(CS6);
    break;
  case 7:
    return(CS7);
    break;
  case 8:
    return(CS8);
    break;
  default:
    return(CS8);
    break;
  }
}

int
cStopSize( int numbits )
{
  if (numbits==2) {
    return(CSTOPB);
  } else {
    return(0);
  }
}

int
cFlowControl( int flowcontrol )
{
  if (flowcontrol) {
    return(CRTSCTS);
  } else {
    return(CLOCAL);
  }
}

int
cParity( enum PARITY_TYPE par )
{
  if (par!=N) {
    if (par==O) {
      return(PARENB | PARODD);
    } else {
      return(PARENB);
    }
  } else {
    return(0);
  }
}

int
cBaudrate( int baudrate )
{
  switch(baudrate) {
  case 0:
    return(B0);
    break;
  case 300:
    return(B300);
    break;
  case 600:
    return(B600);
    break;
  case 1200:
    return(B1200);
    break;
  case 2400:
    return(B2400);
    break;
  case 4800:
    return(B4800);
    break;
  case 9600:
    return(B9600);
    break;
  case 19200:
    return(B19200);
    break;
  case 38400:
    return(B38400);
    break;
  case 57600:
    return(B57600);
    break;
  case 115200:
    return(B115200);
    break;
#ifdef B230400
  case 230400:
    return(B230400);
    break;
#endif

#ifdef B460800  // POSIX doesn't have this one
#warning Including support for baud rate B460800 which is not available in all implementations of termios. \
To enable in Linux, you must edit the serial driver. see this source file for details.
  case 500000:
    /* to use 500k you have to change the entry of B460800 in you kernel:
       /usr/src/linux/drivers/usb/serial/ftdi_sio.h:
       ftdi_8U232AM_48MHz_b460800 = 0x0006    */
    return(B460800);
    break;
#endif

  default:
    return(B9600);
    break;
  }

}

long
bytesWaiting( int sd )
{
  long available=0;
  if ( ioctl( sd, FIONREAD, &available ) == 0 )
    return available;
  else
    return -1;
}

void
DEVICE_set_params( RFLEX_Device dev )
{
  struct termios  ctio;

  tcgetattr(dev.fd,&ctio); /* save current port settings */

  ctio.c_iflag =
    iSoftControl(dev.swf) |
    iParity(dev.parity);
  ctio.c_oflag = 0;
  ctio.c_cflag =
    CREAD                            |
    cFlowControl(dev.hwf || dev.swf) |
    cParity(dev.parity)              |
    cDataSize(dev.databits)          |
    cStopSize(dev.stopbits);

  ctio.c_lflag = 0;
  ctio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
  ctio.c_cc[VMIN]     = 0;   /* blocking read until 0 chars received */

  cfsetispeed ( &ctio, (speed_t) cBaudrate(dev.baud) );
  cfsetospeed ( &ctio, (speed_t) cBaudrate(dev.baud) );

  tcflush(dev.fd, TCIFLUSH);
  tcsetattr(dev.fd,TCSANOW,&ctio);

}

void
DEVICE_set_baudrate( RFLEX_Device dev, int brate )
{
  struct termios  ctio;

  tcgetattr(dev.fd,&ctio); /* save current port settings */

  cfsetispeed ( &ctio, (speed_t) cBaudrate(brate) );
  cfsetospeed ( &ctio, (speed_t) cBaudrate(brate) );

  tcflush(dev.fd, TCIFLUSH);
  tcsetattr(dev.fd,TCSANOW,&ctio);
}

int
DEVICE_connect_port( RFLEX_Device *dev )
{
  if ( ( dev->fd =
	 open( (dev->ttyport), (O_RDWR | O_NOCTTY),0) ) < 0 ) {
    return( -1 );
  }
  DEVICE_set_params( *dev );
  return( dev->fd );
}

int
writeData( int fd, unsigned char *buf, int nChars )
{
  int written = 0;
  while (nChars > 0) {
    written = write( fd, buf, nChars );
    if (written < 0) {
      return FALSE;
    } else {
      nChars -= written;
      buf    += written;
    }
    usleep(1000);
  }
  return TRUE;
}

namespace debug{

inline unsigned short SwapEndian(unsigned short val)
{
    return (val<<8) | (val>>8);
}

inline unsigned long SwapEndian(unsigned long val)
{
    return (val<<24) | ((val<<8) & 0x00ff0000) |
            ((val>>8) & 0x0000ff00) | (val>>24);
}

#pragma pack(push)
#pragma pack(1)
  typedef struct{
    unsigned short pkt_stx; // 2
    char port;   //1
    char packet_id; //1
    char opcode; //1
    char data_length; //1

    unsigned long rv; //4
    unsigned long timestamp; //4
    char axis; //1
    unsigned long position;//4
    unsigned long velocity;//4
    unsigned long accel;//4
    unsigned long torque;//4

    char checksum;//1
    unsigned short pkt_etx;//2
  } Report ;

#pragma pack(pop)
void

printMotSystemReport(unsigned char *buf, int len){
  if (buf[4]!=33) // other possibility is 7 (probably velocity confirmation packet)
    return;

  FILE *out = stdout;

  if (len!=sizeof(Report)){
    fprintf(out, "size of motion message is incorrect!\n");
  }

  for (int i=0;i<len;i++)
    fprintf( out, ".%s%x", buf[i]<16?"0":"", buf[i] );
  
  Report * rep = (Report*) buf;

  fprintf( out, "  time:%.3f axis:%hhu pos:%d vel:%d\n",
      SwapEndian(rep->timestamp) / 250.0, rep->axis,
      (int)SwapEndian(rep->position), 
      (int)SwapEndian(rep->velocity));

  return;
}
}// namespace debug

  int
waitForETX( int fd, unsigned char *buf, int  *len )
{
  static int pos, loop, val, dlen;
#ifdef IO_DEBUG
  int i;
#endif
  pos = 2; loop = 0; dlen = -1;
  while( loop<MAX_NUM_LOOPS ) {
    val = bytesWaiting( fd );
    if (val>0) {
      read( fd, &(buf[pos]), 1 );
      pos++;
      if (pos>5) {
        dlen = buf[5] + 9; /* std length (9 char) + data length - end chars (2)*/
      }
      if (dlen>0 && pos>=dlen) {
        if (buf[dlen-2]==B_ESC && buf[dlen-1]==B_ETX) {
          *len = dlen;
#ifdef IO_DEBUG
          if (buf[2]==2){
            //fprintf( stderr, "-       answer ->" );
            debug::printMotSystemReport(buf, pos);
          }
#else
#ifdef DEBUG
          fprintf( stderr, "(%d)", *len );
#endif
#endif
          return(TRUE);

        }
        else if(buf[2] == 0x06 && pos == dlen)
        {
          //Wait to have one more byte
        }
        else if(buf[2] == 0x06 && pos >= (dlen+1) && buf[dlen-1]==B_ESC && buf[dlen-0]==B_ETX)
        {
          //haunted packet. We have a ghost byte at the 7th position. scrap it and send a corrected paquet
          for (int j = 0;j<(buf[5]+3);j++)
          {
            //shift byte to the left
            buf[j+6] = buf[j+7];
          }

          *len = dlen;

#ifdef IO_DEBUG
          if (buf[2]==2){
            //fprintf( stderr, "- fixed answer ->" );
            debug::printMotSystemReport(buf, pos);
          }
#endif


          return(TRUE);
        } else {
#ifdef IO_DEBUG
          if (buf[2]==2){
            //fprintf( stderr, "- wrong answer ->" );
            debug::printMotSystemReport(buf, pos);
          }
#else
#ifdef DEBUG
          fprintf( stderr, "-" );
#endif
#endif
          return(FALSE);
        }
      }

    } else {
      usleep(100);
      loop++;
    }
  }
#ifdef IO_DEBUG
  //fprintf( stderr, "\n" );
#endif
  return(FALSE);
}

int
waitForAnswer( int fd, unsigned char *buf, int *len )
{
  int loop = 0;
  *len = 0;
  while( loop<MAX_NUM_LOOPS ) {
    if (bytesWaiting( fd )) {
      read( fd, &(buf[*len]), 1 );
      *len = *len+1;
      if (*len>=2 && buf[*len-2]==B_ESC && buf[*len-1]==B_STX ) {
	buf[0] = B_ESC;
	buf[1] = B_STX;
	*len = 2;
	return(waitForETX(fd,buf,len));
      }
    } else {
      usleep(100);
      loop++;
    }
  }
  return(FALSE);
}

