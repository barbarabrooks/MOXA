/*----------------------------------------------------------------------------*
 * SERCOMM - quick and dirty serial comms terminal programme for moxa's       *
 *                                                                            *
 * IM Brooks :  July 2008                                                     *
 *----------------------------------------------------------------------------*/

#include <sys/types.h>                  /* Primitive System Data Type         */
#include <sys/stat.h>                   /* File Characteristics               */
#include <fcntl.h>                      /* File Control Operations            */
#include <termios.h>                    /* general terminal interface         */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>                     /* Symbolic Constants                 */
#include <string.h>
#include <time.h>
#include <sys/time.h>                   /* hi res time functions              */
#include <signal.h>			/* signal handling functions	      */
#include <errno.h>                      /* error message macros               */
#include <dirent.h>                     /* directory functions                */
#include <sys/ioctl.h>			/* misc I/O device operations         */

#ifndef O_BINARY
#define O_BINARY 0
#endif

/*
 * __arm__ is defined on Moxa device - use this as a platform check.
 * Really this should be moxa specific?
 */

#ifdef __arm__
#define MOXA 1
#endif

/* include MOXA device stuff */
#ifdef MOXA
#include <moxadevice.h>
#else
/* Serial port modes defined in moxadevice.h - redefine here if we're not 
   running on a moxa...not that we can use them, but...*/
#define RS232_MODE              0
#define RS485_2WIRE_MODE        1
#define RS422_MODE              2
#define RS485_4WIRE_MODE        3
#endif

/*--- declare global variables -----------------------------------------------*/

int fd;                                 /* generic serial port file handle    */
char *error_message;                    /* pointer to error message string    */

/* serial port properties */
char PORT[32];		/* port device */
tcflag_t baudrate;	/* baudrate flag for port */
char lineterm[3];		/* line termination character flag */

/* misc control variables */
int n;                                  /* generic counter                    */
int alldone;				/* control flag                       */

/* glibc defined structures */
struct termios oldtio, newtio;          /* terminal configuration structures  */


/*--- declare function prototypes --------------------------------------------*/
int init_serialport(char PORT[], int *opmode_ptr); /* initialise serial port  */
void sigterm_handler(int signum);       /* handle termination signals         */
void cleanup(void);			/* clean up before quitting */
void comms(void);

/*----------------------------------------------------------------------------*
 * It all starts here!                                                        *
 *----------------------------------------------------------------------------*/
int main(int argc, char *argv[])
{
  int opmode;				/* serial port protocol mode */
  
  /* check parameters passed from command line */
  if (argc != 5) {
    printf("usage : sercomm device protocol baudrate lineterm\n");
    printf("   eg : sercomm /dev/ttyS0 rs232 B9600 [LF|CRLF]\n");
    exit(-1);
  }

  printf("ctrl-c : quits,\n");
  
  /* install signal handlers - should handle a stop signal gracefully */
  if ( signal(SIGTERM, sigterm_handler ) == SIG_IGN )
    signal(SIGTERM, SIG_IGN);
  if ( signal(SIGINT, sigterm_handler ) == SIG_IGN )
    signal(SIGINT, SIG_IGN);
  alldone = 0;

  /* get serial port protocol */
  sprintf(PORT,argv[1]);
  if (strncasecmp(argv[2],"rs232",5) == 0) {
    opmode = RS232_MODE;
  } else if (strncasecmp(argv[2],"rs422",5) == 0) {
    opmode = RS422_MODE;
  } else if (strncasecmp(argv[2],"rs485_4wire",11) == 0) {
    opmode = RS485_4WIRE_MODE;
  } else if (strncasecmp(argv[2],"rs485_2wire",11) == 0) {
    opmode = RS485_2WIRE_MODE;
  }

  /* get baud rate */
  if (strncasecmp(argv[3],"B4800",5) == 0) {
    baudrate = B4800;
  } else if (strncasecmp(argv[3],"B9600",5) == 0) {
    baudrate = B9600;
  } else if (strncasecmp(argv[3],"B19200",6) == 0) {
    baudrate = B19200;
  } else if (strncasecmp(argv[3],"B38400",6) == 0) {
    baudrate = B38400;
  } else if (strncasecmp(argv[3],"B57600",6) == 0) {
    baudrate = B57600;
  }

  /* get correct line termination characters */
  if (strncasecmp(argv[4],"CRLF",4) == 0) {
    lineterm[0] = 13; lineterm[1] = 10; 
  } else if (strncasecmp(argv[4],"CR",2) == 0) {
    lineterm[0] = 13;
  } else if (strncasecmp(argv[4],"LF",2) == 0) {
    lineterm[0] = 10;
  } else {
    lineterm[0] = 13; lineterm[1] = 10; // default to CRLF if none specified
  }

printf("line termination : %d %d %d\n",lineterm[0],lineterm[1],lineterm[2]);

  /* open and configure serial ports - NB. setting of opmode is a MOXA       *
   * device specific operation. Device modes are defined in <moxadevice.h>   */
  fd = init_serialport(PORT, &opmode);
  
  /* flush serial lines to clear buffers before we start doing anything */
  tcflush(fd,TCIFLUSH);
  
  comms();

  cleanup();
  return(0);
}


/*----------------------------------------------------------------------------*
 * cleanup - cleanup everything before exiting                                *
 *----------------------------------------------------------------------------*/
void cleanup()
{
  /* close all the serial ports */
  close(fd);
}


/*----------------------------------------------------------------------------*
 * sigterm_handler - handle termination signals gracefully                    *
 * ---------------------------------------------------------------------------*/
void sigterm_handler(int signum)
{
  /* just set the 'alldone' flag, and logger should stop, cleanup, and exit */
  alldone = 1;
}


/*----------------------------------------------------------------------------*
 * init_serialport - initialises the named serial port.                       *
 *            This function is based on the example in the Linux              *
 *            Serial Programming HOWTO - see: http://www.linuxdoc.org         *
 *----------------------------------------------------------------------------*/
int init_serialport(char PORT[], int *opmode_ptr)
{
  /* Open serial device for reading and writing and not as controlling tty
     because we don't want to get killed if line noise sends CTRL-C. */
  fd = open(PORT, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    perror(PORT);
    printf("ERROR : failed to open : %s\n",PORT);
    exit(-1);
  }

  /* set the protocol to use on this MOXA port */
#ifdef MOXA
  ioctl(fd, MOXA_SET_OP_MODE, opmode_ptr);
#else
  if (*opmode_ptr != RS232_MODE) {
    printf("RS422/RS485 modes not supported on PC!\n");
  }
#endif
  
  bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

  newtio.c_cflag = baudrate | CRTSCTS | CS8 | CLOCAL | CREAD;
  
  /* IGNPAR  : ignore bytes with parity errors */
  newtio.c_iflag = IGNPAR;
  /* Raw output.*/
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  /* initialize all control characters
     default values can be found in /usr/include/termios.h, and are given
     in the comments, but we don't need them here  */
  newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */
  newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
  newtio.c_cc[VERASE]   = 0;     /* del */
  newtio.c_cc[VKILL]    = 0;     /* @ */
  newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
  newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
  newtio.c_cc[VMIN]     = 0;     /* blocking read until n characters arrive */
  newtio.c_cc[VSWTC]    = 0;     /* '\0'   */
  newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */
  newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
  newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
  newtio.c_cc[VEOL]     = 0;     /* '\0'   */
  newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
  newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
  newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
  newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
  newtio.c_cc[VEOL2]    = 0;     /* '\0'   */

  /* now clean the serial lines and activate the settings for the port */
  tcflush(fd, TCIOFLUSH);
  tcsetattr(fd,TCSANOW,&newtio);

  return(fd);
} 


/*----------------------------------------------------------------------------*
 * function - set_keymode                                                     *
 *          - switch input mode of stdin between canonical and non-canonical  *
 *   parameters - inmode = 0 (non-canonical) or 1 (canonical)                 *
 *----------------------------------------------------------------------------*/
void set_keymode(int inmode)
{
  struct termios keyboard;

  tcgetattr(STDIN_FILENO,&keyboard);         /* get the current settings      */
  keyboard.c_lflag |= ICANON;                /* set canonical mode (default)  */
  keyboard.c_cc[VMIN]=1;                     /* set default VMIN chars to read*/
  if (inmode==0) {
    keyboard.c_lflag &= ~ICANON;             /* enable non-canonical mode     */
    keyboard.c_cc[VMIN]=0;                   /* set VMIN=0 for instant return */
  }
  tcsetattr(STDIN_FILENO,TCSANOW,&keyboard); /* apply the new settings now    */
}


/*----------------------------------------------------------------------------*
 * comms - communicate                                                        *
 *----------------------------------------------------------------------------*/
void comms()
{
  char key;
  char inbuffer[128];
  int nc, gotc, count;

  set_keymode(0); /* non-blocking read */

  while (!alldone) {
    nc=read(STDIN_FILENO,&key,1);
    if (nc > 0) {
      /* trap and handle any control characters */
      if ( key == 3 ) {
        /* CTRL-C quits */
        alldone = 1; 
      } else if ( key == 10 ) {
        /* if we read a LF then send correct line termination characters */
        write(fd,lineterm,strlen(lineterm));
      } else {
        /* write to serial port */
//        printf("\n");
        write(fd, &key, 1);
        usleep(1000);
        gotc = 0;
        count = 0 ;
      }
    }
    nc = 1;
    /* display incoming data stream from serial port */
    while ( nc > 0 ) {
      bzero(inbuffer,sizeof(inbuffer));
      nc = read(fd, inbuffer, sizeof(inbuffer));
      printf("%s",inbuffer);
      fflush(stdout); 
    }
    nc = 0;
  }

  set_keymode(1); /* return to canonical keymode */

}
