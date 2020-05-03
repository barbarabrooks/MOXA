/*----------------------------------------------------------------------------*
 * GPSsync  - syncronises moxa logger to GPS time at chosen intervals         *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * I M Brooks and C E Birch  :                                                *
 * version 1.1 : 10/12/2007                                                   *
 *                                                                            *
 *                                                                            *
 *----------------------------------------------------------------------------*/
#include <sys/types.h> 			/* Primitive System Data Type         */
#include <sys/stat.h> 			/* File Characteristics               */
#include <fcntl.h>			/* File Control Operations            */
#include <termios.h>            	/* general terminal interface         */
#include <stdio.h>
#include <unistd.h>			/* Symbolic Constants                 */
#include <stdlib.h>
#include <string.h>
#include <signal.h>                     /* signal handling functions          */
#include <time.h>
#include <sys/ioctl.h>                  /* misc I/O device operations         */
#include <moxadevice.h>                 /* MOXA hardware specific stuff       */

#ifndef O_BINARY
#define O_BINARY 0
#endif

/* Define serial port properties */
/* Baudrate settings defined in <asm/termbits.h>, included by <termios.h> */

  		
#define BAUDRATE B4800 		/* need to change and recompile if    *
                                         * sonic hardware baudrate is changed */

/*--- declare global variables -----------------------------------------------*/
struct termios oldtio,newtio;  		/* terminal configuration structures  */
char buf_data[255];                     /* buffer to read data                */
const int RMClength = 72;               /* length of RMC message inc CR LF    */
int nc;					/* No. of characters read from port   */
int alldone;                            /* master control flag                */
int fd_GPS;                             /* GPS specific file handles          */
int fd_logfile;                         /* GPS output file handle             */
struct tm GPS_time;                     /* full date and time structure       */
time_t time2use, time2print;            /* time as a number of secs           */
char PORT[25];


/*--- declare function prototypes --------------------------------------------*/
int init_serialport(char PORT[],int *opmode_ptr);  /* initialise serial port */
void sigterm_handler(int signum);
//void write2logfile(int *printtime_ptr);
void write2logfile(time_t printtime);

/* --- Define serial port ----------------------------------------------------*/
//#define PORT_GPS "/dev/ttyM5"        /* Moxa port P1 */


/*----------------------------------------------------------------------------e
 * function - main                                                            *
 *----------------------------------------------------------------------------*/
int main(int argc, char *argv[])
{
  const char LF = {10};
  char abyte;  
  int opmode;                                  /* serial port protocol mode   */
  char filename[64];		               /* filename for GPS output data*/
  const char delimiters [] = ",";              /* token seperater             */
  char *token, *cp, *datestr, *timestr;        /* arrays for tokens           */
  int dd, mo, yy, hh, mi, ss;                  /* time and date variables     */
  int count;
  char oldbuf_data[255];                       /* char array to write to file */

 /* check parameters passed from command line */
  if (argc != 2) {
    printf("usage : GPSsync <port>\nwhere port = '/dev/ttyS0' etc\n");
    exit(-1);
  }
  strcpy(PORT,argv[1]);

  alldone = 0;

    /* install signal handlers */
  if ( signal(SIGTERM, sigterm_handler ) == SIG_IGN )
    signal(SIGTERM, SIG_IGN);
  if ( signal(SIGINT, sigterm_handler ) == SIG_IGN )
    signal(SIGINT, SIG_IGN);

  /* open and configure serial port for GPS - NB. setting of opmode is a MOXA*
   * device specific operation. Device modes are defined in <moxadevice.h>   */
  opmode = RS232_MODE;
  fd_GPS = init_serialport(PORT, &opmode);
  /* flush serial lines to clear buffers before we start doing anything */
  tcflush(fd_GPS,TCIFLUSH);
 
  /* Create a file for the GPS */
  strcpy(filename,"GPSoutput.txt");
  fd_logfile = open(filename, O_WRONLY | O_APPEND | O_CREAT | O_BINARY, 0666);
  
  while (!alldone) {
    abyte = 'a';
    while (abyte!=LF){                      /* read data until end of message */
      nc = read(fd_GPS,&abyte,1);  
    }      
    abyte = 'a';
    count = 0;
    while (abyte!=LF){            /* at end of row read whole of next message */
      nc = read(fd_GPS,&abyte,1);
      if (nc>0){
        buf_data[count] = abyte;
        count=count+1;
      }
   }
    buf_data[count]=0;
    /* use this message to get time */
    cp = strdup (buf_data);                            /* Make writable copy  */
    token = strsep (&cp, delimiters);                  /* token to PGRMF      */
    token = strsep (&cp, delimiters);                  /* token to GPS sec no */
    token = strsep (&cp, delimiters);                  /* token to GPS wk no  */
    datestr = strsep (&cp, delimiters);                /* token for date      */
    timestr = strsep (&cp, delimiters);                /* token for time      */
        
    sscanf(datestr,"%02d%02d%02d",&dd,&mo,&yy);
    sscanf(timestr,"%02d%02d%02d",&hh,&mi,&ss);
    GPS_time.tm_sec = ss;           /* Put parts of the time into a tm struct */
    GPS_time.tm_min = mi;
    GPS_time.tm_hour = hh;
    GPS_time.tm_mday = dd;
    GPS_time.tm_mon = mo-1;
    GPS_time.tm_year = yy+100;
    time2use = mktime(&GPS_time);       /* Convert time into a number of secs */
    time2use = time2use + 1;
    
    strcpy(oldbuf_data,buf_data);       /* GPS message for writing to logfile */

    nc = 0;
    while (nc<1){      /* set system time when first char of next mess logged */
      nc=read(fd_GPS,buf_data,RMClength);      
    }
    time(&time2print);                         /* Get system time for logfile */
    stime(&time2use);                                      /* Set system time */
    
    write2logfile(time2print);         /* Write system time when clock reset */
    write2logfile(time2use);           /* Write time system is set to        */
    write(fd_logfile, oldbuf_data, strlen(oldbuf_data)); /* Write GPS message to logfile */
    alldone = 1;
    nc = read(fd_GPS,&abyte,1);
          
  } /* end of while */
  close(fd_logfile); 
  close(fd_GPS);
  return(0);
}


/*----------------------------------------------------------------------------*
 * function - init_serialport                                                 *
 *          - initialises the serial port.                                    *
 *                                                                            *
 *            This function is based on the example in the Linux              *
 *            Serial Programming HOWTO - see: http://www.linuxdoc.org         *
 *----------------------------------------------------------------------------*/
int init_serialport(char PORT[], int *opmode_ptr)
{
  
  int fd;
  /* Open modem device for reading and writing and not as controlling tty
     because we don't want to get killed if line noise sends CTRL-C. */
   
  fd = open(PORT, O_RDWR | O_NOCTTY); 
  if (fd <0) {
    perror(PORT);
    printf("ERROR : failed to open serial port"); 
    exit(-1); 
  }

  /* set the protocol to use on this MOXA port */
  ioctl(fd, MOXA_SET_OP_MODE, opmode_ptr);

  tcgetattr(fd,&oldtio);          /* save current serial port settings */
  bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

  /* BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
     CRTSCTS : output hardware flow control (only used if the cable has
               all necessary lines. See sect. 7 of Serial-HOWTO)
     CS8     : 8n1 (8bit,no parity,1 stopbit)
     CLOCAL  : local connection, no modem contol
     CREAD   : enable receiving characters 
  */
     
  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
 
  /* IGNPAR  : ignore bytes with parity errors */
  newtio.c_iflag = IGNPAR;
 
  /* Raw output.*/
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
 
  /* initialize all control characters 
     default values can be found in /usr/include/termios.h, and are given
     in the comments, but we don't need them here
  */
  
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
} /* end of init_rocket */


/*----------------------------------------------------------------------------*
 * sigterm_handler - handle termination signals gracefully                    *
 * ---------------------------------------------------------------------------*/
void sigterm_handler(int signum)
{
  close(fd_GPS);
  close(fd_logfile);
  exit(-1);

}


/*----------------------------------------------------------------------------*
 * write2logfile - writes system/GPS time to logfile                          *
 * ---------------------------------------------------------------------------*/
void write2logfile(time_t printtime)
{
  struct tm tm2print;  
  char date[6], time[6];
  const char comma = {44}, colon = {58};
  
  gmtime_r(&printtime,&tm2print);                /* Get time into a tm struct */
  sprintf(date,"%02d%02d%02d",tm2print.tm_mday,tm2print.tm_mon+1,tm2print.tm_year-100);
  write(fd_logfile, &date,6);
  write(fd_logfile, &colon,1);
  sprintf(time,"%02d%02d%02d",tm2print.tm_hour,tm2print.tm_min,tm2print.tm_sec);
  write(fd_logfile, &time,6);
  write(fd_logfile, &comma,1); 
  printf("%d \n",tm2print.tm_min);
  printf("%d \n",tm2print.tm_sec); 
  return;
}  
