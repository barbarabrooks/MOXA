/*----------------------------------------------------------------------------*
 * turblogger - logger for miscellaneous turbulence instruments               *
 *              This version based on ANR's Arran Logger & IMB's SEASAW       *
 *              cruise logger                                                 *
 *                                                                            *
 * I M Brooks  :                                                              *
 * version 1.1 : 3/12/2007                                                    *
 *               This version handles multiple instrument groups, syncing     *
 *               each group separately                                        *
 * version 1.2 : April 2008 - minor mods for Chilbolton2008 cmpaign           *
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
#include <errno.h>			/* error message macros		      */
#include <dirent.h>			/* directory functions		      */
#include <stddef.h>
#include <ctype.h>
#include <sys/ioctl.h>


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
/* Serial port modes defined in moxadevice.h */
#define RS232_MODE              0
#define RS485_2WIRE_MODE        1
#define RS422_MODE              2
#define RS485_4WIRE_MODE        3
#endif

/* include METEK sonic binary file format specification and error codes */
#include "metek.h"
 
#ifndef O_BINARY
#define O_BINARY 0
#endif

/* Define serial ports, properties structure, etc                      */
/* Total number of supported serial ports - max on a MOXA system is 8  */
#define NPORTS 8

/* Support serial port modes */
char *modenames[4] = {"rs232", "rs485_2wire", "rs422", "rs485_4wire"};

/* data stream modes */
#define NUM_DATA_MODES 2
enum datamode {ASCII = 0, BINARY = 1};
char *datamodenames[2] = {"ASCII","BINARY"};

/* Baudrate settings defined in <asm/termbits.h>, included by <termios.h> */
struct port_type {
  char devname[25];    /* /dev/tty*  */
  char instrument[25]; /* unique instrument name/identifier */
  tcflag_t cflag;      /* Control options for port -> most important is the */
                       /* baud rate e.g. B19200. */
  int mode;            /* Serial port mode, e.g. RS232 */
  int msglen;	       /* length of message (or max length if variable) */
  enum datamode datamode; /* 0 = ASCII, 1 = BINARY mode */
  char headbytes[8];   /* start of message byte sequence */  
  int OK;	       /* flag to indicate OK to log - we have discarded any 
                          initial partial data message after startup 
                          0 => not good to log, 1 => OK to log */
  int gotmsg;	       /* flag that whole data message received */
};



/*--- declare global variables -----------------------------------------------*/

const char filesystem[] = "/";
const char datadir[] = "/home/data"; // this should be a link to real data dir
                                     // on high volume USB key, or CF card


int nports_used;
struct port_type portconf[NPORTS];

/*-- file handles --*/
int fd;                                 /* generic serial port file handle    */
int fd_dev[NPORTS];	                /* kit specific file handles	      */

/* array of data buffers, size of [number of ports]x[max size] - max size is  *
 * set to something larger than longest message we expect to get from any     *
 * instrument...340 bytes - a CLASP with 16 channels at 16 bits each          *  *   NB this is sort of out of date now since CLASP is written byte by byte   */
char buf_data[NPORTS][512];

char *error_message;			/* pointer to error message string    */

/* misc control variables */
int n;                                  /* generic counter                    */
int alldone;				/* master control flag	              */
int filelength;				/* length of file in seconds          */

/* glibc defined structures */
struct termios oldtio,newtio;           /* terminal configuration structures  */
struct timeval hirestime;               /* hi res time structure              */
struct tm datetime;                     /* full date and time structure       */

struct timeval sampletime[NPORTS];	        /* sample acquisition time  */

/*--- declare function prototypes --------------------------------------------*/

/* generic system functions */

int init_serialport(struct port_type port);    /* initialise serial port      */
void get_config();			/* get system config from conf file   */
void log_it_all(void);                  /* log sonic output to file           */
void displaysonic(void);                /* display sonic output to terminal   */
void sigterm_handler(int signum);       /* handle termination signals         */
void startup(void);			/* startup tasks */
void cleanup(void);			/* pre-exit cleanup		      */
void save_pid(void);			/* save process id to file	      */
void print_time(void);                  /* print date & time to STDOUT        */
int dir_check(void);
//int is_mounted(char *dirname);	/* check data card is mounted         */

/* data read/write functions */
void new_file(struct port_type port, char *const filename_ptr);
int read_data(int fd, char *data_buffer_ptr, int bufsize);
int read_ascii_data(int fd, char *const data_buffer, int msglen);
int write_message(char filename[], char data_buffer[], int bufsize);

/* instrument specific functions */
int start_mkh_interface(int fd);       /* start MKH interface to inclinometers*/
int stop_mkh_interface(int fd);	       /* stop MKH interface to inclinometers */
void mkh_command(int fd, const char COMMAND, char *response);
				       /* send command to MKH interface, get  *
                                        * response if any                     */
int init_clasp(int fd);                /* initialise CLASP settings           */
int init_AWS(int fd);		       /* initialise AWS */


/*----------------------------------------------------------------------------*
 * It all starts here!                                                        *
 *----------------------------------------------------------------------------*/
int main()
{
  int i;

  /* install signal handlers */
  if ( signal(SIGTERM, sigterm_handler ) == SIG_IGN )
    signal(SIGTERM, SIG_IGN);
  if ( signal(SIGINT, sigterm_handler ) == SIG_IGN )
    signal(SIGINT, SIG_IGN);
  save_pid();

  /* set default values for flags & system variables */
  alldone = 0;		/* not finished yet */
  filelength = 3600;	/* new file at 1-hour intervals */
  /* check the data directory exists */
  dir_check();

  /* Set up system configuration - port/instrument pairs, etc */
  get_config();

  /* open and configure serial ports */
  printf("initialising instruments:\n");
  for (i = 0; i< nports_used; i++) {
    fd_dev[i] = init_serialport(portconf[i]);
  }

  startup();

/* HANDLE INITIAL READ OF FIRST FULL MESSAGE HERE */

  if (!alldone) {  // trap a request to stop before we get started
    log_it_all();
  }

  cleanup();
  return(0);
}


/*----------------------------------------------------------------------------*
 * save_pid - saves process ID for control script                             *
 *----------------------------------------------------------------------------*/
void save_pid()
{
  int fid;
  char pid[5];
  sprintf(pid,"%d", getpid());
//  fid = open("/var/run/logger.pid", O_WRONLY | O_TRUNC | O_CREAT | O_BINARY, 0666);
  fid = open("./logger.pid", O_WRONLY | O_TRUNC | O_CREAT | O_BINARY, 0666);
  write(fid,pid,strlen(pid));
  close(fid);
}


/*----------------------------------------------------------------------------*
 * startup - deal with initital reads of already streaming data feeds to      *
 *           ensure we only get whole data messages                           *
 *----------------------------------------------------------------------------*/
void startup()
{
  int i;
  for (i=0;i<nports_used;i++) {
    tcflush(fd_dev[i],TCIFLUSH);
  }

}


/*----------------------------------------------------------------------------*
 * cleanup - cleanup everything before exiting                                *
 *----------------------------------------------------------------------------*/
void cleanup()
{
  int i;
  for (i=0;i<nports_used;i++) {
    /* close all the serial ports */
    close(fd_dev[i]);
  }
}


/*----------------------------------------------------------------------------*
 * sigterm_handler - handle termination signals gracefully                    *
 * ---------------------------------------------------------------------------*/
void sigterm_handler(int signum)
{
  alldone = 1;
}


/*----------------------------------------------------------------------------*
 * dir_check - check output directory exists                                  *
 * ---------------------------------------------------------------------------*/
int dir_check(void)
{
  DIR *dd;			/* directory stream ID	*/
  struct dirent *dirlist;	/* pointer to directory content structure */
  int dirOK = 0;		/* data dir OK flag */
  char path[40];
  
  dd = opendir(filesystem);
  if ( dd != NULL ) {
    while ( (dirlist = readdir(dd)) ) {
      if ( strcmp(dirlist->d_name,datadir) == 0 ) dirOK = 1;
    }
    closedir(dd);
    if ( !dirOK ) {
      sprintf(path,"%s%s",filesystem,datadir);
      dirOK = mkdir(path , 0666);
    }
  }
  return (dirOK);
}


/*----------------------------------------------------------------------------*
 * init_serialport - initialises the named serial port.                       *
 *            This function is based on the example in the Linux              *
 *            Serial Programming HOWTO - see: http://www.linuxdoc.org         *
 *----------------------------------------------------------------------------*/
int init_serialport(struct port_type port)
{
  int fd;
  /* Open serial device for reading and writing and not as controlling tty
     because we don't want to get killed if line noise sends CTRL-C. */
  fd = open(port.devname, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    perror(port.devname);
    printf("ERROR : failed to open %s\n",port.devname);
    exit(-1);
  }

  /* Set serial port mode on moxa */
#ifdef MOXA
  ioctl(fd,MOXA_SET_OP_MODE,&(port.mode));
#else
  if (port.mode != RS232_MODE) {
    fprintf(stderr,"RS422/485 only supported on moxa devices\nFalling back to RS232\n");
  }
#endif

  bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

  newtio.c_cflag = port.cflag;

  switch (port.mode) {
  case RS232_MODE:
    newtio.c_cflag = newtio.c_cflag | CS8 | CLOCAL | CREAD;
    break;
  case RS485_4WIRE_MODE:
    newtio.c_cflag = newtio.c_cflag | CRTSCTS | CS8 | CLOCAL | CREAD;
    break;
  case RS485_2WIRE_MODE:
    newtio.c_cflag = newtio.c_cflag | CS8 | CLOCAL | CREAD;
    break;
  case RS422_MODE:
    newtio.c_cflag = newtio.c_cflag | CS8 | CLOCAL | CREAD;
    break;
  }

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
 * init_AWS - initialise AWS                                                  *
 *----------------------------------------------------------------------------*/
int init_AWS(int fd)
{
  char buffer[64];
  
  mkh_command(fd, 'A', buffer); 
  mkh_command(fd, 'T', buffer); 
  return(0);
}


/*----------------------------------------------------------------------------*
 * init_clasp - initialise CLASP                                              *
 *----------------------------------------------------------------------------*/
int init_clasp(int fd)
{
  int OK, count;
  char buffer[256];
  char CRLF[2] = {13,10};

  mkh_command(fd, 'A', buffer);

  /* is CLASP responding? */
  OK = 0;
  count = 0;
  while ( !OK && count<10 ) {
    ++count;
    bzero(buffer, sizeof(buffer));
    mkh_command(fd, '?', buffer);
    if ( strstr(buffer, "!")) {
      printf("CLASP responded\n");
      OK = 1;
    }
  }

  if ( !OK ) return(-1);

  /* ensure output mode is timed binary */
  mkh_command(fd, 'I', buffer);
  bzero(buffer, sizeof(buffer));
  mkh_command(fd, 'M', buffer);
  bzero(buffer, sizeof(buffer));
  mkh_command(fd, '2', buffer);
  mkh_command(fd, CRLF[0], buffer);
  mkh_command(fd, CRLF[1], buffer);
  mkh_command(fd, 'B', buffer);
  tcflush(fd,TCIFLUSH);

  return(0);
}


/*----------------------------------------------------------------------------*
 * stop_mkh_interface - stop an MKH interface                                 *
 *----------------------------------------------------------------------------*/
int stop_mkh_interface(int fd)
{
  const char HALT[] = {'H'};
  int nc;
  
  nc = write(fd, HALT, 1);
  fsync(fd);
  if (nc == -1 ) {
    printf("%s\n",strerror(errno));
  }
  return(nc);
}


/*----------------------------------------------------------------------------*
 * start_mkh_interface - start an MKH interface                               *
 *----------------------------------------------------------------------------*/
int start_mkh_interface(int fd)
{
  const char GO[] = {'G'};
  int nc;

  nc = write(fd, GO, 1);
  fsync(fd);
  if (nc == -1 ) {
     printf("%s\n",strerror(errno));
   }
  return(nc);
}


/*----------------------------------------------------------------------------*
 * mkh_command - send command char and get response from MKH interface        *
 *----------------------------------------------------------------------------*/
void mkh_command(int fd, const char COMMAND, char *response)
{
  int nc, gotc, count, ret;
  char buffer[256];

  /* send command to the interface */
  ret = write(fd, &COMMAND, 1);
  fsync(fd);
  if (ret == -1 ) {
     printf("%s\n",strerror(errno));
  }
  usleep(20); /* give MKH interface time to respond..may neeed to change this*/
  count = 0;
  gotc = 0;
  while ( (!gotc || (nc>0)) && (count < 20) ) {
    ++count;
    bzero(buffer, sizeof(buffer));
    nc = read(fd, buffer, sizeof(buffer)); usleep(300);
    fsync(fd);
    if ( nc > 0 ) {
      gotc = 1;
      strcat(response, buffer);
    }
  }
}



/*----------------------------------------------------------------------------*
 * print_time - print date & time to STDOUT 'DD/MM/YY hh:mm:ss'               *
 *----------------------------------------------------------------------------*/
void print_time()
{
  gettimeofday(&hirestime,NULL);
  datetime = *gmtime(&hirestime.tv_sec);
  printf("%02d/%02d/%02d %02d:%02d:%02d : ",
          datetime.tm_mday, datetime.tm_mon+1, datetime.tm_year-100,
          datetime.tm_hour, datetime.tm_min, datetime.tm_sec);
}


/*----------------------------------------------------------------------------*
 * make_filetime - construct the date/time portion of filename                *
 *----------------------------------------------------------------------------*/
void make_filetime(char *filetime_ptr)
{
  char filetime[17];
  
  gettimeofday(&hirestime,NULL);
  datetime = *gmtime(&hirestime.tv_sec);
  
  sprintf(filetime,"%02d%02d%02d_%02d%02d%02d.%02ld", datetime.tm_year-100,
          datetime.tm_mon+1, datetime.tm_mday,  datetime.tm_hour,
          datetime.tm_min, datetime.tm_sec, (hirestime.tv_usec/10000) );
  strcpy(filetime_ptr, filetime);
}


/*----------------------------------------------------------------------------*
 * new_file - make a new (empty) data file                                    *
 *----------------------------------------------------------------------------*/
void new_file(struct port_type port, char *filename_ptr)
{
  char filename[40];
  char path[30];
  int fd;

  sprintf(path,"%s%s",filesystem,datadir);

  /* construct data file filename - note filename_ptr initially contains just *
   * the date/time portion of the name. The full filename includes device name*
   * and port number (+ path)                                                 */
  sprintf(filename,"%s/%s.%s", path, filename_ptr,
	  port.instrument);
  printf("creating file : %s\n",filename);
  fd = open(filename, O_WRONLY | O_TRUNC | O_CREAT | O_BINARY, 0666);
  if ( fd < 0 ) {
    printf("error creating file : %s\n",filename);
    error_message = strerror(errno);
    printf("  %s\n",error_message);
  }
  close(fd);
  /* return the fill filename to the calling function */
  strcpy(filename_ptr, filename); 
}


/*----------------------------------------------------------------------------*
 * read_data - read data from serial port                                     *
 *----------------------------------------------------------------------------*/
int read_data(int fd, char *const data_buffer, int bufsize)
{
  char abyte;
  int nc;

  nc = read(fd, &abyte, 1);
  if (nc > 0) {
    /* shift data buffer contents down 1 */
    for (n=0;n<bufsize-1;++n) {
      data_buffer[n] = data_buffer[n+1];
    }
    /* read new data byte into last element of buffer */
    data_buffer[bufsize-1] = abyte; 
  }
  return(nc);
}


/*----------------------------------------------------------------------------*
 * read_ascii_data - read variable length ascii data from serial port         *
 *----------------------------------------------------------------------------*/
int read_ascii_data(int fd, char *const data_buffer, int msglen)
{
  char abyte;
  int nc;

  nc = read(fd, &abyte, 1);
  if (nc > 0) {
    /* add character read to the end of the message */
    data_buffer[msglen] = abyte;
  }
  return (nc);
}


/*----------------------------------------------------------------------------*
 * write_message - write data message to file                                 *
 *----------------------------------------------------------------------------*/
int write_message(char filename[], char data_buffer[], int bufsize)
{
  int nc;               /* number of characters written */
  int fd;
  
  fd = open(filename, O_WRONLY | O_APPEND | O_BINARY, 0666); 
  if ( fd < 0 ) {
    print_time();
    printf("! error opening file for append : %s\n",filename);
    error_message = strerror(errno);
    printf("  %s\n",error_message);
  }
  
  nc = write(fd,data_buffer,bufsize);
  if ( nc < 0 ) {
    print_time();
    printf("! error writing to : %s\n",filename);
    printf("  buffer = %s\n",data_buffer);
    error_message = strerror(errno);
    printf("  %s\n",error_message);
  }
  close(fd);
  return(nc);
}



/*----------------------------------------------------------------------------*
 * log_it_all - main logging loop                                             *
 *----------------------------------------------------------------------------*/
void log_it_all()
{
  const char LF = {10};
  char filename[NPORTS][40];
  char filetime[17];
  int nc, len;
  int i;
  int starttime;                /* start time of logging interval */
  char timemsg[26];

  /* resync system time to GPS */
  //system("./GPSsync /dev/ttyM5");
  //system("./sync2gps");

  /* Create a new file for each device */
  make_filetime(filetime);
  starttime = hirestime.tv_sec;
  for (i=0;i<nports_used;i++) {
    strcpy(filename[i],filetime);       // temporary filename = date/time
    new_file(portconf[i], filename[i]); // updated on file open
  }

 
  printf("LOGGING!\n");

  while (!alldone) {

    for (i = 0; i<nports_used; i++) {
      switch (portconf[i].datamode) {
      case BINARY:
        len = strlen(portconf[i].headbytes);
	nc = read_data(fd_dev[i], buf_data[i], portconf[i].msglen);
	if (nc > 0) {
	  if ( memcmp(portconf[i].headbytes,buf_data[i],len) == 0 ) {
            portconf[i].gotmsg = 1;
            if (portconf[i].OK == 1) {
              /* got a full data message - write to file and clear buffer */
              nc = write_message(filename[i],buf_data[i],portconf[i].msglen);
              /* append sample time to data message in file */
              bzero(timemsg,sizeof(timemsg));
              memcpy(timemsg,&sampletime[i].tv_sec,
                     sizeof(sampletime[i].tv_sec));
              nc = write_message(filename[i],timemsg,
                                  sizeof(sampletime[i].tv_sec));
              memcpy(timemsg,&sampletime[i].tv_usec,
                     sizeof(sampletime[i].tv_usec));
              nc = write_message(filename[i],timemsg,
                                  sizeof(sampletime[i].tv_usec));

  	      bzero(buf_data[i], portconf[i].msglen);
	    }
            else {
              /* this is first message read, don't have a timestamp for first
                 byte so discared and set OK to log flag true */
   	      portconf[i].OK = 1;
	      bzero(buf_data[i], portconf[i].msglen);
            }
          } 
          else if ( portconf[i].gotmsg == 1) {
            /* This is first byte of new message - get sample time and 
               clear the gotmsg flag */
            gettimeofday(&sampletime[i],NULL);
	    portconf[i].gotmsg = 0;
          }     
	}
	break;

      case ASCII:
	nc = read_ascii_data(fd_dev[i], buf_data[i], strlen(buf_data[i]));
	if (nc > 0) {
	  if ( buf_data[i][strlen(buf_data[i]) - 1] == LF ) {
            /* got a full data message */
            portconf[i].gotmsg = 1;
            if (portconf[i].OK == 1) {
              /* construct time string to add to data message */
              bzero(timemsg,sizeof(timemsg));
              datetime = *gmtime(&sampletime[i].tv_sec);
              sprintf(timemsg,"%04d %02d %02d %02d %02d %02d.%06ld ", 
                      datetime.tm_year+1900, datetime.tm_mon+1, 
                      datetime.tm_mday,  datetime.tm_hour, datetime.tm_min, 
                      datetime.tm_sec, sampletime[i].tv_usec );
              nc = write_message(filename[i], timemsg, strlen(timemsg));
  	      nc = write_message(filename[i], buf_data[i], strlen(buf_data[i]));
	      bzero(buf_data[i], portconf[i].msglen);
	    }
            else {
              /* this is the first message read - might be partial, so 
                 junk it and set OK to log flag true */
   	      portconf[i].OK = 1;
	      bzero(buf_data[i], portconf[i].msglen);
	    }
          }
          else if ( portconf[i].gotmsg == 1 ) {
            /* This is first byte of new message - get sample time and 
               clear the gotmsg flag */
            gettimeofday(&sampletime[i],NULL);
	    portconf[i].gotmsg = 0;
          }
        }
	break;

      } // end of switch
    } // end of for loop through nports


    gettimeofday(&hirestime,NULL);
    /* start new files on interval set in filelength */
    if ( ((hirestime.tv_sec - starttime) >= filelength) ) { 

      /* create new files */
      if (!alldone) {
        make_filetime(filetime); 
        starttime = hirestime.tv_sec;
        for (i=0;i<nports_used;i++) {
          strcpy(filename[i],filetime); // temporarily set filename to date/time
	  new_file(portconf[i], filename[i]); // then update it with full name
        }
      }
    }
    
  }/* end of while (!alldone) loop */
}


/*----------------------------------------------------------------------------*
 * getline - Read a single line in from file                                  *
 *----------------------------------------------------------------------------*/
int getline(int fd, char *line, int maxchar) {
  char c;
  int n;	/* number of chars read on current line of file */
  int err;

  do {
    n = 0;
    err = read(fd, &c,1);
    while ( (err != 0) && (c != '\n') && (c != '#') && (n < maxchar-1)) {
      line[n] = c;
      err = read(fd, &c,1);
      n++;
    }
    line[n] = '\0';

    if ( (n == maxchar-1) || (c == '#') ) {
      /* discard comments, or excess characters on rest of line */
      while ((err == 1) && (c != '\n')) {
	err = read(fd, &c, 1);
      }
    }
  }
  while ((n == 0) && (err == 1));

  if ((n == 0) && (err == 0)) n = -1; /* end of file */
    
  return n;
}


/*----------------------------------------------------------------------------*
 * print_port_config - display port configuration                             *
 *----------------------------------------------------------------------------*/
void print_port_config(struct port_type port) {
  printf("Port  %s : %s : Mode=%s  msglen=%d ", port.devname,
          port.instrument, modenames[port.mode], port.msglen);
  switch (port.cflag) {
  case B9600:
    printf("Baud = 9600\n");
    break;
  case B19200:
    printf("Baud = 19200\n");
    break;
  case B38400:
    printf("Baud = 38400\n");
    break;
  case B57600:
    printf("Baud = 57600\n");
    break;
  case B115200:
    printf("Baud = 115200\n");
    break;
  }
}


/*----------------------------------------------------------------------------*
 * skipwhitespace - return location of next non-whitespace character on line  *
 *----------------------------------------------------------------------------*/
char *skipwhitespace(char *ptr)
{
  while (isspace(*ptr) && (*ptr != '\0')) {
     ptr++;
  }
  return(ptr);
}


/*----------------------------------------------------------------------------*
 * get_config - Read in configuration file for logger.                        *
 *----------------------------------------------------------------------------*/
void get_config() {
  int i, j, k;	// counters
  char fname[] = "./logger.conf";  // config file to read
  int fd;
  char line[256];
  char *ptr, *ptr2;
  int len;
  char temp[25];
  char startbytes[8];  // start or message byte sequence
  int mode,  baud;

  /* Read in file a line at a time, ignore comments with #               *
   * device format is:                                                   *
   *   <devname> <devtype> <mode> <baud> <msglen>                        *
   *   <devname> is the name of the serial port, e.g. /dev/ttyM0         *
   *   <devtype> is the device attached to the port and can be one of:   *
   *             none, metek, gill, CLASP3, CLASP4, licor, DUST,         *
   *             LGR, MTiG                                               *
   *   <mode>    is the serial port mode and can be one of:              *
   *             rs232, rs485_2wire, rs422 and rs485_4wire               *
   *   <baud>    is the serial port baud rate and can be one of:         *
   *             9600, 19200, 38400,...                                  *
   *   <msglen>  Length of data message (or max length if variable)      *
   *   <group>   group number that device belongs to                     *
 
   * general system configuration follows format:                        *
   *   set <option> <value>                                              *
   * current options & valid values are:                                 *
   *   filelength <n>	  : where n is file duration in seconds          *
   *   resync <yes/no>    : resync instruments between files?            *
   *   groups <n>	  : where n = 1..4 (number of groups active)     *

   * (check recent conf file for documentation of any changes to spec)   */
  
  fd = open(fname,O_RDONLY);
  
  i = 0;
  while ((getline(fd, line, sizeof(line)) != -1) && (i < NPORTS) ) {
    ptr = &line[0];

    /* Skip leading whitespace */
    ptr = skipwhitespace(ptr);
    ptr2 = ptr;
      
 
    /* Read in first word - device name or option 'set' */
    while ( !isspace(*ptr2) && (*ptr2 != '\0') ) {
      ptr2++;
    }
    len = ptr2-ptr;
    
    if (strncasecmp(ptr,"set",3) == 0) {
      /* ================================================================== */
      /* =============== this is a generic option setting ================= */
      /* ================================================================== */
      ptr2 = skipwhitespace(ptr2);
      ptr = ptr2;

      /* read in word */
      while (!isspace(*ptr2) && (*ptr2 != '\0')) {
        ptr2++;
      }
      if (strncasecmp(ptr,"filelength",10) == 0) {
        ptr2 = skipwhitespace(ptr2);;
        ptr = ptr2;

        filelength = strtol(ptr,NULL,10);
      }

    } 

    else {
      /* ================================================================== */
      /* ================ this is a device property setting ================ */
      /* ================================================================== */
      if (len >= sizeof(temp)) {
        len = sizeof(temp)-1;
      }
      strncpy(temp,ptr,len);
      temp[len] = '\0';
      strncpy(portconf[i].devname,temp,sizeof(portconf[i].devname));

      ptr2 = skipwhitespace(ptr2);
      ptr = ptr2;

      /* Read in instrument name */
      while (!isspace(*ptr2) && (*ptr2 != '\0')) {
        ptr2++;
      }
      len = ptr2-ptr;
      if (len >= sizeof(temp)) {
        len = sizeof(temp)-1;
      }
      strncpy(temp,ptr,len);
      temp[len] = '\0';
      strncpy(portconf[i].instrument,temp,sizeof(portconf[i].instrument));

      ptr2 = skipwhitespace(ptr2);
      ptr = ptr2;
      
      /* Read in serial mode */
      while (!isspace(*ptr2) && (*ptr2 != '\0')) {
        ptr2++;
      }
      mode = RS232_MODE;
      if (strncasecmp(ptr,"rs232",5) == 0) {
  	mode = RS232_MODE;
      }
      else if (strncasecmp(ptr,"rs485_2wire",11) == 0) {
  	mode = RS485_2WIRE_MODE;
      }
      else if (strncasecmp(ptr,"rs422",5) == 0) {
	mode = RS422_MODE;
      }
      else if (strncasecmp(ptr,"rs485_4wire",11) == 0) {
 	mode = RS485_4WIRE_MODE;
      }

      portconf[i].mode = mode;

      ptr2 = skipwhitespace(ptr2);
      ptr = ptr2;

      /* Read in baud rate */
      while (!isspace(*ptr2) && (*ptr2 != '\0')) {
        ptr2++;
      }
      baud = strtol(ptr,NULL,10);

      switch (baud) {
      case 9600:
        portconf[i].cflag = B9600;
        break;
      case 19200:
        portconf[i].cflag = B19200;
        break;
      case 38400:
        portconf[i].cflag = B38400;
        break;
      case 57600:
        portconf[i].cflag = B57600;
        break;
      case 115200:
        portconf[i].cflag = B115200;
        break;
      default:
        printf("Unknown baud rate \n");
        exit(-1);
        break;
      }

      ptr2 = skipwhitespace(ptr2);
      ptr = ptr2;

      /* read in message length */
      while (!isspace(*ptr2) && (*ptr2 != '\0')) {
        ptr2++;
      }
      portconf[i].msglen = strtol(ptr,NULL,10);
  
      ptr2 = skipwhitespace(ptr2);
      ptr = ptr2;

      /* read in data type: Ascii/Binary */
      while (!isspace(*ptr2) && (*ptr2 != '\0')) {
        ptr2++;
      }
      for (j=0; j<NUM_DATA_MODES;j++) {
        if (strncasecmp(ptr,datamodenames[j],len) == 0) {
          portconf[i].datamode = j;
        }
      } 

      /* deal with ASCII/BINARY mode specific options */
      switch (portconf[i].datamode) {
      case BINARY:
        /* read in binary start of message marker */
        bzero(startbytes,sizeof(startbytes));
        k = 0; 
        while ( (k < sizeof(startbytes)) && (*ptr2 != '\0') ) {

          ptr2 = skipwhitespace(ptr2);
          ptr = ptr2;

          while (!isspace(*ptr2) && (*ptr2 != '\0')) {
            ptr2++;
          }
          startbytes[k] = atoi(ptr);
          k++;
        }
        len = sizeof(portconf[i].headbytes);
        strncpy(portconf[i].headbytes,startbytes,len);
        break;

      case ASCII:
        break;
      }


      i++;

    } /* -- end of device settings -- */

  } /* -- end of configuration file import -- */
 
 
  nports_used = i;
  close(fd);

  for (j=0;j<i;j++) {
    /* set OK to log flag to false - only used for ASCII streams */
    portconf[j].OK = 0;
    portconf[j].gotmsg = 0;
  }

  /* print out the configuration being used */
  printf("\n=== CONFIGURATION =============================================\n");
  for (i=0;i<nports_used;++i) {
    print_port_config(portconf[i]);
  }
  printf("filelength = %d seconds\n",filelength);
  printf("=================================================================\n");

}


