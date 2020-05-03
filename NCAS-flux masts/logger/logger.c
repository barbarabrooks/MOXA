/*----------------------------------------------------------------------------*
 * logger       logger for METEK & Licor turbulence instruments               *
 *              based on IMB's ACSE logger                                    *
 *                                                                            *
 * B J Brooks  :                                                              *
 * version 1.0 : 1/1/2016                                                     *
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
#include <signal.h>			            /* signal handling functions	      */
#include <errno.h>		            	/* error message macros		          */
#include <dirent.h>			            /* directory functions		          */
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

/* Supported instrument types - NB edit all 3 lines below when changing */
#define NUM_DEV_TYPES 8
enum device {none = 0, metek = 1, gill = 2, CLASP3 = 3, licor = 4, AWSascii = 5, CLASP4 = 6, DUST = 7};
char *devnames[8] = {"none","metek","gill","CLASP3","licor","AWSascii", "CLASP4", "DUST"};

/* Define serial port properties structure */
/* Baudrate settings defined in <asm/termbits.h>, included by <termios.h> */
struct port_type {
  int portnum;         /* port number (1-8)                                 */
  char devname[25];
  tcflag_t cflag;      /* Control options for port -> most important is the */
                       /* baud rate e.g. B19200.                            */
  int mode;            /* Serial port mode, e.g. RS232                      */
  enum device dev;     /* One of enumerated list of supported devices       */
  int msglen;	       /* length of message (or max length if variable)     */
  int group;	       /* which group this device belongs to                */
};



/*--- declare global variables ---------------------------------------------*/

const char filesystem[] = "/";
const char datadir[] = "mnt/usbstorage";

int nports_used;
struct port_type portconf[NPORTS];

/*-- file handles --*/
int fd;                                 /* generic serial port file handle  */
int fd_dev[NPORTS];	                    /* kit specific file handles	    */

/* array of data buffers, size of [number of ports]x[max size] - max size is*
 * set to something larger than longest message we expect to get from any   *
 * instrument...340 bytes - a CLASP with 16 channels at 16 bits each        *  
 *   NB this is sort of out of date now since CLASP is written byte by byte */
char buf_data[NPORTS][512];

char *error_message;			       /* pointer to error message string  */

/* misc control variables */
int n;                                 /* generic counter                   */
int alldone;			           	   /* master control flag	            */
int resync_flag;			           /* do we resync between files?       */
int filelength;				           /* length of file in seconds         */
/* instrument group variables */
int ngroups;			               /* number of active instrment groups */
int licorport[4];			           /* index into portconf for licors in *
                                        * each instrument group	            */ 
int n_in_group[4];			           /* number of instruments in group    */
int group_ports[4][8];			       /* port numbers in each group	    */

/* glibc defined structures */
struct termios oldtio,newtio;          /* terminal configuration structures */
struct timeval hirestime;              /* hi res time structure             */
struct tm datetime;                    /* full date and time structure      */


/*--- declare function prototypes ------------------------------------------*/

/* generic system functions */

int init_serialport(struct port_type port);    /* initialise serial port             */
void get_config();			                   /* get system config from conf file   */
void log_it_all(void);                         /* log sonic output to file           */
void displaysonic(void);                       /* display sonic output to terminal   */
void sigterm_handler(int signum);              /* handle termination signals         */
void cleanup(void);			                   /* pre-exit cleanup		             */
void save_pid(void);			               /* save process id to file	         */
void print_time(void);                         /* print date & time to STDOUT        */
int dir_check(void);
//int is_mounted(char *dirname);	           /* check data card is mounted         */

/* data read/write functions */
void new_file(struct port_type port, char *const filename_ptr);
int read_data(int fd, char *data_buffer_ptr, int bufsize);
int read_ascii_data(int fd, char *const data_buffer, int msglen);
int write_message(char filename[], char data_buffer[], int bufsize);

/* instrument specific functions */
void stop_all_instruments();	              /* stop everything that can be         */
void start_instrument(int i);	              /* start an instrument                 */
int sync_to_licor(int fd);	                  /* sync to LICOR data stream           */
//int start_gill_sonic(int fd);	              /* start sonic 			             */
//int stop_gill_sonic(int fd);	              /* stop sonic 	       		         */
//int start_mkh_interface(int fd);            /* start MKH interface to inclinometers*/
//int stop_mkh_interface(int fd);	          /* stop MKH interface to inclinometers */
//void mkh_command(int fd, const char COMMAND, char *response);
				                              /* send command to MKH interface, get  *
                                               * response if any                     */
//int init_clasp(int fd);                     /* initialise CLASP settings           */
//int init_AWS(int fd);		                  /* initialise AWS */


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
  alldone = 0;		                                    /* not finished yet */
  resync_flag = 0;	                                    /* don't resync between files */
  filelength = 3600;	                                /* new file at 1-hour intervals */
  for (i=0;i<4;i++) {
    licorport[i] = -1; 	                                /* set nvalid port number for licor */
  }
  /* check the data directory exists */
  dir_check();

  /* Set up system configuration - port/instrument pairs, etc */
  get_config();

  /* open and configure serial ports */
  printf("initialising instruments:\n");
  for (i = 0; i< nports_used; i++) {
    fd_dev[i] = init_serialport(portconf[i]);

   /* initialise and stop instruments ready to start all together */
    switch (portconf[i].dev) {
    case gill:
      //print_time(); 
      //printf("stopping sonic on %s\n",portconf[i].devname);
      //while (!stop_gill_sonic(fd_dev[i]) && !alldone); 
      break;
    case CLASP3:
      //stop_mkh_interface(fd_dev[i]);
      //init_clasp(fd_dev[i]);
      break;
    case CLASP4:
      //stop_mkh_interface(fd_dev[i]);
	  break;
    case metek:
      break;
    case licor:
      break;
    case AWSascii:
      //stop_mkh_interface(fd_dev[i]);
      //init_AWS(fd_dev[i]);
      break;
    case DUST:
      break;
    case none:
      break;
    }
  }

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
 * cleanup - cleanup everything before exiting                                *
 *----------------------------------------------------------------------------*/
void cleanup()
{
  int i;
  for (i=0;i<nports_used;i++) {
    /* shut down instruments if required */
    switch (portconf[i].dev) {
    case CLASP3:
      //printf("...stopping CLASP interface on %s\n",portconf[i].devname);
      //stop_mkh_interface(fd_dev[i]);
      break;
    case CLASP4:
      //printf("...stopping CLASP interface on %s\n",portconf[i].devname);
      //stop_mkh_interface(fd_dev[i]);
      break;
    case gill:
    case metek:
    case licor:
    case AWSascii:
      //stop_mkh_interface(fd_dev[i]);
      break;
    case DUST:
      break;
    case none:  
      break;
    }
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
  DIR *dd;			        /* directory stream ID	                  */
  struct dirent *dirlist;	/* pointer to directory content structure */
  int dirOK = 0;		    /* data dir OK flag                       */
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

  bzero(&newtio, sizeof(newtio));     /* clear struct for new port settings */

  newtio.c_cflag = port.cflag;

  switch (port.dev) {
  case metek:
    newtio.c_cflag = newtio.c_cflag | CS8 | CLOCAL | CREAD;
    break;
  case gill:
    //newtio.c_cflag = newtio.c_cflag | CRTSCTS | CS8 | CLOCAL | CREAD;
    break;
  case licor:
    newtio.c_cflag = newtio.c_cflag | CS8 | CLOCAL | CREAD;
    break;
  case CLASP3:
    //newtio.c_cflag = newtio.c_cflag | CRTSCTS | CS8 | CLOCAL | CREAD;
    break;
  case CLASP4:
    //newtio.c_cflag = newtio.c_cflag | CRTSCTS | CS8 | CLOCAL | CREAD;
    break;
  case AWSascii:
    //newtio.c_cflag = newtio.c_cflag | CS8 | CLOCAL | CREAD;
    break;
  case DUST:
    //newtio.c_cflag = newtio.c_cflag | CS8 | CLOCAL | CREAD;
    break;
  case none:
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
 * stop_gill_sonic - stop Gill R3 sonic (put into interactive mode)           *
 *----------------------------------------------------------------------------*/
int stop_gill_sonic(int fd) 
{ 
  char IM_COMMAND[4]={'I','M',13,10};	// command to enter interactive mode
  int gotprompt;			// sonic response flag
  int nc;
  int ret;
  char message[512];
  
  gotprompt = 0;
  n = 0;
  bzero(message,sizeof(message)); // zero the message buffer

  // flush the buffer
  tcflush(fd,TCIFLUSH);
  // send command
  ret = write(fd, IM_COMMAND, 4);
  ret = write(fd, IM_COMMAND, 4);
  fsync(fd); 
  usleep(50000); // wait to allow sonic time to respond.
  if (ret == -1 ) {
    printf("%s\n",strerror(errno));
  }
  
  /* look for sonic prompt string "R3> " to be returned...it may take more 
     than one attempt to get sonic into interactive mode.		     */
  
  nc = read(fd,message,sizeof(message)-1);
  if (strstr(message,"R3>")) {
    gotprompt = 1;
  }
  
  return(gotprompt);
}


/*----------------------------------------------------------------------------*
 * start_gill_sonic - start Gill R3 sonic (take out of interactive mode)      *
 *----------------------------------------------------------------------------*/
int start_gill_sonic(int fd)
{
  char EXIT_COMMAND[6]={'e','x','i','t',13,10}; // exit interactive mode
  int nc;
  
  tcflush(fd,TCIFLUSH);
  nc = write(fd, EXIT_COMMAND, strlen(EXIT_COMMAND));
  fsync(fd);
  return(nc);
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
 * sync_to_licor - sync to licor data stream before starting everything else  *
 *----------------------------------------------------------------------------*/
int sync_to_licor(int fd)
{
  int nc;
  char buffer;
  
  tcflush(fd,TCIFLUSH);
  nc = 1;
  while (nc > 0) {
    /* make sure the licor buffer is clear */
    nc = read(fd, &buffer, 1);
  }
  /* now wait until we get the first new message through */
  while (buffer != 10) {
    nc = read(fd, &buffer, 1);
  }
  return(0);
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
  sprintf(filename,"%s/%s.%s%d", path, filename_ptr,
	  devnames[port.dev],port.portnum);
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
 * write_metek_data - write data packet to file                               *
 *----------------------------------------------------------------------------*/
int write_metek_data(char filename[], struct metek_data *data)
{
  int nc;               /* number of characters written */
  int fd;
  
  fd = open(filename, O_WRONLY | O_APPEND | O_BINARY, 0666); 
  if ( fd < 0 ) {
    printf("! error opening file for append : %s\n",filename);
    error_message = strerror(errno);
    printf("  %s\n",error_message);
  }
  
  nc = write(fd,(void *)data,sizeof(struct metek_data));
  if ( nc < 0 ) {
    printf("! error writing to : %s\n",filename);
    error_message = strerror(errno);
    printf("  %s\n",error_message);
  }
  close(fd);
  return(nc);
}


/*----------------------------------------------------------------------------*
 * parse_metek - check Metek data message                                     *
 *               store results in a metek_data struct - follows format of Gill*
 *               sonic binary data format                                     *
 *               returns 1 if a new file needs to be started (packets lost)   *
 *----------------------------------------------------------------------------*/
int parse_metek(char data_buffer[], int bufsize, char filename[],
	        struct metek_data *data)
{
  char *datamsg; 	/* pointer to data message from Metek */
  char msgID;		/* message type indentifier */
  char *ptr;
  int newfile;

  newfile = 0;
  gettimeofday(&hirestime,NULL); 
  
  datamsg = memchr(data_buffer,':',bufsize); 
  if (datamsg != NULL) {
    datamsg = datamsg-1;
    msgID = datamsg[0];
    /* Check if this is a data message */
    switch (msgID) {
      case 'M':
      case 'H':
      case 'D':
        /* Starting new data packet - check if previous one existed (err >= 0)
           and write it out */
        if (data->err != METEK_NO_PREV_DATA) {
	    write_metek_data(filename,data);
        }

        /* Initially we have no error */
        data->err = METEK_NO_ERROR;
        data->errparm = 0;

        /* Parse out data */
        ptr = datamsg;
        ptr = strchr(ptr,'=');
        if (ptr == NULL) {
          data->x = 0;
          data->err = METEK_CORRUPT_MESSAGE;
          data->errparm = 0;
        }
        else {
          ptr++;
          data->x = (int16_t) strtol(ptr,NULL,10);
	  ptr = strchr(ptr,'=');
	}
        if (ptr == NULL) {
          data->y = 0;
          data->err = METEK_CORRUPT_MESSAGE;
          data->errparm = 1;
	}
        else {
          ptr++;  
          data->y = (int16_t) strtol(ptr,NULL,10);
	  ptr = strchr(ptr,'=')+1;
	}
        if (ptr == NULL) {
          data->z = 0;
          data->err = METEK_CORRUPT_MESSAGE;
          data->errparm = 2;
	}
        else {
          ptr++;
          data->z = (int16_t) strtol(ptr,NULL,10);
	  ptr = strchr(ptr,'=');
	}
        if (ptr == NULL) {
          data->t = 0;
          data->err = METEK_CORRUPT_MESSAGE;
          data->errparm = 3;
	}
        else {
          ptr++;
          data->t = (int16_t) strtol(ptr,NULL,10);
	}
        break;
      case 'E':
        /* Error message - need to parse this out */
	printf("%s : %3ld : %s\n",filename,hirestime.tv_usec/1000,datamsg);
        if (strncmp(datamsg,"E:data lost",11) == 0) {
          data->err = METEK_DATA_LOST;
          data->errparm = (int8_t) strtol(&datamsg[12],NULL,10);
        }
        else if (strncmp(datamsg,"E:invalid data",14) == 0) {
          data->err = METEK_INVALID_DATA;
          /* Don't attempt to parse the error - it doesn't help us */
          data->errparm = 0;
        }
        else if (strncmp(datamsg,"E:quality <",11) == 0) {
          data->err = METEK_LOW_QUALITY;
          data->errparm = (int8_t) strtol(&datamsg[12],NULL,10);
        }
        else {
          /* Unknown message - print it out */
	  printf("%s",datamsg);
          data->err = METEK_UNKNOWN_ERROR;
          data->errparm = 0;
        }
        break;
      case 'C':
      case 'R':
      case 'T':
        /* Ignore command, response and time messages */
        break; 
      default:
        printf("Unknown message : %s at %ld\n",datamsg,hirestime.tv_usec/1000);
    }
  }
  return newfile;
}


/*----------------------------------------------------------------------------*
 * start_instrument - start specified instrument                              *
 *----------------------------------------------------------------------------*/
void start_instrument(int i)
{
printf("starting port %s\n",portconf[i].devname);
  tcflush(fd_dev[i],TCIFLUSH);
  switch (portconf[i].dev) {
  case gill:
    start_gill_sonic(fd_dev[i]);
    break;
  case CLASP3:
    start_mkh_interface(fd_dev[i]);
    break;
  case CLASP4:
    start_mkh_interface(fd_dev[i]);
    break;
  case licor:
    break;
  case metek:
    // send a reset command?
    break;
  case AWSascii:
    start_mkh_interface(fd_dev[i]);
    break;
  case DUST:
    break;
  case none:
    break;
  }
}


/*----------------------------------------------------------------------------*
 * stop_all_instruments - stop each instrument in turn                        *
 *----------------------------------------------------------------------------*/
void stop_all_instruments()
{ 
  int i;

  /* flush serial lines to clear buffers before we start doing anything */
  for (i=0;i<nports_used;i++) {
    tcflush(fd_dev[i],TCIFLUSH);
  }
  for (i=0;i<nports_used;i++) {
    tcflush(fd_dev[i],TCIFLUSH);
    switch (portconf[i].dev) {
    case gill:
      //print_time(); 
      //printf("stopping gill sonic on %s\n",portconf[i].devname);
      while (!stop_gill_sonic(fd_dev[i]) && !alldone) ;
      break;
    case CLASP3:
      //print_time();
      //printf("stopping CLASP on %s\n",portconf[i].devname);
      stop_mkh_interface(fd_dev[i]);
      break;
    case CLASP4:
      stop_mkh_interface(fd_dev[i]);
      break;
    case metek:
    case licor:
    case AWSascii:
      stop_mkh_interface(fd_dev[i]);
      break;
    case DUST:
      break;
    case none:
      break;
    }
  }

}

/*----------------------------------------------------------------------------*
 * resync - stop and resync all instruments and start new data files          *
 *----------------------------------------------------------------------------*/
void resync(void)
{ 
  int i, j;


  /* flush serial lines to clear buffers before we start doing anything */
  for (i=0;i<nports_used;i++) {
    tcflush(fd_dev[i],TCIFLUSH);
  }

  /* stop instruments */
  stop_all_instruments();
 
  print_time();
  printf("instruments stopped for resync\n");

  /* resync system time to GPS */
  //system("./GPSsync /dev/ttyM0");
  system("./sync2gps"); // call shell script to run GPSsync - edit shell script
                       // to specify which serial port GPS is attached to

  /* flush the serial buffers again before restarting */
  for (i=0;i<nports_used;i++) {
    tcflush(fd_dev[i],TCIFLUSH);
  }

  if ( !alldone ) {
    for (i=0;i<ngroups;i++) {
      /* if we are syncing to licor - do it*/
      if ( !(licorport[i]<0) ) {
        printf("syncing group %d to licor data stream\n",i+1);
        sync_to_licor(fd_dev[licorport[i]]);
      }
      /* start all instruments in this group */
      for (j=0;j<n_in_group[i];j++) {
        start_instrument(group_ports[i][j]);
      }
    } 

  }
  fflush(stdout);
}


/*----------------------------------------------------------------------------*
 * log_it_all - main logging loop                          *
 *----------------------------------------------------------------------------*/
void log_it_all()
{
  const char BA = {0xBA}; 	/* Gill start of binary message marker */
  const char CR = {13};
  const char LF = {10};
  char filename[NPORTS][40];
  char filetime[17];
  struct metek_data data[NPORTS];
  int nc;
  int i, j;
  int starttime;                /* start time of logging interval */
 
  /* Initially we have no data */
  for (i=0;i<nports_used;i++) {
    data[i].err = METEK_NO_PREV_DATA;
  }

  /* resync system time to GPS */
  //system("./GPSsync /dev/ttyM0");
  system("./sync2gps");

  for (i=0;i<ngroups;i++) {
    /* if we're syncing to licor - do it */
    if ( !(licorport[i]<0) ) {
      printf("syncing group %d to licor data stream on %s\n",
              i+1,portconf[licorport[i]].devname);
      sync_to_licor(fd_dev[licorport[i]]);
    }
    /* start all instruments in this group */
    for (j=0;j<n_in_group[i];j++) {
      start_instrument(group_ports[i][j]);
    }
  } 

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
      switch (portconf[i].dev) {
      case metek:
	/* Read Metek sonic */
	nc = read_data(fd_dev[i], buf_data[i], portconf[i].msglen);
	if (nc > 0) {
	  if ( (buf_data[i][portconf[i].msglen-2] == CR) && 
	       (buf_data[i][portconf[i].msglen-1] == LF) ) {
	    if (parse_metek(buf_data[i],portconf[i].msglen,filename[i], 
                &data[i]) == 1) {
              /* start a new file if packets lost */
              make_filetime(filetime);
              strcpy(filename[i],filetime); 
	      new_file(portconf[i], filename[i]);
	    }
	    bzero(buf_data[i],sizeof(buf_data[i]));
	  }
	}
	break;
      case gill:
	/* Read Gill sonic */
	nc = read_data(fd_dev[i], buf_data[i], portconf[i].msglen);
	if (nc > 0) {
	  if ( (buf_data[i][0] == BA) && (buf_data[i][1] == BA) ) {
            nc = write_message(filename[i],buf_data[i],portconf[i].msglen);
	    bzero(buf_data[i], portconf[i].msglen);
	  }
	}
	break;
      case CLASP3:
        nc = read_ascii_data(fd_dev[i], buf_data[i], strlen(buf_data[i]));
          if (nc > 0) {
            nc = write_message(filename[i], buf_data[i], nc);
            bzero(buf_data[i], portconf[i].msglen);
          }
	break;
      case CLASP4:
        nc = read_ascii_data(fd_dev[i], buf_data[i], strlen(buf_data[i]));
          if (nc > 0) {
            nc = write_message(filename[i], buf_data[i], nc);
            bzero(buf_data[i], portconf[i].msglen);
          }
	break;
      case licor:
	/* Read Licor */
	nc = read_ascii_data(fd_dev[i], buf_data[i], strlen(buf_data[i]));
	if (nc > 0) {
	  if ( buf_data[i][strlen(buf_data[i]) - 1] == LF ) {
	    nc = write_message(filename[i], buf_data[i], strlen(buf_data[i]));
	    bzero(buf_data[i], portconf[i].msglen);
          }
        }
	break;
      case AWSascii:
	nc = read_ascii_data(fd_dev[i], buf_data[i], strlen(buf_data[i]));
	if (nc > 0) {
	  if ( buf_data[i][strlen(buf_data[i]) - 1] == LF ) {
	    nc = write_message(filename[i], buf_data[i], strlen(buf_data[i]));
	    // bzero(buf_data[i], portconf[i].msglen);
	    bzero(buf_data[i], sizeof(buf_data[i]));
          }
        }
        break;
      case DUST:
        nc = read_ascii_data(fd_dev[i], buf_data[i], strlen(buf_data[i]));
        if (nc > 0) {
          if ( buf_data[i][strlen(buf_data[i]) - 1] == LF ) {
            nc = write_message(filename[i], buf_data[i], strlen(buf_data[i]));
            // bzero(buf_data[i], portconf[i].msglen);
            bzero(buf_data[i], sizeof(buf_data[i]));
          }
        }

      case none:
	break;
      } // end of switch
    } // end of for loop through nports

    gettimeofday(&hirestime,NULL);
    /* start new files on interval set in filelength */
    if ( ((hirestime.tv_sec - starttime) >= filelength) ) { 

      /* resync instruments if requested */
      if ( resync_flag ) {
        resync();
      }

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
  printf("Port %s  %s : Mode=%s  msglen=%d grp=%d ",port.devname,
          devnames[port.dev],modenames[port.mode], port.msglen, port.group);
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
  int i, j;
  char fname[] = "./logger.conf";
  int fd;
  char line[256];
  char *ptr, *ptr2;
  int len;
  char dev[25];
  enum device type;
  int mode,  baud;
  int grpi;		// index into group array

  /* Read in file a line at a time, ignore comments with #               *
   * device format is:                                                   *
   *   <devname> <devtype> <mode> <baud> <msglen>                        *
   *   <devname> is the name of the serial port, e.g. /dev/ttyM0         *
   *   <devtype> is the device attached to the port and can be one of:   *
   *             none, metek, gill, CLASP, licor                         *
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
  
  /* initialise n_in_group with zeros */
  for (j=0;j<4;j++) {
   n_in_group[j] = 0;
  }

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
      /* =============== this is a generic option setting ================== */
      ptr2 = skipwhitespace(ptr2);
      ptr = ptr2;

      while (!isspace(*ptr2) && (*ptr2 != '\0')) {
        ptr2++;
      }
      if (strncasecmp(ptr,"filelength",10) == 0) {
        ptr2 = skipwhitespace(ptr2);;
        ptr = ptr2;

        filelength = strtol(ptr,NULL,10);
      }
      else if (strncasecmp(ptr,"resync",6) == 0) {
        ptr2 = skipwhitespace(ptr2);
        ptr=ptr2;

        if (strncasecmp(ptr,"yes",3) == 0) {
          resync_flag = 1;
        } else {
          resync_flag = 0;
        }
      } 
      else if (strncasecmp(ptr,"groups",6) == 0) {
        ptr2 = skipwhitespace(ptr2);
        ptr=ptr2;
        ngroups = strtol(ptr,NULL,10);
      }

    } 
    else {
      /* ================ this is a device property setting ================ */
      if (len >= sizeof(dev)) {
        len = sizeof(dev)-1;
      }
      strncpy(dev,ptr,len);
      dev[len] = '\0';

      ptr2 = skipwhitespace(ptr2);
      ptr = ptr2;

      /* Read in instrument type */
      while (!isspace(*ptr2) && (*ptr2 != '\0')) {
        ptr2++;
      }
      len = ptr2-ptr;
      type = none;
      for (j=0;j<NUM_DEV_TYPES;j++) {
        if (strncasecmp(ptr,devnames[j],len) == 0) {
  	  type = j;
        }
      }
      portconf[i].dev = type;

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

      ptr2 = skipwhitespace(ptr2);
      ptr = ptr2;

      /* Read in baud rate */
      while (!isspace(*ptr2) && (*ptr2 != '\0')) {
        ptr2++;
      }
      baud = strtol(ptr,NULL,10);

      portconf[i].portnum = i+1;
      strncpy(portconf[i].devname,dev,sizeof(portconf[i].devname));
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
      default:
        printf("Unknown baud rate - defaulting to 19200\n");
        portconf[i].cflag = B19200;
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

      /* read in group number */
      while (!isspace(*ptr2) && (*ptr2 != '\0')) {
        ptr2++;
      }
      portconf[i].group = strtol(ptr,NULL,10);
      grpi = portconf[i].group-1;
      n_in_group[grpi]++;
      group_ports[grpi][n_in_group[grpi]-1] = i;	//set port number index
      if (strncasecmp(devnames[portconf[i].dev],"licor",5) == 0) {
        // if this is a licor, set the port flag for resyncing
        licorport[grpi] = i;
      }

      portconf[i].mode = mode;
      i++;
    } /* -- end of device settings -- */

  } /* -- end of configuration file import -- */
 
 
  nports_used = i;
  close(fd);

  /* print out the configuration being used */
  printf("\n=== CONFIGURATION =============================================\n");
  for (i=0;i<nports_used;++i) {
    print_port_config(portconf[i]);
  }
  printf("filelength = %d seconds\n",filelength);
  printf("number of groups = %d\n",ngroups);
  if (resync_flag == 1) {
    printf("resync ON\n");
  } 
  else if (resync_flag == 0) {
    printf("resync OFF\n");
  }
  printf("=================================================================\n");

}


