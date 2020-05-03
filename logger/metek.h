/*----------------------------------------------------------------------------*
 * metek.h - include file for metek binary file format / error codes          *
 *----------------------------------------------------------------------------*/

#ifndef _METEK_H

#define _METEK_H 1

/* Error codes for Metek error messages */
#define METEK_NO_PREV_DATA -1 
#define METEK_NO_ERROR 0
#define METEK_INVALID_DATA 1
#define METEK_LOW_QUALITY 2
#define METEK_DATA_LOST 3
#define METEK_UNKNOWN_ERROR 4
#define METEK_CORRUPT_MESSAGE 5

/* Data structure for binary data */
/* Uses gcc packed attribute to ensure in most compact form */
struct metek_data {
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t t;
  int8_t err;
  int8_t errparm;
} __attribute__((packed));


#endif    /* metek.h */
