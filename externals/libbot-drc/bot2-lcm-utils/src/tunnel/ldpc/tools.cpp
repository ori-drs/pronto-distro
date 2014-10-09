/*
 * The contents of this directory and its sub-directories are 
 * Copyright (c) 1995-2003 by Radford M. Neal
 * 
 * Permission is granted for anyone to copy, use, modify, or distribute these
 * programs and accompanying documents for any purpose, provided this copyright
 * notice is retained and prominently displayed, along with a note saying 
 * that the original programs are available from Radford Neal's web page, and 
 * note is made of any changes made to these programs.  These programs and 
 * documents are distributed without any warranty, express or implied.  As the
 * programs were written for research purposes only, they have not been tested 
 * to the degree that would be advisable in any important application.  All use
 * of these programs is entirely at the user's own risk.
 */


/* This module implements operations on sparse matrices of mod2 elements 
   (bits, with addition and multiplication being done modulo 2).  
  
   All procedures in this module display an error message on standard 
   error and terminate the program if passed an invalid argument (indicative
   of a programming error), or if memory cannot be allocated.  Errors from 
   invalid contents of a file result in an error code being returned to the 
   caller, with no message being printed by this module. 
*/


/* DATA STRUCTURES USED TO STORE A SPARSE MATRIX.  Non-zero entries (ie, 1s)
   are represented by nodes that are doubly-linked both by row and by column,
   with the headers for these lists being kept in arrays.  Nodes are allocated
   in blocks to reduce time and space overhead.  Freed nodes are kept for
   reuse in the same matrix, rather than being freed for other uses, except 
   that they are all freed when the matrix is cleared to all zeros by the
   mod2sparse_clear procedure, or copied into by mod2sparse_copy. 

   Direct access to these structures should be avoided except in low-level
   routines.  Use the macros and procedures defined below instead. */

#include "tools.h"

/* ALLOCATE SPACE AND CHECK FOR ERROR.  Calls 'calloc' to allocate space,
   and then displays an error message and exits if the space couldn't be
   found. */

void *chk_alloc
( unsigned n,		/* Number of elements */
  unsigned size		/* Size of each element */
)
{ 
  void *p;

  p = calloc(n,size);

  if (p==0)
  {
	  fprintf(stderr,"Ran out of memory (while trying to allocate %d bytes)\n", n*size);
	  exit(1);
  }

  return p;
}


#if 0
/* READ AN INTEGER ONE BYTE AT A TIME.  Four bytes are read, ordered from
   low to high order.  These are considered to represent a signed integer,
   in two's complement form.  The value returned is this integer, converted 
   to whatever a C "int" is.  The conversion should work as long as an "int" 
   is at least four bytes, even if it's not in two's complement representation 
   (except for the largest two's complement negative integer).

   If an error or eof is encountered, zero is returned.  The caller can
   check for these events using feof and ferror.  

   The file read from should have been opened as "binary".
*/

int intio_read
( FILE *f   /* File to read from */
)
{ 
  unsigned char b[4];
  int top;
  int i;

  for (i = 0; i<4; i++)
  { if (fread(&b[i],1,1,f) != 1) return 0;
  }

  top = b[3]>127 ? (int)b[3] - 256 : b[3];
  
  return (top<<24) + (b[2]<<16) + (b[1]<<8) + b[0];
}


/* WRITE AN INTEGER ONE BYTE AT A TIME.  Four bytes are written, ordered from
   low to high order.  These are considered to represent a signed integer,
   in two's complement form.  This should work as long as the integer passed
   can be represented in four bytes, even if a C "int" is longer than this.

   The file written to should have been opened as "binary".
*/

void intio_write
( FILE *f,  /* File to write to */
  int v     /* Value to write to file */
)
{ 
  unsigned char b;
  int i;

  for (i = 0; i<3; i++)
  { b = v&0xff;
    fwrite(&b,1,1,f);
    v >>= 8;
  }

  b = v>0 ? v : v+256;
  fwrite(&b,1,1,f);
}
#endif
