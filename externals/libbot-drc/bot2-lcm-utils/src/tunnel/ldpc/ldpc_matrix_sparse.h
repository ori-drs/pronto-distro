/* ldpc_matrix.h - Interface to module for handling sparse mod2 matrices. */
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
   
#ifndef LDPC_MATRIX_SPARSE__
#define LDPC_MATRIX_SPARSE__

#include "ldpc_profile.h"	/* general LDPC codec profiles */
#include "tools.h"


/**
 * Structure representing a non-zero entry, or the header for a row or column.
 */
typedef struct mod2entry
{
	/**
	 * Row and column indexes of this entry, starting at 0, and
	 * with -1 for a row or column header
	 */
#ifdef SPARSE_MATRIX_OPT_SMALL_INDEXES /* memory opt., see ldpc_profile.h */
	short	row;
	short	col;
#else
	int	row;
	int	col;
#endif

	/**
	 * Pointers to entries adjacent in row and column, or to headers.
	 * Free entries are linked by 'left'.
	 */
	struct mod2entry	*left,
				*right,
				*down;
#ifndef SPARSE_MATRIX_OPT_FOR_LDPC_STAIRCASE /* memory opt., see ldpc_profile.h */
	struct mod2entry	*up;
#endif
#if 0
	double pr, lr;	  /* Probability and likelihood ratios - not used  */
			  /*   by the mod2sparse module itself             */
#endif
} mod2entry;

#define Mod2sparse_block 10  /* Number of entries to block together for
                                memory allocation */


/**
 * Block of entries allocated all at once.
 */
typedef struct mod2block
{
	/** Next block that has been allocated. */
	struct mod2block *next;

	/** Entries in this block. */
	mod2entry	entry[Mod2sparse_block];
} mod2block;


/**
 * Representation of a sparse matrix.
 */
typedef struct mod2sparse
{ 
	int n_rows;		  /* Number of rows in the matrix */
	int n_cols;		  /* Number of columns in the matrix */

	mod2entry *rows;	  /* Pointer to array of row headers */
	mod2entry *cols;	  /* Pointer to array of column headers */

	mod2block *blocks;	  /* Blocks that have been allocated */
	mod2entry *next_free;	  /* Next free entry */
} mod2sparse;


/* MACROS TO GET AT ELEMENTS OF A SPARSE MATRIX.  The 'first', 'last', 'next',
   and 'prev' macros traverse the elements in a row or column.  Moving past
   the first/last element gets one to a header element, which can be identified
   using the 'at_end' macro.  Macros also exist for finding out the row 
   and column of an entry, and for finding out the dimensions of a matrix. */

#define mod2sparse_first_in_row(m,i) ((m)->rows[i].right) /* Find the first   */
#define mod2sparse_first_in_col(m,j) ((m)->cols[j].down)  /* or last entry in */
#define mod2sparse_last_in_row(m,i) ((m)->rows[i].left)   /* a row or column  */
#ifndef SPARSE_MATRIX_OPT_FOR_LDPC_STAIRCASE
#define mod2sparse_last_in_col(m,j) ((m)->cols[j].up)
#endif

#define mod2sparse_next_in_row(e) ((e)->right)  /* Move from one entry to     */
#define mod2sparse_next_in_col(e) ((e)->down)   /* another in any of the three */
#define mod2sparse_prev_in_row(e) ((e)->left)   /* possible directions        */
#ifndef SPARSE_MATRIX_OPT_FOR_LDPC_STAIRCASE
#define mod2sparse_prev_in_col(e) ((e)->up) 
#endif

#define mod2sparse_at_end(e) ((e)->row<0) /* See if we've reached the end     */


#define mod2sparse_row(e) ((e)->row)      /* Find out the row or column index */
#define mod2sparse_col(e) ((e)->col)      /* of an entry (indexes start at 0) */

#define mod2sparse_rows(m) ((m)->n_rows)  /* Get the number of rows or columns*/
#define mod2sparse_cols(m) ((m)->n_cols)  /* in a matrix                      */


#if 0
/* POSSIBLE LU DECOMPOSITION STRATEGIES.  For use with mod2sparse_decomp. */

typedef enum mod2sparse_strategy_enum
{ Mod2sparse_first, 
  Mod2sparse_mincol, 
  Mod2sparse_minprod
} mod2sparse_strategy;
#endif // #if 0


/* PROCEDURES TO MANIPULATE SPARSE MATRICES. */

mod2sparse *mod2sparse_allocate (int, int);
void mod2sparse_free            (mod2sparse *);

void mod2sparse_clear    (mod2sparse *);

#if 0
void mod2sparse_copy     (mod2sparse *, mod2sparse *);
#endif // #if 0


void mod2sparse_print       (FILE *, mod2sparse *);
#if 0
int  mod2sparse_write       (FILE *, mod2sparse *);
mod2sparse *mod2sparse_read (FILE *);
#endif // #if 0

mod2entry *mod2sparse_find   (mod2sparse *, int, int);
mod2entry *mod2sparse_insert (mod2sparse *, int, int);
void mod2sparse_delete       (mod2sparse *, mod2entry *);


#if 0
void mod2sparse_transpose (mod2sparse *, mod2sparse *);
void mod2sparse_add       (mod2sparse *, mod2sparse *, mod2sparse *);
void mod2sparse_multiply  (mod2sparse *, mod2sparse *, mod2sparse *);
void mod2sparse_mulvec    (mod2sparse *, char *, char *);

int mod2sparse_equal (mod2sparse *, mod2sparse *);

int mod2sparse_count_row (mod2sparse *, int);
int mod2sparse_count_col (mod2sparse *, int);

void mod2sparse_add_row (mod2sparse *, int, mod2sparse *, int);
void mod2sparse_add_col (mod2sparse *, int, mod2sparse *, int);

int mod2sparse_decomp (mod2sparse *, int, mod2sparse *, mod2sparse *, 
                       int *, int *, mod2sparse_strategy, int, int);

int mod2sparse_tunnel_sub  (mod2sparse *, int *, char *, char *);
int mod2sparse_backward_sub (mod2sparse *, int *, char *, char *);
#endif // #if 0

#ifdef SPARSE_MATRIX_OPT_FOR_LDPC_STAIRCASE
mod2entry * mod2sparse_last_in_col(mod2sparse * m, int i);
#endif


#endif // #ifndef LDPC_MATRIX_SPARSE__


