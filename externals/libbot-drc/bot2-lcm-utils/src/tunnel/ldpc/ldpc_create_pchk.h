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


#ifndef LDPC_CREATE_PCHK__
#define LDPC_CREATE_PCHK__

#include "ldpc_matrix_sparse.h"

typedef enum make_method_enum
{
	Evencol, 	/* Uniform number of bits per column, with number specified */
	Evenboth 	/* Uniform (as possible) over both columns and rows */
} make_method; 


typedef enum SessionType_enum
{
	TypeLDGM,
	TypeSTAIRS,
	TypeTRIANGLE
} SessionType;

mod2sparse* CreatePchkMatrix (  int nbRows, int nbCols, make_method makeMethod, int leftDegree, int seed, bool no4cycle, SessionType type, int verbosity );

#endif

