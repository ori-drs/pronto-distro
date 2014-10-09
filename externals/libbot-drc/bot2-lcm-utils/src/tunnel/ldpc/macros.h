/* $Id: macros.h,v 1.8 2006/07/27 08:22:18 roca Exp $ */
/*
 *  LDPC/LDGM FEC Library.
 *  (c) Copyright 2002-2006 INRIA - All rights reserved
 *  Main authors: Christoph Neumann (christoph.neumann@inrialpes.fr)
 *                Vincent Roca      (vincent.roca@inrialpes.fr)
 *		  Julien Laboure   (julien.laboure@inrialpes.fr)
 *
 *  This copyright notice must be retained and prominently displayed,
 *  along with a note saying that the original programs are available from
 *  Vincent Roca's web page, and note is made of any changes made to these
 *  programs.  
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#ifndef MACROS_H
#define MACROS_H

#include <stdio.h>


/****** general macros ******/

/*
 * print to stdout
 */
#define PRINT(a) { printf a; fflush(stdout); }

/*
 * same with a verbosity level
 */
#define PRINT_LVL(l, a) if (verbosity >= (l)) {				\
				printf a;				\
				fflush(stdout);				\
			}

/*
 * Trace with a level in DEBUG mode only
 */
#ifdef DEBUG
#define TRACE_LVL(l, a) if (m_verbosity >= (l)) {				\
				printf a;				\
				fflush(stdout);				\
			}
#else
#define TRACE_LVL(l, a)
#endif

/*
 * print and exit
 */
#define EXIT(a) { printf a; fflush(stdout); exit(-1); }

/*
 * conversion
 */
#define BUFFER_TO_INT32(x) ((*(x)<<24) + (*(x+1)<<16) + (*(x+2)<<8) + (*(x+3)))

/*
 * timeval structure management
 */
#ifdef timersub
#undef timersub
#endif
#define timersub(a, b, result)						\
	do {								\
		(result).tv_sec = (a).tv_sec - (b).tv_sec;		\
		(result).tv_usec = (a).tv_usec - (b).tv_usec;		\
		if ((result).tv_usec < 0) {				\
			--(result).tv_sec;				\
			(result).tv_usec += 1000000;			\
		}							\
	} while (0)

/*
 * assertion in DEBUG mode
 */
#ifdef ASSERT
#undef ASSERT
#endif
#ifdef DEBUG
#define ASSERT(c)	if (!(c)) { \
				fprintf(stderr, "ASSERT [%s:%d] failed\n", \
					__FILE__, __LINE__);		\
				fflush(stderr);				\
				exit(-1);				\
			}
#else /* DEBUG */
#define ASSERT(c)
#endif /* DEBUG */


#endif /* MACROS_H */
