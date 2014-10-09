/* $Id: ldpc_types.h,v 1.1 2006/09/06 14:36:49 roca Exp $ */
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


#ifndef LDPC_TYPES_H
#define LDPC_TYPES_H

/****** type specifications ******/

#ifndef UINT32
#define INT8    char
#define INT16   short
#define UINT8   unsigned char
#define UINT16  unsigned short
#if defined(__LP64__) || (__WORDSIZE == 64) /* 64 bit architectures */
#define INT32   int		// yes, it's also true in LP64!
#define UINT32  unsigned int	// yes, it's also true in LP64!
#else  /* 32 bit architectures */
#define INT32   int		// int creates less compilations pbs than long
#define UINT32  unsigned int	// int creates less compilations pbs than long
#endif /* 32/64 architectures */
#endif /* !UINT32 */

#ifndef UINT64
#ifdef WIN32
#define INT64   __int64
#define UINT64  __uint64
#else  /* UNIX */
#define INT64   long long
#define UINT64  unsigned long long
#endif /* OS */
#endif /* !UINT64 */


#endif /* LDPC_TYPES_H */
