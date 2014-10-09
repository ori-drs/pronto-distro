/* $Id: ldpc_profile.h,v 1.11 2006/09/06 14:36:49 roca Exp $ */
/* 
 *  LDPC/LDGM FEC Library.
 *  (c) Copyright 2002-2006 INRIA - All rights reserved
 *  Main authors: Christoph Neumann (christoph.neumann@inrialpes.fr)
 *                Vincent Roca      (vincent.roca@inrialpes.fr)
 *		  Laurent Fazio     (STMicroelectronics)
 *		  Julien Laboure    (julien.laboure@inrialpes.fr)
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


/****** GENERAL SETUP OPTIONS; EDIT AS APPROPRIATE ****************************/

/**
 * Choose the type of decoder: the trivial iterative decoding algo or the Gauss
 * elimination algo. Only one of them must be chosen.
 * By default, the standard decoder is DECODER_ITERATIVE.
 */
#define DECODER_ITERATIVE


/**
 * Enable the partial sum optimization during decoding.
 * This optimization is effective but adds some more complexity
 * to the decoder.
 */
#define PART_SUM_OPTIMIZATION


/**
 * Enable/disable external memory management support.
 */
#define EXTERNAL_MEMORY_MGMT_SUPPORT


#ifdef DECODER_ITERATIVE
/**
 * This define reduces significantly the memory consumption of sparse matrices
 * (the matrix size is decreased by 1/6). See ldpc_matrix_sparse.h|cpp files.
 *
 * But there is a drawback... With LDPC-Triangle, the initialization time 
 * becomes quite important when activating this option!!!
 * In all other cases (LDGM and LDPC-Staircase) we did not noticed any impact
 * on init/encoding/decoding times. So use it unless you are interested in the
 * Triangle codes.
 *
 * There is also another drawback: with a Gauss elimination algorithm, it
 * creates additional processing at the decoder. So use this optimization
 * only in DECODER_ITERATIVE mode.
 */
//#define SPARSE_MATRIX_OPT_FOR_LDPC_STAIRCASE
#endif


/**
 * This define reduces significantly the memory consumption of sparse matrices
 * (the matrix size is decreased by 1/6). See ldpc_matrix_sparse.h|cpp files.
 *
 * But there is a drawback... The row and column index of each entry of the
 * matrix is stored in a signed short integer (instead of signed integer),
 * limiting the maximum matrix size to 32768 x 32768. Therefore we must have
 * k <= n <= 32767. In other words, it limits the maximum source block size
 * and/or FEC Expansion Ratio.
 *
 * Finally, this optimization is only significant in 32-bit architectures.
 * With LP64 architectures, padding will be needed, adding 32 more bits,
 * which makes this optimization useless...
 */
#if defined (__LP64__) || (__WORDSIZE == 64)
// useless with 64-bit architectures
#else
//#define SPARSE_MATRIX_OPT_SMALL_INDEX
#endif


/**
 * Enable the count of XOR operations, for performance monitoring.
 * The counter will not distinguish between 64-bit XORs (with 64-bit
 * architectures), 32-bit XORs, and 8-bit XORs.
 */
#define PERF_COUNT_XOR


