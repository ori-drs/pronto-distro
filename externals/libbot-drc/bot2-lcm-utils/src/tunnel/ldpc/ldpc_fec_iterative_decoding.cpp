/* $Id: ldpc_fec_iterative_decoding.cpp,v 1.2 2006/09/05 15:59:49 roca Exp $ */
/*
 *  LDPC/LDGM FEC Library.
 *  (c) Copyright 2002-2006 INRIA - All rights reserved
 *  Main authors: Christoph Neumann (christoph.neumann@inrialpes.fr)
 *                Vincent Roca      (vincent.roca@inrialpes.fr)
 *		  Laurent Fazio     (STMicroelectronics)
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

#include "ldpc_fec.h"
#include "macros.h"
#ifdef WIN32
#include <conio.h>
#endif


#if defined(DECODER_ITERATIVE) /* { */

/******************************************************************************/
/*
 * Decoder using the Iterative Decoding Algorithm.
 */


/******************************************************************************
 * DecodingStepWithSymbol: Perform a new decoding step with a new (given) symbol.
 * This is the legacy front end to the DecodingStepWithSymbol() method. The actual
 * work will be done in the other DecodingStepWithSymbol() method.
 * => See header file for more informations.
 */ 
ldpc_error_status
LDPCFecSession::DecodingStepWithSymbol (void*	symbol_canvas[],
					void*	new_symbol,
					int	new_symbol_seqno,
					bool	store_symbol)
{
	void	*new_symbol_dst;	// temp variable used to store symbol

	ASSERT(new_symbol);
	ASSERT(new_symbol_seqno >= 0);
	ASSERT(new_symbol_seqno < m_nbTotalSymbols);
	ASSERT(m_initialized);
	ASSERT(m_sessionFlags & FLAG_DECODER);

	// Fast path. If store symbol is not set, then call directly
	// the full DecodingStepWithSymbol() method to avoid duplicate processing.
	if (store_symbol == false) {
		return(DecodingStepWithSymbol(symbol_canvas, new_symbol, new_symbol_seqno)); 
	}
	// Step 0: check if this is a fresh symbol, otherwise return
	if ((mod2sparse_last_in_col(m_pchkMatrix, GetMatrixCol(new_symbol_seqno))->row < 0)
	    || (IsSourceSymbol(new_symbol_seqno) && (symbol_canvas[new_symbol_seqno] != NULL))
	    || (IsParitySymbol(new_symbol_seqno) && (m_parity_symbol_canvas[new_symbol_seqno - m_nbSourceSymbols] != NULL))) {
		// Symbol has already been processed, so skip it
#ifdef DEBUG
		if (this->m_verbosity >= 1) {
			printf("LDPCFecSession::DecodingStepWithSymbol: %s symbol %d already received or rebuilt, ignored\n",
				(IsSourceSymbol(new_symbol_seqno)) ? "Source" : "Parity",
				new_symbol_seqno); }
#endif
		return LDPC_OK;
	}
	// Step 1: Store the symbol in a permanent array if the caller wants it.
	// It concerns only source symbols, since parity symbols are only stored in
	// permanent array if we have a memory gain by doing so, which will
	// be defined later on in the full DecodingStepWithSymbol() method.
	if (IsSourceSymbol(new_symbol_seqno)) {
		ASSERT(store_symbol);
		// Call any required callback, or allocate memory, and
		// copy the symbol content in it.
		// This is typically something which is done when this
		// function is called recursively, for newly decoded
		// symbols.
		if (this->m_decodedSymbol_callback != NULL) {
			new_symbol_dst = m_decodedSymbol_callback(
						m_context_4_callback,
						m_symbolSize,
						new_symbol_seqno);
		} else {
			new_symbol_dst = (void *)malloc(m_symbolSize);
		}
		if (new_symbol_dst == NULL) {
			fprintf(stderr, "LDPCFecSession::DecodingStepWithSymbol: ERROR, out of memory!\n");
			return LDPC_ERROR;
		}
		// Copy data now
		memcpy(GetBufferPtrOnly(new_symbol_dst), GetBuffer(new_symbol), m_symbolSize);
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
		if (m_storeData_callback != NULL) {
			m_storeData_callback(m_context_4_callback,
					new_symbol_dst);
		}
#endif
	} else {
		new_symbol_dst = new_symbol;
	}
	/* continue decoding with the full DecodingStepWithSymbol() method */
	return(DecodingStepWithSymbol(symbol_canvas, new_symbol_dst, new_symbol_seqno)); 
}


/******************************************************************************
 * DecodingStepWithSymbol: Perform a new decoding step with a new (given) symbol.
 * => See header file for more informations.
 *
 * This function relies on the following simple algorithm:
 *
 * Given a set of linear equations, if one of them has only one
 * remaining unknown variable, then the value of this variable is
 * that of the constant term.
 * Replace this variable by its value in all remaining linear
 * equations, and reiterate. The value of several variables can
 * therefore be found by this recursive algorithm.
 *
 * In practice, an incoming symbol contains the value of the associated
 * variable, so replace its value in all linear equations in which
 * it is implicated. Then apply the above algorithm and see if decoding
 * can progress by one or more steps.
 *
 * For instance, if {s1, s2} are source symbols, and {f1} a parity symbol:
 *    | s1 + s2 + f1 = A      (eq. 1)
 *    |      s2 + f1 = B      (eq. 2)
 * Now if the node receives symbol s2, he replaces its value in the two
 * equations, he then finds f1, he replaces its value in the first equation
 * and finds s1.
 */
ldpc_error_status
LDPCFecSession::DecodingStepWithSymbol(	void*	symbol_canvas[],
				void*	new_symbol,
				int	new_symbol_seqno)
{
	mod2entry	*e = NULL;	// entry ("1") in parity check matrix
	mod2entry	*delMe;		// temp: entry to delete in row/column
	void		*currChk;	// temp: pointer to Partial sum
	int		row;		// temp: current row value
	int		*CheckOfDeg1 = NULL; // table of check nodes of degree
					// one after the processing of new_symbol
	int		CheckOfDeg1_nb = 0; // number of entries in table
	int		CheckOfDeg1_listSize = 0; // size of the memory block
					// allocated for the table
#ifdef PART_SUM_OPTIMIZATION
	bool		keep_symbol;	// true if it's worth to store new_symbol
					// in this function, in case it's a parity
					// symbol, and independantly from the
					// store_symbol argument.
#endif

	ASSERT(new_symbol);
	ASSERT(new_symbol_seqno >= 0);
	ASSERT(new_symbol_seqno < m_nbTotalSymbols);
	ASSERT(m_initialized);
	ASSERT(m_sessionFlags & FLAG_DECODER);

	// Step 0: check if this is a fresh symbol, otherwise return
	if ((mod2sparse_last_in_col(m_pchkMatrix, GetMatrixCol(new_symbol_seqno))->row < 0)
	    || (IsSourceSymbol(new_symbol_seqno) && (symbol_canvas[new_symbol_seqno] != NULL))
	    || (IsParitySymbol(new_symbol_seqno) && (m_parity_symbol_canvas[new_symbol_seqno - m_nbSourceSymbols] != NULL))) {
		// Symbol has already been processed, so skip it
		TRACE_LVL(1, ("LDPCFecSession::DecodingStepWithSymbol: %s symbol %d already received or rebuilt, ignored\n",
			(IsSourceSymbol(new_symbol_seqno)) ? "Source" : "Parity",
			new_symbol_seqno))
		return LDPC_OK;
	}
	TRACE_LVL(1, ("LDPCFecSession::DecodingStepWithSymbol: Processing NEW %s symbol: seq=%d\n",
		(IsSourceSymbol(new_symbol_seqno)) ? "Source" : "Parity",
		new_symbol_seqno))
	// First, make sure data is available for this new symbol. Must
	// remain valid throughout this function...
	GetBuffer(new_symbol);

	// Step 1: Store the symbol in a permanent array. It concerns only DATA
	// symbols. Parity symbols are only stored in permanent array, if we have
	// a memory gain by doing so (and not creating new partial sums)
	if (IsSourceSymbol(new_symbol_seqno)) {
		// Source symbol
		// There's no need to allocate anything, nor to call
		// anything. It has already been done by the caller...
		symbol_canvas[new_symbol_seqno] = new_symbol;
		if (IsDecodingComplete(symbol_canvas)) {
			// Decoding is now finished, return...
			TRACE_LVL(1, ("LDPCFecSession::DecodingStepWithSymbol: decoding is finished!\n"))
			return LDPC_OK;
		}
#ifdef PART_SUM_OPTIMIZATION
		keep_symbol = true;
#endif
	} else {
		// Parity symbol
		// Check if parity symbol should be stored or if partial
		// sum should be stored
#ifdef PART_SUM_OPTIMIZATION
		if (m_triangleWithSmallFECRatio) {
			// In this case, the symbol will never be stored into
			// permanent array, but directly added to partial sum
			keep_symbol = false;
		} else {
			// The symbol will be stored if more than 1 partial sum
			// is needed
			const int	max_allowed_PS = 1; // threshold before 
							// storing symbol in array
			int		PS_to_create = 0; // nb of partial sums that
							//  would have to be allocated

			// count the number of PS that would be created
			// if we don't keep this parity symbol.
			for (e = mod2sparse_first_in_col(m_pchkMatrix,
							GetMatrixCol(new_symbol_seqno));
			    !mod2sparse_at_end(e);
			    e = mod2sparse_next_in_col(e))
			{
				if (m_checkValues[e->row] == NULL &&
				    m_nb_unknown_symbols[e->row] > 2) {
					PS_to_create++;
					if (PS_to_create > max_allowed_PS) {
						break;	// no need to see further
					}
				}
			}
			// now take a decision...
			if (PS_to_create > max_allowed_PS) {
				// Parity symbol will be stored in a permanent array
				// Alloc the buffer...
				keep_symbol = true;
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
				if (m_allocTmpBuffer_callback != NULL) {
					m_parity_symbol_canvas[new_symbol_seqno - m_nbSourceSymbols] =
							m_allocTmpBuffer_callback(
								m_context_4_callback,
								m_symbolSize);
				} else
#endif //EXTERNAL_MEMORY_MGMT_SUPPORT
				{
					m_parity_symbol_canvas[new_symbol_seqno - m_nbSourceSymbols] =
							(void *)malloc(m_symbolSize);
				}
				// copy the content...
				memcpy(GetBufferPtrOnly(m_parity_symbol_canvas[new_symbol_seqno - m_nbSourceSymbols]),
					GetBufferPtrOnly(new_symbol), m_symbolSize);
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
				// and store it permanently.
				if (m_storeData_callback) {
					m_storeData_callback(m_context_4_callback,
							m_parity_symbol_canvas[new_symbol_seqno - m_nbSourceSymbols]);
				}
#endif //EXTERNAL_MEMORY_MGMT_SUPPORT
			} else {
				// Parity symbol will only be added to partial sums
				keep_symbol = false;
			}
		}
#else  // !PART_SUM_OPTIMIZATION
		if (m_triangleWithSmallFECRatio) {
			// In this case, the symbol will never be stored into
			// permanent array, but directly added to partial sum
		} else {
			// Check if parity symbol should be stored or if
			// partial sum should be stored
			bool	store_parity = false;
			for (e = mod2sparse_first_in_col(m_pchkMatrix,
					    GetMatrixCol(new_symbol_seqno));
			    !mod2sparse_at_end(e);
			    e = mod2sparse_next_in_col(e))
			{
				if (m_nb_unknown_symbols[e->row] > 2) {
					store_parity = true;
					break;
				}
			}
			if (store_parity) {
				// Parity symbol will be stored in a permanent array
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
				if (m_allocTmpBuffer_callback != NULL) {
					m_parity_symbol_canvas[new_symbol_seqno - m_nbSourceSymbols] =
							m_allocTmpBuffer_callback(
								m_context_4_callback,
								m_symbolSize);
				} else
#endif //EXTERNAL_MEMORY_MGMT_SUPPORT
				{
					m_parity_symbol_canvas[new_symbol_seqno - m_nbSourceSymbols] =
							(void *)malloc(m_symbolSize);
				}
				// copy the content...
				memcpy(GetBufferPtrOnly(m_parity_symbol_canvas[new_symbol_seqno - m_nbSourceSymbols]),
					GetBufferPtrOnly(new_symbol), m_symbolSize);
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
				// and store it permanently.
				if (m_storeData_callback) {
					m_storeData_callback(m_context_4_callback,
							m_parity_symbol_canvas[new_symbol_seqno - m_nbSourceSymbols]);
				}
#endif //EXTERNAL_MEMORY_MGMT_SUPPORT
			}
			// else parity symbol will only be added to partial sums
		}
#endif  // !PART_SUM_OPTIMIZATION
	}

//printf("0: start step 2 for symbol seq=%d\n", new_symbol_seqno);
	// Step 2: Inject the symbol value in each equation it is involved
	// (if partial sum already exists or if partial sum should be created)
	for (e = mod2sparse_first_in_col(m_pchkMatrix, GetMatrixCol(new_symbol_seqno));
	     !mod2sparse_at_end(e); ) {
		// for a given row, ie for a given equation where this symbol
		// is implicated, do the following:
		row = e->row;
		m_nb_unknown_symbols[row]--;	// symbol is known
		currChk = m_checkValues[row];	// associated check
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
		if (currChk != NULL) {
			// the currChk temp buffer has already been allocated and
			// stored in the system. Make sure data is still available.
			if (m_getData_callback != NULL) {
				m_getData_callback(m_context_4_callback, currChk);
//printf("1: currChk already exists; getData, databuf=x%x\n", GetBufferPtrOnly(currChk));
			}
		}
#endif
		if (currChk == NULL &&
#ifdef PART_SUM_OPTIMIZATION
		    ((keep_symbol == false) || (m_nb_unknown_symbols[row] == 1))
#else
		    ((m_nb_unknown_symbols[row] == 1) || m_triangleWithSmallFECRatio)
#endif
		) {
			// we need to allocate a PS (i.e. check node)
			// and add symbol to it, because the parity symbol
			// won't be kept (keep_symbol == false), or it is the
			// last missing symbol of this equation, or because
			// or some particular situation where it is non sense
			// no to allocate a PS (m_triangleWithSmallFECRatio).
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
			if (m_allocTmpBuffer_callback != NULL) {
				currChk = m_allocTmpBuffer_callback(
							m_context_4_callback,
							m_symbolSize);
//printf("2: currChk does not exist; allocTmpBuf, databuf=x%x\n", GetBufferPtrOnly(currChk));
			} else
#endif
			{
				currChk = (void*) calloc(m_symbolSize, 1);
			}
			if ((m_checkValues[row] = currChk) == NULL) {
				goto no_mem;
			}
		}
		if (currChk != NULL) {
			// there's a partial sum for this row...
			if (m_nbSymbols_in_equ[row] > 1) {
				// Security: make sure data is available by
				// calling GetBuffer(new_symbol) rather than
				// GetBufferPtrOnly(new_symbol).
				// we can add the symbol content to this PS
//printf("3: before add to currChk, databuf=x%x\n", GetBufferPtrOnly(currChk));
				AddToSymbol(GetBufferPtrOnly(currChk),
					    GetBuffer(new_symbol));
//printf("3': after add to currChk, databuf=x%x\n", GetBufferPtrOnly(currChk));
			}
			// else this is useless, since new_symbol is the last
			// symbol of this equation, and its value is necessary
			// equal to the PS. Their sum must be 0 (we don't
			// check it).

			// remove the symbol from the equation (this entry
			// is now useless)
			delMe = e;
			e = mod2sparse_next_in_col(e);
			mod2sparse_delete(m_pchkMatrix, delMe);
			m_nbSymbols_in_equ[row]--;
			if (IsParitySymbol(new_symbol_seqno)) {
				m_nbEqu_for_parity[new_symbol_seqno - m_nbSourceSymbols]--;
			}

			// Inject all permanently stored symbols (DATA and parity)
			// into partial sum
#ifdef PART_SUM_OPTIMIZATION
			if ((m_nb_unknown_symbols[row] == 1) ||
			    ((keep_symbol == false) && (m_triangleWithSmallFECRatio == false)))
#else
			if (m_triangleWithSmallFECRatio == false)
#endif
			{
				// Inject all permanently stored symbols
				// (DATA and parity) into this partial sum.
				// Requires to scan the equation (ie row).
				mod2entry	*tmp_e;	// curr symbol in this equ
				int		tmp_seqno;// corresponding seq no
				void		*tmp_symbol; // corresponding symbol pointer

				for (tmp_e = mod2sparse_first_in_row(m_pchkMatrix, row);
				     !mod2sparse_at_end(tmp_e); ) {

					tmp_seqno = GetSymbolSeqno(tmp_e->col);
					if (IsParitySymbol(tmp_seqno)) {
						tmp_symbol = m_parity_symbol_canvas[tmp_seqno - m_nbSourceSymbols];
					} else {
						// waiting for
						// (m_nb_unknown_symbols[row] == 1)
						// to add source symbols is
						// useless... it's even slower
						// in that case!
						tmp_symbol = symbol_canvas[tmp_seqno];
					}
					if (tmp_symbol != NULL) {
						// add the symbol content now
//printf("4: ready to add to currChk the tmp_symbol seq=%d, databuf=x%x\n", tmp_seqno, GetBufferPtrOnly(currChk));
						AddToSymbol(
							GetBufferPtrOnly(currChk),
							GetBuffer(tmp_symbol));
//printf("5: add to currChk done, databuf=x%x\n", GetBufferPtrOnly(currChk));
						// delete the entry
						delMe = tmp_e;
						tmp_e =  mod2sparse_next_in_row(tmp_e);
						mod2sparse_delete(m_pchkMatrix, delMe);
						m_nbSymbols_in_equ[row]--;
						if (IsParitySymbol(tmp_seqno)) {
							m_nbEqu_for_parity[tmp_seqno - m_nbSourceSymbols]--;
							// check if we can delete
							// parity symbol altogether
							if (m_nbEqu_for_parity[tmp_seqno - m_nbSourceSymbols] == 0) {
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
								if (m_freeSymbol_callback != NULL) {
									m_freeSymbol_callback(
										m_context_4_callback,
										tmp_symbol);
								} else
#endif
								{
									free(tmp_symbol);
								}
								m_parity_symbol_canvas[tmp_seqno - m_nbSourceSymbols] = NULL;
							}
						}
					} else {
						// this symbol not yet known,
						// switch to next one in equ
						tmp_e =  mod2sparse_next_in_row(tmp_e);
					}
				}
			}
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
			// store the partial sum now we are sure it has been
			// completely updated
			if (m_storeData_callback != NULL) {
				m_storeData_callback(m_context_4_callback,
							currChk);
//printf("6: currChk stored\n");
			}
#endif
		} else {
			// here m_checkValues[row] is NULL, ie. the partial
			// sum has not been allocated
			e = mod2sparse_next_in_col(e);
		}
		if (m_nbSymbols_in_equ[row] == 1) {
			// register this entry for step 3 since the symbol
			// associated to this equation can now be decoded...
			if (CheckOfDeg1 == NULL) {
				// allocate memory for the table first
				CheckOfDeg1_listSize = 4;
				if ((CheckOfDeg1 = (int*)
						calloc(CheckOfDeg1_listSize,
						sizeof(int*))) == NULL) {
					goto no_mem;
				}
			} else if (CheckOfDeg1_nb == CheckOfDeg1_listSize) {
				// not enough size in table, add some more
				CheckOfDeg1_listSize += 4;
				if ((CheckOfDeg1 = (int*)realloc(CheckOfDeg1,
							CheckOfDeg1_listSize * sizeof(int*))) == NULL) {
					goto no_mem;
				}
			}
			CheckOfDeg1[CheckOfDeg1_nb++] = row;
		}
	}

	// Step 3: Check if a new symbol has been decoded and take appropriate
	// measures ...
	int	decoded_symbol_seqno;	// sequence number of decoded symbol
	//for (int i = 0; i < CheckOfDeg1_nb; i++) 
	for (CheckOfDeg1_nb--; CheckOfDeg1_nb >= 0; CheckOfDeg1_nb--) {
		if (IsDecodingComplete(symbol_canvas)) {
			// decoding has just finished, no need to do anything else
			break;
		}
		// get the index (ie row) of the partial sum concerned
		row = CheckOfDeg1[CheckOfDeg1_nb];
		if (m_nbSymbols_in_equ[row] == 1) {
			// A new decoded symbol is available...
			// NB: because of the recursion below, we need to
			// check that all equations mentioned in the
			// CheckOfDeg1 list are __still__ of degree 1.
			e = mod2sparse_first_in_row(m_pchkMatrix, row);
			ASSERT(!mod2sparse_at_end(e) &&
				mod2sparse_at_end(e->right))
			decoded_symbol_seqno = GetSymbolSeqno(e->col);
			// remove the entry from the matrix
			currChk = m_checkValues[row];	// remember it
			m_checkValues[row] = NULL;
			m_nbSymbols_in_equ[row]--;
			if (IsParitySymbol(decoded_symbol_seqno)) {
				m_nbEqu_for_parity[decoded_symbol_seqno - m_nbSourceSymbols]--;
			}
			mod2sparse_delete(m_pchkMatrix, e);
			TRACE_LVL(1, ("LDPCFecSession::DecodingStepWithSymbol: => REBUILT %s symbol %d\n",
					(IsParitySymbol(decoded_symbol_seqno)) ? "Parity" : "Source",
					decoded_symbol_seqno))
			if (IsSourceSymbol(decoded_symbol_seqno)) {
				// source symbol.
				void	*decoded_symbol_dst;// temp variable used to store symbol

				// First copy it into a permanent symbol.
				// Call any required callback, or allocate memory, and
				// copy the symbol content in it.
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
				if (this->m_decodedSymbol_callback != NULL) {
					decoded_symbol_dst =
						m_decodedSymbol_callback(
							m_context_4_callback,
							m_symbolSize,
							decoded_symbol_seqno);
				} else
#endif
				{
					decoded_symbol_dst =
						(void *)malloc(m_symbolSize);
				}
				if (decoded_symbol_dst == NULL) {
					goto no_mem;
				}
				memcpy(GetBufferPtrOnly(decoded_symbol_dst),
					GetBuffer(currChk), m_symbolSize);
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
				if (m_storeData_callback != NULL) {
					m_storeData_callback(m_context_4_callback,
							decoded_symbol_dst);
				}
#endif
				// Free partial sum which is no longer used.
				// It's important to free it before calling
				// DecodingStepWithSymbol recursively to reduce max
				// memory requirements.
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
				if (m_freeSymbol_callback != NULL) {
					m_freeSymbol_callback(m_context_4_callback,
							currChk);
				} else
#endif
				{
					free(currChk);	
				}
				// And finally call this method recursively
				DecodingStepWithSymbol(symbol_canvas, decoded_symbol_dst,
							decoded_symbol_seqno);

			} else {

				// Parity symbol.
				// Call this method recursively first...
				DecodingStepWithSymbol(symbol_canvas, currChk,
							decoded_symbol_seqno);
				// Then free the partial sum which is no longer needed.
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
				if (m_freeSymbol_callback != NULL) {
					m_freeSymbol_callback(m_context_4_callback,
							currChk);
				} else
#endif
				{
					free(currChk);	
				}
			}
		}
	}
	if (CheckOfDeg1 != NULL) {
		free(CheckOfDeg1);
	}
	return LDPC_OK;

no_mem:
	fprintf(stderr, "LDPCFecSession::DecodingStepWithSymbol: ERROR, out of memory!\n");
	return LDPC_ERROR;
}

#endif // #if defined(DECODER_ITERATIVE) /* } */
