/* $Id: ldpc_fec.cpp,v 1.91 2006/09/06 14:36:49 roca Exp $ */
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



/******************************************************************************
 * LDPCFecSession Contructor.
 */
LDPCFecSession::LDPCFecSession()
{
	memset(this, 0, sizeof(*this));
	m_verbosity = -1;
}


/******************************************************************************
 * LDPCFecSession Destructor.
 */
LDPCFecSession::~LDPCFecSession()
{
        EndSession();
}


/******************************************************************************
 * InitSession : Initializes the LDPC session.
 * => See header file for more informations.
 */
ldpc_error_status
LDPCFecSession::InitSession (	int nbSourceSymbols,
				int nbParitySymbols,
				int symbolSize,
				int flags,
				int seed,
				SessionType codecType,
				int leftDegree)
{
	mod2entry	*e;

	m_initialized	= false;
	m_sessionFlags	= flags;
	m_sessionType	= codecType;
	m_symbolSize	= symbolSize;

#ifdef PERF_COUNT_XOR
	m_nbXor = 0;
#endif

#if defined (__LP64__) || (__WORDSIZE == 64)
	// symbolSize is not necessarily a multiple of 8, but >> 3 will divide
	// it by 8 and keep the integral part automatically.
	m_symbolSize64	= symbolSize >> 3;
#endif 
	m_symbolSize32	= symbolSize >> 2;
	m_symbolSize32rem = symbolSize % 4;	// Remaining bytes when the symbol
						// size is not multiple of 32 bits.
	if (nbSourceSymbols + nbParitySymbols > this->GetMaxN()) {
		fprintf(stderr, "LDPCFecSession::InitSession: ERROR: the total number of symbols (%d) is too large (%d max)\n",
			nbSourceSymbols + nbParitySymbols, this->GetMaxN());
#ifdef SPARSE_MATRIX_OPT_SMALL_INDEX
		fprintf(stderr, "NB: this codec has been compiled in SPARSE_MATRIX_OPT_SMALL_INDEX mode. You can #undef it in src/ldpc_profile.h file to remove this limitation.\n");
#endif
		return LDPC_ERROR;
	}
	m_nbSourceSymbols	= nbSourceSymbols;
	m_nbParitySymbols	= nbParitySymbols;

	// sanity check: setting a left degree != 3 is only meaningfull with
	// LDGM codes, not with LDPC-Staircase/Triangle codes.
	if ((m_sessionType != TypeLDGM) && (leftDegree != 3)) {
		fprintf(stderr, "LDPCFecSession::InitSession: ERROR: setting a leftDegree!=3 (here %d) is only meaningfull with LDGM codes, not with LDPC-Staircase/Triangle codes!\n", leftDegree);
		return LDPC_ERROR;
	}
	m_leftDegree	= leftDegree;
	m_firstNonDecoded = 0;

	if (this->m_verbosity >= 1) {
		if (m_sessionType == TypeLDGM) {
			printf("Initializing LDPC FEC Session...\n - Source symbols = %d\n - Parity symbols = %d\n - Symbol size = %d\n - Edges per source symbols = %d\n", m_nbSourceSymbols, m_nbCheck, m_symbolSize, m_leftDegree = leftDegree);
		} else if (m_sessionType == TypeSTAIRS) {
			printf("Initializing LDPC STAIRCASE FEC Session...\n - Source symbols = %d\n - Parity symbols = %d\n - Symbol size = %d\n - Edges per source symbols = %d\n", m_nbSourceSymbols, m_nbCheck, m_symbolSize, m_leftDegree = leftDegree);
		} else if (m_sessionType == TypeTRIANGLE) {
			printf("Initializing LDPC TRIANGLE FEC Session...\n - Source symbols = %d\n - Parity symbols = %d\n - Symbol size = %d\n - Edges per source symbols = %d\n", m_nbSourceSymbols, m_nbCheck, m_symbolSize, m_leftDegree = leftDegree);
		} 
	}

	// generate parity check matrix... 
	if (this->m_verbosity >= 1) {
		printf("Generating Parity Check Matrix (H)...");
	}
	m_pchkMatrix = CreatePchkMatrix(m_nbCheck, m_nbSourceSymbols + m_nbParitySymbols, Evenboth, m_leftDegree, seed, false, m_sessionType, this->m_verbosity);
	if (m_pchkMatrix == NULL) {
		fprintf(stderr, "LDPCFecSession::InitSession: ERROR: call to CreatePchkMatrix failed!\n");
		return LDPC_ERROR;
	}
	if (this->m_verbosity >= 1) {
		printf("Done!\n");
	}

	if (m_sessionFlags & FLAG_CODER) {
		m_nb_unknown_symbols_encoder = (int*)calloc(m_nbCheck, sizeof(int));
		if (m_nb_unknown_symbols_encoder == NULL) {
			fprintf(stderr, "LDPCFecSession::InitSession: ERROR: call to calloc failed for m_nb_unknown_symbols_encoder!\n");
			return LDPC_ERROR;
		}

		for (int row = 0; row < m_nbCheck; row++) {
			mod2entry *e;
			for (e = mod2sparse_first_in_row(m_pchkMatrix, row);
			     !mod2sparse_at_end(e);
			     e = mod2sparse_next_in_row(e)) {
				m_nb_unknown_symbols_encoder[row]++;
			}
		}
	} else {
		m_nb_unknown_symbols_encoder = NULL;
	}

	if (m_sessionFlags & FLAG_DECODER) {
		// allocate all internal tables
		if (((m_checkValues	= (void**)calloc(m_nbCheck, sizeof(void*))) == NULL) ||
		    ((m_nbSymbols_in_equ = (int*)calloc(m_nbCheck, sizeof(int))) == NULL) ||
		    ((m_nb_unknown_symbols = (int*)calloc(m_nbCheck, sizeof(int))) == NULL) ||
		    ((m_nbEqu_for_parity = (int*)calloc(m_nbCheck, sizeof(int))) == NULL) ||
		    ((m_parity_symbol_canvas = (void**)calloc(m_nbCheck, sizeof(void*))) == NULL)) {
			fprintf(stderr, "LDPCFecSession::InitSession: ERROR: call to calloc failed for m_parity_symbol_canvas!\n");
			return LDPC_ERROR;
		}
		// and update the various tables now
		for (int row = 0; row < m_nbCheck; row++) {
			for (e = mod2sparse_first_in_row(m_pchkMatrix, row);
			     !mod2sparse_at_end(e);
			     e = mod2sparse_next_in_row(e))
			{
				m_nbSymbols_in_equ[row]++;
				m_nb_unknown_symbols[row]++;
			}
		}
		for (int seq = m_nbSourceSymbols; seq < m_nbTotalSymbols; seq++) {
			for (e = mod2sparse_first_in_col(m_pchkMatrix,
						    GetMatrixCol(seq));
			     !mod2sparse_at_end(e);
			     e = mod2sparse_next_in_col(e))
			{
				m_nbEqu_for_parity[seq - m_nbSourceSymbols]++;
			}
		}
	} else {
		// CODER session
		m_checkValues = NULL;
		m_nbSymbols_in_equ = NULL;
		m_nb_unknown_symbols = NULL;
		m_nbEqu_for_parity = NULL;
		m_parity_symbol_canvas = NULL;
	}
	if ((m_sessionType == TypeTRIANGLE) && ((m_nbTotalSymbols/m_nbSourceSymbols) < 2.0)) {
		m_triangleWithSmallFECRatio = true;
	} else {
		m_triangleWithSmallFECRatio = false;
	}
#ifdef DEBUG
	if (this->m_verbosity >= 2) {
		printf("Pchk Matrix:\n");
		mod2sparse_print(stdout, m_pchkMatrix);
	}
#endif
	m_initialized = true;
	//printf("Pchk Matrix:\n");
	//mod2sparse_print(stdout, m_pchkMatrix);
	return LDPC_OK;
}


/******************************************************************************
 * SetDecodedFunctions: Call the function whenever BuildParitySymbol decodes
 * a new parity or source symbol.
 * => See header file for more informations.
 */
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
ldpc_error_status
LDPCFecSession::SetCallbackFunctions (
		void* (*DecodedSymbol_callback)		(void *context, int size,
							 int symbol_seqno),
		void* (*AllocTmpBuffer_callback)	(void *context, int size),
		void* (*GetData_callback)		(void *context, void *symbol),
		void* (*GetDataPtrOnly_callback)	(void *context, void *symbol),
		ldpc_error_status (*StoreData_callback) (void *context, void *symbol),
		ldpc_error_status (*FreeSymbol_callback)	(void *context, void *symbol),
		void*	context_4_callback)
{
	// sanity checks first
	if (DecodedSymbol_callback) {
		if (!(m_sessionFlags & FLAG_DECODER)) {
			fprintf(stderr, "LDPCFecSession::SetCallbackFunctions: ERROR: specifying DecodedSymbol_callback is only valid in DECODER mode\n");
			return LDPC_ERROR;
		}
	}
	if (AllocTmpBuffer_callback) {
		if (!(m_sessionFlags & FLAG_DECODER)) {
			fprintf(stderr, "LDPCFecSession::SetCallbackFunctions: ERROR: specifying AllocTmpBuffer_callback is only valid in DECODER mode\n");
			return LDPC_ERROR;
		}
	}
	if (FreeSymbol_callback) {
		if (!(m_sessionFlags & FLAG_DECODER)) {
			fprintf(stderr, "LDPCFecSession::SetCallbackFunctions: ERROR: specifying FreeSymbol_callback is only valid in DECODER mode\n");
			return LDPC_ERROR;
		}
	}
	// then remember everything
	m_decodedSymbol_callback = DecodedSymbol_callback;
	m_allocTmpBuffer_callback = AllocTmpBuffer_callback;
	m_getData_callback = GetData_callback;
	m_getDataPtrOnly_callback = GetDataPtrOnly_callback;
	m_storeData_callback = StoreData_callback;
	m_freeSymbol_callback = FreeSymbol_callback;
	m_context_4_callback = context_4_callback;
	return LDPC_OK;
}

#else  /* !EXTERNAL_MEMORY_MGMT_SUPPORT */

ldpc_error_status
LDPCFecSession::SetCallbackFunctions (
		void* (*DecodedSymbol_callback)	(void *context, int size, int symbol_seqno),
		void*	context_4_callback)
{
	// sanity checks first
	if (DecodedSymbol_callback) {
		if (!(m_sessionFlags & FLAG_DECODER)) {
			fprintf(stderr, "LDPCFecSession::SetCallbackFunctions: ERROR: specifying DecodedSymbol_callback is only valid in DECODER mode\n");
			return LDPC_ERROR;
		}
	}
	// then remember everything
	m_decodedSymbol_callback = DecodedSymbol_callback;
	m_context_4_callback = context_4_callback;
	return LDPC_OK;
}
#endif /* !EXTERNAL_MEMORY_MGMT_SUPPORT */


/******************************************************************************
 * EndSession : Ends the LDPC session, and cleans up everything.
 * => See header file for more informations.
 */
void
LDPCFecSession::EndSession()
{
	if (m_initialized == true) {

		m_initialized = false;
#if defined(DECODER_ITERATIVE)

		mod2sparse_free(m_pchkMatrix);
		free(m_pchkMatrix);	/* mod2sparse_free does not free it! */
#endif // #if defined(DECODER_ITERATIVE)

		if (m_checkValues != NULL) {
			for (int i = 0; i < m_nbCheck; i++) {
				if (m_checkValues[i] != NULL) {
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
					if (m_freeSymbol_callback != NULL) {
						m_freeSymbol_callback(m_context_4_callback,
								   m_checkValues[i]);
					} else
#endif
					{
						free(m_checkValues[i]);
					}
				}
			}
			free(m_checkValues);
		}
		if (m_parity_symbol_canvas != NULL) {
			for (int i = 0; i < m_nbCheck; i++) {
				if (m_parity_symbol_canvas[i] != NULL) {
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
					if (m_freeSymbol_callback != NULL) {
						m_freeSymbol_callback(m_context_4_callback,
								   m_parity_symbol_canvas[i]);
					} else
#endif
					{
						free(m_parity_symbol_canvas[i]);
					}
				}
			}
			free(m_parity_symbol_canvas);
		}
		if (m_nbSymbols_in_equ != NULL) {
			free(m_nbSymbols_in_equ);
		}
		if (m_nbEqu_for_parity != NULL) {
			free(m_nbEqu_for_parity);
		}
		if (m_nb_unknown_symbols != NULL) {
			free(m_nb_unknown_symbols);
		}
		if (m_nb_unknown_symbols_encoder != NULL) {
			free(m_nb_unknown_symbols_encoder);
		}
	}
	// and now init everything!
	memset(this, 0, sizeof(*this));
}


/******************************************************************************
 * SetVerbosity: Sets the verbosity level.
 * => See header file for more informations.
 */
void
LDPCFecSession::SetVerbosity(int	verb)
{
	this->m_verbosity = verb;
}


/******************************************************************************
 * MoreAbout:Prints version number and copyright information about this codec.
 * => See header file for more informations.
 */
void
LDPCFecSession::MoreAbout (FILE		*out)
{
	fprintf(out, "LDPC/LDGM large block FEC codec - Version 2.1, September 7th, 2006\n");
	fprintf(out, "  Copyright (c) 2002-2006 INRIA - All rights reserved\n");
	fprintf(out, "  Authors: C. Neumann, V. Roca, L. Fazio, J. Laboure\n");
	fprintf(out, "  This codec contains code from R. Neal:\n");
	fprintf(out, "  Copyright (c) 1995-2003 by Radford M. Neal\n");
	fprintf(out, "  See the associated LICENCE.TXT file for licence information\n");
	switch (m_sessionType) {
	case TypeLDGM:
		fprintf(out, "  LDGM codec ");
		break;
	case TypeSTAIRS:
		fprintf(out, "  LDPC-Staircase codec ");
		break;
	case TypeTRIANGLE:
		fprintf(out, "  LDPC-Triangle codec ");
		break;
	}
#if defined(DECODER_ITERATIVE)
		fprintf(out, "(Iterative Decoding)\n");
#endif
}


/******************************************************************************
 * Calculates the XOR sum of two symbols: to = to + from.
 * => See header file for more informations.
 */
void
LDPCFecSession::AddToSymbol	(void	*to,
				void	*from)
{
	unsigned int		i;
#if defined (__LP64__) || (__WORDSIZE == 64) // {
	// 64-bit machines
	/* First perform as many 64-bit XORs as needed... */
	UINT64		*t = (UINT64*)to;	// to pointer to 64-bit integers
	UINT64		*f = (UINT64*)from;	// from pointer to 64-bit integers
	for (i = m_symbolSize64; i > 0; i--) {
		*t ^= *f;
		t++;
		f++;
#ifdef PERF_COUNT_XOR
		m_nbXor++;
#endif
	}
	UINT32		*t32 = (UINT32*)t;	// to pointer to 32-bit integers
	UINT32		*f32 = (UINT32*)f;	// from pointer to 32-bit integers
	/* then perform a 32-bit XOR if needed... */
	if ((m_symbolSize64 << 1) < m_symbolSize32) {
		*(UINT32*)t32 ^= *(UINT32*)f32;
		t32++;
		f32++;
#ifdef PERF_COUNT_XOR
		m_nbXor++;
#endif
	} 
	/* finally perform as many 8-bit XORs as needed if symbol size is not
	 * multiple of 32 bits... */
	if (m_symbolSize32rem > 0) {
		for (i = 0; i < m_symbolSize32rem; i++) {
			*(UINT8*)((UINT8*)t32 + i) ^= *(UINT8*)((UINT8*)f32 + i);
#ifdef PERF_COUNT_XOR
			m_nbXor++;
#endif
		}
	}

#else //defined (__LP64__) || (__WORDSIZE == 64) } {

	// 32-bit machines
	UINT32		*t32 = (UINT32*)to;	// to pointer to 32-bit integers
	UINT32		*f32 = (UINT32*)from;	// from pointer	to 32-bit integers
	/* First perform as many 32-bit XORs as needed... */
	for (i = m_symbolSize32; i > 0; i--) {
		*t32 ^= *f32;
		t32++;
		f32++;
#ifdef PERF_COUNT_XOR
		m_nbXor++;
#endif
	}
	/* finally perform as many 8-bit XORs as needed if symbol size is not
	 * multiple of 32 bits... */
	if (m_symbolSize32rem > 0) {
		for (i = 0; i < m_symbolSize32rem; i++) {
			*(UINT8*)((UINT8*)t32 + i) ^= *(UINT8*)((UINT8*)f32 + i);
#ifdef PERF_COUNT_XOR
			m_nbXor++;
#endif
		}
	}
#endif //defined (__LP64__) || (__WORDSIZE == 64) }
}


/******************************************************************************
 * BuildParitySymbol: Builds a new parity symbol.
 * => See header file for more informations.
 */
ldpc_error_status
LDPCFecSession::BuildParitySymbol (void* symbol_canvas[],
				int paritySymbol_index,
				void* paritySymbol)
{
	uintptr_t	*fec_buf;	// buffer for this parity symbol
	uintptr_t	*to_add_buf;	// buffer for the  source.parity symbol to add
        mod2entry	*e;
	int seqno;

	ASSERT(paritySymbol_index >= 0);
	ASSERT(paritySymbol_index < m_nbParitySymbols);
	ASSERT(paritySymbol != NULL);
	ASSERT(m_initialized);
	ASSERT(m_sessionFlags & FLAG_CODER);

	fec_buf = (uintptr_t*)GetBufferPtrOnly(paritySymbol);
	memset(fec_buf, 0, m_symbolSize);	// reset buffer (security)

	ASSERT(m_sessionType == TypeSTAIRS ||
		m_sessionType == TypeTRIANGLE ||
		m_sessionType == TypeLDGM);
	e = mod2sparse_first_in_row(m_pchkMatrix, paritySymbol_index);
	ASSERT(!mod2sparse_at_end(e));
	while (!mod2sparse_at_end(e)) {
		// paritySymbol_index in {0.. n-k-1} range, so this test is ok
		if (e->col != paritySymbol_index) {
			// don't add paritySymbol to itself
			seqno = GetSymbolSeqno(e->col);
			to_add_buf = (uintptr_t*)
					GetBuffer(symbol_canvas[seqno]);
			if (to_add_buf == NULL) {
				fprintf(stderr, "LDPCFecSession::BuildParitySymbol: FATAL ERROR, symbol %d is not allocated!\n", seqno);
				return LDPC_ERROR;
			}
			AddToSymbol(fec_buf, to_add_buf);
		}
		e = mod2sparse_next_in_row(e);
	}
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
	if (m_storeData_callback) {
		m_storeData_callback(m_context_4_callback, paritySymbol);
	}
#endif
#ifdef DEBUG
	if (this->m_verbosity >= 1) {
		printf("LDPCFecSession::BuildParitySymbol: parity symbol seq=%d created\n",
			paritySymbol_index);
	}
#endif
	return LDPC_OK;
}


/******************************************************************************
 * BuildParitySymbol: Builds a new parity symbol.
 * => See header file for more informations.
 */
ldpc_error_status
LDPCFecSession::BuildParitySymbolsPerCol (void*	symbol_canvas[],
					int	symbol_index,
					int*	built_parity_symbols[],
					int*	nb_built_parity_symbols)
{

	mod2entry	*e;
	uintptr_t	*data;
	uintptr_t	*paritySymbol;

	ASSERT(m_initialized);
	ASSERT(m_sessionFlags & FLAG_CODER);

	ASSERT(m_sessionType== TypeSTAIRS || m_sessionType == TypeTRIANGLE || m_sessionType== TypeLDGM);

	*nb_built_parity_symbols = 0;	
#ifdef DEBUG
	if (this->m_verbosity >= 1) {
		printf("LDPCFecSession::BuildParitySymbolsPerCol: column=%d processed\n", symbol_index);
	}
#endif
	e = mod2sparse_first_in_col(m_pchkMatrix, GetMatrixCol(symbol_index));
	ASSERT(!mod2sparse_at_end(e));

	while (!mod2sparse_at_end(e)) {
		if (e->row != GetMatrixCol(symbol_index)) {
			data = (uintptr_t*) GetBuffer(symbol_canvas[symbol_index]);
			if (symbol_canvas[GetSymbolSeqno(e->row)] == NULL) {
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
				if (m_allocTmpBuffer_callback)
					symbol_canvas[GetSymbolSeqno(e->row)] = m_allocTmpBuffer_callback(m_context_4_callback,m_symbolSize);
				else
#endif
					symbol_canvas[GetSymbolSeqno(e->row)] = (char*)calloc(m_symbolSize, 1);
			}
			paritySymbol = (uintptr_t*) GetBufferPtrOnly(symbol_canvas[GetSymbolSeqno(e->row)]);
		
			if (data == NULL) {
				fprintf(stderr, "LDPCFecSession::BuildParitySymbol: FATAL ERROR, symbol %d is not allocated!\n", symbol_index);
				return LDPC_ERROR;
			}
			AddToSymbol(paritySymbol, data);
			m_nb_unknown_symbols_encoder[e->row]--;
			if (m_nb_unknown_symbols_encoder[e->row] == 1) {
				(*nb_built_parity_symbols)++;
				if (*nb_built_parity_symbols == 1) {
					*built_parity_symbols = (int*)
						calloc(1,sizeof(int));
				} else {
					*built_parity_symbols = (int*)
						realloc((void*) *built_parity_symbols, (*nb_built_parity_symbols)*sizeof(int));
				}
				(*built_parity_symbols)[(*nb_built_parity_symbols) - 1] = e->row;
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
				if (m_storeData_callback) {
					m_storeData_callback(m_context_4_callback, paritySymbol);
				}
#endif						
			}
			
		}		
		e = mod2sparse_next_in_col(e);
	}
	
	/* Call recursively this function with new parity symbols as parameter*/
	int temp = *nb_built_parity_symbols;
	for (int i = 0; i < temp; i++) {
		int	recursive_nb_built_parity_symbols = 0;
		int	*recursive_built_parity_symbols = NULL;

		BuildParitySymbolsPerCol(symbol_canvas,
					*built_parity_symbols[i] + m_nbSourceSymbols,
					&recursive_built_parity_symbols,
					&recursive_nb_built_parity_symbols);
		*built_parity_symbols = (int*) realloc((void*) *built_parity_symbols,((*nb_built_parity_symbols) + recursive_nb_built_parity_symbols)*sizeof(int));
		for (int j = 0; j < recursive_nb_built_parity_symbols; j++) {
			(*built_parity_symbols)[*nb_built_parity_symbols+j] =
						recursive_built_parity_symbols[j];
		}
		*nb_built_parity_symbols = *nb_built_parity_symbols +
					recursive_nb_built_parity_symbols;

		if (recursive_built_parity_symbols != NULL) {
			free(recursive_built_parity_symbols);
			recursive_built_parity_symbols = NULL;
		}
	}
	return LDPC_OK;
}


/******************************************************************************
 * GetMatrixCol:
 * => See header file for more informations.
 */
int
LDPCFecSession::GetMatrixCol(int symbolSeqno)
{
	if (symbolSeqno < m_nbSourceSymbols) {
		/* source symbol */
		return (symbolSeqno + m_nbParitySymbols);
	} else {
		/* parity symbol */
		return (symbolSeqno - m_nbSourceSymbols);
	}
}


/******************************************************************************
 * GetSymbolSeqno:
 * => See header file for more informations.
 */
int
LDPCFecSession::GetSymbolSeqno(int matrixCol)
{
	int colInOrder;

	colInOrder = matrixCol;
	if (colInOrder < m_nbParitySymbols) {
		/* parity symbol */
		return (colInOrder + m_nbSourceSymbols);
	} else {
		/* source symbol */
		return (colInOrder - m_nbParitySymbols);
	}
}


/******************************************************************************
 * IsAlreadyKnown: Returns true if the symbol has already been received
 * or decoded (i.e. if it is already known), false otherwise.
 * => See header file for more informations.
 */
bool
LDPCFecSession::SymbolAlreadyKnown (void*	symbol_canvas[],
				    int		new_symbol_seqno)
{
	if ((mod2sparse_last_in_col(m_pchkMatrix, GetMatrixCol(new_symbol_seqno))->row < 0)
	    || (IsSourceSymbol(new_symbol_seqno) && (symbol_canvas[new_symbol_seqno] != NULL))
	    || (IsParitySymbol(new_symbol_seqno) && (m_parity_symbol_canvas[new_symbol_seqno - m_nbSourceSymbols] != NULL))) {
		// No entry in the column associated to this symbol.
		// Means symbol has already been processed, so skip it.
#ifdef DEBUG
		if (this->m_verbosity >= 1) {
			printf("LDPCFecSession::SymbolAlreadyKnown: %s symbol %d already received or rebuilt\n",
				(new_symbol_seqno < m_nbSourceSymbols) ? "Source" : "Parity",
				new_symbol_seqno);
		}
#endif
		return true;
	} else {
#ifdef DEBUG
		if (this->m_verbosity >= 1) {
			printf("LDPCFecSession::SymbolAlreadyKnown: %s symbol %d not received or rebuilt\n",
				(new_symbol_seqno < m_nbSourceSymbols) ? "Source" : "Parity",
				new_symbol_seqno);
		}
#endif
		return false;
	}
}


/******************************************************************************
 * IsDecodingComplete: Checks if all DATA symbols have been received/rebuilt.
 * => See header file for more informations.
 */
bool
LDPCFecSession::IsDecodingComplete(void*	symbol_canvas[])
{
	if (!m_initialized) {
		fprintf(stderr, "LDPCFecSession::IsDecodingComplete: ERROR: LDPC Session is NOT initialized!\n");
		return false;
	}

	for (int i = m_firstNonDecoded; i < m_nbSourceSymbols; i++) {
		if (symbol_canvas[i] == NULL) {
			/* not yet decoded! */
			m_firstNonDecoded = i;	/* remember for next time */
			return false;
		}
	}
	return true;
}

