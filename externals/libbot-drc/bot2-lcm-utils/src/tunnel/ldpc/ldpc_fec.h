/* $Id: ldpc_fec.h,v 1.61 2006/09/06 14:36:49 roca Exp $ */
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

#ifndef LDPC_FEC_H /* { */
#define LDPC_FEC_H

#include <math.h>
#include <sys/types.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#ifndef WIN32
#include <inttypes.h>
#else
#include <STDDEF.H>
#endif

#include "ldpc_profile.h"	/* defines the general compilation profile */

#include "ldpc_types.h"
#include "ldpc_matrix_sparse.h"
#include "ldpc_create_pchk.h"


/****** CONSTANT AND CLASS DEFINITION *****************************************/

/**             
 * Error status returned by functions.
 */             
enum ldpc_error_status {
	LDPC_OK = 0,
	LDPC_ERROR = 1
};


/**
 * Is the session a coding or decoding session, or both.
 */
#define FLAG_CODER	0x00000001
#define FLAG_DECODER	0x00000002
#define FLAG_BOTH (FLAG_DECODER|FLAG_CODER)


/**
 * This is the LDPC FEC session class, where all the context information
 * is kept for encoding/decoding this block. To "k" source symbols,
 * the LDPC codec can add "n-k" parity (or FEC) symbols,
 * for a total of "n" symbols. Source symbols are numbered {0; k-1}
 * and parity symbols {k; n-1}.
 * There must be one such FEC session instance per FEC block.
 *
 * When LDPCFecSession and LDPCFecScheme are both used, the LDPCFecSession
 * MUST be initialized first (with InitSession()), THEN the LDPCFecScheme
 * (with InitScheme()).
 *
 * WARNING: the following class contains a lot of checking code that
 * is only available in DEBUG mode (set -DDEBUG on the compiling line).
 * Whenever used with a new application, first validate your code in
 * DEBUG mode, and switch to production code only in a second step...
 */
class LDPCFecSession {
public:

/**
 * LDPCFecSession Contructor and Destructor.
 */
	LDPCFecSession ();
	~LDPCFecSession ();


/**
 * InitSession: Initializes the LDPC session.
 * @param nbSourceSymbols	(IN) number of source symbols (i.e. k).
 * @param nbParitySymbols	(IN) number of parity symbols (i.e. n-k).
 *			Be careful that n-k cannot be less than the left
 *			degree (i.e. 3 by default), otherwise an error is
 *			returned.
 * @param symbolSize	(IN) symbol size in bytes. It does NOT need to be
 *			multiple of 4, any value is accepted.
 * @param flags		(IN) session flags (FLAG_CODER, FLAG_DECODER, ...).
 * @param seed		(IN) seed used to build the parity check matrix (H).
 * @param codecType	(IN) Type of codec algorithm and matrix to use.
 *			Can be on of TypeLDGM, TypeSTAIRS, or TypeTRIANGLE.
 * @param leftDegree	(IN) number of equations in which a symbol is involved.
 *			3 (default) is the optimal value for TypeSTAIRS
 *			and TypeTRIANGLE codes, DO NOT change.
 *			With TypeLDGM, higher values are usually preferable
 *			(see INRIA Research Report 5225, June 2004).
 * @return		Completion status (LDPC_OK or LDPC_ERROR).
 */
	ldpc_error_status InitSession  (int	nbSourceSymbols,
					int	nbParitySymbols,
					int	symbolSize,
					int	flags = FLAG_BOTH,
					int	seed = 1,
					SessionType	codecType = TypeTRIANGLE,
					int	leftDegree = 3);


/**
 * SetCallbackFunctions: Set the various callback functions for this session.
 *
 * - The DecodedSymbol callback function is called each time a source symbol
 *   is decoded by the DecodingStepWithSymbol() function. What this function does is
 *   application-dependant, but it must return a pointer to a data buffer,
 *   left uninitialized, of the appropriate size.
 *   In EXTERNAL_MEMORY_MGMT_SUPPORT mode, this function returns an opaque
 *   symbol pointer. The associated buffer, where actual data will be stored,
 *   must be retrieved via the GetData callback.
 *
 * In EXTERNAL_MEMORY_MGMT_SUPPORT mode, the following callbacks are defined:
 * - The AllocTmpBuffer callback is called each time a temporary buffer is
 *   required by the system, e.g. to store a partial sum (check node). This
 *   function returns a symbol pointer, and accessing the data buffer requires
 *   a call to the GetData callback. The associated data buffer MUST be 
 *   initialized to '0' by the callback.
 * - The GetData callback is called each time the data associated to a symbol
 *   must be read. What this function does is application-dependant.
 * - The StoreData callback is called each time a symbol's buffer has been
 *   updated and must be stored reliably by the memory mgmt system.
 *   What this function does is application-dependant.
 * - The FreeSymbol callback is called each time a symbol (or temporary buffer)
 *   is no longer required and can be free'd by the memory mgmt system.
 *
 * All callback functions require an opaque context parameter, that is the
 * same parameter as the one given to DecodingStepWithSymbol().
 *
 * @param DecodedSymbol_callback	(IN) Pointer to an application's callback.
 * 				Given the size of a newly created source symbol
 *				and its sequence number, this function enables
 *				the callee to allocate a symbol structure.
 * 				This function returns a pointer to the data
 *				buffer allocated or to the symbol in
 *				EXTERNAL_MEMORY_MGMT_SUPPORT mode.
 *				This callback is never called when decoding
 *				a parity symbol!
 *
 * @param AllocTmpBuffer_callback (IN) Pointer to an application's callback.
 *				Valid in EXTERNAL_MEMORY_MGMT_SUPPORT mode.
 *				Given the desired buffer size, this function
 *				allocates a symbol that will contain a buffer
 *				of appropriate size and initialized to '0'.
 *
 * @param GetData_callback	(IN) Pointer to an application's callback.
 *				Valid in EXTERNAL_MEMORY_MGMT_SUPPORT mode.
 *				Given the symbol pointer, this function
 *				returns the data buffer, after making sure
 *				that this latter is available and up-to-date.
 *
 * @param GetDataPtrOnly_callback (IN) Pointer to an application's callback.
 *				Valid in EXTERNAL_MEMORY_MGMT_SUPPORT mode.
 *				Same as GetData_callback, except that no
 *				check is made to make sure data is available
 *				and up-to-date. It makes sense when buffer
 *				has just been allocated before, for instance
 *				because this is a destination buffer in a
 *				memcpy() syscall.
 *
 * @param StoreData_callback	(IN) Pointer to an application's callback.
 *				Valid in EXTERNAL_MEMORY_MGMT_SUPPORT mode.
 *				Given the symbol pointer, this function stores
 *				data reliably in the memory mgmt system.
 *
 * @param FreeSymbol_callback	(IN) Pointer to an application's callback.
 *				Valid in EXTERNAL_MEMORY_MGMT_SUPPORT mode.
 *				This function will be called with a symbol
 *				pointer, so that the external memory mgmt
 *				system can free the associated buffer.
 *
 * @param context_4_callback (IN) Pointer to context that will be passed
 * 				to the callback function (if any). This
 * 				context is not interpreted by this function.
 *
 * @return			Completion status (LDPC_OK or LDPC_ERROR).
 */
	ldpc_error_status SetCallbackFunctions (
		void* (*DecodedSymbol_callback)	(void	*context,
						 int	size,
						 int	symbol_seqno),
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
		void* (*AllocTmpBuffer_callback)(void	*context,
						 int	size),
		void* (*GetData_callback)	(void	*context,
						 void	*symbol),
		void* (*GetDataPtrOnly_callback)(void	*context,
						 void	*symbol),
		ldpc_error_status (*StoreData_callback)
						(void	*context,
						 void	*symbol),
		ldpc_error_status (*FreeSymbol_callback)
						(void	*context,
						 void	*symbol),
#endif /* EXTERNAL_MEMORY_MGMT_SUPPORT */
		void*	context_4_callback = NULL);


/**
 * EndSession: Ends the LDPC session, cleans up everything.
 */
	void EndSession ();


/**
 * IsInitialized: Check if the LDPC session has been initialized.
 * @return	  TRUE if the session is ready and initialized, FALSE if not.
 */
	bool IsInitialized ();


/**
 * Set the verbosity level.
 * @param verb		(IN) new verbosity level (0: no trace, 1: all traces)
 */
	void SetVerbosity (int	verb);


/**
 * Prints version number and copyright information about this codec.
 * @param out		(IN) FILE handle where the string should be written.
 */
	void MoreAbout (FILE	*out);


/**
 * Returns the maximum encoding block length (n parameter).
 * This limit is not LDPC-* specific that are nature large bloc FEC codes,
 * meaning that (k,n) can both be very very large. This is a codec specific
 * limit, due to the way the codec is implemented.
 * See ldpc_profile.h:
 *	If SPARSE_MATRIX_OPT_SMALL_INDEX is defined, then
 *		k <= n < 2^15;
 *	Else
 *		k <= n < 2^31
 * The limits are essentially over the n parameter, but given the
 * desired FEC Expansion ratio n/k (or the code rate, k/n), it will also
 * limit the source block length (k parameter).
 * @return			Maximum n value (A.K.A. encoding block length).
 */
	int	GetMaxN ();


/**
 * Build a new parity symbol.
 * @param symbol_canvas	(IN)	Array of source and parity symbols.
 *				This is a table of n pointers to buffers
 *				containing the source and parity symbols.
 * @param paritySymbol_index	(IN)	Index of parity symbol to build in {0.. n-k-1}
 *				range (!)
 * @param paritySymbol	(IN-OUT) Pointer to the parity symbol buffer that will
 *				be built. This buffer MUST BE allocated
 *				before,	but NOT cleared (memset(0)) since
 *				this function will do it.
 * @return			Completion status (LDPC_OK or LDPC_ERROR).
 */
	ldpc_error_status BuildParitySymbol (void*		symbol_canvas[],
					  int		paritySymbol_index,
					  void*		paritySymbol); 


/**
 * Build a new parity symbol.
 * @param symbol_canvas	(IN)	Array of source and parity symbols.
 *				This is a table of n pointers to buffers
 *				containing the source and parity symbols.
 * @param symbol_index	(IN)	Index of column/symbol.
 * @param built_parity_symbols_indices (OUT)
 * @param nb_built_parity_symbols (OUT)
 * @return			Completion status (LDPC_OK or LDPC_ERROR).
 */
	ldpc_error_status BuildParitySymbolsPerCol (void*	symbol_canvas[],
						 int	symbol_index,
						 int*	built_parity_symbols_indices[],
						 int*	nb_built_parity_symbols); 


/**
 * Perform a new decoding step thanks to the newly received symbol.
 * @param symbol_canvas	(IN-OUT) Global array of received or rebuilt source
 * 				symbols (parity symbols need not be stored here).
 *				This is a table of k pointers to buffers.
 * 				This array must be cleared (memset(0)) upon
 * 				the first call to this function. It will be
 * 				automatically updated, with pointers to
 * 				symbols received or decoded, by this function.
 * @param new_symbol	(IN)	Pointer to the buffer containing the new symbol.
 * @param new_symbol_seqno	(IN)	New symbol's sequence number in {0.. n-1} range.
 * @param store_symbol	(IN)	true if the function needs to allocate memory,
 *				copy the symbol content in it, and call
 *				any required callback.
 *				This is typically done when this function is
 *				called recursively, for newly decoded symbols,
 *				or under special circunstances (e.g. perftool).
 * @return			Completion status (LDPC_OK or LDPC_ERROR).
 */
	ldpc_error_status DecodingStepWithSymbol (void*	symbol_canvas[],
					 void*	new_symbol,
					 int	new_symbol_seqno,
					 bool	store_symbol);


/**
 * Perform a new decoding step thanks to the newly received symbol.
 * Same as the other DecodingStepWithSymbol method, without the store_symbol argument
 * (prefered solution).
 * @param symbol_canvas	(IN-OUT) Global array of received or rebuilt source
 * 				symbols (parity symbols need not be stored here).
 *				This is a table of k pointers to buffers.
 * 				This array must be cleared (memset(0)) upon
 * 				the first call to this function. It will be
 * 				automatically updated, with pointers to
 * 				symbols received or decoded, by this function.
 * @param new_symbol	(IN)	Pointer to the buffer containing the new symbol.
 * @param new_symbol_seqno	(IN)	New symbol's sequence number in {0.. n-1} range.
 * @return			Completion status (LDPC_OK or LDPC_ERROR).
 */
	ldpc_error_status DecodingStepWithSymbol (void*	symbol_canvas[],
					 void*	new_symbol,
					 int	new_symbol_seqno);


/**
 * Returns true if the symbol has already been received
 * or decoded (i.e. if it is already known), false otherwise.
 * @param symbol_canvas	(IN)	Array of received/rebuilt source symbols.
 * @param new_symbol_seqno	(IN)	New symbol's sequence number in {0.. n-1} range.
 * @return			TRUE if this symbol has already been received
 * 				or decoded.
 */
	bool SymbolAlreadyKnown (void*	symbol_canvas[],
				 int	new_symbol_seqno);


/**
 * Checks if all DATA symbols have been received/rebuilt.
 * @param symbol_canvas	(IN)	Array of received/rebuilt source symbols.
 * @return			TRUE if all DATA symbols have been received
 * 				or decoded.
 */
	bool IsDecodingComplete (void*	symbol_canvas[] );


#ifdef PERF_COUNT_XOR
/**
 * Returns the number of XOR operations performed since last reset.
 * The counter will not distinguish between 64-bit XORs (with 64-bit
 * architectures), 32-bit XORs, and 8-bit XORs.
 * @return		number of 64/32/8-bit XOR operations
 */
	unsigned int GetNbXor (void);


/**
 * Resets the XOR counter.
 */
	void ResetNbXor (void);
#endif 


/****** PROTECTED MEMBERS ******************************************************/

protected:

	/**
	 * Return true if this is a DATA source symbol.
	 * @param symbolSeqno	(IN) symbol sequence number in {O; n-1} range
         * @return		true if source symbol, false if parity symbol
	 */
	bool	IsSourceSymbol	(int	symbolSeqno);

	/**
	 * Return true if this is a parity (AKA FEC) symbol.
	 * @param symbolSeqno	(IN) symbol sequence number in {O; n-1} range
         * @return		true if parity symbol, false if source symbol
	 */
	bool	IsParitySymbol	(int	symbolSeqno);

	/**
	 * symbol sequence number to column index translation.
	 * @param symbolSeqno	(IN) symbol sequence number in {O; n-1} range
         * @return		corresponding column number in matrix
	 */
	int	GetMatrixCol	(int symbolSeqno);

	/**
	 * Internal column index to symbol sequence number translation.
	 * @param matrixCol	(IN) column number in matrix
	 * @return		corresponding symbol sequence number in
	 *			{O; n-1} range
	 */
	int	GetSymbolSeqno	(int matrixCol);

	/**
	 * Get the data buffer associated to a symbol stored in the
	 * symbol_canvas[] / m_parity_symbol_canvas[] / m_checkValues[] tables.
	 * This function is usefull in EXTERNAL_MEMORY_MGMT_SUPPORT mode
	 * when a the Alloc/Get/Store/Free callbacks are used, but it does
	 * nothing in other mode. This is due to the fact that with these
	 * callbacks, the various canvas do not point to data buffers but
	 * to intermediate structures, and therefore accessing the associated
	 * buffer needs extra processing.
	 * @param symbol	(IN) pointer stored in the various canvas
	 * @return		associated buffer
	 */
	void	*GetBuffer	(void	*symbol);

	/**
	 * Same as GetBuffer, except that this call does not use the
	 * GetData_callback but GetDataPtrOnly_callback instead.
	 * For instance, in EXTERNAL_MEMORY_MGMT_SUPPORT, it will not
	 * make sure that data is actually available and up-to-date,
	 * perhaps because this is a destination buffer in a memcpy
	 * that has just been allocated!
	 * @param symbol	(IN) pointer stored in the various canvas
	 * @return		associated buffer
	 */
	void	*GetBufferPtrOnly	(void	*symbol);

	/**
	 * Calculates the XOR sum of two symbols: to = to + from.
	 * @param to		(IN/OUT) source symbol
	 * @param from		(IN/OUT) symbol added to the source symbol
	 */
	void	AddToSymbol	(void	*to,
				 void	*from);


	bool	m_initialized;	// is TRUE if session has been initialized
	int	m_sessionFlags;	// Mask containing session flags
				// (FLAG_CODER, FLAG_DECODER, ...)
	SessionType	m_sessionType;	// Type of the session. Can be one of
					// LDGM, LDPC STAIRS, and
					// LDPC TRIANGLE.
	int	m_verbosity;	// verbosity level:	0 means no trace
				//			1 means infos
				//			2 means debug

	unsigned int	m_symbolSize;	// Size of symbols in BYTES
#if defined (__LP64__) || (__WORDSIZE == 64)
	unsigned int	m_symbolSize64;	// Size of symbols in 64bits unit
					// (m_symbolSize64 = floor(m_symbolSize/8.0))
					// we use floor since symbol size
					// can be a multiple of 32 bits.
#endif
	unsigned int	m_symbolSize32;	// Size of symbols in 32bits unit
					// (m_symbolSize32 = m_symbolSize/4)
	unsigned int	m_symbolSize32rem; // Remaining bytes when the symbol
					// size is not multiple of 32 bits.
					// (m_symbolSize32rem = m_symbolSize%4)

	int	m_nbSourceSymbols;	// number fo source symbol (K)
	int	m_nbParitySymbols;	// number of parity symbol (=m_nbCheck)
#	define	m_nbCheck m_nbParitySymbols
#	define m_nbTotalSymbols (m_nbParitySymbols+m_nbSourceSymbols)


	mod2sparse*	m_pchkMatrix;	// Parity Check matrix in sparse mode 
					// format. This matrix is also used as
					// a generator matrix in LDGM-* modes.

	int		m_leftDegree;	// Number of equations per data symbol

	// Encoder specific...
	int*		m_nb_unknown_symbols_encoder; // Array: nb unknown symbols
					// per check node. Used during per column
					// encoding.

	// Decoder specific...
	void**		m_checkValues;	// Array: current check-nodes value.
					// Each entry is the sum (XOR) of some
					// or all of the known symbols in this
					// equation.
	int*		m_nbSymbols_in_equ;// Array: nb of variables per check
					// node, ie. per equation
	int		m_firstNonDecoded; // index of first symbol not decoded.
					// Used to know whether decoding is
					// finished or not.
	int*		m_nb_unknown_symbols; // Array: nb unknown symbols per check node
	int*		m_nbEqu_for_parity; // Array: nb of equations where
					// each parity symbol is included
	void**		m_parity_symbol_canvas; //Canvas of stored parity symbols.

#if 0
	uintptr_t* 	m_builtSymbol; 	// symbol built by decoder, used for 
					// recursive calls of DecodingStepWithSymbol
#endif
	bool		m_triangleWithSmallFECRatio;
					// with LDGM Triangle and a small FEC
					// ratio (ie. < 2), some specific
					// behaviors are needed...

#ifdef PERF_COUNT_XOR
	unsigned int	m_nbXor;	// 64/32/8-bit XOR counter for statisitics
#endif 
	// Callbacks
	void* (*m_decodedSymbol_callback) (void* context, int size, int symbol_seqno);
					// Function called each time a new
					// source symbol is decoded.
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
	void* (*m_allocTmpBuffer_callback)(void *context, int size);
					// Function called each time a
					// temporary buffer is required
	void* (*m_getData_callback)	(void *context, void *symbol);
					// Function called each time the data
					// associated to a symbol must be read
	void* (*m_getDataPtrOnly_callback) (void *context, void *symbol);
					// Function called each time we need
					// a ptr to the data buffer associated
					// to a symbol
	ldpc_error_status (*m_storeData_callback) (void *context, void *symbol);
					// Function called each time a symbol's
					// buffer has been updated and must be
					// stored reliably
	ldpc_error_status (*m_freeSymbol_callback) (void *context, void *symbol);
					// Function called each time a symbol
					// (or tmp buffer) can be free'd
#endif /* EXTERNAL_MEMORY_MGMT_SUPPORT */
	void*		m_context_4_callback; // used by callback functions
};


//------------------------------------------------------------------------------
// Inlines for all classes follow
//------------------------------------------------------------------------------

inline bool
LDPCFecSession::IsInitialized( )
{
	return m_initialized;
}

inline int
LDPCFecSession::GetMaxN ()
{
#ifdef SPARSE_MATRIX_OPT_SMALL_INDEX
	return 0x7FFF;
#else
	return 0x7FFFFFFF;
#endif
}

inline bool
LDPCFecSession::IsSourceSymbol	(int	symbolSeqno)
{
	return ((symbolSeqno < m_nbSourceSymbols) ? true : false);
}

inline bool	
LDPCFecSession::IsParitySymbol	(int	symbolSeqno)
{
	return ((symbolSeqno < m_nbSourceSymbols) ? false : true);
}

inline void*
LDPCFecSession::GetBuffer	(void	*symbol)
{
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
	if (m_getData_callback) {
		return (m_getData_callback(m_context_4_callback, symbol));
	} else
#endif
		return symbol;		// nothing to do here
}

inline void*
LDPCFecSession::GetBufferPtrOnly	(void	*symbol)
{
#ifdef EXTERNAL_MEMORY_MGMT_SUPPORT
	if (m_getDataPtrOnly_callback) {
		return (m_getDataPtrOnly_callback(m_context_4_callback, symbol));
	} else
#endif
		return symbol;		// nothing to do here
}

inline unsigned int
LDPCFecSession::GetNbXor (void)
{
	return m_nbXor;
}

inline void
LDPCFecSession::ResetNbXor (void)
{
	m_nbXor=0;
}

#endif /* } LDPC_FEC_H */
