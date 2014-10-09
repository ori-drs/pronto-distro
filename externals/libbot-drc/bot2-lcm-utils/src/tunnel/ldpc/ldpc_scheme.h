/* $Id: ldpc_scheme.h,v 1.12 2006/09/06 14:36:49 roca Exp $ */
/* 
 *  LDPC/LDGM FEC Scheme.
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
 
 
#ifndef LDPC_FEC_SCHEME_H /* { */
#define LDPC_FEC_SCHEME_H

#include "ldpc_types.h"


/**
 * Class that implements parts of the LDPC-Staircase/Triangle FEC Scheme,
 * as defined in draft-ietf-rmt-bb-fec-ldpc-01.txt (or later version).
 * It defines the notion of packet, i.e. the grouping of several symbols
 * in the same transmission unit. Depending on the initialization, the
 * LDPCFECScheme class can either define internally the optimal number of
 * symbols per packet, or take it as a parameter.
 * Using this class makes the symbol(s) <=> packet mapping almost transparent
 * to the user. In that case, packet creation (SENDER) and packet processing
 * (RECEIVER) are completely managed by this class (e.g. there is no need
 * to call the LDPCFECSession::DecodingStepWithSymbol() method any more).
 *
 * When LDPCFecSession and LDPCFecScheme are both used, the LDPCFecSession
 * MUST be initialized first (with InitSession()), THEN the LDPCFecScheme
 * (with InitScheme()).
 */
class LDPCFecScheme : public LDPCFecSession {

/****** PUBLIC MEMBERS *********************************************************/
public:
	/**
	 * LDPCFecScheme Constructor and Destructor.
	 */
	LDPCFecScheme ();
	~LDPCFecScheme ();

	/**
	 * Determine the optimal symbol size when the object size and packet
	 * size are both known.
	 *
	 * This function defines the optimal symbol size in order to maximize
	 * the erasure recovery efficiency. The actual number of symbols per
	 * packet can then be retrieved by means of the getNbSymbolsPerPkt()
	 * function. This value must be an integer, i.e. that the packet size
	 * must be a multiple of the resulting symbol size (which also depends
	 * on the object size). It usually means that the packet size must be
	 * a multiple of a certain power of 2. If not possible, an error is
	 * returned.
	 *
	 * When the object size is not multiple of the symbol size, it is left
	 * to the application to handle the possible padding of the last source
	 * symbol.
	 *
	 * @param objectSize	(IN) the object size (bytes).
	 * @param pktSize	(IN) the packet size (bytes). Depending on the
	 *			number of symbols per packet, the packet
	 *			size must sometimes be a multiple of 4 or 8.
	 * @param symbolSize	(OUT) opimal symbol size determined by this
	 *			function.
	 * @param nbSourceSymbols
	 *			(OUT) corresponding number of source symbols.
	 * @return		Completion status (LDPC_OK or LDPC_ERROR).
	 */
	ldpc_error_status DetermineSymbolSize
					(INT64		objectSize,
					 int		pktSize,
					 int		*symbolSize,
					 int		*nbSourceSymbols);

	  /**
	 * Initialize the LDPC scheme.
	 * Note that the packet size must be a multiple of the symbol size.
	 * @param symbolSize	(IN) the symbol size, probably calculated by 
	 *			the DetermineSymbolSize() function above.
	 * @param pktSize	(IN) the packet size (bytes).
	 * @return		Completion status (LDPC_OK or LDPC_ERROR).
	 */
	ldpc_error_status InitScheme   (int	symbolSize,
					int	pktSize);

	/**
	 * Build the packet of the index from an appropriate number of
	 * symbols. Used by a sender.
	 * There are always getNbSymbolsPerPkt() symbols per packet, even
	 * when the number of source or repair symbols are not multiple of
	 * the number of symbols per packet.
	 * Note that only homogeneous packets are created, i.e. packets
	 * consisting either of source symbols (AKA source packets), or
	 * or of repair symbols (AKA repair packets).
	 *
	 * @param pktIdx	(IN) Index of the packet to build, in
	 *			[0; getNbPkts() - 1] range.
	 * @param pktBuffer	Data buffer where the packet should be
	 *			written.
	 * @param ESIofFirstSymbol
	 *			(OUT) ESI of the first symbol chosen
	 *			to be included in this packet.
	 * @return		Completion status (LDPC_OK or LDPC_ERROR).
	 */
	ldpc_error_status BuildPkt (int		pktIdx,
				    void**	pktBuffer,
				    void*	symbol_canvas[],
				    int*	ESIofFirstSymbol);

	/**
	 * Split a received packet into the set of its constituting symbols.
	 * @param pktBuffer	(IN) Data buffer containing the packet
	 *			received.
	 * @param ESIofFirstSymbol
	 *			(IN) ESI of the first symbol of the packet.
	 * @param GeneratedSymbols[]
	 *			(OUT) table containing pointers to all the
	 *			symbols of the packet, including the first one.
	 *			There are always getNbSymbolsPerPkt() entries.
	 *			This table is allocated by the function and
	 *			must be free'ed by the caller.
	 * @param ESIofSymbols	(OUT) table containing the ESI of all the
	 *			symbols of the packet, including the first one.
	 *			There are always getNbSymbolsPerPkt() entries.
	 *			This table is allocated by the function and
	 *			must be free'ed by the caller.
	 * @return		Completion status (LDPC_OK or LDPC_ERROR).
	 */
	ldpc_error_status DecomposePkt (void* 	pktBuffer,
					int	ESIofFirstSymbol,
					void**	GeneratedSymbols[],
					int	*ESIofSymbols[]);

	/**
	 * Perform a new decoding step thanks to the newly received packet.
	 * This is the same as LDPCFecSession::DecodingStepWithSymbol() but
	 * with a packet as input rather than a symbol.
	 * @param symbol_canvas	(IN-OUT) Global array of received or rebuilt
	 * 			source symbols (parity symbols need not be
	 *			stored here).
	 *			This is a table of k pointers to buffers.
	 * 			This array must be cleared (memset(0)) upon
	 * 			the first call to this function. It will be
	 * 			automatically updated, with pointers to
	 * 			symbols received or decoded, by this function.
	 * @param pktBuffer	(IN) Pointer to the buffer containing the new
	 *			packet.
	 * @param ESIofFirstSymbol
	 *			(IN) ESI of the first symbol of the packet.
	 * @param store_symbol	(IN) true if the function needs to allocate memory,
	 *			copy the symbol content in it, and call
	 *			any required callback.
	 *			This is typically done when this function is
	 *			called recursively, for newly decoded symbols,
	 *			or under special circunstances (e.g. perftool).
	 * @return		Completion status (LDPC_OK or LDPC_ERROR).
	 */
	ldpc_error_status DecodingStepWithPkt  (void*	symbol_canvas[],
						void* 	pktBuffer,
						int	ESIofFirstSymbol,
						bool	store_symbol);

	/**
	 * Perform a new decoding step thanks to the newly received packet.
	 * Same as the other DecodingStepWithSymbol method, without the
	 * store_symbol argument (prefered solution).
	 * @param symbol_canvas	(IN-OUT) Global array of received or rebuilt
	 * 			source symbols (parity symbols need not be
	 *			stored here).
	 *			This is a table of k pointers to buffers.
	 * 			This array must be cleared (memset(0)) upon
	 * 			the first call to this function. It will be
	 * 			automatically updated, with pointers to
	 * 			symbols received or decoded, by this function.
	 * @param pktBuffer	(IN) Pointer to the buffer containing the new
	 *			packet.
	 * @param ESIofFirstSymbol
	 *			(IN) ESI of the first symbol of the packet.

	 * @return		Completion status (LDPC_OK or LDPC_ERROR).
	 */
	ldpc_error_status DecodingStepWithPkt  (void*	symbol_canvas[],
						void* 	pktBuffer,
						int	ESIofFirstSymbol);

	/**
	 * Return the number of symbols that are grouped in the same packet
	 * (AKA symbol group).
	 * There are always this number of symbols per packet, even
	 * when the number of source or repair symbols is not multiple of
	 * the number of symbols per packet.
	 * @return	Number of symbols grouped in any packet.
	 */
	int getNbSymbolsPerPkt (void);

	/**
	 * Return the number of source packets (i.e. consisting only
	 * of source symbols).
	 * It might differ from the number of source symbols since
	 * several symbols can be grouped in the same packet.
	 * @return	Number of source packets.
	 */
	int getNbSourcePkts (void);

	/**
	 * Return the number of parity packets (i.e. consisting only
	 * of parity symbols).
	 * It might differ from the number of parity symbols since
	 * several symbols can be grouped in the same packet.
	 * @return	Number of parity packets.
	 */
	int getNbParityPkts (void);
	
	/**
	 * Return the number of source symbols 
	 * It might differ from the number of parity packets since
	 * several symbols can be grouped in the same packet.
	 * @return	Number of parity packets.
	 */
	int getNbSourceSymbols (void);

	/**
	 * Return the number of parity symbols 
	 * It might differ from the number of parity packets since
	 * several symbols can be grouped in the same packet.
	 * @return	Number of parity packets.
	 */
	int getNbParitySymbols (void);


/****** PRIVATE MEMBERS ********************************************************/
protected:
	
	bool		m_SchemeInitialized;

#	define		m_nbSymbols	(m_nbParitySymbols + m_nbSourceSymbols)
	int		m_symbolSize;
	int		m_pktSize;
	int		m_nbSourcePkts;
	int		m_nbParityPkts;
#	define		m_nbPkts	(m_nbParityPkts + m_nbSourcePkts)	
	int		m_nbSymbolsPerPkt;

	/** Table where the index is the ESI, the value is the transmission index. */
	int* 		m_ESItoTxseq;

	/** Table where the index is transmission index, the value is the ESI. */
	int* 		m_txseqToESI;
};


//------------------------------------------------------------------------------
// Inlines for all classes follow
//------------------------------------------------------------------------------

inline int
LDPCFecScheme::getNbSymbolsPerPkt ()
{
	return m_nbSymbolsPerPkt;
}
	
inline int
LDPCFecScheme::getNbSourcePkts ()
{
	return m_nbSourcePkts;
}

inline int
LDPCFecScheme::getNbSourceSymbols ()
{
	return m_nbSourceSymbols;
}

inline int
LDPCFecScheme::getNbParitySymbols ()
{
	return m_nbParitySymbols;
}

inline int
LDPCFecScheme::getNbParityPkts ()
{
	return m_nbParityPkts;
}

#endif /* } LDPC_FEC_SCHEME_H */
