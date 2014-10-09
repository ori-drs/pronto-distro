/* $Id: ldpc_scheme.cpp,v 1.13 2006/09/06 14:55:39 roca Exp $ */
/* 
 *  LDPC/LDGM FEC Scheme.
 *  (c) Copyright 2002-2006 INRIA - All rights reserved
 *  Main authors: Christoph Neumann (christoph.neumann@inrialpes.fr)
 *                Vincent Roca      (vincent.roca@inrialpes.fr)
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
#include "ldpc_scheme.h"
#include "ldpc_rand.h"


/******************************************************************************
 * LDPCFecScheme Contructor.
 */
LDPCFecScheme::LDPCFecScheme()
{
	memset(this, 0, sizeof(*this));
}
 
 
/******************************************************************************
 * LDPCFecScheme Destructor.
 */
LDPCFecScheme::~LDPCFecScheme()
{
	if(m_ESItoTxseq)
		free(m_ESItoTxseq);

	if(m_txseqToESI)
		free(m_txseqToESI);

	// and now init everything!
	memset(this, 0, sizeof(*this));
}


/******************************************************************************
 * => See header file for more informations.
 */
ldpc_error_status 
LDPCFecScheme::DetermineSymbolSize (INT64	objectSize,
				    int		pktSize,
				    int		*symbolSize,
				    int		*nbSourceSymbols)
{
	int	nbPkt;		// number of packet for this object
	int	ss;		// optimal symbol size
	nbPkt = (int)ceil((double)objectSize / (double)pktSize);
	if (nbPkt >= 4000) {
		ss = pktSize;
	} else if (nbPkt >= 2000) {
		// target: at least 4000 symbols
		if ((pktSize % 2) != 0) {
			fprintf(stderr,
			"LDPCFecScheme::InitScheme: ERROR, pktSize (%d bytes) IS NOT multiple of 2\n", pktSize);
			return LDPC_ERROR;
		}
		ss = pktSize >> 1;
#if 0
		if ((ss % 4) != 0) {
			fprintf(stderr,
			"LDPCFecScheme::InitScheme: ERROR, pktSize (%d bytes) IS NOT multiple of 8 and the resulting symbolSize (%d bytes) IS NOT multiple of 4\n", pktSize, ss);
			return LDPC_ERROR;
		}
#endif
	} else if (nbPkt >= 1000) {
		// target: at least 4000 symbols
		if ((pktSize % 4) != 0) {
			fprintf(stderr,
			"LDPCFecScheme::InitScheme: ERROR, pktSize (%d bytes) IS NOT multiple of 4\n", pktSize);
			return LDPC_ERROR;
		}
		ss = pktSize >> 2;
#if 0
		if ((ss % 4) != 0) {
			fprintf(stderr,
			"LDPCFecScheme::InitScheme: ERROR, pktSize (%d bytes) IS NOT multiple of 16 and the resulting symbolSize (%d bytes) IS NOT multiple of 4\n", pktSize, ss);
			return LDPC_ERROR;
		}
#endif
	} else if (nbPkt >= 500) {
		// target: at least 4000 symbols
		if ((pktSize % 8) != 0) {
			fprintf(stderr,
			"LDPCFecScheme::InitScheme: ERROR, pktSize (%d bytes) IS NOT multiple of 8\n", pktSize);
			return LDPC_ERROR;
		}
		ss = pktSize >> 3;
#if 0
		if ((ss % 4) != 0) {
			fprintf(stderr,
			"LDPCFecScheme::InitScheme: ERROR, pktSize (%d bytes) IS NOT multiple of 32 and the resulting symbolSize (%d bytes) IS NOT multiple of 4\n", pktSize, ss);
			return LDPC_ERROR;
		}
#endif
	} else {
		if ((pktSize % 16) != 0) {
			fprintf(stderr,
			"LDPCFecScheme::InitScheme: ERROR, pktSize (%d bytes) IS NOT multiple of 16\n", pktSize);
			return LDPC_ERROR;
		}
		ss = pktSize >> 4;
#if 0
		if ((ss % 4) != 0) {
			fprintf(stderr,
			"LDPCFecScheme::InitScheme: ERROR, pktSize (%d bytes) IS NOT multiple of 64 and the resulting symbolSize (%d bytes) IS NOT multiple of 4\n", pktSize, ss);
			return LDPC_ERROR;
		}
#endif
	}
	*symbolSize = ss;
	*nbSourceSymbols = (int)ceil((double)objectSize / (double)ss);
	return LDPC_OK;
}


/******************************************************************************
 *
 * => See header file for more informations.
 */
ldpc_error_status 
LDPCFecScheme::InitScheme  (int	symbolSize,
			    int	pktSize)
{
	int	randInd = 0;
	int	backup = 0;

	if (m_initialized == false) {
		fprintf(stderr, "LDPCFecScheme::InitScheme: ERROR, LDPC Session needs to be initialized before\n");
		return LDPC_ERROR;
	}

	if (m_SchemeInitialized == true) {
		fprintf(stderr, "LDPCFecScheme::InitScheme: ERROR, Scheme has already been initialized\n");
		return LDPC_ERROR;
	}

	if (pktSize < symbolSize) {
		fprintf(stderr, "LDPCFecScheme::InitScheme: ERROR, bad packet size %d; must be at least be equal to the symbol size %d\n",
			pktSize, symbolSize);
		return LDPC_ERROR;
	}

	m_symbolSize = symbolSize;
	m_pktSize = pktSize;

	m_nbSymbolsPerPkt = (int) floor((double) m_pktSize/(double) m_symbolSize);
	m_nbSourcePkts 	= (int) ceil((double) m_nbSourceSymbols/(double) m_nbSymbolsPerPkt);
	m_nbParityPkts	= (int) ceil((double) m_nbParitySymbols/(double) m_nbSymbolsPerPkt);	

	if (m_nbSymbolsPerPkt == 1) {
		if (m_pktSize != m_symbolSize) {
			fprintf(stderr, "LDPCFecScheme::InitScheme: ERROR, pktSize %d is not equal to symbolSize %d\n",
				m_pktSize, m_symbolSize);
			return LDPC_ERROR;
		}
		// We do not want to randomize the ESI <-> PktIDx.
	} else {
		// init the two tables that map esi <-> txsequence
		m_ESItoTxseq = (int*) calloc(m_nbParitySymbols, sizeof(int));
		m_txseqToESI = (int*) calloc(m_nbParitySymbols, sizeof(int));

		for(int i = 0; i < m_nbParitySymbols; i++) {
			m_ESItoTxseq[i] = i;
			m_txseqToESI[i] = i;
		}
		for(int i = 0; i < m_nbParitySymbols; i++) {
			backup			= m_ESItoTxseq[i];
			randInd			= ldpc_rand(m_nbParitySymbols);
			m_ESItoTxseq[i]		= m_ESItoTxseq[randInd];
			m_ESItoTxseq[randInd]	= backup;
			m_txseqToESI[m_ESItoTxseq[i]]		=  i;
			m_txseqToESI[m_ESItoTxseq[randInd]]	= randInd;
		}
	}

	m_SchemeInitialized = true;
	return LDPC_OK;
}


/******************************************************************************
 *
 * => See header file for more informations.
 */
ldpc_error_status 
LDPCFecScheme::BuildPkt (int	pktIdx,
			 void**	pktBuffer,
			 void*	symbol_canvas[],
			 int*	ESIofFirstSymbol)
{
	int	j;

	ASSERT(pktIdx >= 0);
	ASSERT(pktIdx < m_nbPkts);
	ASSERT(symbol_canvas);
	ASSERT(ESIofFirstSymbol);

	if (*pktBuffer == NULL) {
		*pktBuffer = (char*)calloc(m_pktSize, 1);
	}
	ASSERT(pktBuffer);
	if (m_nbSymbolsPerPkt == 1) {
		/*
		 * simple case, one symbol per packet
		 */
		*ESIofFirstSymbol = pktIdx;
		memcpy((*(char**)pktBuffer),
			symbol_canvas[*ESIofFirstSymbol],
			m_symbolSize);
	} else {
		/*
		 * complex case, several symbols per packet
		 */
		if (pktIdx < m_nbSourcePkts) {
			/* generate a source packet */
			*ESIofFirstSymbol = (pktIdx * m_nbSymbolsPerPkt);
			ASSERT(*ESIofFirstSymbol < m_nbSourceSymbols);
			for (j = 0; j < m_nbSymbolsPerPkt; j++) {
				memcpy((*(char**)pktBuffer) + j * m_symbolSize,
					symbol_canvas[(j + *ESIofFirstSymbol)
							% m_nbSourceSymbols],
					m_symbolSize);
			}
		} else {
			/* generate a parity (repair) packet */
			*ESIofFirstSymbol =
				m_nbSourceSymbols +
				m_txseqToESI[((pktIdx - m_nbSourcePkts) * m_nbSymbolsPerPkt)
						% m_nbParitySymbols];	
			for (j = 0; j < m_nbSymbolsPerPkt; j++) {
				memcpy((*(char**)pktBuffer) + j * m_symbolSize,
					symbol_canvas[m_nbSourceSymbols +
						      m_txseqToESI[(j + (pktIdx - m_nbSourcePkts)
								    * m_nbSymbolsPerPkt)
								    % m_nbParitySymbols]],
					m_symbolSize);
			}
		}
	}
	if (this->m_verbosity >= 1) 
		printf("LDPCFecScheme::BuildPkt: Packet %i build, with ESI %i as first symbol\n",
			pktIdx, *ESIofFirstSymbol);
	return LDPC_OK;
}


/******************************************************************************
 *
 * => See header file for more informations.
 */
ldpc_error_status 
LDPCFecScheme::DecomposePkt (void* 	pktBuffer,
			     int	ESIofFirstSymbol,
			     void**	GeneratedSymbols[],
			     int*	ESIofSymbols[])
{
	int j;

	ASSERT(pktBuffer);

	*ESIofSymbols = (int*) calloc(m_nbSymbolsPerPkt, sizeof(int));
	*GeneratedSymbols = (void**) calloc(m_nbSymbolsPerPkt, sizeof(void*));
 
	if (m_nbSymbolsPerPkt == 1) {
		/*
		 * simple case, one symbol per packet
		 */
		(*ESIofSymbols)[0] = ESIofFirstSymbol;
		(*GeneratedSymbols)[0] = pktBuffer;
	} else {
		/*
		 * complex case, several symbols per packet
		 */
		if (ESIofFirstSymbol < m_nbSourceSymbols) {	
			/* source packet */
			for (j = 0; j < m_nbSymbolsPerPkt; j++) {
				(*ESIofSymbols)[j] = (ESIofFirstSymbol + j) % m_nbSourceSymbols;
				(*GeneratedSymbols)[j] = ((char*)pktBuffer) + j * m_symbolSize;
			}
		} else { 
			/* parity packet */
			for (j = 0; j < m_nbSymbolsPerPkt; j++) {	
				(*ESIofSymbols)[j] = m_nbSourceSymbols 
						+ m_txseqToESI[(j + m_ESItoTxseq[ESIofFirstSymbol
								- m_nbSourceSymbols]) % m_nbParitySymbols];
				(*GeneratedSymbols)[j] = ((char*)pktBuffer) + j * m_symbolSize;
			}
		}
	}
	return LDPC_OK;
}


/******************************************************************************
 *
 * => See header file for more informations.
 */
ldpc_error_status 
LDPCFecScheme::DecodingStepWithPkt (void*	symbol_canvas[],
				    void*	pktBuffer,
				    int		ESIofFirstSymbol)
{
	ASSERT(pktBuffer);
	return DecodingStepWithPkt (symbol_canvas, pktBuffer, ESIofFirstSymbol, false);
}				


/******************************************************************************
 *
 * => See header file for more informations.
 */
ldpc_error_status 
LDPCFecScheme::DecodingStepWithPkt (void*	symbol_canvas[],
				    void*	pktBuffer,
				    int		ESIofFirstSymbol,
				    bool	store_symbol)
{
	int		*ESIofSymbols = NULL;
	char		**generatedSymbols = NULL;
	ldpc_error_status	err;

	ASSERT(pktBuffer);

	err = DecomposePkt( pktBuffer, ESIofFirstSymbol,
				(void ***)&generatedSymbols, &ESIofSymbols);
	if (err == LDPC_ERROR)
		goto error;

	for (int i = 0; i < m_nbSymbolsPerPkt; i++) {
		err = DecodingStepWithSymbol(symbol_canvas, generatedSymbols[i],
						ESIofSymbols[i], store_symbol);
		if (err == LDPC_ERROR)
			goto error;
		if (IsDecodingComplete(symbol_canvas) == true) {
			// no need to consider the remaining symbols
			break;
		}
	}
	if (generatedSymbols != NULL)
		free(generatedSymbols);
	if (ESIofSymbols != NULL) 
		free(ESIofSymbols);	
	return LDPC_OK;

error:
	if (generatedSymbols != NULL)
		free(generatedSymbols);
	if (ESIofSymbols != NULL)
		free(ESIofSymbols);			
	return LDPC_ERROR;
}

