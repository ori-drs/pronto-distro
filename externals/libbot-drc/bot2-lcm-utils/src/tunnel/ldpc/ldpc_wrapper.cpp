/*
 * ldpc_wrapper.cpp
 *
 *  Created on: Mar 5, 2009
 *      Author: abachrac
 */

/*BASED OFF OF:
 * $Id: perf_tool2.cpp,v 1.7 2006/09/05 15:59:49 roca Exp $ */

/*  LDPC/LDGM extended performance tool.
 *  (c) Copyright 2002-2006 INRIA - All rights reserved
 *  Main authors: Christoph Neumann (christoph.neumann@inrialpes.fr)
 *                Vincent Roca      (vincent.roca@inrialpes.fr)
 *                Julien Laboure   (julien.laboure@inrialpes.fr)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307,
 *  USA.
 */

#include <stdlib.h>
#include <stdio.h>
#include "ldpc_wrapper.h"
#include <math.h>

#include <getopt.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

int ldpc_wrapper::init(int objSize_, int pktSize_, double fec_ratio_, int typeFlag_)
{
  pktSize = pktSize_;
  objSize = objSize_;
  fec_ratio = fec_ratio_;
  typeFlag = typeFlag_;
  /*
   * step 1: initialize the LDPC FEC session/scheme
   */
  MyFecScheme = new LDPCFecScheme();
  if (MyFecScheme->DetermineSymbolSize(objSize, pktSize, &symbolSize, &nbDATA) == LDPC_ERROR) {
    fprintf(stderr, "ERROR: MyFecScheme->DetermineSymbolSize() failed. Check the -ps value provided.\n");
    exit(1);
  }
  if (fec_ratio < 1.0) {
    fprintf(stderr, "ERROR: invalid FEC ratio %.3f (must be >= 1.0).\nMake sure one has been specified with -fec<n>\n",
        fec_ratio);
    exit(1);
  }
  nbFEC = (int) ceil((fec_ratio - 1.0) * nbDATA);
  nbSYMBOLS = nbDATA + nbFEC;

  if (MyFecScheme->InitSession(nbDATA, nbFEC, symbolSize, typeFlag, 23, TypeTRIANGLE, 3) == LDPC_ERROR) {
    fprintf(stderr, "ERROR: Unable to initialize LDPC_TRIANGLE FEC session!\n");
    exit(1);
  }
  /* then initialize the FEC Scheme */
  if (MyFecScheme->InitScheme(symbolSize, pktSize) == LDPC_ERROR) {
    fprintf(stderr, "ERROR: Unable to initialize LDPC_TRIANGLE FEC scheme!\n");
    exit(1);
  }

  /* and now adjust the various counters */

  nbSymbolsPerPkt = MyFecScheme->getNbSymbolsPerPkt();
  nbDATAPkts = MyFecScheme->getNbSourcePkts();
  nbFECPkts = MyFecScheme->getNbParityPkts();
  nbPKT = nbDATAPkts + nbFECPkts;
  if (objSize == 0)
    objSize = nbDATA * symbolSize;

  //  printf(
  //      "data_symbols=%d  fec_symbols=%d  symbol_size=%d  nb_symbol_per_pkt=%d  total_nb_pkts=%d  pkt_size=%d  object_size=%d  left_degree=3\n",
  //      nbDATA, nbFEC, symbolSize, nbSymbolsPerPkt, nbPKT, pktSize, objSize);

  //allocate space for the symbols
  if ((data = (uint8_t**) calloc(nbSYMBOLS, sizeof(uint8_t*))) == NULL) {
    printf("ERROR: CANNOT ALLOCATE data buffer array!\n");
    exit(1);
  }
  packetNum = 0;
  return 0;
}

ldpc_wrapper::~ldpc_wrapper()
{
  //  fprintf(stderr, "cleanup\n");
  /*
   * close and free everything
   */
  MyFecScheme->EndSession(); //tell it to remove the coding matrix too
  delete MyFecScheme;
  /* free all data and FEC packets created by the source */
  for (int symbolseq = 0; symbolseq < nbSYMBOLS; symbolseq++) {
    free(data[symbolseq]);
  }
  free(data);
}

ldpc_enc_wrapper::ldpc_enc_wrapper(uint8_t * data_to_send, int objSize_, int packetSize, double fec_rate) //initializer for encoder
{

  init(objSize_, packetSize, fec_rate, FLAG_CODER);

  /*
   * step 2: allocate space for  symbols
   */
  for (int sourceseq = 0; sourceseq < nbDATA; sourceseq++) {
    if ((data[sourceseq] = (uint8_t*) calloc(symbolSize, 1)) == NULL) {
      printf("ERROR: CANNOT ALLOCATE data buffers!\n");
      exit(1);
    }
  }
  /* allocate FEC symbol buffers */
  for (int fecseq = 0; fecseq < nbFEC; fecseq++) {
    if ((data[fecseq + nbDATA] = (uint8_t*) calloc(symbolSize, 1)) == NULL) {
      printf("ERROR: CANNOT ALLOCATE FEC buffers!\n");
      exit(1);
    }
  }
  encodeData(data_to_send);
}

int ldpc_enc_wrapper::getNextPacket(uint8_t * pktBuf, int16_t * ESI)
{
  if (packetNum >= nbPKT) {
    printf("ERROR: can't generate more packets\n");
    exit(1);
  }

  int ESI_;
  MyFecScheme->BuildPkt(packetNum, (void**) &pktBuf, (void**) data, &ESI_);
  *ESI = (int16_t) ESI_;
  //  printf("PktIdx %i mapped to first symbol ESI %i\n", packetNum, ESI);

  packetNum++;
  return packetNum >= nbPKT;
}

ldpc_dec_wrapper::ldpc_dec_wrapper(int objSize_, int packetSize, double fec_rate)
{
  init(objSize_, packetSize, fec_rate, FLAG_DECODER);
}

int ldpc_dec_wrapper::processPacket(uint8_t * pktBuf, int16_t ESI)
{
  MyFecScheme->DecodingStepWithPkt((void**) data, pktBuf,(uint32_t) ESI, true);
  /* done, incr the step counter now */
  packetNum++;
  /* check if completed if we received nbDATA packets or more */
  if (packetNum >= nbDATAPkts && MyFecScheme->IsDecodingComplete((void**) data)) {
    return 1;
  }
  if (packetNum >= nbPKT) {
    return -1;
  }
  return 0;

}

int ldpc_wrapper::getObject(uint8_t * buf)
{
  if (!(packetNum >= nbDATAPkts && MyFecScheme->IsDecodingComplete((void**) data))) {
    return -1;
  }
  else {
    for (int i = 0; i < nbDATA; i++) {
      int numB = objSize - i * symbolSize;
      if (numB > symbolSize)
        numB = symbolSize;
      memcpy(buf + i * symbolSize, data[i], numB);
    }
    return 0;
  }

}

int ldpc_enc_wrapper::encodeData(uint8_t * data_to_send)
{
  /*
   * step 3: generate the original DATA symbols
   */
  for (int sourceseq = 0; sourceseq < nbDATA; sourceseq++) {
    uint8_t * p = data_to_send + sourceseq * symbolSize;
    int numB = objSize - sourceseq * symbolSize;
    if (numB > symbolSize)
      numB = symbolSize;
    memcpy(data[sourceseq], p, numB);
  }

  /* and now do FEC encoding */
  for (int fecseq = 0; fecseq < nbFEC; fecseq++) {
    MyFecScheme->BuildParitySymbol((void**) data, fecseq, data[fecseq + nbDATA]);
  }
  return 0;
}
