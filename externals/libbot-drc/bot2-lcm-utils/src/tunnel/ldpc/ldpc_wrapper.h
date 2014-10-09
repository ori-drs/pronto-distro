/*
 * ldpc_wrapper.h
 *
 *  Created on: Mar 5, 2009
 *      Author: abachrac
 */

#ifndef LDPC_WRAPPER_H_
#define LDPC_WRAPPER_H_

#include <unistd.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>

#include "ldpc_fec.h"
#include "ldpc_scheme.h"
#include "macros.h"

class ldpc_wrapper {
public:
  ldpc_wrapper()
  {
  }
  ~ldpc_wrapper();

  inline int getNumPackets()
  {
    return nbPKT;
  }

  //used for both
  int getObject(uint8_t * pktBuf);

protected:
  int init(int objSize_, int pktSize_, double fec_rate_, int typeFlag);
  int typeFlag;

  int nbDATA; /* k parameter */
  int nbFEC; /* n - k parameter */
  int nbSYMBOLS; /* n parameter */

  int pktSize; /* packet size */
  int symbolSize; /* symbol size */

  int nbSymbolsPerPkt;
  int nbDATAPkts;
  int nbFECPkts;
  int nbPKT;

  int objSize;
  double fec_ratio;
  LDPCFecScheme * MyFecScheme;

  uint8_t **data; /* filled with original data symbols AND  built FEC symbols */
  int packetNum; /* number of packets sent/received */

};

class ldpc_enc_wrapper: public ldpc_wrapper {
public:
  //encoder stuff:
  ldpc_enc_wrapper(uint8_t * data_to_send, int objSize, int packetSize, double fec_rate); //initializer for encoder
  int getNextPacket(uint8_t * pktBuf, int16_t * ESI);
  int encodeData(uint8_t * data_to_send);
};

class ldpc_dec_wrapper: public ldpc_wrapper {
public:
  //decoder stuff:
  ldpc_dec_wrapper(int objSize, int packetSize, double fec_rate); //initializer for decoder
  int processPacket(uint8_t * newPkt, int16_t ESI);
};
#endif /* LDPC_WRAPPER_H_ */
