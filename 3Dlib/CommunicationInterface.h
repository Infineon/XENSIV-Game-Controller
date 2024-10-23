#ifndef TLX493D_COMMUNICATION_INTERFACE_H
#define TLX493D_COMMUNICATION_INTERFACE_H


// std includes
#include <stdbool.h>
#include <stdint.h>
#include "tlx493d_types.h"


#ifdef __cplusplus

extern "C" {

#endif


typedef struct TLx493D_t  TLx493D_t;


bool tlx493d_transfer(TLx493D_t *sensor, uint8_t txBuffer[], uint8_t txLen, uint8_t rxBuffer[], uint8_t rxLen);


#ifdef __cplusplus

}

#endif


void tlx493d_deinitCommunication(TLx493D_t *sensor);


#endif // TLX493D_COMMUNICATION_INTERFACE_H
