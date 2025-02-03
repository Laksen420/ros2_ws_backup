
#ifndef SRC_IO_LIGHT_H_
#define SRC_IO_LIGHT_H_

#include "CAENRFIDTypes_Light.h"
#include "Protocol_Light.h"

typedef struct IOBuffer
{
    uint8_t         *memory;
    uint16_t         size;
    int16_t          rpos;
    int16_t          wpos;
} IOBuffer_t;

#define sizeAVP(avptype, len) (AVP_HEADLEN + (uint16_t)(len))

void getAntNames(char ** Array[], int16_t* n);
void getSrcNames(char ** Array[], int16_t* n);
void addHeader(uint16_t CmdID, IOBuffer_t* buf, uint16_t size);
void addAVP(IOBuffer_t *buf, uint16_t len, uint16_t wtype, void *value);
int16_t getAVP(IOBuffer_t *buf, uint16_t wtype, void *value);
int16_t sendReceive(CAENRFIDReader* reader, IOBuffer_t* txbuf, IOBuffer_t* rxbuf);
int16_t sendAbort(CAENRFIDReader* reader);
int16_t receiveFramedTag(CAENRFIDReader* reader, bool* has_tag, CAENRFIDTag* Tag,
                         bool* has_result_code);


#endif /* SRC_IO_LIGHT_H_ */