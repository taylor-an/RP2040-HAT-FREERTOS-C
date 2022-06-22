#ifndef _TCP_H_
#define _TCP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//#define _TCP_DEBUG_

#ifndef DATA_BUF_SIZE
#define DATA_BUF_SIZE   2048
#endif

int32_t tcps(uint8_t sn, uint16_t port);
int32_t tcps_send(uint8_t sn, uint8_t* buf, int32_t len);

#ifdef __cplusplus
}
#endif

#endif
