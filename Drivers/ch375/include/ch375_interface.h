#ifndef CH375_INTERFACE_H
#define CH375_INTERFACE_H

#include <stdint.h>
typedef struct CH375_Context_t CH375_Context_t;

#define CH375_CMD(_x) ((uint16_t)((_x) | 0x0100))
#define CH375_DATA(_x) ((uint16_t)((_x) | 0x0000))

enum CH375_ERRNO{
    CH375_SUCCESS = 0,
    CH375_ERROR = -1,
    CH375_PARAM_INVALID = -2,
    ch375_writeCmd_FAILD = -3,
    ch375_readData_FAILD = -4,
    CH375_NO_EXIST = -5,
    CH375_TIMEOUT = -6,
    CH375_NOT_FOUND = -7,    // can't find ch375 board.
};

typedef int (*func_write_cmd)(CH375_Context_t *context, uint8_t cmd);
typedef int (*func_write_data)(CH375_Context_t *context, uint8_t data);
typedef int (*func_read_data)(CH375_Context_t *context, uint8_t *data);
// @return 0 not intrrupt, other interrupted
typedef int (*func_query_int)(CH375_Context_t *context);

void *ch375_getPriv(CH375_Context_t *context);
int ch375_closeContext(CH375_Context_t *context);
int ch375_openContext(CH375_Context_t **context, 
    func_write_cmd write_cmd,
    func_write_data write_data,
    func_read_data read_data,
    func_query_int query_int,
    void *priv);

#endif /* CH375_INTERFACE_H */