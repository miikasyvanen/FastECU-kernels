#ifndef _CRC_H
#define _CRC_H

#include "stypes.h"

u16 crc16(const u8 *data, u32 siz);
u32 crc32(const u8 *data, u32 siz);
u32 buffer_crc32(u8 *flashbuffer, u32 len);

#endif
