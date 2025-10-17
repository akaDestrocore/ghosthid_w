#ifndef BSWAP_H
#define BSWAP_H

#include <stdint.h>

static inline uint16_t bswap16(uint16_t x) {
    return (uint16_t)((x << 8) | (x >> 8));
}
static inline uint32_t bswap32(uint32_t x) {
    return ((x & 0x000000FFU) << 24) |
           ((x & 0x0000FF00U) << 8) |
           ((x & 0x00FF0000U) >> 8) |
           ((x & 0xFF000000U) >> 24);
}
static inline uint64_t bswap64(uint64_t x) {
    return ((uint64_t)bswap32((uint32_t)(x & 0xFFFFFFFFULL)) << 32) |
           (uint64_t)bswap32((uint32_t)(x >> 32));
}

/* assume little-endian host in most Zephyr boards; if you need big-endian set HOST_WORDS_BIGENDIAN */
#ifndef HOST_WORDS_BIGENDIAN
#define le16_to_cpu(x) (x)
#define le32_to_cpu(x) (x)
#define cpu_to_le16(x) (x)
#define cpu_to_le32(x) (x)
#else
#define le16_to_cpu(x) bswap16(x)
#define le32_to_cpu(x) bswap32(x)
#define cpu_to_le16(x) bswap16(x)
#define cpu_to_le32(x) bswap32(x)
#endif

#endif /* BSWAP_H */
