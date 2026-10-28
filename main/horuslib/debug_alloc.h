/* ESP-IDF portability shim for upstream Codec2 allocation macros.
 * Original project optionally uses debug allocation tracking. */
#ifndef HORUS_DEBUG_ALLOC_H
#define HORUS_DEBUG_ALLOC_H
#include <stdlib.h>
#define CALLOC(n, size) calloc((n), (size))
#define FREE(ptr) free((ptr))
#endif
