#ifndef __OVERLAYKEY_H_
#define __OVERLAYKEY_H_

#define USE_128_BITS 1

#ifdef USE_128_BITS
#include "OverlayKey128.h"
#else
#include "OverlayKeyGMP.h"
#endif

#endif
