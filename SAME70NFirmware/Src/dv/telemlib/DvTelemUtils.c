#include "DvTelemUtils.h"

#ifdef _MSC_VER

    #include <time.h>

    void restartStopWatch(DvStopWatchState* state) {
        struct _timespec64 ts;
        _timespec64_get(&ts, TIME_UTC);
        state->m_initSeconds = ts.tv_sec;
        state->m_initNanoSeconds = ts.tv_nsec;
    }

    uint32_t elapsedMsStopWatch(DvStopWatchState* state) {
        struct _timespec64 ts;
        _timespec64_get(&ts, TIME_UTC); 
        int64_t secondsDiff = ts.tv_sec - state->m_initSeconds;
        long nanoSecsDiff = ts.tv_nsec - state->m_initNanoSeconds;
        uint32_t msDiff = (uint32_t)((secondsDiff * 1000) + (nanoSecsDiff / 1000000));
        return msDiff;
    }

#elif defined(__ATPL360B__)

    // setup and incremented in conjunction with the SysTick_Handler() function.
    extern volatile uint32_t g_ul_ms_ticks;

    void restartStopWatch(DvStopWatchState* state) {
        uint32_t currMs = g_ul_ms_ticks;
        state->m_initSeconds = currMs / 1000;
        state->m_initNanoSeconds = (currMs % 1000) * 1000000;
    }

    uint32_t elapsedMsStopWatch(DvStopWatchState* state) {
        int64_t currMs = g_ul_ms_ticks;
        int64_t prevMs = (state->m_initSeconds * 1000);
        prevMs += (state->m_initNanoSeconds / 1000000);
        if (currMs < prevMs) {
            currMs += UINT32_MAX;
        }
        return (uint32_t)(currMs - prevMs);
    }

#elif defined(__clang__) || defined(__GNUC__)

    #include <time.h>

    void restartStopWatch(DvStopWatchState* state) {
        struct timespec ts;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
        state->m_initSeconds = ts.tv_sec;
        state->m_initNanoSeconds = ts.tv_nsec;
    }

    uint32_t elapsedMsStopWatch(DvStopWatchState* state) {
        struct timespec ts;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
        int64_t secondsDiff = ts.tv_sec - state->m_initSeconds;
        long nanoSecsDiff = ts.tv_nsec - state->m_initNanoSeconds;
        uint32_t msDiff = (uint32_t)((secondsDiff * 1000) + (nanoSecsDiff / 1000000));
        return msDiff;
    }

#else
    #pragma error("Compiling on unknown platform. Good luck!")
#endif


bool lappedMsStopWatch(DvStopWatchState* state, uint32_t milliseconds) {
    uint32_t tooLong = milliseconds * 5;
    uint32_t elapsed = elapsedMsStopWatch(state);
    if (elapsed < milliseconds) {
        return false;
    }
    else if (elapsed < tooLong) {
        int64_t nanoSecResult = (state->m_initNanoSeconds + ((milliseconds % 1000) * 1000000));
        int64_t secondsDiff = (milliseconds / 1000) + (nanoSecResult / 1000000000);
        state->m_initSeconds += secondsDiff;
        state->m_initNanoSeconds = nanoSecResult % 1000000000;
        return true;
    }
    else {
        restartStopWatch(state);
        return true;
    }
}

// from: https://barrgroup.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
/**********************************************************************
* Filename:    crc.h & crc.c
* Copyright (c) 2000 by Michael Barr.  This software is placed into
* the public domain and may be used for any purpose.  However, this
* notice must not be changed or removed and no warranty is either
* expressed or implied by its publication or distribution.
**********************************************************************/

#define POLYNOMIAL			0x8FDB
#define INITIAL_REMAINDER	0xFFFF
#define FINAL_XOR_VALUE		0x0000
#define REFLECT_DATA		FALSE
#define REFLECT_REMAINDER	FALSE
#define WIDTH    (8 * sizeof(uint16_t))
#define TOPBIT   (1 << (WIDTH - 1))

//#if (REFLECT_DATA == TRUE)
//#undef  REFLECT_DATA
//#define REFLECT_DATA(X)			((unsigned char) reflect((X), 8))
//#else
#undef  REFLECT_DATA
#define REFLECT_DATA(X)			(X)
//#endif

//#if (REFLECT_REMAINDER == TRUE)
//#undef  REFLECT_REMAINDER
//#define REFLECT_REMAINDER(X)	((uint16_t) reflect((X), WIDTH))
//#else
#undef  REFLECT_REMAINDER
#define REFLECT_REMAINDER(X)	(X)
//#endif

//static unsigned long reflect(unsigned long data, unsigned char nBits) {
//	unsigned long  reflection = 0x00000000;
//	unsigned char  bit;
//	for (bit = 0; bit < nBits; ++bit) {
//		if (data & 0x01) {
//			reflection |= (1 << ((nBits - 1) - bit));
//		}
//		data = (data >> 1);
//	}
//	return (reflection);
//}

static uint16_t s_crcTable[256];
static void crcInit(void) {
    uint16_t remainder = 0;
    unsigned char bit = 0;
    for (int dividend = 0; dividend < 256; ++dividend) {
        remainder = (uint16_t)(dividend << (WIDTH - 8));
        for (bit = 8; bit > 0; --bit) {
            if (remainder & TOPBIT) {
                remainder = (uint16_t)((remainder << 1) ^ POLYNOMIAL);
            }
            else {
                remainder = (uint16_t)(remainder << 1);
            }
        }
        s_crcTable[dividend] = remainder;
    }
}

uint16_t calcTelemDataCrc(const void* data, int size) {
    static bool initCrc = false;
    if (!initCrc) {
        crcInit();
        initCrc = true;
    }
    unsigned char* const message = (unsigned char*)data;
    uint16_t remainder = (uint16_t)(INITIAL_REMAINDER);
    for (int byte = 0; byte < size; ++byte) {
        unsigned char index = (unsigned char)(REFLECT_DATA(message[byte]) ^ (remainder >> (WIDTH - 8)));
        remainder = (uint16_t)(s_crcTable[index] ^ (remainder << 8));
    }
    return (uint16_t)(REFLECT_REMAINDER(remainder) ^ FINAL_XOR_VALUE);
}

