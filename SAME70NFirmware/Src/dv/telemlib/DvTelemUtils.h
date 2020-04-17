#ifndef DV_TELEM_UTILS_H
#define DV_TELEM_UTILS_H

#include "DvTelemInterop.h"

// stopwatch state, implemented differently on each platform
typedef struct {
    int64_t m_initSeconds;
    long m_initNanoSeconds;
} DvStopWatchState;

void restartStopWatch(DvStopWatchState* state);
uint32_t elapsedMsStopWatch(DvStopWatchState* state);
bool lappedMsStopWatch(DvStopWatchState* state, uint32_t milliseconds);

// crc calculations...
uint16_t calcTelemDataCrc(const void* data, int size);

#endif // DV_TELEM_UTILS_H 
