/*
 * dv_plc.h
 *
 * Created: 2019-12-12 3:40:27 PM
 *  Author: wrinch
 */ 


#ifndef DV_PLC_H_
#define DV_PLC_H_

#include "dv/telemlib/DvTelemInterop.h"
#include "thirdparty/g3/mac_rt/include/MacRt.h"

void dv_plc_init(int role, uint32_t deviceId);
void dv_plc_setAddress(uint8_t address);
void dv_plc_handleRxData(void);
void dv_plc_handleTxData(void);

#endif /* DV_PLC_H_ */