/*
 * dv_handler.h
 *
 * Created: 12/9/2019 3:52:38 PM
 *  Author: liu
 */ 


#ifndef DV_HANDLER_H_
#define DV_HANDLER_H_

#define ROLE_COORDINATOR 1
#define ROLE_SLAVE       0
#define HANDLER_BUFFER_SIZE (5 * 1024)

void dvhandler_init(uint8_t role, uint32_t deviceId);
void dvhandler_update(void);
uint8_t dvhandler_getCurrentAddress(void);

void dvhandler_bytesFromSpi(uint8_t* data, int size);
void dvhandler_bytesFromPlc(uint8_t* data, int size);
int dvhandler_bytesToSpi(uint8_t* buffer, int buffSize);
int dvhandler_bytesToPlc(uint8_t* buffer, int buffSize);

#endif /* DV_HANDLER_H_ */