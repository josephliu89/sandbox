/*
 * dv_efc.c
 *
 * Created: 11/13/2019 4:29:50 PM
 *  Author: liu
 */ 

#include "asf.h"
#include "dv_efc.h"
#include "dv/telemlib/DvTelemUtils.h"



const uint32_t sectorAddresses[] = {
    SECTOR0_SUBSECTOR0,
    SECTOR0_SUBSECTOR1,
    SECTOR0_SUBSECTOR2,
    SECTOR1,
    SECTOR2,
    SECTOR3,
    SECTOR4,
    SECTOR5,
    SECTOR6,
    SECTOR7
};

/* Memory map extern for test function */
extern uint32_t __shared__;

/*!
 * \brief  Test code for embedded flash read/writes. Writes 'TEST' to memory location pointed to by memory
 * \       location __shared__ which initialized in linker script
 * \param  none
 */
void test_efc(void) 
{     
    uint8_t data_to_store[4];
    uint8_t resultant[4] = { 0 };
    uint32_t unique_id[4];
 	
    data_to_store[0] = 'T';
    data_to_store[1] = 'E';
    data_to_store[2] = 'S';
    data_to_store[3] = 'T';
 	
    if (flash_write((uint32_t)&__shared__, data_to_store, sizeof(data_to_store), 0) == 0) {
        printf("Wrote to flash!\r\n");
    } else {
        printf("Failed to write to flash!\r\n");
    }
 	
    memcpy(resultant, &__shared__, 4);
    for (uint8_t i = 0; i < sizeof(resultant); i++) {
        printf("resultant[%i]: %c\r\n", i, resultant[i]);
    }
 	
    flash_read_unique_id(unique_id, 4);
    for (uint8_t i = 0; i < 4; i++) {
        printf("resultant[%i]: 0x%08lX\r\n", i, unique_id[i]);
    }
}

/*!
 * \brief   Writes to flash memory location from content buffer
 * \       
 * \param   data            Pointer to buffer holding contents to move to flash
 * \param   byte_length     Number of bytes to copy
 * \param   offset          Writing offset position from base memory
 */
void write_efc(uint8_t *data, uint8_t byte_length, uint8_t offset) {
	if (flash_write((uint32_t)(&__shared__ + offset), data, byte_length, 0) != FLASH_RC_OK) {
		DV_error("Failed to write to flash!");
	}
}

/*!
 * \brief   Reads flash memory location and copies content into buffer
 * \       
 * \param   rx_buffer       Pointer to buffer to hold copied contents
 * \param   byte_length     Number of bytes to copy
 * \param   offset          Offset position from memory to start copy     
 */
void read_efc(uint8_t *rx_buffer, uint8_t byte_length, uint8_t offset) {
	
	uint8_t rx_int_buffer[byte_length];
	memcpy(rx_int_buffer, (&__shared__ + offset), byte_length);
	for(uint8_t i = 0; i < byte_length; i++) {
		rx_buffer[i] = rx_int_buffer[i];
	}
}

/*!
 * \brief   Reads unique ID programmed by vendor in protected flash region and returns CRC of it
 * \        as unique ID
 * \       
 * \param   none
 *
 * \retval  32-bit value of two CRC16s of unique ID
 */
uint32_t read_uid(void) {
	uint32_t unique_id[4];
	/* Read vendor 128-bit unique ID */
	if (flash_read_unique_id(unique_id, 4) == 0) {
	    uint16_t uid_crc[2];
        uid_crc[0] = calcTelemDataCrc(unique_id, (int)(sizeof(uint32_t) * 2));
        uid_crc[1] = calcTelemDataCrc(unique_id + 2, (int)(sizeof(uint32_t) * 2));   
        uint32_t uid = uid_crc[1] << 16 | uid_crc[0];
        return uid;
    }
    else {
		DV_error("Failed to read unique id from flash!");
        return rand();        
    }        
}

/*!
 * \brief   Erases consecutive sectors inclusively
 * \       
 * \param   startIndex   Starting position of sector to wipe
 * \param   endIndex     Ending position of sector to wipe (inclusive)
 * \
 * \retval  Status from performing flash_erase_sector(). See typedef of flash_rc_t.
 */
flash_rc_t erase_partitions(uint8_t startIndex, uint8_t endIndex) {
    uint32_t ul_rc;

    for (uint8_t i = startIndex; i <= endIndex; i++) {
        ul_rc = flash_erase_sector(sectorAddresses[i]);

        if (ul_rc != FLASH_RC_OK) {
            DV_error("Flash erase error on sector %i", i);
            return FLASH_RC_ERROR;
        }
    }

    return FLASH_RC_OK;
}

/*!
 * \brief   Erases consecutive sectors inclusively
 * \       
 * \param   startAddr       Starting position of flash to compare
 * \param   dataToCompare   Buffer location to compare
 * \param   numOfBytes      Number of bytes to compare
 * \
 * \retval  Status from performing data validation. See typedef of flash_rc_t.
 */
flash_rc_t validate_data(const uint32_t *startAddr, const uint8_t *dataToCompare, uint32_t numOfBytes) {
    uint8_t tmp;
    uint8_t *ptr = &tmp;
    
    for (uint32_t i = 0; i < numOfBytes; i++) {
        memcpy(ptr, (uint32_t *)(((uint32_t)startAddr) + i), 1);
        if ((*ptr) != *dataToCompare) {
            return FLASH_RC_ERROR;
        }
        //else {
            //DV_info("Address: 0x%08lx\tFlash data: %u\tOriginal data: %u", ((uint32_t)start_address + i), *ptr, *original_data);
        //}
        dataToCompare++;  
    }
    return FLASH_RC_OK;
}