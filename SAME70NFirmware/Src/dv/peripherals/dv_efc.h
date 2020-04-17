/*
 * dv_efc.h
 *
 * Created: 11/13/2019 4:30:40 PM
 *  Author: liu
 */ 

#ifndef DV_EFC_H_
#define DV_EFC_H_

/* Memory map sector definitions for SAME70 with 1MB of internal flash*/
#define SECTOR0_SUBSECTOR0      (IFLASH_ADDR)                       // 0x00400000
#define SECTOR0_SUBSECTOR1      (IFLASH_ADDR + 0x00002000)          // 0x00402000
#define SECTOR0_SUBSECTOR2      (IFLASH_ADDR + 0x00004000)          // 0x00404000
#define SECTOR1                 (IFLASH_ADDR + 0x00020000)          // 0x00420000
#define SECTOR2                 (SECTOR1 + 0x00020000)              // 0x00440000
#define SECTOR3                 (SECTOR2 + 0x00020000)              // 0x00460000
#define SECTOR4                 (SECTOR3 + 0x00020000)              // 0x00480000
#define SECTOR5                 (SECTOR4 + 0x00020000)              // 0x004A0000
#define SECTOR6                 (SECTOR5 + 0x00020000)              // 0x004C0000
#define SECTOR7                 (SECTOR6 + 0x00020000)              // 0x004E0000

/* Memory map offsets */
#define	uid_offset			0x00

typedef enum flash_program {
    FLASH_PRG_SUCCESS = 0,
    FLASH_PRG_ERROR
} flashProgram_t;

void test_efc(void);
void write_efc(uint8_t *data, uint8_t byte_length, uint8_t offset);
void read_efc(uint8_t *rx_buffer, uint8_t byte_length, uint8_t offset);
uint32_t read_uid(void);
flash_rc_t erase_partitions(uint8_t startIndex, uint8_t endIndex);
flash_rc_t validate_data(const uint32_t *startAddr, const uint8_t *dataToCompare, uint32_t numOfBytes);

#endif /* DV_EFC_H_ */