#ifndef DV13131B_H
#define DV13131B_H

/* SPI-I2C packet format definitions */
#define SPI_COM             0
#define I2C0_COM            1
#define I2C1_COM            2
#define I2C2_COM            3
#define MASK_BITS           (0x08)
#define MASK_RW             (0x04)
#define MASK_INT            (0x03)
#define POS_BITS            (0x03)
#define POS_RW              (0x02)
#define DUMMY_BYTE          (0xAB)

/* Acknowledgment codes */
#define ACK                 (0xAA)
#define NACK                (0xBB)
#define INT_INVALID         (0xCC)
#define REG_INVALID         (0xDD)
#define LEN_INVALID         (0xEE)

/* Z-Board (DV13131B) specific registers */
#define REG_IR_CTRL         (0x00)
#define REG_IR_VAL          (0x01)
#define REG_ANALOG0         (0x02)
#define REG_ANALOG1         (0x03)
#define REG_G_LED           (0x04)
#define REG_R_LED           (0x05)

/* Spine Board (DV12851D) specific registers */
#define DEV_ID_AD7991_1     (0x29)

/* SPI configuration */
#define SPI_DBC           30        // sleep between SPI bytes/commands

#pragma message("@TODO: May need to adjust process time to accommodate for maximimum I2C device read/write times")
/* PROCESS_TIME define may vary:
 *  Z-Board access requires 250 us minimum
 *  Spine Board I2C ADC read-back requires 400 us minimum
 *  LDC1614 read-back requires 650 us minimum
 *  All other I2C devices in the chain have not been tested
 */
#define PROCESS_TIME        650

#define CS_ASSERTED         1
#define CS_DEASSERTED       0

typedef struct {
    uint8_t header;
    uint8_t devAddr;
    uint8_t regAddr;
    uint8_t data[2];
} commPacket;

typedef enum {
    SPI_FAILED = 0,
    SPI_SUCCESS = 1
} spiState;

void sampleSpiInit(void);
spiState writeZBoard(uint8_t intf, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t numBytes);
spiState writeZBoardMulti(uint8_t intf, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t numBytes);
spiState readZBoard(uint8_t intf, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t numBytes);
spiState readZBoardMulti(uint8_t intf, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t numBytes);
void enableIr(void);
void disableIr(void);
void enableGLed(void);
void disableGLed(void);
void enableRLed(void);
void disableRLed(void);
spiState testZBoardIntf(void);
uint16_t getIrAnalogVal(void);
uint16_t getAnalogVal(uint8_t channel);
uint16_t getSpineBoardAdc(uint8_t channel);

#endif //DV13131B_H