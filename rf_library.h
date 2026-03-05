#include "msp430.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifndef RF_LIBRARY_H_
#define RF_LIBRARY_H_

#define LED_PIN BIT0

#define DUMMY 0x00

#define COMMAND_STROBE_START 0x30
#define COMMAND_STROBE_END 0x3D

#define FIFO_ADDRESS 0x3F

#define WRITE_BIT 0x00
#define READ_BIT 0x80

#define NO_BURST 0x00
#define BURST 0x40

#define SINGLE_TX_BYTE 0x3F
#define BURST_TX_BYTE 0x7F
#define SINGLE_RX_BYTE 0xBF
#define BURST_RX_BYTE 0xFF

struct rf_settings
{
    uint8_t address;
    uint8_t value;
};

// values found in datasheet
enum state_machine
{
    IDLE,
    RX,
    TX,
    FSTXON,
    CALIBRATE,
    SETTLING,
    RXFIFO_OVERFLOW,
    TXFIFO_OVERFLOW
};

enum rw_config_registers
{
    IOCFG2,
    IOCFG1,
    IOCFG0,
    FIFOTHR,
    SYNC1,
    SYNC0,
    PKTLEN,
    PKTCTRL1,
    PKTCTRL0,
    ADDR,
    CHANNR,
    FSCTRL1,
    FSCTRL0,
    FREQ2,
    FREQ1,
    FREQ0,
    MDMCFG4,
    MDMCFG3,
    MDMCFG2,
    MDMCFG1,
    MDMCFG0,
    DEVIATN,
    MCSM2,
    MCSM1,
    MCSM0,
    FOCCFG,
    BSCFG,
    AGCCTRL2,
    AGCCTRL1,
    AGCCTRL0,
    WOREVT1,
    WOREVT0,
    WORCTRL,
    FREND1,
    FREND0,
    FSCAL3,
    FSCAL2,
    FSCAL1,
    FSCAL0,
    RCCTRL1,
    RCCTRL0,
    FSTEST,
    PTEST,
    AGCTEST,
    TEST2,
    TEST1,
    TEST0
};

enum command_strobes
{
    SRES = 0x30,
    SFSTXON,
    SXOFF,
    SCAL,
    SRX,
    STX,
    SIDLE,
    SWORD = 0x38,
    SPWD,
    SFRX,
    SFTX,
    SWORRST,
    SNOP,
    PATABLE,
    TX_FIFO
};

enum burst_command_strobes
{
    PARTNUM = 0x30,
    VERSION,
    FREQEST,
    LQI,
    RSSI,
    MARCSTATE,
    WORTIME1,
    WORTIME0,
    PKTSTATUS,
    VCO_VC_DAC,
    TXBYTES,
    RXBYTES,
    RCCTRL1_STATUS,
    RCCTRL0_STATUS,
    RX_FIFO = 0x3F
};

class rf_library
{
public:
    rf_library(uint8_t csn_pin);

    void config_SPI();
    void config_radio(); // using baud 115.2k for now w/ 2-FSK
    uint8_t read_single_byte(uint8_t address);
    uint8_t write_single_byte(uint8_t address, uint8_t write);
    void read_burst(uint8_t address);
    void write_burst(uint8_t address, uint8_t *data, uint8_t length);
    void write_string(uint8_t address, const char *string);
    uint8_t command_strobe(uint8_t address);

private:
    /* data */
    uint8_t _csn_pin;
};

/* RF_LIBRARY_H_ */
#endif
