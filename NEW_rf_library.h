/*

This is a re-do of the rf_library.cpp

//// PIN ASSIGNMENTS ////

CSN  = P5.4
MOSI = P4.4
MISO = P4.3
SCLK = P5.3
GDO0 = P1.0
GDO2 = P1.7

*/

#pragma once
#include <msp430.h>
#include <stdint.h>

#define CSN_DIR P5DIR
#define CSN_OUT P5OUT
#define CSN_PIN BIT4

#define MISO_IN P4IN
#define MISO_PIN BIT3
#define MOSI_PIN BIT4

#define SCLK_PIN BIT3

#define GDO0_IN P1IN
#define GDO0_PIN BIT0

#define GDO2_IN P1IN
#define GDO2_PIN BIT7

// register map
#define IOCFG2      0x00  // GDO2 output pin configuration
#define IOCFG1      0x01  // GDO1 output pin configuration
#define IOCFG0      0x02  // GDO0 output pin configuration
#define FIFOTHR     0x03  // RX FIFO and TX FIFO thresholds
#define SYNC1       0x04  // Sync word, high byte
#define SYNC0       0x05  // Sync word, low byte
#define PKTLEN      0x06  // Packet length
#define PKTCTRL1    0x07  // Packet automation control
#define PKTCTRL0    0x08  // Packet automation control
#define ADDR        0x09  // Device address
#define CHANNR      0x0A  // Channel number
#define FSCTRL1     0x0B  // Frequency synthesizer control
#define FSCTRL0     0x0C  // Frequency synthesizer control
#define FREQ2       0x0D  // Frequency control word, high byte
#define FREQ1       0x0E  // Frequency control word, middle byte
#define FREQ0       0x0F  // Frequency control word, low byte
#define MDMCFG4     0x10  // Modem configuration
#define MDMCFG3     0x11  // Modem configuration
#define MDMCFG2     0x12  // Modem configuration
#define MDMCFG1     0x13  // Modem configuration
#define MDMCFG0     0x14  // Modem configuration
#define DEVIATN     0x15  // Modem deviation setting
#define MCSM2       0x16  // Main radio control state machine configuration
#define MCSM1       0x17  // Main radio control state machine configuration
#define MCSM0       0x18  // Main radio control state machine configuration
#define FOCCFG      0x19  // Frequency offset compensation configuration
#define BSCFG       0x1A  // Bit synchronization configuration
#define AGCCTRL2    0x1B  // AGC control
#define AGCCTRL1    0x1C  // AGC control
#define AGCCTRL0    0x1D  // AGC control
#define WOREVT1     0x1E  // WOR event timeout, high byte
#define WOREVT0     0x1F  // WOR event timeout, low byte
#define WORCTRL     0x20  // Wake on radio control
#define FREND1      0x21  // Front end RX configuration
#define FREND0      0x22  // Front end TX configuration
#define FSCAL3      0x23  // Frequency synthesizer calibration
#define FSCAL2      0x24  // Frequency synthesizer calibration
#define FSCAL1      0x25  // Frequency synthesizer calibration
#define FSCAL0      0x26  // Frequency synthesizer calibration
#define RCCTRL1     0x27  // RC oscillator configuration
#define RCCTRL0     0x28  // RC oscillator configuration
#define FSTEST      0x29  // Frequency synthesizer calibration control
#define PTEST       0x2A  // Production test
#define AGCTEST     0x2B  // AGC test
#define TEST2       0x2C  // Various test settings
#define TEST1       0x2D  // Various test settings
#define TEST0       0x2E  // Various test settings
#define PATABLE     0x3E  // Burst access

// fifo registers
#define RXFIFO      0x3F
#define TXFIFO      0x3F

// command cmdStrobes
#define SRES        0x30  // Reset chip
#define SCAL        0x33  // Calibrate synthesizer
#define SRX         0x34  // Enable receive
#define STX         0x35  // Enable transmit
#define SIDLE       0x36  // Go idle
#define SFRX        0x3A  // Flush RX FIFO
#define SFTX        0x3B  // Flush TX FIFO
#define SNOP        0x3D  // No operation

// header byte masks
#define WRITE_BIT           0x00
#define NO_BURST            0x00
#define WRITE_BURST         0x40
#define READ_SINGLE         0x80
#define READ_BURST          0xC0

// packet length
#define PACKET_LEN  2 // one for SpO2, another for heart-rate

class RF_module
{
public:
    void begin();       // combination of spi init, and loading registers
    void reset();       // reset strobes
    void sendPacket(uint8_t spo2, uint8_t heartRate);
    void receivePacket(uint8_t &spo2, uint8_t &heartRate);
    void setIdle();
    uint8_t getStatus();

private:
    void spiInit();
    void loadConfigRegisters();
    uint8_t spiTransfer(uint8_t byte);
    void csAssert();
    void csDeassert();
    void writeRegister(uint8_t address, uint8_t val);
    uint8_t readRegister(uint8_t address);
    void writeBurst(uint8_t address, uint8_t *data, uint8_t length);
    void readBurst(uint8_t address, uint8_t *data, uint8_t length);
    uint8_t cmdStrobe(uint8_t cmd);
    static void delayMicroseconds(uint16_t us);
}