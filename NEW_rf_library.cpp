/*

This is a re-do of the rf_library.cpp

---- PIN ASSIGNMENTS ----

CSN  = P5.4
MOSI = P4.4
MISO = P4.3
SCLK = P5.3
GDO0 = P1.0
GDO2 = P1.7

The GDOX pins are digital output pins that can be configured with:
    IOCFG0.GDO0_CFG, IOCFG2.GDO2_CFG

Useful configurations: (page 62)

    0x00 - "Associated w/ RX FIFO: Asserts when RX FIFO is filled at or above the RX FIFO threshold.
            De-asserts when RX FIFO is drained below the same threshold."

    0x01 - "Associated w/ RX FIFO: Asserts when RX FIFO is filled at or above the RX FIFO threshold or the end of packet is reached.
            De-asserts when the RX FIFO is empty"

    0x02 - "Associated w/ TX FIFO: Asserts when TX FIFO is filled at or above the TX FIFO threshold.
            De-asserts when the TX FIFO is below the same threshold."

    0x03 - "Associated w/ TX FIFO: Asserts when TX FIFO is full.
            De-asserts when the TX FIFO is drained below the TX FIFO threshold."

    0x04 - "Asserts when the RX FIFO has overflowed. De-asserts when the FIFO is flushed."

    0x05 - "Asserts when the TX FIFO has underflowed. De-asserts when the FIFO is flushed."

    0x06 - "Asserts when sync word has been sent / received, and de-asserts at the end of the packet.
            In RX, the pin will also deassert when a packet is discarded due to address or maximum length filtering or when the radio enters RXFIFO_OVERFLOW state.
            In TX the pin will de-assert if the TX FIFO underflows"

    0x07 - "Asserts when a packet has been received with CRC OK. De-asserts when the first byte is read from the RX FIFO."

---- GDO0 ----  (page 62 of datasheet)

    Used for:


---- GDO2 ----  (page 62 of datasheet)

    Used for:


---- header byte format ----

Bit 7    = R/W   (0 = write | 1 = read)     (page 30)
Bit 6    = Burst (0 = single | 1 = burst)
Bits 5-0 = Address

*/

#include "NEW_rf_library.h"

void RF_module::spiInit(){
    // set chip select
    CSN_DIR |= CSN_PIN;
    CSN_OUT |= CSN_PIN;

    // set GDO0 & GDO2, no pull-up resistor
    P1DIR &= ~(GDO0_PIN | GDO2_PIN);
    P1REN &= ~(GDO0_PIN | GDO2_PIN);

    // enable eUSCI_B1, since P4.4 & P4.3 correspond to pin 8 & 9
    P4SEL0 |= (MOSI_PIN | MISO_PIN);
    P4SEL1 &= ~(MOSI_PIN | MISO_PIN);

    // set SCLK
    P5SEL0 |= SCLK_PIN;
    P5SEL1 &= ~SCLK_PIN;

    // reset to start config
    UCB1CTLW0 = UCSWRST;

    UCB1CTLW0 |= (UCMST | UCSYNC | UCMSB | UCCKPH | UCSSEL__SMCLK); // using SMCLK, 1 MHz

    UCB1BRW = 0x0001; // no clock divider

    // release reset
    UCB1CTLW0 &= ~UCSWRST;
}

// consolidates the 4-line pipeline of:
//      wait transmit -> send byte -> wait buffer -> read buffer
uint8_t RF_module::spiTransfer(uint8_t byte){
    while(!(UCB1IFG & UCTXIFG)); // wait until tx buffer is empty
    UCB1TXBUF = byte; // send byte
    while(!(UCB1IFG & UCRXIFG)); // wait until rx buffer has byte
    return UCB1RXBUF; // return byte
}

// datasheet states (page 29):
// "When CSn is pulled low, the MCU must wait until CC1101 SO pin goes low"
void RF_module::csAssert(){
    CSN_OUT &= ~CSN_PIN;        // pull csn low
    while (MISO_IN & MISO_PIN); // wait for MISO to go low
}

// csn must go high to state the end of a transaction
void RF_module::csDeassert(){
    CSN_OUT |= CSN_PIN; // pull csn high
}

// writes the 'val' byte to register 'address'
//      using the & operator w/ 0x3F ensures the two MSBs are 0
void RF_module::writeRegister(uint8_t address, uint8_t val){
    csAssert();
    spiTransfer(address & 0x3F);    // send header
    spiTransfer(val);               // send data
    csDeassert();
}

// reads a single byte from 'address', from the datasheet (page 32):
// "For register addresses in the range 0x30 -> 0x3D, the burst bit is used
//      to select between status registers when burst bit is one"
// if we don't do this, it runs the risk of sending a command strobe
// READ_BURST includes the READ & BURST bit
uint8_t RF_module::readRegister(uint8_t address){
    csAssert();

    // send header
    if(address >= 0x30 && address <= 0x3D) spiTransfer(READ_BURST | (address & 0x3F));
    else spiTransfer(READ_SINGLE | (address & 0x3F));

    uint8_t response = spiTransfer(0x00);   // send dummy byte
    csDeassert();
    return response;
}

// writes *data a length amount of times
void RF_module::writeBurst(uint8_t address, uint8_t *data, uint8_t length){
    csAssert();
    spiTransfer(WRITE_BURST | (address & 0x3F));                // send header
    for(uint8_t i = 0; i < length; i++) spiTransfer(data[i]);   // send data
    csDeassert();
}

// reads address length amount of times and stores in *data
void RF_module::readBurst(uint8_t address, uint8_t *data, uint8_t length){
    csAssert();
    spiTransfer(READ_BURST | (address & 0x3F));                         // send header
    for(uint8_t i = 0; i < length; i++) data[i] = spiTransfer(0x00);    // read data
    csDeassert();
}

// command strobe, returns a status byte (page 31)
// sends single byte instructions, where internal sequences will be started (page 32)
// SIDLE will clear all pending commands
uint8_t RF_module::strobe(uint8_t cmd){
    csAssert();
    uint8_t response = spiTransfer(cmd);
    csDeassert();
    return response;
}

// get status is a helper to let us know what status the cc1101 is in via SNOP (page 67)
uint8_t RF_module::getStatus(){
    return strobe(SNOP);
}

// calibration will not be performed when SIDLE is called (page 52)
// to calibrate the cc1101 takes various amounts of time, check page 54
void RF_module::setIdle(){
    strobe(SIDLE);
}

// we do this version of a delay since the infrastructure to build a microsecond delay isn't worth the time
static void delayMicroseconds(uint16_t us){
    us >>= 2;
    while(us--){
        __no_operation();
    }
}

// manual sequence (page 51)
void RF_module::reset(){
    // set SCLK to 1 and SI to 0

    // strobe cs low / high
    csDeassert();
    delayMicroseconds(5);
    csAssert();
    delayMicroseconds(10);
    csDeassert();

    // wait for 40us at minimum, use 50 just in case
    delayMicroseconds(50);

    // set cs high and send SRES
    csAssert();
    while(MISO_IN & MISO_PIN);  // wait for MISO to go low
    spiTransfer(SRES);
    while(MISO_IN & MISO_PIN); // wait for MISO to go low

    csDeassert();
}

void RF_module::loadConfigRegisters(){
    writeRegister(IOCFG2, 0x2E);        // goes high when RX FIFO >= threshold
    writeRegister(IOCFG0, 0x06);        // we use this to check when a SYNC word has been received
    writeRegister(FIFOTHR, 0x47);       // rx fifo and tx fifo thresholds
    writeRegister(PKTLEN, PACKET_LEN);  // only send 2 bytes (not including CRC)
    writeRegister(PKTCTRL1, 0x04);      // append status bytes enabled
    writeRegister(PKTCTRL0, 0x04);      // enable CRC
    writeRegister(FSCTRL1, 0x06);       // 152.3 kHz IF
    writeRegister(FREQ2, 0x23);         // high byte
    writeRegister(FREQ1, 0x31);         // middle byte
    writeRegister(FREQ0, 0x3B);         // low byte
    writeRegister(MDMCFG4, 0xC5);       // modem config
    writeRegister(MDMCFG3, 0x83);       // modem config
    writeRegister(MDMCFG2, 0x03);       // modem config
    writeRegister(MDMCFG1, 0x22);       // modem config
    writeRegister(MDMCFG0, 0xF8);       // modem config
    writeRegister(DEVIATN, 0x34);       // modem deviation
    writeRegister(MCSM1, 0x00);         // main radio control
    writeRegister(MCSM0, 0x18);         // main radio control
    writeRegister(FOCCFG, 0x16);        // frequency offset compensation
    writeRegister(AGCCTRL2, 0x43);      // AGC control
    writeRegister(AGCCTRL1, 0x49);      // AGC control
    writeRegister(AGCCTRL0, 0x91);      // AGC control
    writeRegister(FREND1, 0x56);        // front end RX config
    writeRegister(FREND0, 0x10);        // front end TX config
    writeRegister(FSCAL3, 0xE9);        // frequency synthesizer calibration
    writeRegister(FSCAL2, 0x2A);        // frequency synthesizer calibration
    writeRegister(FSCAL1, 0x00);        // frequency synthesizer calibration
    writeRegister(FSCAL0, 0x1F);        // frequency synthesizer calibration
    writeRegister(TEST2, 0x81);         // various test settings
    writeRegister(TEST1, 0x35);         // various test settings
    writeRegister(TEST0, 0x09);         // various test settings

    uint8_t paTable[1] = {0xC3};
    writeBurst(PATABLE, paTable, 1);
}

void RF_module::begin(){
    spiInit();
    reset();
    loadConfigRegisters();
}