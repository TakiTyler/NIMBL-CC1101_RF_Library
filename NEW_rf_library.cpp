/*

This is a re-do of the rf_library.cpp

---- PIN ASSIGNMENTS ----

CSN  = P5.4
MOSI = P4.4
MISO = P4.3
SCLK = P5.3
GDO0 = P1.0
GDO2 = P1.7

---- GDO0 (ATEST) ----

    Used for:
    Test signals
    FIFO status signals
    Clear channel indicator
    Clock output
    Serial output RX data
    Serial input TX data

---- GDO2 ----

    Used for:
    Test signals
    FIFO status signals
    Clear channel indicator
    Clock output
    Serial output RX data

---- header byte format ----

Bit 7    = R/W   (0 = write | 1 = read)
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

// datasheet states:
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

// reads a single byte from 'address', from the datasheet:
// "For register addresses in the range 0x30 -> 0x3D, the burst bit is used
//      to select between status registers when burst bit is one"
// if we don't do this, it runs the risk of sending a command strobe
void RF_module::readRegister(uint8_t address){
    csAssert();
    spiTransfer(address | 0x80);            // send header
    uint8_t response = spiTransfer(0x00);   // send dummy byte
    csDeassert();
    return response;
}