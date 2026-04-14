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

#define LED_1 BIT5
#define LED_2 BIT2
#define LED_3 BIT6
#define LED_4 BIT7
#define LED_5 BIT0

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
uint8_t RF_module::cmdStrobe(uint8_t cmd){
    csAssert();
    uint8_t response = spiTransfer(cmd);
    csDeassert();
    return response;
}

// get status is a helper to let us know what status the cc1101 is in via SNOP (page 67)
uint8_t RF_module::getStatus(){
    return cmdStrobe(SNOP);
}

// calibration will not be performed when SIDLE is called (page 52)
// to calibrate the cc1101 takes various amounts of time, check page 54
void RF_module::setIdle(){
    cmdStrobe(SIDLE);
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

// double-check ALL register values
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

// sends a single 2-byte package
void RF_module::sendPacket(uint8_t spo2, uint8_t heartRate){
    cmdStrobe(SIDLE);               // clean slate
    cmdStrobe(SFTX);                // flush tx fifo
    uint8_t packet[PACKET_LEN] = {spo2, heartRate};
    writeBurst(TXFIFO, packet, PACKET_LEN);
    cmdStrobe(STX);                 // send packet
    while(!(GDO0_IN & GDO0_PIN));   // confirms that we finish calibration & transmitted preamble
    while(GDO0_IN & GDO0_PIN);      // last CRC byte clocked out
}

bool RF_module::receivePacket(uint8_t &spo2, uint8_t &heartRate){
    cmdStrobe(SIDLE);               // clean slate
    cmdStrobe(SFRX);                // flush rx fifo
    cmdStrobe(SRX);                 // begin listening
    while(!(GDO0_IN & GDO0_PIN));   // chip matched sync word
    while(GDO0_IN & GDO0_PIN);      // last CRC byte clocked
    uint8_t status = getStatus();   // check bits 6 to 4, 110 means overflow
    if(((status >> 4) & 0x07) == 0x06){ // overflow
        cmdStrobe(SIDLE);
        cmdStrobe(SFRX);
        return false;
    }
    uint8_t buffer[4]; // 2 payload bytes + 2 CRC bytes
    readBurst(RXFIFO, buffer, 4)
    if(!(buf[3] & 0x80)) return false;
    spo2 = buffer[0];
    heartRate = buffer[1];
    return true;
}

void delay_ms(uint16_t ms)
{
    // timer B0, 32kHz clock, count up, clock divider = 1
    TB0CTL = (TBSSEL__ACLK | MC__UP | ID__1 | TBCLR);

    // set the count value, about 32 counts per ms
    TB0CCR0 = 33 * ms;

    // clear the timer interrupt
    TB0CCTL0 &= ~CCIFG;

    // enable clock interrupt
    TB0CCTL0 |= CCIE;

    // enable low power mode & interrupts
    __bis_SR_register(LPM3_bits + GIE);

    // disable timer after we wake
    TB0CTL = MC__STOP;
}

#pragma vector = TIMER0_B0_VECTOR
__interrupt void Timer_B0_ISR(void)
{
    __bic_SR_register_on_exit(LPM3_bits); // wake CPU & exit low-power
}

// set up for 115k baud
static void initUART()
{
    P1SEL0 |= BIT4 | BIT5;

    UCA0CTLW0 |= UCSWRST;       // eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK; // choose smclk

    UCA0BR0 = 8;
    UCA0BR1 = 0x00;
    UCA0MCTLW = 0xD600;

    UCA0CTLW0 &= ~UCSWRST; // release from reset
}

static void uart_write_byte(uint8_t c)
{
    while (!(UCA0IFG & UCTXIFG)); // wait for tx buffer to be empty
    UCA0TXBUF = c;
}

static void uart_write_string(const char *str)
{
    while (*str) uart_write_byte((uint8_t)*str++);
}

static void uart_write_uint8(uint8_t val)
{
    uart_write_byte('0' + (val / 100));
    uart_write_byte('0' + (val % 100) / 10);
    uart_write_byte('0' + (val % 10));
}

static void initLEDs(){
    P3DIR |= (LED_1 | LED_2 | LED_3);
    P4DIR |= LED_4;
    P5DIR |= LED_5;

    P3OUT &= ~(LED_1 | LED_2 | LED_3);
    P4OUT &= ~LED_4;
    P5OUT &= ~LED_5;
}

void TX_test_main(){
    // stop watchdog & gpio high-impedance
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    // toggle if we see uart commands
    bool good_receive, do_uart = false;
    RF_module radio;
    uint8_t spo2 = 12, heartRate = 34, status, state;

    // starting both uart & radio
    initLEDs();
    if(do_uart) initUART();
    radio.begin();
    __bis_SR_register(GIE); // enable interrupts
    P3OUT |= LED_1;         // made it past init phase

    if(do_uart) uart_write_string("CC1101 Transmit is ready\r\n");

    while(1){
        P5OUT ^= LED_5; // blink LED to show transaction
        if(do_uart) uart_write_string("Sending packet...\r\n");
        radio.sendPacket(spo2, heartRate); // should send 12 & 34

        status = radio.getStatus();
        state = (status >> 4) & 0x07;   // 7th bit should always be 0, we don't care about other bytes
        if (state == 0x06 || state == 0x07) {
            P3OUT |= LED_3;
            if(do_uart) uart_write_string("UNDERFLOW\r\n");
        }
        else{
            P4OUT |= LED_4;
            if(do_uart){
                uart_write_string("status = ");
                uart_write_uint8(state);
                uart_write_string("\r\n");
            }
        }

        delay_ms(1000); // transmit once per seconds
    }
}

void RX_test_main(){

    // stop watchdog & gpio high-impedance
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    // toggle if we see uart commands
    bool good_receive, do_uart = true;
    RF_module radio;
    uint8_t spo2, heartRate, status, state;

    // starting both uart & radio
    initUART();
    radio.begin();
    __bis_SR_register(GIE); // enable interrupts

    uart_write_string("CC1101 Receive is ready\r\n");

    while(1){
        good_receive = radio.receivePacket(spo2, heartRate);
        if(good_receive){
            uart_write_string("GOOD RECEIVE  SpO2 = ");
            uart_write_uint8(spo2);
            uart_write_string(", Heart Rate = ");
            uart_write_uint8(heartRate);
            uart_write_string("\r\n");
        }
        else{
            // check if an overflow or CRC fail (page 31 for status meanings)
            status = radio.getStatus();
            state = (status >> 4) & 0x07;   // 7th bit should always be 0, we don't care about other bytes
            if (state == 0x06 || state == 0x07) uart_write_string("OVERFLOW\r\n");
            else {
                uart_write_string("CRC FAIL  status = ");
                uart_write_uint8(state);
                uart_write_string("\r\n");
            }
        }
    }
}