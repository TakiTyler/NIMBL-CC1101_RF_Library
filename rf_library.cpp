#include "rf_library.h"

#define SO_PIN BIT3
#define SI_PIN BIT4
#define SCLK_PIN BIT3
#define CSN_PIN BIT4

// auto generate using SmartRF
const struct rf_settings cc1101_config[] =
{
    {IOCFG2, 0x29},         // GDO2 Output Pin Configuration
    {IOCFG1, 0x2E},         // GDO1 Output Pin Configuration
    {IOCFG0, 0x06},         // GDO0 Output Pin Configuration
    {FIFOTHR, 0x47},        // RX FIFO and TX FIFO Thresholds
    {SYNC1, 0xD3},          // Sync Word, High Byte
    {SYNC0, 0x91},          // Sync Word, Low Byte
    {PKTLEN, 0xFF},         // Packet Length
    {PKTCTRL1, 0x04},       // Packet Automation Control
    {PKTCTRL0, 0x05},       // Packet Automation Control
    {ADDR, 0x00},           // Device Address
    {CHANNR, 0x00},         // Channel Number
    {FSCTRL1, 0x08},        // Frequency Synthesizer Control
    {FSCTRL0, 0x00},        // Frequency Synthesizer Control
    {FREQ2, 0x23},          // Frequency Control Word, High Byte
    {FREQ1, 0x31},          // Frequency Control Word, Middle Byte
    {FREQ0, 0x3B},          // Frequency Control Word, Low Byte
    {MDMCFG4, 0x6C},        // Modem Configuration
    {MDMCFG3, 0x22},        // Modem Configuration
    {MDMCFG2, 0x03},        // Modem Configuration
    {MDMCFG1, 0x22},        // Modem Configuration
    {MDMCFG0, 0xF8},        // Modem Configuration
    {DEVIATN, 0x47},        // Modem Deviation Setting
    {MCSM2, 0x07},          // Main Radio Control State Machine Configuration
    {MCSM1, 0x30},          // Main Radio Control State Machine Configuration
    {MCSM0, 0x18},          // Main Radio Control State Machine Configuration
    {FOCCFG, 0x1D},         // Frequency Offset Compensation Configuration
    {BSCFG, 0x6C},          // Bit Synchronization Configuration
    {AGCCTRL2, 0xC7},       // AGC Control
    {AGCCTRL1, 0x00},       // AGC Control
    {AGCCTRL0, 0xB2},       // AGC Control
    {WOREVT1, 0x87},        // High Byte Event0 Timeout
    {WOREVT0, 0x6B},        // Low Byte Event0 Timeout
    {WORCTRL, 0xFB},        // Wake On Radio Control
    {FREND1, 0xB6},         // Front End RX Configuration
    {FREND0, 0x10},         // Front End TX Configuration
    {FSCAL3, 0xEA},         // Frequency Synthesizer Calibration
    {FSCAL2, 0x2A},         // Frequency Synthesizer Calibration
    {FSCAL1, 0x00},         // Frequency Synthesizer Calibration
    {FSCAL0, 0x1F},         // Frequency Synthesizer Calibration
    {RCCTRL1, 0x41},        // RC Oscillator Configuration
    {RCCTRL0, 0x00},        // RC Oscillator Configuration
    {FSTEST, 0x59},         // Frequency Synthesizer Calibration Control
    {PTEST, 0x7F},          // Production Test
    {AGCTEST, 0x3F},        // AGC Test
    {TEST2, 0x81},          // Various Test Settings
    {TEST1, 0x35},          // Various Test Settings
    {TEST0, 0x09},          // Various Test Settings
    {PARTNUM, 0x00},        // Chip ID
    {VERSION, 0x04},        // Chip ID
    {FREQEST, 0x00},        // Frequency Offset Estimate from Demodulator
    {LQI, 0x00},            // Demodulator Estimate for Link Quality
    {RSSI, 0x00},           // Received Signal Strength Indication
    {MARCSTATE, 0x00},      // Main Radio Control State Machine State
    {WORTIME1, 0x00},       // High Byte of WOR Time
    {WORTIME0, 0x00},       // Low Byte of WOR Time
    {PKTSTATUS, 0x00},      // Current GDOx Status and Packet Status
    {VCO_VC_DAC, 0x00},     // Current Setting from PLL Calibration Module
    {TXBYTES, 0x00},        // Underflow and Number of Bytes
    {RXBYTES, 0x00},        // Overflow and Number of Bytes
    {RCCTRL1_STATUS, 0x00}, // Last RC Oscillator Calibration Result
    {RCCTRL0_STATUS, 0x00}, // Last RC Oscillator Calibration Result
};

rf_library::rf_library(uint8_t csn_pin) 
{
    _csn_pin = csn_pin; 
}

void rf_library::config_SPI()
{
    UCB1CTLW0 = UCSWRST; // reset to start config

    // enable: master mode, synchronous mode, MSB, caputre on first edge, idle low
    UCB1CTLW0 |= (UCMST | UCSYNC | UCMSB | UCCKPH | UCSSEL__SMCLK);

    UCB1BRW = 0x0002; // set clock divider

    // setting SCLK for SPI
    P5SEL0 |= SCLK_PIN;
    P5SEL1 &= ~(SCLK_PIN);

    // setting SI & SO
    P4SEL0 |= SO_PIN | SI_PIN;
    P4SEL1 &= ~(SO_PIN | SI_PIN);

    // setting chip select
    P5DIR |= CSN_PIN;
    P5OUT |= CSN_PIN;

    UCB1CTLW0 &= ~UCSWRST; // clear reset to stop config
}

void rf_library::config_radio()
{
    uint8_t num_registers;

    command_strobe(SRES); // reset radio

    // find number of registers we need to write
    num_registers = sizeof(cc1101_config) / sizeof(cc1101_config[0]);

    // loop through all registers
    for(int i = 0; i < num_registers; i++)
    {
        write_single_byte(cc1101_config[i].address, cc1101_config->value);
    }

    // to change power level, write to PATABLE
    write_single_byte(PATABLE, 0xC6); // 0xC6 = default power
}

uint8_t rf_library::read_single_byte(uint8_t address)
{
    uint8_t data_byte, status_byte;
    uint8_t header = (READ_BIT | NO_BURST | address); // read 1 byte

    P5OUT &= ~CSN_PIN;      // pull low to start communication
    while (P4IN & SO_PIN);  // wait for SO pin to go low

    // send header information //
    while (!(UCB1IFG & UCTXIFG));   // wait until we can transmit
    UCB1TXBUF = header;             // send header
    while (!(UCB1IFG & UCRXIFG));   // wait for receive byte
    status_byte = UCB1RXBUF;        // read buffer

    // read data //
    while (!(UCB1IFG & UCTXIFG));
    UCB1TXBUF = DUMMY;              // send dummy bytes
    while (!(UCB1IFG & UCRXIFG));
    data_byte = UCB1RXBUF;          // read data

    P5OUT |= CSN_PIN; // pull high to end communication

    return data_byte; // return data for reads
}

uint8_t rf_library::write_single_byte(uint8_t address, uint8_t write)
{
    uint8_t data_byte, status_byte;
    uint8_t header = (WRITE_BIT | NO_BURST | address); // write 1 byte

    P5OUT &= ~CSN_PIN;      // pull low to start communication
    while (P4IN & SO_PIN);  // wait for SO pin to go low

    // send header information //
    while (!(UCB1IFG & UCTXIFG));   // wait until we can transmit
    UCB1TXBUF = header;             // send header
    while (!(UCB1IFG & UCRXIFG));   // wait for receive byte
    status_byte = UCB1RXBUF;        // read buffer

    // write data //
    while (!(UCB1IFG & UCTXIFG));
    UCB1TXBUF = write;              // send bytes to write
    while (!(UCB1IFG & UCRXIFG));
    data_byte = UCB1RXBUF;          // read data

    P5OUT |= CSN_PIN; // pull high to end communication

    return status_byte; // return status for writes
}

// IF GDO0 is set to high, call this function
void rf_library::read_burst(uint8_t address)
{
    uint8_t dummy, length;
    uint8_t header = (READ_BIT | BURST | address); // read many bytes

    P5OUT &= ~CSN_PIN;      // pull low to start communication
    while (P4IN & SO_PIN);  // wait for SO pin to go low

    // send header information //
    while (!(UCB1IFG & UCTXIFG));   // wait until we can transmit
    UCB1TXBUF = header;             // send header
    while (!(UCB1IFG & UCRXIFG));   // wait for receive byte
    dummy = UCB1RXBUF;              // read buffer to clear flag

    // read length byte //
    while (!(UCB1IFG & UCTXIFG));
    UCB1TXBUF = FIFO_ADDRESS;       // read buffer
    while (!(UCB1IFG & UCRXIFG));
    length = UCB1RXBUF;             // get length

    // read length bytes of data //
    for (int i = 0; i < length; i++)
    {
        while (!(UCB1IFG & UCTXIFG));
        UCB1TXBUF = FIFO_ADDRESS;
        while (!(UCB1IFG & UCRXIFG));
        dummy = UCB1RXBUF;
    }

    P5OUT |= CSN_PIN; // pull high to end communication
}

void rf_library::write_burst(uint8_t address, uint8_t *data, uint8_t length)
{
    uint8_t dummy;
    uint8_t header = (WRITE_BIT | BURST | address); // write many bytes

    P5OUT &= ~CSN_PIN;      // pull low to start communication
    while (P4IN & SO_PIN);  // wait for SO pin to go low

    // send header information //
    while (!(UCB1IFG & UCTXIFG));   // wait until we can transmit
    UCB1TXBUF = header;             // send header
    while (!(UCB1IFG & UCRXIFG));   // wait for receive byte
    dummy = UCB1RXBUF;              // read buffer to clear flag

    // write many bytes of data //
    for (int i = 0; i < length; i++)
    {
        while (!(UCB1IFG & UCTXIFG));
        UCB1TXBUF = data[i];
        while (!(UCB1IFG & UCRXIFG));
        dummy = UCB1RXBUF;
    }

    P5OUT |= CSN_PIN; // pull high to end communication
}

void rf_library::write_string(uint8_t address, const char *string)
{
    uint8_t dummy;
    uint8_t header = (WRITE_BIT | BURST | address);
    uint8_t str_length = strlen(string);

    if (str_length > 63) return; // CC1101 can't handle 64 length

    P5OUT &= ~CSN_PIN;      // pull low to start communication
    while (P4IN & SO_PIN);  // wait for SO pin to go low

    // send header information //
    while (!(UCB1IFG & UCTXIFG));   // wait until we can transmit
    UCB1TXBUF = header;             // send header
    while (!(UCB1IFG & UCRXIFG));   // wait for receive byte
    dummy = UCB1RXBUF;              // read buffer to clear flag

    // write data //
    while (!(UCB1IFG & UCTXIFG));
    UCB1TXBUF = str_length;         // send dummy bytes
    while (!(UCB1IFG & UCRXIFG));
    dummy = UCB1RXBUF;              // read data

    // write many bytes of data //
    for (int i = 0; i < str_length; i++)
    {
        while (!(UCB1IFG & UCTXIFG));
        UCB1TXBUF = string[i];
        while (!(UCB1IFG & UCRXIFG));
        dummy = UCB1RXBUF;
    }

    P5OUT |= CSN_PIN; // pull high to end communication

    command_strobe(STX); // trigger a transmit
}

// this only writes a byte, doesn't read one back
uint8_t rf_library::command_strobe(uint8_t address)
{
    uint8_t status_byte;
    uint8_t header = (WRITE_BIT | NO_BURST | address); // write 1 byte;

    P5OUT &= ~CSN_PIN;      // pull low to start communication
    while (P4IN & SO_PIN);  // wait for SO pin to go low

    // send header information //
    while (!(UCB1IFG & UCTXIFG));   // wait until we can transmit
    UCB1TXBUF = header;             // send header
    while (!(UCB1IFG & UCRXIFG));   // wait for receive byte
    status_byte = UCB1RXBUF;        // read buffer

    P5OUT |= CSN_PIN; // pull high to end communication

    return status_byte; // return status for writes
}

void test_rf()
{
    // stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // disable high-impedance mode
    PM5CTL0 &= ~LOCKLPM5;

    // led feedback
    P1DIR |= LED_PIN;
    P1OUT &= ~LED_PIN;

    // instantiate & config
    rf_library myRadio(CSN_PIN);
    myRadio.config_SPI();

    // send reset strob
    myRadio.command_strobe(SRES);

    //// test 1 (who am i) ////
    uint8_t part_num = myRadio.read_single_byte(PARTNUM);

    //// test 2 (read / write) ////
    uint8_t test_val = 0x33;
    myRadio.write_single_byte(PKTLEN, test_val);
    uint8_t read_back = myRadio.read_single_byte(PKTLEN);

    //// validation ////
    if(part_num == 0x00 && read_back == test_val)
    {
        // if we have success, turn on LED
        P1OUT |= LED_PIN;
    }
    else
    {
        // if we have an error, flash LEDs
        volatile uint16_t counter = 65000;
        while(1)
        {
            P1OUT ^= LED_PIN;
            while(counter != 0){counter--;}
            counter = 65000;
        }
    }

    while(1){}

    return;
}