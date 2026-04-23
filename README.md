# CC1101 RF Transceiver Library

## Overview
This module contains a custom C++ driver for interfacing the **TI CC1101 Sub-1 GHz RF Transceiver** with a **TI MSP430** microcontroller. It serves as the wireless backbone for a low-power pulse oximeter project, enabling the reliable transmission of patient biometric data (SpO2 and Heart Rate) from a wearable device to a remote base station.

## NIMBL Libraries

*   [SparkFun_MAX32664_Library](https://github.com/TakiTyler/NIMBL-SparkFun_MAX32664_Library)
*   [HT1621_LCD_Library](https://github.com/TakiTyler/NIMBL-HT1621_LCD_Library)
*   [CC1101_RF_Library](https://github.com/TakiTyler/NIMBL-CC1101_RF_Library)
*   [Website_and_Python](https://github.com/TakiTyler/NIMBL-Website_and_Python)

## Hardware Interface
The library utilizes the MSP430's `eUSCI_B1` hardware SPI module for fast, reliable communication, alongside dedicated GPIO pins for radio state monitoring.

**SPI Pins:**
*   **CSN (Chip Select):** `P5.4`
*   **SCLK (Clock):** `P5.3`
*   **MOSI (Data Out):** `P4.4`
*   **MISO (Data In):** `P4.3`

**Status Pins:**
*   **GDO0:** `P1.0` (Used for packet sync/transmission confirmation)
*   **GDO2:** `P1.7`

## Features
*   **Hardware SPI Integration:** Leverages the MSP430 `eUSCI_B1` module for high-speed hardware SPI, reducing CPU overhead compared to bit-banging.
*   **Automated Packet Handling:** Built-in methods (`sendPacket`, `receivePacket`) handle payload formatting, TX/RX FIFO flushing, and command strobes automatically.
*   **State Machine Management:** Handles CC1101 idle states, calibration, and buffer overflow recovery seamlessly under the hood.
*   **Pre-configured Radio Settings:** Includes heavily optimized radio configurations (`loadConfigRegisters`) for Sub-1 GHz transmission, including CRC filtering and packet length automation.
*   **UART Debugging Tooling:** Includes optional UART initializers and print helpers for monitoring TX/RX statuses over a serial terminal.

## Repository Structure
*   `rf_library.h`: Defines the CC1101 register map, command strobes, pin configurations, and the `RF_module` class interface.
*   `rf_library.cpp`: Implements the SPI pipeline, burst read/writes, configuration loading, and high-level TX/RX sequences.

## Usage Example
Below is a high-level look at how to initialize the radio and perform basic Transmission (TX) and Reception (RX).

### Transmitting Data (Wearable Device)
```cpp
#include "msp430.h"
#include "new_rf_library.h"

void main(void) {
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // 1. Initialize the RF module
    RF_module radio;
    radio.begin(); 

    uint8_t current_spo2 = 98;
    uint8_t current_bpm = 115;

    while(1) {
        // 2. Transmit the 2-byte payload
        radio.sendPacket(current_spo2, current_bpm);

        // Delay until next reading...
    }
}
```

### Receiving Data (Base Station)
```cpp
#include "msp430.h"
#include "new_rf_library.h"

void main(void) {
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // 1. Initialize the RF module
    RF_module radio;
    radio.begin();

    uint8_t rx_spo2 = 0;
    uint8_t rx_bpm = 0;

    while(1) {
        // 2. Listen for incoming packets
        bool success = radio.receivePacket(rx_spo2, rx_bpm);

        if (success) {
            // Data successfully received and CRC passed!
            // Process/Display rx_spo2 and rx_bpm...
        }
    }
}
```

## Development Environment
*   **IDE:** Code Composer Studio (CCS) v12+
*   **Compiler:** TI MSP430 Compiler
*   **Target MCU:** TI MSP430FR2476

## Acknowledgments
This library relies heavily on Texas Instruments CC1101 data sheets and SmartRF Studio for generating the optimal RF register values.