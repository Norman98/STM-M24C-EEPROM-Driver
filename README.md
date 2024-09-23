# STM M24C EEPROM Driver

This repository contains a C++ driver for the **STM M24C EEPROM series** with I2C communication support. The driver is written to be platform-independent by using a virtual I2C interface that must be implemented for your specific platform (e.g., STM32, ESP32). Currently, the driver is specialized for the **M24C16** model but can be easily extended to support other EEPROM sizes in the M24C family.

## Features

- **Platform-Independent I2C Interface**: Easily integrate with any platform by implementing the abstract `I2C_M24C` class.
- **EEPROM Models Supported**: 
  - M24C16 (16Kb)
  - Easy extension for M24C32, M24C64, and others by adding specializations.
- **Memory Operations**:
  - Byte, halfword, and block read/write
  - Chip erase and page erase
- **Error Handling**: Continuous polling until I2C errors are resolved.
- **EEPROM Paging Support**: Automatically handles paging based on EEPROM model's page size.

## Getting Started

### Requirements

- **C++17 or later**
- **I2C Communication Support**: Implement the `I2C_M24C` interface for your platform.

### Adding the Library

Clone this repository or download the source code into your project.

```bash
git clone https://github.com/Norman98/STM-M24C-EEPROM-Driver.git
