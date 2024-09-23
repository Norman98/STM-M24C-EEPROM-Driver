# STM M24C EEPROM Driver

This repository contains a C++ driver for the **STM M24C EEPROM series**. It provides robust read, write, and erase operations through an abstract I2C interface, making it platform-independent and flexible. The driver is written to be platform-independent by using a virtual I2C interface that must be implemented for your specific platform (e.g., STM32, ESP32). Currently, the driver is specialized for the **M24C16** model but can be easily extended to support other EEPROM sizes in the M24C family.

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
```

### Usage
1. Implement the I2C Interface
You need to implement the I2C_M24C class, which serves as an abstraction for your platform-specific I2C communication.

2. Instantiate the EEPROM Driver
After implementing your platform-specific I2C_M24C, pass it to the EepromM24C class template.

```cpp
#include "EepromM24C.hpp"
#include "YourPlatformI2C.hpp"

// Instantiate your I2C interface
YourPlatformI2C i2c_instance;

// Instantiate EEPROM driver for M24C16
EepromM24C<EepromM24CModel::M24C16> eeprom(i2c_instance);
```

3. Perform Operations
Use the following methods to read, write, and erase data on the EEPROM.

```
// Write a byte
eeprom.WriteByte(0x000A, 0xFF);

// Read a byte
uint8_t data = eeprom.ReadByte(0x000A);

// Erase a page
eeprom.ErasePage(0x0000);

// Erase the entire chip
eeprom.ChipErase();
```

### Extending to Other Models
To add support for other models like M24C32, M24C64, etc., simply specialize the EepromModelTraits for the desired model.

Example:

```cpp
template <>
struct EepromModelTraits<EepromM24CModel::M24C32> {
    static constexpr uint8_t PAGE_SIZE = 32;
    static constexpr uint16_t MEMORY_SIZE = 4096;
};
```

### License
This project is licensed under the MIT License - see the LICENSE file for details.

### Author
Norman Dryś
