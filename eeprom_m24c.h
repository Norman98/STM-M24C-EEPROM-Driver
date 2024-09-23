
/*
 * ----------------------------------
 * STM EEPROM series M24C driver
 *
 * Author: Norman Dryś
 * Version: 1.0.0
 * Last change: 2024-09-09
 * ----------------------------------
 */

#pragma once

#include <stdint.h>


// ========================================== I2C Interface ==========================================

/**
 * @brief Interface for I2C communication. Derived classes should implement this platform-specific logic.
*/
class I2C_M24C
{
public:

    /**
     * @brief Enum representing I2C communication modes.
     */
    enum I2CMode
    {
        TX = 0, /**< Transmission mode */
        RX = 1, /**< Reception mode */
    };

    /**
     * @brief Resets, Configures and enables the I2C peripheral
     */
    virtual void Init() = 0;

    /**
     * @brief Polls the EEPROM. I2C START condition included
     * @param device_id The address of the device to communicate with.
     * @param mode The mode of communication (TX/RX).
     * @param set_pos_bit (STM32) Indicates whether to set the POS bit (true for setting POS, false otherwise).
     */
    virtual void StartPolling(uint8_t device_id, I2CMode mode, bool set_pos_bit = false) = 0;

    /**
     * @brief Check if the I2C state indicates an error
     * @return true if there is an error, false otherwise.
     */
    virtual bool IsStateError() = 0;

    /**
     * @brief Reads a single byte from the I2C bus. I2C STOP condition included
     * @return The byte read from the I2C bus.
     */
    virtual uint8_t ReadByte() = 0;

    /**
     * @brief Reads a halfword (16-bit) from the I2C bus. I2C STOP condition included.
     * EepromM24C "WriteHalfWord" method stores data in little-endian format.
     * @return The halfword value read from the I2C bus.
     */
    virtual uint16_t ReadHalfWord() = 0;

    /**
     * @brief Reads multiple bytes from the I2C bus. I2C STOP condition included
     * @param output Pointer to the buffer where the read bytes will be stored.
     * @param size The number of bytes to read from the I2C bus.
     */
    virtual void ReadMultipleBytes(uint8_t *output, uint16_t size) = 0;

    /**
     * @brief Writes a single byte to the I2C bus
     * @param data The byte of data to write to the I2C bus.
     */
    virtual void WriteByte(uint8_t data) = 0;

    /**
     * @brief Sends an I2C STOP condition
     */
    virtual void Stop() = 0;
};

// ========================================= Eeprom M24C ==========================================

/**
 * @brief Specific memory models in the EEPROM M24C series.
 */
enum class EepromM24CModel
{
    M24C16, // Tested
    // M24C32,
    // M24C64,
};

/**
 * @brief Traits to define model-specific constants.
 * @tparam model The EEPROM model from EepromM24CModel enum.
 */
template <EepromM24CModel model>
struct EepromModelTraits;

/**
 * @brief Specialization for EEPROM model M24C16.
 */
template <>
struct EepromModelTraits<EepromM24CModel::M24C16>
{
    static constexpr uint8_t PAGE_SIZE = 16;
    static constexpr uint16_t MEMORY_SIZE = 2048;
};

// Specializations for other models can be added as needed.
// template<>
// struct EepromModelTraits<EepromM24CModel::M24C32> {
//     static constexpr uint8_t PAGE_SIZE = 32;
//     static constexpr uint16_t MEMORY_SIZE = 4096;
// };

// template<>
// struct EepromModelTraits<EepromM24CModel::M24C64> {
//     static constexpr uint8_t PAGE_SIZE = 32;
//     static constexpr uint16_t MEMORY_SIZE = 8192;
// };

/**
 * @brief STM EEPROM series M24C driver.
 *
 * This template class provides methods to interact with EEPROM devices in the M24C series via I2C.
 *
 * @tparam model The EEPROM model type from the EepromM24CModel enum.
 */
template <EepromM24CModel model>
class EepromM24C
{
public:
    static constexpr uint8_t PAGE_SIZE = EepromModelTraits<model>::PAGE_SIZE;      /**< Page size in bytes for the specified model */
    static constexpr uint16_t MEMORY_SIZE = EepromModelTraits<model>::MEMORY_SIZE; /**< Total memory size in bytes for the specified model */

    EepromM24C(I2C_M24C &i2c_instance) : i2c(i2c_instance) {} // Dependency injection of I2C instance

    void WriteByte(uint16_t address, uint8_t value);
    void WriteHalfWord(uint16_t address, uint16_t value);
    void WriteBlock(void *data, uint16_t address, uint16_t block_size);

    uint8_t ReadByte(uint16_t address);
    uint16_t ReadHalfWord(uint16_t address);
    void ReadBlock(void *data, uint16_t address, uint16_t block_size);

    void ChipErase();
    void ErasePage(uint16_t address);

private:
    static constexpr uint8_t DEVICE_ID = 0b10100000;               /**< I2C device ID for the EEPROM */
    static constexpr uint8_t CHIP_ENABLE_ADRESS_MASK = 0b00001110; /**< Mask to extract relevant address bits for chip enable */
    static constexpr uint8_t CHIP_ENABLE_ADRESS_SHIFT = 7;         /**< Shift to align address bits for chip enable */
    /**
     * @brief Generates the device select code based on the EEPROM address.
     * @param address The EEPROM address.
     * @return uint8_t The device select code.
     */
    uint8_t HandleDeviceSelectCode(uint16_t address) const
    {
        return DEVICE_ID | ((address >> CHIP_ENABLE_ADRESS_SHIFT) & CHIP_ENABLE_ADRESS_MASK);
    };
    void WritePage(void *data, uint16_t address, uint8_t data_size);

    I2C_M24C &i2c; // Reference to the I2C interface
};

// ========================================= Eeprom M24C Implementation ==========================================

/**
 * @brief Writes a byte to the specified address.
 * @param address The EEPROM address to write to.
 * @param value The byte value to write.
 */
template <EepromM24CModel model>
void EepromM24C<model>::WriteByte(uint16_t address, uint8_t value)
{
    uint8_t device_code = HandleDeviceSelectCode(address);

    do
    {
        if (i2c.IsStateError())
        {
            i2c.Init();
        }

        i2c.StartPolling(device_code, i2c.TX);
        i2c.WriteByte(static_cast<uint8_t>(address));
        i2c.WriteByte(value);
        i2c.Stop();

    } while (i2c.IsStateError());
}

/**
 * @brief Writes a 16-bit halfword to the specified address.
 * @param address The EEPROM address to write to (must be even).
 * @param value The 16-bit value to write.
 */
template <EepromM24CModel model>
void EepromM24C<model>::WriteHalfWord(uint16_t address, uint16_t value)
{
    uint8_t device_code = HandleDeviceSelectCode(address);

    do
    {
        if (i2c.IsStateError())
        {
            i2c.Init();
        }

        i2c.StartPolling(device_code, i2c.TX);
        i2c.WriteByte(static_cast<uint8_t>(address));
        i2c.WriteByte(static_cast<uint8_t>(value));
        i2c.WriteByte(static_cast<uint8_t>(value >> 8));
        i2c.Stop();

    } while (i2c.IsStateError());
}

/**
 * @brief Writes a page of data to the EEPROM.
 * @param data Pointer to the data to write.
 * @param address The starting address of the page.
 * @param data_size The size of the data to write.
 */
template <EepromM24CModel model>
void EepromM24C<model>::WritePage(void *data_ptr, uint16_t address, uint8_t data_size)
{
    uint8_t *data = reinterpret_cast<uint8_t*>(data_ptr);
    uint8_t device_code = HandleDeviceSelectCode(address);

    do
    {
        if (i2c.IsStateError())
        {
            i2c.Init();
        }

        i2c.StartPolling(device_code, i2c.TX);
        i2c.WriteByte(static_cast<uint8_t>(address));

        for (uint8_t i = 0; i < data_size; i++)
        {
            i2c.WriteByte(*(data + i));
        }

        i2c.Stop();

    } while (i2c.IsStateError());
}

/**
 * @brief Writes a block of data to the EEPROM.
 * @param data Pointer to the data to write.
 * @param address The starting address for the block. Must be a multiple of PAGE_SIZE if the block spans one or more pages.
 * @param data_size The size of the data block.
 */
template <EepromM24CModel model>
void EepromM24C<model>::WriteBlock(void *data_ptr, uint16_t address, uint16_t data_size)
{
    uint8_t *data = reinterpret_cast<uint8_t*>(data_ptr);
    uint16_t remaining_full_pages = data_size / PAGE_SIZE;

    while (remaining_full_pages >= 1)
    {
        WritePage(data, address, PAGE_SIZE);

        data += PAGE_SIZE;
        address += PAGE_SIZE;
        remaining_full_pages--;
    }

    WritePage(data, address, data_size % PAGE_SIZE);
}

/**
 * @brief Reads a byte from the specified address.
 * @param address The EEPROM address to read from.
 * @return The byte value read from the address.
 */
template <EepromM24CModel model>
uint8_t EepromM24C<model>::ReadByte(uint16_t address)
{
    uint8_t device_code = HandleDeviceSelectCode(address);
    uint8_t read_value;

    do
    {
        if (i2c.IsStateError())
        {
            i2c.Init();
        }

        i2c.StartPolling(device_code, i2c.TX);
        i2c.WriteByte(static_cast<uint8_t>(address));
        i2c.StartPolling(device_code, i2c.RX);
        read_value = i2c.ReadByte();

    } while (i2c.IsStateError());

    return read_value;
}

/**
 * @brief Reads a 16-bit halfword from the specified address.
 * @param address The EEPROM address to read from (must be even).
 * @return The 16-bit value read from the address.
 */
template <EepromM24CModel model>
uint16_t EepromM24C<model>::ReadHalfWord(uint16_t address)
{
    uint8_t device_code = HandleDeviceSelectCode(address);
    uint16_t read_value = 0;

    do
    {
        if (i2c.IsStateError())
        {
            i2c.Init();
        }

        i2c.StartPolling(device_code, i2c.TX, 1);
        i2c.WriteByte(static_cast<uint8_t>(address));
        i2c.StartPolling(device_code, i2c.RX);
        read_value = i2c.ReadHalfWord();

    } while (i2c.IsStateError());

    return read_value;
}

/**
 * @brief Reads a block of data from the EEPROM.
 * @param data Pointer to the buffer to store the read data.
 * @param address The starting address for the block. Must be a multiple of PAGE_SIZE if the block spans one or more pages.
 * @param data_size The size of the data block.
 */
template <EepromM24CModel model>
void EepromM24C<model>::ReadBlock(void *data_ptr, uint16_t address, uint16_t data_size)
{
    uint8_t *data = reinterpret_cast<uint8_t*>(data_ptr);
    uint8_t device_code = HandleDeviceSelectCode(address);

    do
    {
        if (i2c.IsStateError())
        {
            i2c.Init();
        }

        i2c.StartPolling(device_code, i2c.TX);
        i2c.WriteByte(static_cast<uint8_t>(address));
        i2c.StartPolling(device_code, i2c.RX);
        i2c.ReadMultipleBytes(data, data_size);

    } while (i2c.IsStateError());
}

/**
 * @brief Erases a page by filling it with 0xFF.
 * @param address The start address of the page to erase.
 */
template <EepromM24CModel model>
void EepromM24C<model>::ErasePage(uint16_t address)
{
    uint8_t device_code = HandleDeviceSelectCode(address);

    do
    {
        if (i2c.IsStateError())
        {
            i2c.Init();
        }

        i2c.StartPolling(device_code, i2c.TX);
        i2c.WriteByte(static_cast<uint8_t>(address));

        for (uint8_t i = 0; i < PAGE_SIZE; i++)
        {
            i2c.WriteByte(0xFF);
        }

        i2c.Stop();

    } while (i2c.IsStateError());
}

/**
 * @brief Erases the entire EEPROM by filling it with 0xFF.
 */
template <EepromM24CModel model>
void EepromM24C<model>::ChipErase()
{
    for (int i = 0; i < MEMORY_SIZE; i += PAGE_SIZE)
    {
        ErasePage(i);
    }
}