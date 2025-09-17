/*
 * ----------------------------------
 * ST EEPROM series M24 driver
 *
 * Author: Norman Dryś
 * Version: 2.0.0
 * Last change: 2024-09-09
 * ----------------------------------
 */

#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>

namespace m24
{
// ========================================== I2C Interface ==========================================

/**
 * @brief Interface for I2C communication. Derived classes should implement this platform-specific logic.
 */
class I2cInterface
{
public:
    /**
     * @brief Enum representing I2C communication modes.
     */
    enum class Mode
    {
        TX, /**< Transmission mode */
        RX, /**< Reception mode */
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
    virtual void StartPolling(uint8_t device_id, Mode mode, bool set_pos_bit = false) = 0;

    /**
     * @brief Check if the I2C state indicates an error
     * @return true if there is an error, false otherwise.
     */
    virtual bool StateError() = 0;

    /**
     * @brief Reads a single byte from the I2C bus. I2C STOP condition included
     * @return The byte read from the I2C bus.
     */
    virtual uint8_t ReadByte() = 0;

    /**
     * @brief Reads a halfword (16-bit) from the I2C bus. I2C STOP condition included.
     * EepromDriver "WriteHalfWord" method stores data in little-endian format.
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

// ======================================= EepromDriver M24 ========================================

/**
 * @brief Specific memory models in the ST EEPROM M24 series.
 */
enum class MemoryVersion
{
    Kb1,
    Kb2,
    Kb4,
    Kb8,
    Kb16,
    Kb32,
    Kb64,
    Kb128,
    Kb256,
    Kb512,
    Mb1,
    Mb2,
};

/**
 * @brief Status codes for EEPROM operations
 */
enum class Status
{
    OK,                /**< Operation completed successfully */
    I2C_BUS_ERROR,     /**< I2C bus error occurred */
    INVALID_ADDRESS,   /**< Address is out of bounds or invalid */
    INVALID_PARAMETER, /**< Invalid parameter provided */
    NULL_POINTER,      /**< Null pointer passed as parameter */
};

/**
 * @brief STM EEPROM series M24C driver.
 *
 * @tparam M The EEPROM model type from the MemoryVersion enum.
 */
template <MemoryVersion M, uint8_t ChipEnableAddr = 0>
class EepromDriver
{
public:
    static_assert(GetChipEnableBits() && (ChipEnableAddr < (1 << GetChipEnableBits())), "Too many chip enable bits");
    static_assert(!GetChipEnableBits() && (ChipEnableAddr == 0), "This model doesn't support chip enabling addressing");

    static constexpr uint32_t MEMORY_SIZE = (1 << M) * 1024; /**< Total memory size in bytes */
    static constexpr uint16_t PAGE_SIZE   = GetPageSize();   /**< Page size in bytes */

    EepromDriver(I2cInterface &i2c_instance) : _i2c(i2c_instance) {}

    Status WriteByte(uint32_t address, uint8_t value);
    Status WriteHalfWord(uint32_t address, uint16_t value);
    Status WriteBlock(void *data, uint32_t address, uint16_t block_size);
    Status WriteIdPage(uint32_t address, uint8_t value);

    int8_t ReadByte(uint32_t address);
    int16_t ReadHalfWord(uint32_t address);
    Status ReadBlock(void *data, uint32_t address, uint16_t block_size);

    Status ChipErase();
    Status PageErase(uint32_t page_address);

private:
    static constexpr uint8_t DEVICE_SELECT_CODE_MEMORY  = 0b1010'0000 | (ChipEnableAddr << (4 - GetChipEnableBits()));
    static constexpr uint8_t DEVICE_SELECT_CODE_ID_PAGE = 0b0001'0000 | DEVICE_SELECT_CODE_MEMORY;
    static constexpr uint8_t CHIP_ENABLE_ADDR_MASK      = 0b0000'1110 & (0b0000'1110 >> GetChipEnableBits());

    constexpr bool AddressOutOfBounds(uint32_t address) { return address >= MEMORY_SIZE ? true : false; }
    constexpr uint16_t GetPageSize();
    constexpr uint8_t GetChipEnableBits();
    constexpr uint8_t HandleDeviceSelectCode(uint32_t address, bool special_func = false);
    bool WritePage(void *data, uint32_t address, uint8_t data_size);
    void SendAddress(uint32_t address);
    template <typename Operation>
    Status ExecuteWithRetry(Operation &&op, uint8_t max_retries = 3);

    I2cInterface &_i2c;
};

// =================================== EepromDriver Implementation ===================================

/**
 * @brief Writes a byte to the specified address.
 */
template <MemoryVersion M, uint8_t ChipEnableAddr>
Status EepromDriver<M, ChipEnableAddr>::WriteByte(uint32_t address, uint8_t value)
{
    if (AddressOutOfBounds(address))
        return Status::INVALID_ADDRESS;

    uint8_t device_select_code = HandleDeviceSelectCode(address);

    Status result = ExecuteWithRetry([&]() {
        _i2c.StartPolling(device_select_code, I2cInterface::Mode::TX);
        SendAddress(address);
        _i2c.WriteByte(value);
        _i2c.Stop();
    });

    return result;
}

/**
 * @brief Writes a 16-bit halfword to the specified address.
 * @param address The EEPROM address to write to (must be even).
 * @param value The 16-bit value to write.
 */
template <MemoryVersion M, uint8_t ChipEnableAddr>
Status EepromDriver<M, ChipEnableAddr>::WriteHalfWord(uint32_t address, uint16_t value)
{
    if (AddressOutOfBounds(address))
        return Status::INVALID_ADDRESS;

    uint8_t device_select_code = HandleDeviceSelectCode(address);

    Status result = ExecuteWithRetry([&]() {
        _i2c.StartPolling(device_select_code, I2cInterface::Mode::TX);
        SendAddress(address);
        _i2c.WriteByte(static_cast<uint8_t>(value));
        _i2c.WriteByte(static_cast<uint8_t>(value >> 8));
        _i2c.Stop();
    });

    return result;
}

/**
 * @brief Writes a block of data to the EEPROM.
 * @param data Pointer to the data to write.
 * @param address The starting address for the block. Must be a multiple of PAGE_SIZE if the block spans one or more
 * pages.
 * @param data_size The size of the data block.
 */
template <MemoryVersion M, uint8_t ChipEnableAddr>
Status EepromDriver<M, ChipEnableAddr>::WriteBlock(void *data_ptr, uint32_t address, uint16_t data_size)
{
    if (AddressOutOfBounds(address))
        return Status::INVALID_ADDRESS;

    if (!data_ptr)
        return Status::NULL_POINTER;

    uint8_t *data                 = reinterpret_cast<uint8_t *>(data_ptr);
    uint16_t remaining_full_pages = data_size / PAGE_SIZE;

    while (remaining_full_pages >= 1)
    {
        if (!WritePage(data, address, PAGE_SIZE))
            return Status::I2C_BUS_ERROR;

        data += PAGE_SIZE;
        address += PAGE_SIZE;
        remaining_full_pages--;
    }

    return WritePage(data, address, data_size % PAGE_SIZE) ? Status::OK : Status::I2C_BUS_ERROR;
}

/**
 * @brief Reads a byte from the specified address.
 * @param address The EEPROM address to read from.
 * @return The byte value read from the address.
 */
template <MemoryVersion M, uint8_t ChipEnableAddr>
int8_t EepromDriver<M, ChipEnableAddr>::ReadByte(uint32_t address)
{
    if (AddressOutOfBounds(address))
        return Status::INVALID_ADDRESS;

    uint8_t device_select_code = HandleDeviceSelectCode(address);
    int8_t read_value          = 0;

    Status result = ExecuteWithRetry([&]() {
        _i2c.StartPolling(device_select_code, I2cInterface::Mode::TX);
        SendAddress(address);
        _i2c.StartPolling(device_select_code, I2cInterface::Mode::RX);
        read_value = _i2c.ReadByte();
    });

    return result == Status::OK ? read_value : 0;
}

/**
 * @brief Reads a 16-bit halfword from the specified address.
 * @param address The EEPROM address to read from (must be even).
 * @return The 16-bit value read from the address.
 */
template <MemoryVersion M, uint8_t ChipEnableAddr>
int16_t EepromDriver<M, ChipEnableAddr>::ReadHalfWord(uint32_t address)
{
    if (AddressOutOfBounds(address))
        return Status::INVALID_ADDRESS;

    uint8_t device_select_code = HandleDeviceSelectCode(address);
    int16_t read_value         = 0;

    Status result = ExecuteWithRetry([&]() {
        _i2c.StartPolling(device_select_code, I2cInterface::Mode::TX, true);
        SendAddress(address);
        _i2c.StartPolling(device_select_code, I2cInterface::Mode::RX);
        read_value = _i2c.ReadHalfWord();
    });

    return result == Status::OK ? read_value : 0;
}

/**
 * @brief Reads a block of data from the EEPROM.
 * @param data Pointer to the buffer to store the read data.
 * @param address The starting address for the block. Must be a multiple of PAGE_SIZE if the block spans one or more
 * pages.
 * @param data_size The size of the data block.
 */
template <MemoryVersion M, uint8_t ChipEnableAddr>
Status EepromDriver<M, ChipEnableAddr>::ReadBlock(void *data_ptr, uint32_t address, uint16_t data_size)
{
    if (AddressOutOfBounds(address))
        return Status::INVALID_ADDRESS;

    if (!data_ptr)
        return Status::NULL_POINTER;

    uint8_t *data              = reinterpret_cast<uint8_t *>(data_ptr);
    uint8_t device_select_code = HandleDeviceSelectCode(address);

    Status result = ExecuteWithRetry([&]() {
        _i2c.StartPolling(device_select_code, I2cInterface::Mode::TX);
        SendAddress(address);
        _i2c.StartPolling(device_select_code, I2cInterface::Mode::RX);
        _i2c.ReadMultipleBytes(data, data_size);
    });

    return result
}

/**
 * @brief Erases a page by filling it with 0xFF.
 * @param address The start address of the page to erase.
 */
template <MemoryVersion M, uint8_t ChipEnableAddr>
Status EepromDriver<M, ChipEnableAddr>::PageErase(uint32_t page_address)
{
    if (page_address % PAGE_SIZE != 0)
        return Status::INVALID_ADDRESS;

    uint8_t device_select_code = HandleDeviceSelectCode(page_address);

    Status result = ExecuteWithRetry([&]() {
        _i2c.StartPolling(device_select_code, I2cInterface::Mode::TX);
        SendAddress(page_address);

        for (size_t i = 0; i < PAGE_SIZE; i++)
            _i2c.WriteByte(0xFF);

        _i2c.Stop();
    });

    return result;
}

/**
 * @brief Erases the entire EEPROM by filling it with 0xFF.
 */
template <MemoryVersion M, uint8_t ChipEnableAddr>
Status EepromDriver<M, ChipEnableAddr>::ChipErase()
{
    for (size_t i = 0; i < MEMORY_SIZE; i += PAGE_SIZE)
    {
        Status result = PageErase(i);
        if (result != Status::OK)
            return result;
    }

    return Status::OK;
}

/**
 * @brief Writes to identification page (D variants).
 */
template <MemoryVersion M, uint8_t ChipEnableAddr>
Status EepromDriver<M, ChipEnableAddr>::WriteIdPage(uint32_t address_in_page, uint8_t value)
{
    if (address_in_page >= PAGE_SIZE)
        return Status::INVALID_ADDRESS;

    uint8_t device_select_code = DEVICE_SELECT_CODE_ID_PAGE;

    bool success = ExecuteWithRetry([&]() {
        _i2c.StartPolling(device_select_code, I2cInterface::Mode::TX);
        SendAddress(address_in_page);
        _i2c.WriteByte(value);
        _i2c.Stop();
    });

    return success ? Status::OK : Status::I2C_BUS_ERROR;
}

template <MemoryVersion M, uint8_t ChipEnableAddr>
constexpr uint16_t EepromDriver<M, ChipEnableAddr>::GetPageSize()
{
    switch (M)
    {
    default:
        return 16;
    case MemoryVersion::Kb32:
    case MemoryVersion::Kb64:
        return 32;
    case MemoryVersion::Kb128:
    case MemoryVersion::Kb256:
        return 64;
    case MemoryVersion::Kb512:
        return 128;
    case MemoryVersion::Mb1:
    case MemoryVersion::Mb2:
        return 256;
    }
}

template <MemoryVersion M, uint8_t ChipEnableAddr>
constexpr uint8_t EepromDriver<M, ChipEnableAddr>::GetChipEnableBits()
{
    switch (M)
    {
    default:
        return 3;
    case MemoryVersion::Kb4:
    case MemoryVersion::Mb1:
        return 2;
    case MemoryVersion::Kb8:
    case MemoryVersion::Mb2:
        return 1;
    case MemoryVersion::Kb16:
        return 0;
    }
}

/**
 * @brief Generates the device select code based on the EEPROM address.
 */
template <MemoryVersion M, uint8_t ChipEnableAddr>
constexpr uint8_t EepromDriver<M, ChipEnableAddr>::HandleDeviceSelectCode(uint32_t address, bool special_func)
{
    uint8_t device_select_code = DEVICE_SELECT_CODE_MEMORY;

    switch (M)
    {
    case MemoryVersion::Kb4:
    case MemoryVersion::Kb8:
    case MemoryVersion::Kb16:
        device_select_code |= ((address >> 7) & CHIP_ENABLE_ADDR_MASK);
    case MemoryVersion::Mb1:
    case MemoryVersion::Mb2:
        device_select_code |= ((address >> 15) & CHIP_ENABLE_ADDR_MASK);
    default:
    }

    if (special_func)
        device_select_code |= 0b0001'0000;

    return device_select_code;
}

/**
 * @brief Writes a page of data to the EEPROM.
 * @param data Pointer to the data to write.
 * @param address The starting address of the page.
 * @param data_size The size of the data to write.
 */
template <MemoryVersion M, uint8_t ChipEnableAddr>
bool EepromDriver<M, ChipEnableAddr>::WritePage(void *data_ptr, uint32_t address, uint8_t data_size)
{
    uint8_t *data              = reinterpret_cast<uint8_t *>(data_ptr);
    uint8_t device_select_code = HandleDeviceSelectCode(address);

    return ExecuteWithRetry([&]() {
        _i2c.StartPolling(device_select_code, I2cInterface::Mode::TX);
        SendAddress(address);

        for (size_t i = 0; i < data_size; i++)
            _i2c.WriteByte(data[i]);

        _i2c.Stop();
    });
}

template <MemoryVersion M, uint8_t ChipEnableAddr>
void EepromDriver<M, ChipEnableAddr>::SendAddress(uint32_t address)
{
    if constexpr (M >= MemoryVersion::Kb32)
        _i2c.WriteByte(static_cast<uint8_t>(address >> 8));

    // Send low byte for all models
    _i2c.WriteByte(static_cast<uint8_t>(address));
}

/**
 * @brief Executes I2C operations with automatic retry handling.
 * @param op Lambda containing the I2C operation to execute.
 * @param max_retries Maximum number of retry attempts.
 * @return true if operation succeeded, false if all retries exhausted.
 */
template <MemoryVersion M, uint8_t ChipEnableAddr>
template <typename Operation>
Status EepromDriver<M, ChipEnableAddr>::ExecuteWithRetry(Operation &&op, uint8_t max_retries)
{
    uint8_t retries = 0;

    do
    {
        if (_i2c.StateError())
            _i2c.Init();

        op(); // Execute the I2C operation

    } while (_i2c.StateError() && ++retries < max_retries);

    return _i2c.StateError() ? Status::I2C_BUS_ERROR : Status::OK;
}

} // namespace m24
