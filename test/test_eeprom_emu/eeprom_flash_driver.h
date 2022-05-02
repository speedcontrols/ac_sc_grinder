#ifndef __EEPROM_FLASH_DRIVER__
#define __EEPROM_FLASH_DRIVER__

#define EEPROM_EMU_BANK_SIZE 1024

#include <stdint.h>

class EepromFlashDriver
{
public:
    EepromFlashDriver()
    {
        for (uint32_t i = 0; i < BankSize*2; i++) memory[i] = 0xFF;
    }

    static const uint32_t BankSize = EEPROM_EMU_BANK_SIZE;

    uint8_t memory[BankSize*2];

    void erase(uint8_t bank)
    {
        for (uint32_t i = 0; i < BankSize; i++) memory[bank*BankSize + i] = 0xFF;
    }

    uint16_t read_u16(uint8_t bank, uint32_t addr)
    {
        uint32_t ofs = bank*BankSize + addr;

        return uint16_t(memory[ofs] + (memory[ofs+1] << 8));
    }

    void write_u16(uint8_t bank, uint32_t addr, uint16_t data)
    {
        uint32_t ofs = bank*BankSize + addr;

        memory[ofs] = (uint8_t)data & 0xFF;
        memory[ofs+1] = (uint8_t)(data >> 8) & 0xFF;
    }
};

#endif
