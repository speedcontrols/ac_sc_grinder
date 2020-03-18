#ifndef __EEPROM_EMU__
#define __EEPROM_EMU__


#include <stdint.h>

/* Driver
class FlashDriver {
public:
    enum { BankSize = XXXX };
    static void erase(uint8_t bank) {};

    static uint32_t read(uint8_t bank, uint16_t addr);

    // If 32bit write requires multiple operations, those MUST follow from
    // least significant bits to most significant bits.
    static uint32_t write(uint8_t bank, uint16_t addr, uint32_t data);
}
*/

/*
    Bank structure:

    [ 8 bytes bank marker ] [ 8 bytes record ] [ 8 bytes record ]...

    Bank Marker:

    - [ 0x77EE, 0xFFFF,    VERSION, 0xFFFF ] => active, current
    - [ 0x77EE, NOT_EMPTY, VERSION, 0xFFFF ] => ready to erase (!active)
    - [ 0xFFFF, 0xFFFF,    0xFFFF,  0xFFFF ] => erased OR on progress of transfer

    Data record:

    [ 0x55AA, address_16, data_lo_16, data_hi_16 ]

    2-phase commits used to guarantee data consistency. Commit mark (0x55AA) is
    stored separate, because many modern flashes use additional ECC bits and not
    allow old data partial override with zero bits.

    Value 0x55AA at record start means write was completed with success
*/

template <typename FLASH_DRIVER, uint16_t VERSION = 0xCC33>
class EepromEmu
{
    enum {
        EMPTY = 0xFFFF,
        RECORD_SIZE = 8,
        BANK_HEADER_SIZE = 8,
        COMMIT_MARK = 0x55AA,
        BANK_MARK = 0x77EE,
        BANK_DIRTY_MARK = 0x5555
    };

    bool initialized = false;
    uint8_t current_bank = 0;
    uint32_t next_write_offset;

    bool is_clear(uint8_t bank)
    {
        for (uint32_t i = 0; i < FLASH_DRIVER::BankSize; i += 2) {
            if (flash.read_u16(bank, i) != EMPTY) return false;
        }
        return true;
    }

    bool is_active(uint8_t bank)
    {
        if ((flash.read_u16(bank, 0) == BANK_MARK) &&
            (flash.read_u16(bank, 2) == EMPTY) &&
            (flash.read_u16(bank, 4) == VERSION) &&
            (flash.read_u16(bank, 6) == EMPTY)) return true;

        return false;
    }

    uint32_t find_write_offset()
    {
        uint32_t ofs = BANK_HEADER_SIZE;

        for (; ofs <= FLASH_DRIVER::BankSize - RECORD_SIZE; ofs += RECORD_SIZE)
        {
            if ((flash.read_u16(current_bank, ofs + 0) == EMPTY) &&
                (flash.read_u16(current_bank, ofs + 2) == EMPTY) &&
                (flash.read_u16(current_bank, ofs + 4) == EMPTY) &&
                (flash.read_u16(current_bank, ofs + 6) == EMPTY)) break;
        }

        return ofs;
    }

    void move_bank(uint8_t from, uint8_t to, uint16_t ignore_addr=UINT16_MAX)
    {
        if (!is_clear(to)) flash.erase(to);

        uint32_t dst_end_addr = BANK_HEADER_SIZE;

        for (uint32_t ofs = BANK_HEADER_SIZE; ofs < next_write_offset; ofs += RECORD_SIZE)
        {
            // Skip invalid records
            if (flash.read_u16(from, ofs + 0) != COMMIT_MARK) continue;

            uint16_t addr = flash.read_u16(from, ofs + 2);

            // Skip variable with ignored address
            if (addr == ignore_addr) continue;

            uint16_t lo   = flash.read_u16(from, ofs + 4);
            uint16_t hi   = flash.read_u16(from, ofs + 6);

            bool more_fresh_exists = false;

            // Check if more fresh record exists
            for (uint32_t i = ofs + RECORD_SIZE; i < next_write_offset; i += RECORD_SIZE)
            {
                // Skip invalid records
                if (flash.read_u16(from, i + 0) != COMMIT_MARK) continue;

                // Skip different addresses
                if (flash.read_u16(from, i + 2) != addr) continue;

                // More fresh (=> already copied) found
                more_fresh_exists = true;
                break;
            }

            if (more_fresh_exists) continue;

            flash.write_u16(to, dst_end_addr + 2, addr);
            flash.write_u16(to, dst_end_addr + 4, lo);
            flash.write_u16(to, dst_end_addr + 6, hi);
            flash.write_u16(to, dst_end_addr + 0, COMMIT_MARK);
            dst_end_addr += RECORD_SIZE;
        }

        // Mark new bank active
        flash.write_u16(to, 0, BANK_MARK);
        flash.write_u16(to, 4, VERSION);

        current_bank = to;
        next_write_offset = dst_end_addr;

        // Clean old bank in 2 steps to avoid UB: destroy header & run erase
        flash.write_u16(from, 2, BANK_DIRTY_MARK);
        flash.write_u16(from, 6, BANK_DIRTY_MARK);
        flash.erase(from);
    }

    void init()
    {
        initialized = true;

        if (is_active(0)) current_bank = 0;
        else if (is_active(1)) current_bank = 1;
        else
        {
            // Both banks have no valid markers => prepare first one
            if (!is_clear(0)) flash.erase(0);
            flash.write_u16(0, 0, BANK_MARK);
            flash.write_u16(0, 4, VERSION);
            current_bank = 0;
        }

        next_write_offset = find_write_offset();
        return;
    }

public:
    FLASH_DRIVER flash;

    uint32_t read_u32(uint32_t addr, uint32_t dflt)
    {
        if (!initialized) init();

        // Reverse scan, stop on first valid
        for (uint32_t ofs = next_write_offset;;)
        {
            if (ofs <= BANK_HEADER_SIZE) break;

            ofs -= RECORD_SIZE;

            if (flash.read_u16(current_bank, ofs + 0) != COMMIT_MARK) continue;
            if (flash.read_u16(current_bank, ofs + 2) != addr) continue;

            uint16_t lo = flash.read_u16(current_bank, ofs + 4);
            uint16_t hi = flash.read_u16(current_bank, ofs + 6);

            return (hi << 16) + lo;
        }

        return dflt;
    }

    void write_u32(uint32_t addr, uint32_t val)
    {
        if (!initialized) init();

        uint8_t bank = current_bank;

        // Don't write the same value
        uint32_t previous = read_u32(addr, val+1);
        if (previous == val) return;

        // Check free space and swap banks if needed
        if (next_write_offset + RECORD_SIZE > FLASH_DRIVER::BankSize)
        {
            move_bank(bank, bank ^ 1, (uint16_t)addr);
            bank ^= 1;
        }

        // Write data
        flash.write_u16(bank, next_write_offset + 2, (uint16_t)addr);
        flash.write_u16(bank, next_write_offset + 4, val & 0xFFFF);
        flash.write_u16(bank, next_write_offset + 6, (uint16_t)(val >> 16) & 0xFFFF);
        flash.write_u16(bank, next_write_offset + 0, COMMIT_MARK);
        next_write_offset += RECORD_SIZE;
    }

    float read_float(uint32_t addr, float dflt)
    {
        union { uint32_t i; float f; } x;
        x.f = dflt;
        x.i = read_u32(addr, x.i);
        return x.f;
    }

    void write_float(uint32_t addr, float val)
    {
        union { uint32_t i; float f; } x;
        x.f = val;
        write_u32(addr, x.i);
    }
};

#endif
