#ifdef UNIT_TEST

#include <unity.h>

#include "eeprom_emu.h"
#include "eeprom_flash_driver.h"


#include <stdio.h>
#include <string.h>

/*void mem_dump(EepromEmu<EepromFlashDriver> &eeprom)
{
    for (uint32_t i = 0; i < eeprom.flash.BankSize; i++)
    {
        printf("0x%.2X ", eeprom.flash.memory[i]);
    }

    printf("\n\n");

    for (uint32_t i = eeprom.flash.BankSize; i < eeprom.flash.BankSize*2; i++)
    {
        printf("0x%.2X ", eeprom.flash.memory[i]);
    }

    printf("\n\n");
}*/


void test_eeprom_write() {
    EepromEmu<EepromFlashDriver, 0x4499> eeprom;

    eeprom.write_u32(3, 0x0000AA99);
    // bank Marker + address + value
    uint8_t expected[] = {
        0xEE, 0x77, 0xFF, 0xFF, 0x99, 0x44, 0xFF, 0xFF, // Bank header
        0xAA, 0x55,                 // commit mark
        0x03, 0x00,                 // addr
        0x99, 0xAA, 0x00, 0x00,     // data
        0xFF // free space start
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, eeprom.flash.memory, sizeof(expected));
}

void test_eeprom_write_over() {
    EepromEmu<EepromFlashDriver, 0x4499> eeprom;

    eeprom.write_u32(3, 0x0000AA99);
    eeprom.write_u32(3, 0x5577CCEE);

    uint8_t expected[] = {
        0xEE, 0x77, 0xFF, 0xFF, 0x99, 0x44, 0xFF, 0xFF, // Bank header
        0xAA, 0x55,                 // commit mark
        0x03, 0x00,                 // addr
        0x99, 0xAA, 0x00, 0x00,     // data
        0xAA, 0x55,                 // commit mark
        0x03, 0x00,                 // addr
        0xEE, 0xCC, 0x77, 0x55,     // data
        0xFF // free space start
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, eeprom.flash.memory, sizeof(expected));
}

void test_eeprom_write_skip_the_same() {
    EepromEmu<EepromFlashDriver, 0x4499> eeprom;

    eeprom.write_u32(3, 0x0000AA99);
    eeprom.write_u32(3, 0x0000AA99);
    eeprom.write_u32(3, 0x00000000);

    // Only 2 records should exist (1 old + 1 new)
    uint8_t expected[] = {
        0xEE, 0x77, 0xFF, 0xFF, 0x99, 0x44, 0xFF, 0xFF, // Bank header
        0xAA, 0x55,                 // commit mark
        0x03, 0x00,                 // addr
        0x99, 0xAA, 0x00, 0x00,     // data
        0xAA, 0x55,                 // commit mark
        0x03, 0x00,                 // addr
        0x00, 0x00, 0x00, 0x00,     // data
        0xFF // free space start
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, eeprom.flash.memory, sizeof(expected));
}

void test_eeprom_read() {
    EepromEmu<EepromFlashDriver> eeprom;

    eeprom.write_u32(3, 0x0000AA99);
    TEST_ASSERT_EQUAL_HEX32(0x0000AA99, eeprom.read_u32(3, 0));
    eeprom.write_u32(3, 0x5577CCEE);
    TEST_ASSERT_EQUAL_HEX32(0x5577CCEE, eeprom.read_u32(3, 0));
}

void test_eeprom_version_match() {
    EepromEmu<EepromFlashDriver, 0x4499> eeprom;

    // bank Marker + address + value
    uint8_t raw_content[] = {
        0xEE, 0x77, 0xFF, 0xFF, 0x99, 0x44, 0xFF, 0xFF, // Bank header
        0xAA, 0x55,                 // commit mark
        0x03, 0x00,                 // addr
        0x99, 0xAA, 0x00, 0x00,     // data
        0xFF // free space start
    };
    memcpy(eeprom.flash.memory, raw_content, sizeof(raw_content));

    // Correct version tag should allow data use
    TEST_ASSERT_EQUAL_HEX32(0x0000AA99, eeprom.read_u32(3, 0));

    EepromEmu<EepromFlashDriver, 0x6633> eeprom2;
    memcpy(eeprom2.flash.memory, raw_content, sizeof(raw_content));

    // Wrong version should invalidate data
    TEST_ASSERT_EQUAL_HEX32(0x00000000, eeprom2.read_u32(3, 0));
    // Make sure eeprom operations are ok
    eeprom2.write_u32(3, 0x0000AA99);
    TEST_ASSERT_EQUAL_HEX32(0x0000AA99, eeprom2.read_u32(3, 0));

}


void test_eeprom_bank_move() {
    EepromEmu<EepromFlashDriver, 0x4499> eeprom;

    uint8_t empty[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    // Fill bank to cause data move to next one
    uint16_t capacity = (eeprom.flash.BankSize - 8) / 8;


    // Prepare first record
    eeprom.write_u32(7, 0x00000000);
    capacity--;

    // Occupy all bank memory with second record
    uint32_t data = 0x7777CCCC;
    for (int i = 0; i < capacity; i++)
    {
        eeprom.write_u32(3, data);
        data ^= 0xFFFFFFFF;
    }

    // Overflow
    eeprom.write_u32(3, 0x0000AA99);

    // Old bank should become clean
    TEST_ASSERT_EQUAL_HEX8_ARRAY(empty, eeprom.flash.memory, sizeof(empty));

    // New bank should contain only 2 records
    uint8_t expected[] = {
        0xEE, 0x77, 0xFF, 0xFF, 0x99, 0x44, 0xFF, 0xFF, // Bank header
        0xAA, 0x55,                 // commit mark
        0x07, 0x00,                 // addr
        0x00, 0x00, 0x00, 0x00,     // data
        0xAA, 0x55,                 // commit mark
        0x03, 0x00,                 // addr
        0x99, 0xAA, 0x00, 0x00,     // data
        0xFF // free space start
    };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, eeprom.flash.memory + eeprom.flash.BankSize, sizeof(expected));
}

void test_eeprom_float() {
    EepromEmu<EepromFlashDriver> eeprom;

    eeprom.write_float(3, 0.15F);
    TEST_ASSERT_EQUAL_HEX32(0.15F, eeprom.read_float(3, 0.0F));
    eeprom.write_float(3, 0.44F);
    TEST_ASSERT_EQUAL_HEX32(0.44F, eeprom.read_float(3, 0.0F));
}



int main() {
    UNITY_BEGIN();
    RUN_TEST(test_eeprom_write);
    RUN_TEST(test_eeprom_write_over);
    RUN_TEST(test_eeprom_write_skip_the_same);
    RUN_TEST(test_eeprom_read);
    RUN_TEST(test_eeprom_bank_move);
    RUN_TEST(test_eeprom_float);
    RUN_TEST(test_eeprom_version_match);
    return UNITY_END();
}

#endif
