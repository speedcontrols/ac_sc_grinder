#ifndef __EEPROM_FLOAT__
#define __EEPROM_FLOAT__

/*
  Wrapper to read/write float numbers to emulated eeprom.
  All ops are translated to uint16_t read/write.

  Note 1. eeprom_float_init() must be called before start.
  Note 2. Address must be > 0 and < 64 for 1K flash sector size.
*/


#include "eeprom_emu.h"

#include "stm32f1xx_hal.h"


class FlashDriver {
public:
  enum { PageSize = FLASH_PAGE_SIZE };

  static void load(int page, void* ptr) {
    memcpy(ptr, (void*) (FLASH_BASE + page * PageSize), PageSize);
  }

  static void erase(int page, int num) {
    FLASH_EraseInitTypeDef s_eraseinit;
    s_eraseinit.TypeErase   = FLASH_TYPEERASE_PAGES;
    s_eraseinit.PageAddress = FLASH_BASE + page * PageSize;
    s_eraseinit.NbPages     = num;
    uint32_t page_error = 0;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&s_eraseinit, &page_error);
    HAL_FLASH_Lock();
  }

  static void save(int page, const void* ptr) {
    uint16_t *p_src = (uint16_t *)ptr;
    uint16_t *p_dst = (uint16_t *)(FLASH_BASE + page * PageSize);

    HAL_FLASH_Unlock();

    for (int i = 0; i < PageSize / 2; i++)
    {
      if (p_src[i] == p_dst[i]) continue;
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_BASE + page * PageSize + i * 2, p_src[i]);
    }

    HAL_FLASH_Lock();
  }
};


// stm32f103c8t6 - 64 of 1k pages total, 0..63, use 2 last ones.
RomVars<FlashDriver, (64 - 1 - 2) * FLASH_PAGE_SIZE> eeprom;

// Each float has takes 3 uint16_t records:
//
// 0 - successeful commit mark
// 1 - high bits
// 2 - low bits
//
// That's needed to detect uninitialized memory, when data not exists at all.
// On second override mark is not changed.

#define EEPROM_COMMIT_MARK 0x12EF

float eeprom_float_read(int addr, float default_value)
{
  if (eeprom[addr * 3] != EEPROM_COMMIT_MARK) return default_value;

  uint32_t hi = eeprom[addr * 3 + 1];
  uint32_t lo = eeprom[addr * 3 + 2];

  union { uint32_t i; float f; } x;

  x.i = (hi << 16) | lo;

  return x.f;
}

void eeprom_float_write(int addr, float val)
{
  union { uint32_t i; float f; } x;

  x.f = val;
  uint32_t data = x.i;

  eeprom[addr * 3 + 1] = (uint16_t)(data >> 16);
  eeprom[addr * 3 + 2] = data & 0xFFFF;
  eeprom[addr * 3] = EEPROM_COMMIT_MARK;
}

void eeprom_float_init()
{
  eeprom.init();
}


#endif
