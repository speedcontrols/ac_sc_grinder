#ifndef __EEPROM_EMU__
#define __EEPROM_EMU__


// Generic implementation of rom-based variables, saved across power cycles.
// see http://jeelabs.org/2015/04/01/emulating-eeprom/

#include <stdint.h>
#include <string.h>


template < typename FLASH_DRIVER, int BASE >
class RomVars {
  typedef struct { uint16_t key, val; } Tuple;

  enum { Empty = 0xFFFF };
  enum { NumVars = FLASH_DRIVER::PageSize / sizeof (Tuple) };
  enum { BasePage = BASE / FLASH_DRIVER::PageSize };

  class Ref {
    RomVars& owner;
    int index;
  public:
    Ref (RomVars* rv, int i) : owner (*rv), index (i) {}
    operator uint16_t () const { return owner.at(index); }
    uint16_t operator= (uint16_t n) const { return owner.set(index, n); }
  };

public:
  void init () {
    if (!reusePage(BasePage) && !reusePage(BasePage + 1)) {
      fill = NumVars / 2;
      memset(map, 0, sizeof map);
      memset(tuples, 0xFF, sizeof tuples);
    }
  }

  Ref operator[] (int index) {
    return Ref (this, index);
  }

private:
  uint16_t& at (int index) {
    int m = map[index];
    return m == 0 ? values[index] : tuples[m].val;
  }

  uint16_t set (int index, uint16_t newVal) {
    uint16_t& oldVal = at(index);
    if (newVal != oldVal) {
      if (values[0] == Empty) {
        values[0] = BasePage;
        flash.erase(BasePage, 2);
      }

      if (oldVal == Empty) {
        oldVal = newVal;
        flash.save(values[0], tuples);
      } else if (fill < NumVars) {
        tuples[fill].key = index;
        tuples[fill].val = newVal;
        map[index] = fill++;
        flash.save(values[0], tuples);
      } else {
        combineChanges();
        values[index] = newVal;
        flipAndSave();
      }
    }
    return newVal;
  }

  bool reusePage (int page) {
    flash.load(page, tuples);
    if (values[0] != page)
      return false;

    memset(map, 0, sizeof map);
    for (fill = NumVars / 2; fill < NumVars; ++fill) {
      uint16_t key = tuples[fill].key;
      if (key >= NumVars)
        break;
      map[key] = fill;
    }
    return true;
  }

  void combineChanges () {
    for (int i = 0; i < NumVars; ++i)
      if (map[i] != 0)
        values[i] = tuples[map[i]].val;

    fill = NumVars / 2;
    memset(map, 0, sizeof map);
    memset(tuples + fill, 0xFF, sizeof tuples / 2);
  }

  void flipAndSave () {
    uint16_t page = values[0] ^ 1;
    values[0] = Empty;
    flash.save(page, tuples);
    values[0] = page;
    flash.save(page, tuples);
    flash.erase(page ^ 1, 1);
  }

  union {
    uint16_t values [NumVars];
    Tuple tuples [NumVars];
  };
  uint8_t map [NumVars];
  uint16_t fill;
  FLASH_DRIVER flash;
};


#endif
