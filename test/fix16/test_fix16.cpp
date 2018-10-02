#ifdef UNIT_TEST

#include <unity.h>

#include "../src/math/fix16_math.h"


void test_fix16_div() {
  TEST_ASSERT_EQUAL(fix16_div(F16(2000), F16(1000)), F16(2));
}


int main() {
  UNITY_BEGIN();
  RUN_TEST(test_fix16_div);
  UNITY_END();
}


#endif
