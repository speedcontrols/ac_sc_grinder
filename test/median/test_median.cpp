#ifdef UNIT_TEST

#include <unity.h>

#include "../src/math/fix16_math.h"
#include "../src/math/median.h"

// samples from `/doc/data`
fix16_t data[] = {
    F16(0.4754093567),
    F16(0.3471043578),
    F16(0.0465602837),
    F16(0.172412614),
    F16(0.2909671788),
    F16(0.3985208895),
    F16(0.2898536896),
    F16(0.3071515892),
    F16(0.3279415274),
    F16(0.2148582766),
    F16(0.1171775899),
    F16(0.3562473233),
    F16(0.2116588542),
    F16(-0.0394103774),
    F16(-0.0007017544),
    F16(-0.1201406498),
    F16(-0.0312891207),
    F16(0.127527819),
    F16(0.1715315315),
    F16(0.3352715655),
    F16(0.4044117647),
    F16(0.2655273788),
    F16(0.188442623),
    F16(0.1345081967),
    F16(0.1654635609),
    F16(0.219397619),
    F16(0.0919010417),
    F16(0.0854198113),
    F16(-0.037311706),
    F16(0.0315125673),
    F16(0.2712424395),
    F16(0.2084039548),
    F16(0.4149436937),
    F16(0.5023453608),
    F16(0.3418415493),
    F16(0.2317910448),
    F16(0.1164992331),
    F16(0.1329140127),
    F16(0.2571724138),
    F16(0.1439221014),
    F16(0.1501388889),
    F16(0.0294411765),
    F16(0.144832636)
};

const int data_len = sizeof(data)/sizeof(data[0]);


void test_median_0_el() {
    MedianIteratorTemplate<fix16_t, 64> m;
    // 0 entries
    TEST_ASSERT_EQUAL(m.result(), 0);
}


void test_median_1_el() {
    MedianIteratorTemplate<fix16_t, 64> m;
    int i = 0;
    // 1 entry
    m.add(data[i++]);
    TEST_ASSERT_EQUAL(m.result(), data[0]);
}


void test_median_2_el() {
    MedianIteratorTemplate<fix16_t, 64> m;
    int i = 0;
    // 2 entries
    m.add(data[i++]);
    m.add(data[i++]);
    TEST_ASSERT_EQUAL(m.result(), (data[0] + data[1]) / 2);
}


void test_median_3_el() {
    MedianIteratorTemplate<fix16_t, 64> m;
    int i = 0;
    // 3 entries
    m.add(data[i++]);
    m.add(data[i++]);
    m.add(data[i++]);
    TEST_ASSERT_EQUAL(m.result(), data[1]);
}


void test_median_4_el() {
    MedianIteratorTemplate<fix16_t, 64> m;
    int i = 0;
    // 4 entries
    m.add(data[i++]);
    m.add(data[i++]);
    m.add(data[i++]);
    m.add(data[i++]);
    TEST_ASSERT_EQUAL(m.result(), (data[1] + data[3]) / 2);
}


void test_median_5_el() {
    MedianIteratorTemplate<fix16_t, 64> m;
    int i = 0;
    // 5 entries
    m.add(data[i++]);
    m.add(data[i++]);
    m.add(data[i++]);
    m.add(data[i++]);
    m.add(data[i++]);
    TEST_ASSERT_EQUAL(m.result(), data[4]);
}


void test_median_overflow() {
    MedianIteratorTemplate<fix16_t, 4> m;
    int i = 0;
    // 4 entries
    m.add(data[i++]);
    m.add(data[i++]);
    m.add(data[i++]);
    m.add(data[i++]);
    // Overflow, should be ignored
    m.add(data[i++]);
    m.add(data[i++]);
    TEST_ASSERT_EQUAL(m.result(), (data[1] + data[3]) / 2);
}


void test_median_64() {
    MedianIteratorTemplate<fix16_t, 64> m;

    for (int i = 0; i < data_len; i++) m.add(data[i]);

    TEST_ASSERT_EQUAL(m.result(), F16(0.188446));
}


int main() {
    UNITY_BEGIN();
    RUN_TEST(test_median_0_el);
    RUN_TEST(test_median_1_el);
    RUN_TEST(test_median_2_el);
    RUN_TEST(test_median_3_el);
    RUN_TEST(test_median_4_el);
    RUN_TEST(test_median_5_el);
    RUN_TEST(test_median_overflow);
    RUN_TEST(test_median_64);
    UNITY_END();
}


#endif
