#ifndef SELFTEST_H
#define SELFTEST_H
#include <stdint.h>
#define SELFTEST_N 12
extern const uint32_t selftest_inputs[SELFTEST_N];
uint32_t selftest(const volatile uint32_t *in, int n, volatile uint8_t *scratch);
#endif
