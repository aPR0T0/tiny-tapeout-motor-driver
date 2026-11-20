/* Host build of the self test: prints the signature the chip must produce. */
#include <stdio.h>
#include "selftest.h"
int main(void)
{
    static volatile uint8_t scratch[8];
    printf("0x%08X\n", selftest(selftest_inputs, SELFTEST_N, scratch));
    return 0;
}
