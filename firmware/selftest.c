/* ISA self test.  Built for the chip (RV32E) and for the host PC; both must
   produce the same signature.  Inputs come through a volatile pointer so the
   compiler cannot fold anything. */
#include <stdint.h>
#include "selftest.h"

static uint32_t mix(uint32_t h, uint32_t v)
{
    h ^= v;
    h = (h << 5) | (h >> 27);
    return (h + 0x7F4A7C15u) ^ (h >> 11);
}

__attribute__((noinline)) static int32_t clamp_s(int32_t v, int32_t lo, int32_t hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static const int8_t   tab8[8]  = {-128, -1, 0, 1, 77, -77, 127, 42};
static const int16_t  tab16[6] = {-32768, -12345, -1, 0, 12345, 32767};

uint32_t selftest(const volatile uint32_t *in, int n, volatile uint8_t *scratch)
{
    uint32_t h = 0x12345678u;
    for (int i = 0; i < n; i++) {
        uint32_t a = in[i];
        uint32_t b = in[(i + 1) % n];
        int32_t sa = (int32_t)a, sb = (int32_t)b;
        h = mix(h, a + b);
        h = mix(h, a - b);
        h = mix(h, a ^ b);
        h = mix(h, a | (b >> 3));
        h = mix(h, a & ~b);
        h = mix(h, a << (b & 31));
        h = mix(h, a >> (b & 31));
        h = mix(h, (uint32_t)(sa >> (b & 31)));
        h = mix(h, (uint32_t)(sa < sb) | ((uint32_t)(a < b) << 1));
        h = mix(h, (uint32_t)clamp_s(sa, -100000, 100000));
        h = mix(h, a * b);                  /* __mulsi3 */
        if (b != 0) {
            h = mix(h, a / b);
            h = mix(h, a % b);
            if (!(sa == INT32_MIN && sb == -1)) {
                h = mix(h, (uint32_t)(sa / sb));
                h = mix(h, (uint32_t)(sa % sb));
            }
        }
        /* byte / halfword stores and sign / zero extending loads */
        scratch[i & 7] = (uint8_t)a;
        ((volatile uint16_t *)scratch)[(i >> 1) & 3] ^= (uint16_t)(b >> 7);
        h = mix(h, (uint32_t)(int32_t)(int8_t)scratch[(i + 3) & 7]);
        h = mix(h, scratch[(i + 5) & 7]);
        h = mix(h, (uint32_t)(int32_t)((volatile int16_t *)scratch)[i & 3]);
        h = mix(h, ((volatile uint16_t *)scratch)[(i + 1) & 3]);
        h = mix(h, (uint32_t)(int32_t)tab8[i & 7]);
        h = mix(h, (uint32_t)(int32_t)tab16[i % 6]);
    }
    return h;
}

const uint32_t selftest_inputs[SELFTEST_N] = {
    0x00000000u, 0xFFFFFFFFu, 0x80000000u, 0x7FFFFFFFu, 0x00000001u, 0x12345678u,
    0xDEADBEEFu, 0x0000001Fu, 0x00000020u, 0xFFFF8000u, 0x00008000u, 0xA5A5A5A5u,
};
