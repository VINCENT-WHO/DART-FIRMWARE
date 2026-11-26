/* 无第三方库的滤波回归：不可变输入九邻居参考、重复调用和缓存哨兵。 */
#include "green_detector.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int dart_test_median(uint8_t *mask, unsigned count, uint8_t *scratch, unsigned scratch_size);
static uint8_t original[DART_DET_PIXELS], expected[DART_DET_PIXELS];
static uint8_t guarded[DART_DET_PIXELS + 64], rows[3 * DART_DET_WIDTH + 64];
static uint32_t seed = 20261126;
static uint32_t random32(void)
{
    seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
    return seed;
}

static void reference(void)
{
    for (int y = 0; y < DART_DET_HEIGHT; ++y)
        for (int x = 0; x < DART_DET_WIDTH; ++x) {
            unsigned ones = 0;
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    int yy = y + dy, xx = x + dx;
                    if (yy < 0) yy = 0;
                    if (yy >= DART_DET_HEIGHT) yy = DART_DET_HEIGHT - 1;
                    if (xx < 0) xx = 0;
                    if (xx >= DART_DET_WIDTH) xx = DART_DET_WIDTH - 1;
                    ones += original[yy * DART_DET_WIDTH + xx] != 0;
                }
            expected[y * DART_DET_WIDTH + x] = ones >= 5 ? 255 : 0;
        }
}

int main(void)
{
    const unsigned cases = 50;
    for (unsigned pattern = 0; pattern < cases; ++pattern) {
        for (int y = 0; y < DART_DET_HEIGHT; ++y)
            for (int x = 0; x < DART_DET_WIDTH; ++x) {
                int on;
                switch (pattern) {
                case 0: on = 0; break;
                case 1: on = 1; break;
                case 2: on = (x + y) & 1; break;
                case 3: on = x & 1; break;
                case 4: on = y & 1; break;
                case 5: on = x < 2 || y < 2 || x >= DART_DET_WIDTH - 2 || y >= DART_DET_HEIGHT - 2; break;
                case 6: on = x > 20 && x < 100 && y > 40 && y < 200; break;
                case 7: on = (x == 0 || x == DART_DET_WIDTH - 1) && (y == 0 || y == DART_DET_HEIGHT - 1); break;
                default: on = random32() % 101 < (pattern - 8) * 100 / (cases - 9); break;
                }
                original[y * DART_DET_WIDTH + x] = on ? 255 : 0;
            }
        memset(guarded, 0xa5, sizeof(guarded));
        memset(rows, 0x5a, sizeof(rows));
        memcpy(guarded + 32, original, sizeof(original));
        for (int repeat = 0; repeat < 2; ++repeat) {
            reference();
            assert(dart_test_median(guarded + 32, DART_DET_PIXELS,
                                   rows + 32, 3 * DART_DET_WIDTH) == 0);
            assert(!memcmp(guarded + 32, expected, sizeof(expected)));
            for (unsigned i = 0; i < 32; ++i) {
                assert(guarded[i] == 0xa5 && guarded[sizeof(guarded) - 1 - i] == 0xa5);
                assert(rows[i] == 0x5a && rows[sizeof(rows) - 1 - i] == 0x5a);
            }
            memcpy(original, expected, sizeof(original));
        }
    }
    printf("PASS: median %u fixtures, %u pixel comparisons, repeated calls and guards\n",
           cases, cases * 2 * DART_DET_PIXELS);
    return 0;
}
