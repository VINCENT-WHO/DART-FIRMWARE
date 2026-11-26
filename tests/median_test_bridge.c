/* 仅主机测试编译此桥接，直接调用生产文件中的static滤波函数。
 * 不向板端增加测试命令或导出接口，也不运行检测/评分逻辑。 */
#ifndef DART_MEDIAN_SOURCE
#define DART_MEDIAN_SOURCE "../h750/green_detector.c"
#endif
#include DART_MEDIAN_SOURCE

#if defined(_WIN32)
#define TEST_EXPORT __declspec(dllexport)
#else
#define TEST_EXPORT
#endif

TEST_EXPORT int dart_test_median(uint8_t *mask, unsigned count,
                                  uint8_t *scratch, unsigned scratch_size)
{
    if (!mask || !scratch || count != W * H || scratch_size != 3 * W) return -1;
    median3(mask, (uint8_t (*)[W])scratch);
    return 0;
}
