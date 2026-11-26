"""通用二值掩膜回归：调用生产滤波函数，与独立参考及OpenCV逐像素比较。

不运行目标检测或飞行控制，不把滤波等价性当作实际检出率/帧率验收。
"""
from pathlib import Path
import argparse
import ctypes
import json
import os
import shutil
import subprocess

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
WIDTH, HEIGHT = 320, 240


def fixtures():
    yy, xx = np.indices((HEIGHT, WIDTH))
    zero = np.zeros((HEIGHT, WIDTH), np.uint8)
    one = np.full_like(zero, 255)
    yield 'all_black', zero
    yield 'all_white', one
    for name, data in [('checker', (xx + yy) % 2), ('horizontal_lines', yy % 2),
                       ('vertical_lines', xx % 2), ('diagonal', (xx - yy) % 7 < 3)]:
        yield name, (data * 255).astype(np.uint8)
    for y, x in [(0, 0), (0, WIDTH-1), (HEIGHT-1, 0), (HEIGHT-1, WIDTH-1),
                 (HEIGHT//2, WIDTH//2), (1, 1), (HEIGHT-2, WIDTH-2)]:
        for reverse in (False, True):
            image = one.copy() if reverse else zero.copy()
            image[y, x] = 0 if reverse else 255
            yield 'impulse_%d_%d_%d' % (y, x, reverse), image
    edge = zero.copy()
    edge[:2] = 255; edge[-2:] = 255; edge[:, :2] = 255; edge[:, -2:] = 255
    yield 'border_frame', edge
    rectangle = zero.copy(); rectangle[40:200, 60:260] = 255
    rectangle[110:130, 150:170] = 0
    yield 'rectangle_hole', rectangle
    rng = np.random.default_rng(20261124)
    for density in (.01, .1, .3, .5, .7, .9, .99):
        for repeat in range(4):
            yield 'random_%.2f_%d' % (density, repeat), (rng.random(zero.shape) < density).astype(np.uint8) * 255


def reference(image):
    # 从完整、未改变的输入分别取九个邻居；独立于生产代码的行缓存实现。
    p = np.pad(image != 0, 1, mode='edge')
    count = np.zeros(image.shape, np.uint8)
    for dy in range(3):
        for dx in range(3):
            count += p[dy:dy+HEIGHT, dx:dx+WIDTH]
    return np.where(count >= 5, 255, 0).astype(np.uint8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT/'h750/green_detector.c')
    parser.add_argument('--out', type=Path, default=ROOT/'build/median-fix/validation/median')
    parser.add_argument('--expect-mismatch', action='store_true')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    cc = shutil.which('gcc')
    if not cc:
        raise SystemExit('Host GCC required')
    # 编译前拷到无中文的输出路径；来源保留在报告里，原文件不改。
    source_copy = args.out/'filter_source.c'
    shutil.copy2(args.source, source_copy)
    library = args.out/('median_test.dll' if os.name == 'nt' else 'median_test.so')
    subprocess.run([cc, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
        '-Wno-misleading-indentation', '-shared', '-I'+str(ROOT/'h750'),
        '-DDART_MEDIAN_SOURCE="'+source_copy.resolve().as_posix()+'"',
        str(ROOT/'tests/median_test_bridge.c'), '-o', str(library), '-lm'], check=True)
    module = ctypes.CDLL(str(library.resolve()))
    run = module.dart_test_median
    pointer = ctypes.POINTER(ctypes.c_uint8)
    run.argtypes = [pointer, ctypes.c_uint, pointer, ctypes.c_uint]
    run.restype = ctypes.c_int
    checked = mismatches = guard_failures = 0
    cases = []
    for name, original in fixtures():
        expected = reference(original)
        assert np.array_equal(expected, cv2.medianBlur(original, 3))
        guarded = np.full(original.size + 64, 0xa5, np.uint8)
        actual = guarded[32:-32].reshape(original.shape)
        actual[:] = original
        row_guard = np.full(WIDTH * 3 + 64, 0x5a, np.uint8)
        scratch = row_guard[32:-32]
        assert run(actual.ctypes.data_as(pointer), actual.size,
                   scratch.ctypes.data_as(pointer), scratch.size) == 0
        diff = int(np.count_nonzero(actual != expected))
        guards = (np.all(guarded[:32] == 0xa5) and np.all(guarded[-32:] == 0xa5)
                  and np.all(row_guard[:32] == 0x5a) and np.all(row_guard[-32:] == 0x5a))
        if not guards:
            guard_failures += 1
        checked += original.size; mismatches += diff
        cases.append({'name': name, 'different_pixels': diff, 'guards_pass': bool(guards)})
        # 同一缓存再次调用要得到二次滤波的标准结果，不能依赖上次残留的缓存行。
        expected_second = reference(expected)
        assert run(actual.ctypes.data_as(pointer), actual.size,
                   scratch.ctypes.data_as(pointer), scratch.size) == 0
        diff_second = int(np.count_nonzero(actual != expected_second))
        checked += actual.size; mismatches += diff_second
        cases[-1]['second_call_different_pixels'] = diff_second
    assert guard_failures == 0
    success = mismatches > 0 if args.expect_mismatch else mismatches == 0
    report = {'result': 'PASS' if success else 'FAIL', 'source': str(args.source),
        'scope': 'Binary 3x3 median only; no detector executed', 'fixtures': len(cases),
        'pixels_checked_including_repeated_calls': checked,
        'different_pixels': mismatches, 'guard_failures': guard_failures,
        'reference': 'Independent immutable-input majority + OpenCV medianBlur(3)',
        'expected_negative_control': args.expect_mismatch, 'cases': cases,
        'hardware_test': 'NOT RUN', 'frame_rate_improvement': 'NOT MEASURED'}
    (args.out/'validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('result', 'fixtures',
        'pixels_checked_including_repeated_calls', 'different_pixels', 'guard_failures',
        'expected_negative_control')}, indent=2))
    if not success:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
