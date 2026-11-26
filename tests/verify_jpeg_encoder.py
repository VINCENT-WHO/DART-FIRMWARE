"""录像JPEG空间方向回归：只测试编码器，不运行检测、标注或MCU下载。

在Windows主机上编译同一份jpeg_enc.c，生成非对称亮度/色度图案，
解码后比较像素位置。只确认“可解码”不足以检出8x8块被转置的错误。
"""
from pathlib import Path
import argparse
import ctypes
import json
import os
import shutil
import subprocess
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
WIDTH, HEIGHT = 320, 240
CAPACITY = 59392


def transpose_blocks(plane):
    """构造错误方向参考；它不能作为正确解码图的验收基准。"""
    height, width = plane.shape
    return plane.reshape(height // 8, 8, width // 8, 8).transpose(0, 3, 2, 1).reshape(height, width)


def to_bgr(y, cb, cr):
    # JPEG使用全范围YCbCr；不要误用有限范围的YUYV到RGB转换来验收。
    cb = np.repeat(np.repeat(cb.astype(float) - 128, 2, axis=0), 2, axis=1)
    cr = np.repeat(np.repeat(cr.astype(float) - 128, 2, axis=0), 2, axis=1)
    y = y.astype(float)
    return np.clip(np.stack((y + 1.772 * cb,
                            y - 0.344136 * cb - 0.714136 * cr,
                            y + 1.402 * cr), axis=2), 0, 255).round().astype(np.uint8)


def image_fixtures():
    yy, xx = np.indices((HEIGHT, WIDTH))
    neutral = np.full((HEIGHT // 2, WIDTH // 2), 128, np.uint8)
    fixtures = []
    fixtures.append(('uniform', np.full((HEIGHT, WIDTH), 150, np.uint8), neutral, neutral, False))
    # 每个块内仅一个方向变化，横纵写反就会有很大的像素误差。
    fixtures.append(('horizontal_ramp', (35 + 25 * (xx % 8)).astype(np.uint8), neutral, neutral, True))
    fixtures.append(('vertical_ramp', (35 + 25 * (yy % 8)).astype(np.uint8), neutral, neutral, True))
    text = np.full((HEIGHT, WIDTH), 180, np.uint8)
    cv2.putText(text, 'JPEG TEST 123', (12, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.85, 10, 2, cv2.LINE_AA)
    cv2.line(text, (10, 70), (290, 180), 235, 4)
    for x in range(20, 300, 16):
        cv2.rectangle(text, (x, 95), (x + 4, 210), 45, -1)
    fixtures.append(('asymmetric_text', text, neutral, neutral, True))
    cy, cx = np.indices(neutral.shape)
    cb = (90 + 10 * (cx % 8)).astype(np.uint8)
    cr = (90 + 10 * (cy % 8)).astype(np.uint8)
    fixtures.append(('asymmetric_chroma', np.full((HEIGHT, WIDTH), 128, np.uint8), cb, cr, True))
    return fixtures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--encoder', type=Path, default=ROOT / 'h750/jpeg_enc.c')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if os.name != 'nt':
        raise RuntimeError('This host runner currently uses a MinGW Windows DLL')
    if args.out.exists() and any(args.out.iterdir()):
        raise RuntimeError('Test output directory must be new or empty')
    args.out.mkdir(parents=True, exist_ok=True)
    compiler = shutil.which('gcc')
    if not compiler:
        raise RuntimeError('Host GCC is required; do not use ARM GCC')
    library = args.out.resolve() / 'jpeg_test.dll'
    subprocess.run([compiler, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-shared', '-I' + str(ROOT / 'h750'), str(args.encoder), '-o', str(library)], check=True)
    module = ctypes.CDLL(str(library))
    encode = module.jpeg_encode_yuv422
    pointer = ctypes.POINTER(ctypes.c_uint8)
    encode.argtypes = [pointer, ctypes.c_int, pointer, ctypes.POINTER(ctypes.c_uint32)]
    encode.restype = ctypes.c_int
    results, failures, previews = {}, [], []
    for name, y, cb, cr, directional in image_fixtures():
        yuyv = np.empty((HEIGHT, WIDTH, 2), np.uint8)
        yuyv[:, :, 0] = y
        yuyv[:, 0::2, 1] = np.repeat(cb, 2, axis=0)
        yuyv[:, 1::2, 1] = np.repeat(cr, 2, axis=0)
        output = np.empty(CAPACITY, np.uint8)
        count = ctypes.c_uint32(CAPACITY)
        status = encode(yuyv.ctypes.data_as(pointer), 75, output.ctypes.data_as(pointer), ctypes.byref(count))
        if status or not 0 < count.value <= CAPACITY:
            raise RuntimeError(f'{name}: encoder status/size invalid: {status}/{count.value}')
        encoded = output[:count.value].copy()
        (args.out / (name + '.jpg')).write_bytes(encoded.tobytes())
        decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if decoded is None or decoded.shape != (HEIGHT, WIDTH, 3):
            raise RuntimeError(f'{name}: JPEG did not decode to the expected size')
        expected = to_bgr(y, cb, cr)
        wrong = to_bgr(transpose_blocks(y), transpose_blocks(cb), transpose_blocks(cr))
        error = float(np.abs(decoded.astype(float) - expected).mean())
        wrong_error = float(np.abs(decoded.astype(float) - wrong).mean())
        passed = error < 8 and (not directional or error < wrong_error * 0.45)
        results[name] = {'jpeg_bytes': count.value, 'mae_to_correct_pixels': error,
                         'mae_to_transposed_pixels': wrong_error, 'passed': passed}
        if not passed:
            failures.append(name)
        if name == 'asymmetric_text':
            previews = [expected, decoded]
    # 容量边界必须报错，且不能写到调用方缓冲外。
    backing = np.full(96, 0xA5, np.uint8)
    short_count = ctypes.c_uint32(32)
    status = encode(yuyv.ctypes.data_as(pointer), 75, backing.ctypes.data_as(pointer), ctypes.byref(short_count))
    if status != -2 or not np.all(backing[32:] == 0xA5):
        failures.append('small_buffer_guard')
    if previews:
        image = np.hstack(previews)
        ok, png = cv2.imencode('.png', image)
        if not ok:
            raise RuntimeError('Could not encode test preview')
        (args.out / 'source_vs_decoded.png').write_bytes(png.tobytes())
    report = {'result': 'FAIL' if failures else 'PASS', 'fixtures': results,
              'small_buffer_guard': 'FAIL' if 'small_buffer_guard' in failures else 'PASS',
              'failures': failures, 'hardware': 'NOT RUN'}
    (args.out / 'validation.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
