"""验证硬件JPEG输入重排、输出边界和适配层回调；不声称模拟了JPEG外设。"""
from pathlib import Path
import ctypes
import json
import os
import shutil
import subprocess

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'build/hw-jpeg/validation'
OUT.mkdir(parents=True, exist_ok=True)
CC = shutil.which('gcc')
if not CC:
    raise SystemExit('Host GCC required')
flags = [CC, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror']
mock = OUT / ('test_jpeg_callbacks.exe' if os.name == 'nt' else 'test_jpeg_callbacks')
subprocess.run(flags + ['-I' + str(ROOT / 'tests/jpeg_hal_mock'), '-I' + str(ROOT / 'h750'),
    str(ROOT / 'tests/test_jpeg_hw_callbacks.c'), str(ROOT / 'h750/jpeg_hw.c'),
    str(ROOT / 'h750/jpeg_mcu.c'), '-o', str(mock)], check=True)
subprocess.run([str(mock)], check=True)

pointer = ctypes.POINTER(ctypes.c_uint8)
rng = np.random.default_rng(20261124)
frame = rng.integers(0, 256, (240, 320, 2), dtype=np.uint8)
original = frame.copy()
results = {}
for swap in (0, 1):
    library = OUT / ('mcu_swap%d.dll' % swap)
    subprocess.run(flags + ['-shared', '-DDART_DET_C_UV_SWAP=%d' % swap,
        str(ROOT / 'h750/jpeg_mcu.c'), '-o', str(library)], check=True)
    module = ctypes.CDLL(str(library))
    pack = module.jpeg_pack_mcu_strip
    pack.argtypes = [pointer, ctypes.c_uint32, pointer, ctypes.c_uint32]
    pack.restype = ctypes.c_int
    y = frame[:, :, 0]
    cb = frame[:, (1 if swap else 0)::2, 1].astype(np.uint16)
    cr = frame[:, (0 if swap else 1)::2, 1].astype(np.uint16)
    cb = ((cb[::2] + cb[1::2]) // 2).astype(np.uint8)
    cr = ((cr[::2] + cr[1::2]) // 2).astype(np.uint8)
    reconstructed_y = np.empty_like(y)
    reconstructed_cb = np.empty_like(cb)
    reconstructed_cr = np.empty_like(cr)
    expected_strips = []
    for strip in range(15):
        guarded = np.full(7680 + 64, 0xa5, dtype=np.uint8)
        data = guarded[32:-32]
        assert pack(frame.ctypes.data_as(pointer), strip, data.ctypes.data_as(pointer), data.size) == 0
        expected_strips.append(data.copy())
        assert np.all(guarded[:32] == 0xa5) and np.all(guarded[-32:] == 0xa5)
        for mx, mcu in enumerate(data.reshape(20, 384)):
            # 独立按空间坐标重建，再逐像素与原图比较，检出块序、转置、UV和舍入错误。
            for block in range(4):
                yy = strip * 16 + (block // 2) * 8
                xx = mx * 16 + (block % 2) * 8
                reconstructed_y[yy:yy+8, xx:xx+8] = mcu[block*64:(block+1)*64].reshape(8, 8)
            reconstructed_cb[strip*8:strip*8+8, mx*8:mx*8+8] = mcu[256:320].reshape(8, 8)
            reconstructed_cr[strip*8:strip*8+8, mx*8:mx*8+8] = mcu[320:384].reshape(8, 8)
    assert np.array_equal(reconstructed_y, y)
    assert np.array_equal(reconstructed_cb, cb) and np.array_equal(reconstructed_cr, cr)
    assert np.array_equal(frame, original)
    # 32位优化不得依赖输入/输出地址对齐；完整条带需与独立空间参考一致。
    for input_offset in range(4):
        input_guard = np.full(frame.size + 68, 0x53, np.uint8)
        unaligned = input_guard[32+input_offset:32+input_offset+frame.size]
        unaligned[:] = frame.reshape(-1)
        snapshot = input_guard.copy()
        for output_offset in range(4):
            for strip in range(15):
                output_guard = np.full(7680 + 68, 0xa5, np.uint8)
                data = output_guard[32+output_offset:32+output_offset+7680]
                assert pack(unaligned.ctypes.data_as(pointer), strip, data.ctypes.data_as(pointer), data.size) == 0
                assert np.array_equal(data, expected_strips[strip])
                assert np.all(output_guard[:32+output_offset] == 0xa5)
                assert np.all(output_guard[32+output_offset+7680:] == 0xa5)
        assert np.array_equal(input_guard, snapshot)
    # 相邻行色度0/255：验证双通道求和不串位且平均向下取整为127。
    edge = np.full_like(frame, 255)
    edge[::2, :, 1] = 0
    data = np.empty(7680, np.uint8)
    assert pack(edge.ctypes.data_as(pointer), 0, data.ctypes.data_as(pointer), data.size) == 0
    blocks = data.reshape(20,384)
    assert np.all(blocks[:,:256] == 255) and np.all(blocks[:,256:] == 127)
    rejected = np.full(7680, 0x7b, dtype=np.uint8)
    assert pack(frame.ctypes.data_as(pointer), 15, rejected.ctypes.data_as(pointer), rejected.size) == -1
    assert pack(frame.ctypes.data_as(pointer), 0, rejected.ctypes.data_as(pointer), rejected.size - 1) == -1
    assert np.all(rejected == 0x7b)
    results[str(swap)] = 'PASS: full-frame Y/Cb/Cr, all 16 alignment pairs, chroma overflow/rounding, guards, immutable input'

    class Stream(ctypes.Structure):
        _fields_ = [('data', pointer), ('capacity', ctypes.c_uint32),
                    ('used', ctypes.c_uint32), ('overflow', ctypes.c_uint32)]
    begin = module.jpeg_stream_begin
    begin.argtypes = [ctypes.POINTER(Stream), pointer, ctypes.c_uint32]
    append = module.jpeg_stream_append
    append.argtypes = [ctypes.POINTER(Stream), pointer, ctypes.c_uint32]
    finish = module.jpeg_stream_finish
    finish.argtypes = [ctypes.POINTER(Stream), ctypes.POINTER(ctypes.c_uint32)]
    finish.restype = ctypes.c_int
    for padding in range(4):
        payload = np.array([255, 216, 1, 2, 3, 255, 217] + [0x6a] * padding, np.uint8)
        guarded = np.full(64, 0xa5, np.uint8)
        output = guarded[16:48]
        stream = Stream()
        begin(ctypes.byref(stream), output.ctypes.data_as(pointer), output.size)
        # SOI/EOI可跨两次回调；追加层不应依赖块边界。
        for chunk in (payload[:1], payload[1:6], payload[6:]):
            append(ctypes.byref(stream), chunk.ctypes.data_as(pointer), chunk.size)
        size = ctypes.c_uint32(99)
        assert finish(ctypes.byref(stream), ctypes.byref(size)) == 0 and size.value == 7
        assert np.all(guarded[:16] == 0xa5) and np.all(guarded[48:] == 0xa5)
    begin(ctypes.byref(stream), output.ctypes.data_as(pointer), 4)
    append(ctypes.byref(stream), payload.ctypes.data_as(pointer), payload.size)
    assert finish(ctypes.byref(stream), ctypes.byref(size)) == -2 and size.value == 0

report = {'result': 'PASS', 'pixel_reordering': results,
    'output_stream': 'PASS: chunk boundaries, padding, capacity',
    'hal_callbacks_mock': 'PASS', 'hardware_encoding': 'NOT RUN',
    'actual_frame_rate': 'NOT MEASURED',
    'note': 'Host packing tests and mock HAL do not validate the real JPEG peripheral or its bitstream'}
(OUT / 'hw_jpeg_host_tests.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(report, ensure_ascii=False, indent=2))
