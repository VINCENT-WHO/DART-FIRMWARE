"""主机验证：机载绿灯检测管线 vs 电脑端 OpenCV 结果。

流程：
1. 读取绿灯识别结果目录中的 annotated.avi（与原始录像同源的帧序列）；
2. 每帧 RGB -> BT.601 全幅 YCbCr -> 4:2:2 YUYV，模拟 OV5640 的量化损失；
3. 调用主机编译的 onboard_pipeline.exe 跑板端"检测->标注->JPEG 编码"；
4. 与电脑端 centers.csv 逐帧对比：检出状态一致率、质心与半径偏差；
5. 抽样解码机内 JPEG 验证可解码性，统计编码尺寸与耗时数据。

用法：python verify_onboard_detector.py <结果目录> <输出目录> [质量]
"""
import csv
import json
import shutil
import struct
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PIPELINE = ROOT / "build/tests/onboard/onboard_pipeline.exe"


def rgb_to_yuyv(frame_bgr):
    """BGR 帧 -> YUYV422 字节流（模拟 OV5640 输出：BT.601 全幅 + 水平 2:1 色度）。"""
    b, g, r = frame_bgr[:, :, 0].astype(np.int32), frame_bgr[:, :, 1].astype(np.int32), \
        frame_bgr[:, :, 2].astype(np.int32)
    y = (0.299 * r + 0.587 * g + 0.114 * b + 0.5).astype(np.uint8)
    cb = (-0.168736 * r - 0.331264 * g + 0.5 * b + 128.5).astype(np.uint8)
    cr = (0.5 * r - 0.418688 * g - 0.081312 * b + 128.5).astype(np.uint8)
    height, width = y.shape
    yuyv = np.zeros((height, width, 2), dtype=np.uint8)
    yuyv[:, :, 0] = y
    yuyv[:, 0::2, 1] = cb[:, 0::2]      # 4:2:2：色度水平共享（取偶数列）
    yuyv[:, 1::2, 1] = cr[:, 0::2]
    return yuyv.tobytes()


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    result_dir = Path(sys.argv[1])
    out_dir = Path(sys.argv[2])
    quality = int(sys.argv[3]) if len(sys.argv) > 3 else 75
    if not PIPELINE.exists():
        print("onboard_pipeline.exe 不存在，先编译：")
        print("  gcc -std=c11 -O2 -Ih750 tests/onboard_pipeline.c h750/green_detector.c "
              "h750/green_annotate.c h750/jpeg_enc.c -o build/tests/onboard/onboard_pipeline.exe")
        return 1

    frames_dir = out_dir / "frames"
    det_dir = out_dir / "det"
    for folder in (frames_dir, det_dir):
        if folder.exists():
            shutil.rmtree(folder)
        folder.mkdir(parents=True)

    # 1) 解帧并模拟相机输出。
    cap = cv2.VideoCapture(str(result_dir / "annotated.avi"))
    if not cap.isOpened():
        print("无法打开 annotated.avi")
        return 1
    index = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        (frames_dir / f"frame_{index:05d}.bin").write_bytes(rgb_to_yuyv(frame))
        index += 1
    cap.release()
    total = index
    print(f"模拟帧数: {total}")

    # 2) 跑板端管线。
    proc = subprocess.run([str(PIPELINE), str(frames_dir), str(det_dir), str(quality)],
                          capture_output=True, text=True)
    print(proc.stdout.strip())
    if proc.returncode:
        print(proc.stderr)
        return 1

    # 3) 与电脑端 centers.csv 对比。
    with (result_dir / "centers.csv").open(encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    with (det_dir / "centers_onboard.csv").open(encoding="utf-8-sig") as f:
        mine = list(csv.DictReader(f))
    if len(rows) != total or len(mine) != total:
        print(f"帧数不一致: annotated={total} pc_csv={len(rows)} onboard={len(mine)}")
        return 1

    match_flag = 0
    center_devs, radius_devs, mismatches = [], [], []
    for i in range(total):
        pc_flag = int(rows[i]["检测到目标"])
        ob_flag = int(mine[i]["检出标志"])
        if pc_flag == ob_flag:
            match_flag += 1
        else:
            mismatches.append({"帧": i, "电脑": pc_flag, "机载": ob_flag})
            continue
        if pc_flag:
            pcx, pcy = float(rows[i]["中心X_px"]), float(rows[i]["中心Y_px"])
            obx, oby = float(mine[i]["质心X_像素"]), float(mine[i]["质心Y_像素"])
            center_devs.append(((pcx - obx) ** 2 + (pcy - oby) ** 2) ** 0.5)
            radius_devs.append(abs(float(rows[i]["标注半径_px"]) - float(mine[i]["半径_像素"])))

    # 4) 抽样解码机内 JPEG，验证可解码且尺寸正确。
    decoded, jpeg_sizes = 0, []
    for i in range(0, total, 25):
        path = det_dir / f"det_{i:05d}.jpg"
        img = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if img is None or img.shape[:2] != (240, 320):
            print(f"JPEG 解码失败: {path}")
            return 1
        decoded += 1
    for i in range(total):
        jpeg_sizes.append((det_dir / f"det_{i:05d}.jpg").stat().st_size)

    center_arr = np.array(center_devs) if center_devs else np.array([0.0])
    radius_arr = np.array(radius_devs) if radius_devs else np.array([0.0])
    report = {
        "结果目录": str(result_dir),
        "帧数": total,
        "检出状态一致率": round(match_flag / total, 4),
        "状态不一致帧": mismatches[:20],
        "共同检出帧数": len(center_devs),
        "质心偏差_px": {
            "平均": round(float(center_arr.mean()), 3),
            "最大": round(float(center_arr.max()), 3),
            "中位数": round(float(np.median(center_arr)), 3),
        },
        "半径偏差_px": {
            "平均": round(float(radius_arr.mean()), 3),
            "最大": round(float(radius_arr.max()), 3),
        },
        "JPEG质量": quality,
        "JPEG字节": {
            "平均": int(np.mean(jpeg_sizes)),
            "最大": int(np.max(jpeg_sizes)),
            "最小": int(np.min(jpeg_sizes)),
        },
        "抽样解码帧数": decoded,
    }
    (out_dir / "validation_onboard.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
