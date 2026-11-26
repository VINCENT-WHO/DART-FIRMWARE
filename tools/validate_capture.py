"""Decode and measure a real paired DLG/AVI acquisition without altering it."""
from pathlib import Path
import argparse,csv,json,statistics
from decode_log import decode_file

def rows(path):
    if not path.exists():return []
    with path.open(encoding='utf-8-sig',newline='') as stream:return list(csv.DictReader(stream))

def intervals(data,key):
    stamps=[int(row[key]) for row in data]
    delta=[(b-a)%2**32 for a,b in zip(stamps,stamps[1:])]
    if not delta:return None
    return {'min':min(delta),'median':statistics.median(delta),'max':max(delta),
            'mean':statistics.mean(delta),'measured_hz':1e6*len(delta)/sum(delta) if sum(delta) else None}

def validate(log,out):
    log=Path(log);out=Path(out)
    result=decode_file(log,out)
    imu=rows(out/'imu.csv');frames=rows(out/'frames.csv')
    result['imu_intervals_us']=intervals(imu,'f1_time_us')
    result['frame_intervals_us']=intervals(frames,'h7_frame_end_us')
    for field in ['gyro_fresh','accel_fresh','spi_error','late']:
        result[field+'_count']=sum(row[field]=='True' for row in imu)
    result['invalid_imu_samples']=sum(row['gyro_valid']!='True' or row['accel_valid']!='True' for row in imu)
    result['session_success']=any(e['text'].startswith('FINAL failed=0') for e in result['events'])
    video=log.parent/'VIDEO.AVI' if log.name.upper()=='IMU.DLG' else log.with_suffix('.AVI')
    if video.exists():
        try:import cv2
        except ImportError:result['video_decode']='NOT RUN: install opencv-python'
        else:
            cap=cv2.VideoCapture(str(video));count=0;dimensions=set()
            result['avi_header_fps']=cap.get(cv2.CAP_PROP_FPS)
            while True:
                ok,frame=cap.read()
                if not ok:break
                dimensions.add((frame.shape[1],frame.shape[0]));count+=1
            cap.release()
            result['video_decode']='PASS' if count==len(frames) and count>0 else 'FAIL'
            result['decoded_frames']=count;result['dimensions']=sorted(dimensions)
    result['integrity_pass']=(result['session_success'] and not result['truncated_tail']
        and not result['crc_or_format_errors'] and not result['imu_sequence_gaps']
        and not result['spi_error_count'] and not result['late_count'] and not result['invalid_imu_samples']
        and (not video.exists() or result.get('video_decode')=='PASS'))
    (out/'capture_validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',type=Path);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(validate(args.log,args.out),ensure_ascii=False,indent=2))
