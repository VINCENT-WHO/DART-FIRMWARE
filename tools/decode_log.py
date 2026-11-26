"""Export .DLG IMU/frame/synchronization records. Video is the paired .AVI."""
from pathlib import Path
import argparse,csv,json,struct,re
from wire import records,signed_delta
IMU=struct.Struct('<III6hHHI')
def decode_file(path,out):
    path=Path(path);out=Path(out);out.mkdir(parents=True,exist_ok=True)
    packets,errors,truncated=records(path.read_bytes())
    sync=[];imu=[];frames=[];events=[]
    for kind,seq,p in packets:
        if len(p)<4:errors+=1;continue
        received=struct.unpack_from('<I',p)[0];payload=p[4:]
        if kind==18 and len(payload)==12:
            sent,source,arrived=struct.unpack('<III',payload);rtt=(arrived-sent)%2**32
            sync.append({'h7_sent_us':sent,'f1_us':source,'h7_received_us':arrived,'rtt_us':rtt})
        elif kind==16 and len(payload)==32:
            values=IMU.unpack(payload);imu.append((received,values))
        elif kind==17 and len(payload)==12:
            i,n,sensor=struct.unpack('<III',payload);frames.append({'index':i,'h7_frame_end_us':received,'jpeg_bytes':n,'camera_completed_counter':sensor})
        elif kind==19:events.append({'h7_us':received,'text':payload.decode('ascii','replace')})
        else:errors+=1
    best=min(sync,key=lambda s:s['rtt_us']) if sync else None
    # 新固件使用触发样本为时间零点；兼容旧版没有TRIGGER事件的文件。
    trigger=None
    for event in events:
        match=re.search(r'TRIGGER f1_us=(\d+) h7_us=(\d+)',event['text'])
        if match:trigger={'f1_us':int(match[1]),'h7_us':int(match[2])};break
    rows=[];gaps=0;previous=None
    for received,v in imu:
        seq,t,st,*rest=v;gx,gy,gz,ax,ay,az,flags,reserved,missed=rest
        if previous is not None:gaps+=(seq-previous-1)%2**32
        previous=seq
        mapped=(best['h7_sent_us']+best['rtt_us']/2+signed_delta(t,best['f1_us'])) if best else None
        relative=signed_delta(t,trigger['f1_us'])/1e6 if trigger else None
        rows.append({'sequence':seq,'time_from_trigger_s':relative,'f1_time_us':t,'h7_rx_us':received,'h7_estimated_sample_us':mapped,
          'gyro_x_dps':gx*2000/32768,'gyro_y_dps':gy*2000/32768,'gyro_z_dps':gz*2000/32768,
          'acc_x_g':ax*24/32768,'acc_y_g':ay*24/32768,'acc_z_g':az*24/32768,
          'accel_sensor_time_24bit':st,'flags':flags,'gyro_valid':bool(flags&1),'accel_valid':bool(flags&2),
          'gyro_fresh':bool(flags&8),'accel_fresh':bool(flags&4),'spi_error':bool(flags&16),'late':bool(flags&32),'missed_ticks':missed})
    for filename,data in [('imu.csv',rows),('frames.csv',frames),('clock_sync.csv',sync)]:
        if data:
            with (out/filename).open('w',newline='',encoding='utf-8-sig') as f:
                writer=csv.DictWriter(f,fieldnames=list(data[0]));writer.writeheader();writer.writerows(data)
    complete=any(e['text'].startswith('FINAL failed=') for e in events)
    summary={'source':str(path),'trigger':trigger,'imu_samples':len(rows),'video_frames_logged':len(frames),'crc_or_format_errors':errors,'session_closed_marker':complete,
             'truncated_tail':truncated,'imu_sequence_gaps':gaps,'sync_pairs':len(sync),'best_sync':best,'events':events,
             'timing_note':'Sync midpoint is an estimate; RTT/2 bounds transport asymmetry only. Frame timestamp is DMA/frame end, not exposure time.',
             'axes_note':'Raw sensor axes; board-to-airframe mounting rotation and calibration are not yet applied.'}
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    return summary
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('log');p.add_argument('--out',default='decoded');a=p.parse_args()
    print(json.dumps(decode_file(a.log,a.out),ensure_ascii=False,indent=2))
