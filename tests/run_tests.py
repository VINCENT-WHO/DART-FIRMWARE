from pathlib import Path
import csv,json,os,shutil,struct,subprocess,sys,tempfile
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from wire import encode,decode,records,signed_delta
from decode_log import decode_file
out=ROOT/'build/tests';out.mkdir(parents=True,exist_ok=True)
cc=shutil.which('gcc')
if not cc:raise SystemExit('Host GCC required for C protocol validation')
exe=out/('test_protocol.exe' if os.name=='nt' else 'test_protocol')
subprocess.run([cc,'-std=c11','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-I'+str(ROOT/'common'),str(ROOT/'tests/test_protocol.c'),str(ROOT/'common/protocol.c'),str(ROOT/'common/avi.c'),'-o',str(exe)],check=True)
subprocess.run([str(exe),str(out/'avi_header.bin')],check=True)
auto_exe=out/('test_auto_record.exe' if os.name=='nt' else 'test_auto_record')
subprocess.run([cc,'-std=c11','-Wall','-Wextra','-Werror','-I'+str(ROOT/'common'),str(ROOT/'tests/test_auto_record.c'),str(ROOT/'common/auto_record.c'),'-o',str(auto_exe)],check=True)
subprocess.run([str(auto_exe)],check=True)
exposure_exe=out/('test_camera_exposure.exe' if os.name=='nt' else 'test_camera_exposure')
subprocess.run([cc,'-std=c11','-O2','-Wall','-Wextra','-Werror','-I'+str(ROOT/'h750'),
    str(ROOT/'tests/test_camera_exposure.c'),'-o',str(exposure_exe)],check=True)
subprocess.run([str(exposure_exe)],check=True)
profile_exe=out/('test_record_profile.exe' if os.name=='nt' else 'test_record_profile')
subprocess.run([cc,'-std=c11','-O2','-Wall','-Wextra','-Werror','-I'+str(ROOT/'h750'),
    str(ROOT/'tests/test_record_profile.c'),str(ROOT/'h750/record_profile.c'),'-o',str(profile_exe)],check=True)
subprocess.run([str(profile_exe)],check=True)
detector_exe=out/('test_rgb_detector.exe' if os.name=='nt' else 'test_rgb_detector')
subprocess.run([cc,'-std=c11','-O2','-Wall','-Wextra','-Werror',
    '-Wno-misleading-indentation','-I'+str(ROOT/'h750'),
    str(ROOT/'tests/test_rgb_detector.c'),str(ROOT/'h750/green_detector.c'),
    '-o',str(detector_exe),'-lm'],check=True)
subprocess.run([str(detector_exe)],check=True)
roi_exe=out/('test_detector_roi.exe' if os.name=='nt' else 'test_detector_roi')
subprocess.run([cc,'-std=c11','-O2','-Wall','-Wextra','-Werror',
    '-Wno-misleading-indentation','-I'+str(ROOT/'h750'),
    str(ROOT/'tests/test_detector_roi.c'),'-o',str(roi_exe),'-lm'],check=True)
subprocess.run([str(roi_exe)],check=True)
median_exe=out/('test_median_sliding.exe' if os.name=='nt' else 'test_median_sliding')
subprocess.run([cc,'-std=c11','-O2','-Wall','-Wextra','-Werror',
    '-Wno-misleading-indentation','-I'+str(ROOT/'h750'),
    str(ROOT/'tests/test_median_sliding.c'),str(ROOT/'tests/median_test_bridge.c'),
    '-o',str(median_exe),'-lm'],check=True)
subprocess.run([str(median_exe)],check=True)
for n in range(81):
    payload=bytes((i*31)%256 for i in range(n));assert decode(encode(19,n,payload))==(19,n,payload)
assert signed_delta(50,0xfffffff0)==66
imu1=struct.pack('<III6hHHI',0,0xfffffff0,12,100,-100,200,0,0,1365,15,0,0)
imu2=struct.pack('<III6hHHI',2,984,13,101,-99,201,0,0,1366,15,0,0)
items=[encode(19,0,struct.pack('<I',1000)+b'BEGIN'),
 encode(18,1,struct.pack('<IIII',2000,1900,0xfffffff0,2100)),
 encode(16,2,struct.pack('<I',2200)+imu1),encode(16,3,struct.pack('<I',3200)+imu2),
 encode(17,4,struct.pack('<IIII',3000,0,1500,1)),encode(19,5,struct.pack('<I',4000)+b'END')]
source=out/'fixture.DLG';source.write_bytes(b''.join(items))
summary=decode_file(source,out/'decoded');assert summary['imu_samples']==2 and summary['imu_sequence_gaps']==1 and summary['sync_pairs']==1 and summary['video_frames_logged']==1
assert summary['crc_or_format_errors']==0 and not summary['truncated_tail']
with (out/'decoded/imu.csv').open(encoding='utf-8-sig') as f:
    rows=list(csv.DictReader(f));assert float(rows[1]['h7_estimated_sample_us'])-float(rows[0]['h7_estimated_sample_us'])==1000
data=bytearray(items[2]);data[-3]^=8
parsed,errors,truncated=records(items[0]+data+items[3]+items[4][:-2]);assert errors==1 and truncated and len(parsed)==2
try:
    from PIL import Image
    from io import BytesIO
    frame=BytesIO();Image.new('RGB',(320,240),(50,120,190)).save(frame,format='JPEG');jpeg=frame.getvalue()
    movi=b'';offsets=[]
    for i in range(3):
        offsets.append(4+len(movi));movi+=b'00dc'+struct.pack('<I',len(jpeg))+jpeg+(b'\0' if len(jpeg)&1 else b'')
    idx=b'idx1'+struct.pack('<I',48)+b''.join(b'00dc'+struct.pack('<III',16,o,len(jpeg)) for o in offsets)
    header=bytearray((out/'avi_header.bin').read_bytes());struct.pack_into('<I',header,4,224+len(movi)+len(idx)-8);struct.pack_into('<I',header,216,len(movi)+4)
    avi=out/'fixture.AVI';avi.write_bytes(header+movi+idx)
    import cv2
    cap=cv2.VideoCapture(str(avi));decoded=0
    while True:
        ok,frame=cap.read()
        if not ok:break
        assert frame.shape[:2]==(240,320);decoded+=1
    cap.release();assert decoded==3
    avi_result='PASS'
except ImportError:print('Pillow/OpenCV unavailable: skipped generated JPEG/AVI decode');avi_result='NOT RUN'
report={'result':'PASS','c_randomized_cases':10000,'python_wire_lengths':81,'crc_corruption_and_truncated_tail':'PASS','timestamp_wrap_and_sync':'PASS','imu_gap_detection':'PASS','generated_avi_decode':avi_result,'record_profile_capacity_errors_csv_reset':'PASS','rgb_detection_area_centroid_input_unchanged_and_timing':'PASS','fixed_exposure_register_logic':'PASS','sliding_median_immutable_reference_and_guards':'PASS','hardware':'NOT RUN'}
report['roi_bounds_halo_motion_reacquire_reset']='PASS'
(out/'validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
