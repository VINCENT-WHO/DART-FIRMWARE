"""使用板载 DartDAP 备份、烧录、回读验证 F103；默认使用原生 CMake 产物。"""
from pathlib import Path
from datetime import datetime
import argparse,hashlib,json,subprocess
ROOT=Path(__file__).resolve().parents[1]
def tclpath(p):return '{'+str(p.resolve()).replace('\\','/')+'}'
def main():
    p=argparse.ArgumentParser();p.add_argument('--openocd',default='openocd');p.add_argument('--scripts',type=Path)
    p.add_argument('--program',action='store_true');p.add_argument('--restore',type=Path);a=p.parse_args()
    if a.program and a.restore:p.error('Choose program or restore')
    out=ROOT.parent/'固件备份'/('f103_'+datetime.now().strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True)
    cfg=ROOT/'tools/f103_openocd.cfg';report={'target':'STM32F103C8T6','backup_verified':False,'programmed':False,'readback_verified':False}
    def run(label,command):
        cmd=[a.openocd]+(['-s',str(a.scripts)] if a.scripts else [])+['-f',str(cfg),'-c',command]
        r=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,errors='replace',timeout=60)
        (out/(label+'.txt')).write_text(r.stdout,encoding='utf-8')
        if r.returncode:raise RuntimeError(label+' failed; see '+str(out))
    try:
        run('backup','init; halt; dump_image '+tclpath(out/'original.bin')+' 0x08000000 65536; dump_image '+tclpath(out/'original_again.bin')+' 0x08000000 65536; dump_image '+tclpath(out/'capacity.bin')+' 0x1FFFF7E0 2; resume; shutdown')
        original=(out/'original.bin').read_bytes()
        if len(original)!=65536 or original!=(out/'original_again.bin').read_bytes():raise RuntimeError('Backup mismatch')
        if int.from_bytes((out/'capacity.bin').read_bytes(),'little')!=64:raise RuntimeError('Expected nominal64KiB MCU')
        report['backup_verified']=True;report['original_sha256']=hashlib.sha256(original).hexdigest()
        if not a.program and not a.restore:print('Backup complete:',out);return
        if a.restore:
            data=a.restore.read_bytes()
            if len(data)!=65536:raise RuntimeError('Restore requires exact64KiB backup')
        else:
            data=(ROOT/'build/native/f103/dart_f103.bin').read_bytes();m=json.loads((ROOT/'build/native/f103/manifest.json').read_text())
            if m['target']!='STM32F103C8T6' or hashlib.sha256(data).hexdigest()!=m['sha256']:raise RuntimeError('Candidate mismatch')
            if not 256<len(data)<=65536:raise RuntimeError('Candidate size exceeds F103 flash')
            data=data+b'\xff'*(65536-len(data))
        candidate=out/'candidate_full64k.bin';candidate.write_bytes(data)
        run('program','init; reset halt; flash write_image erase '+tclpath(candidate)+' 0x08000000 bin; verify_image '+tclpath(candidate)+' 0x08000000 bin; dump_image '+tclpath(out/'readback.bin')+' 0x08000000 65536; reset run; shutdown')
        report['programmed']=True
        if (out/'readback.bin').read_bytes()!=data:raise RuntimeError('Readback mismatch')
        report['readback_verified']=True;print('PASS:64KiB backup and programmed readback verified',out)
    finally:(out/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
if __name__=='__main__':main()
