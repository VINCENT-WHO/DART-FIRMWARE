"""通过 ROM DFU 备份、烧录、回读验证 H750；不修改选项字节和TF数据。"""
from pathlib import Path
from datetime import datetime
import argparse,hashlib,json,re,subprocess

ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--cli',required=True);p.add_argument('--serial',default='200364500000')
    p.add_argument('--program',action='store_true');p.add_argument('--restore',type=Path)
    a=p.parse_args()
    if a.program and a.restore:p.error('Choose program or restore')
    out=ROOT.parent/'固件备份'/('h750_'+datetime.now().strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True)
    report={'target':'STM32H750VBT6','serial':a.serial,'flash_size':131072,'backup_verified':False,'programmed':False,'readback_verified':False}
    def run(name,args):
        cmd=[a.cli,'-q','-c','port=USB1','sn='+a.serial,*args]
        r=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,errors='replace',timeout=60)
        s=re.sub(r'\x1b\[[0-9;]*[A-Za-z]','',r.stdout);(out/(name+'.txt')).write_text(s,encoding='utf-8')
        if r.returncode or re.search(r'(^|\n)\s*Error:',s,re.I):raise RuntimeError(name+' failed; see '+str(out))
    try:
        run('flash_capacity',['-u','0x1FF1E880','0x4',str(out/'capacity.bin')])
        if int.from_bytes((out/'capacity.bin').read_bytes()[:2],'little')!=128:raise RuntimeError('Expected nominal128KiB device')
        run('backup',['-u','0x08000000','0x20000',str(out/'original.bin')])
        run('backup_verify',['-u','0x08000000','0x20000',str(out/'original_again.bin')])
        original=(out/'original.bin').read_bytes()
        if len(original)!=131072 or original!=(out/'original_again.bin').read_bytes():raise RuntimeError('Backup readback differs')
        report['backup_verified']=True;report['original_sha256']=hashlib.sha256(original).hexdigest()
        if not a.program and not a.restore:print('Backup complete:',out);return
        if a.restore:
            data=a.restore.read_bytes()
            if len(data)!=131072:raise RuntimeError('Restore must be an exact128KiB backup')
        else:
            manifest=json.loads((ROOT/'build/native/h750/manifest.json').read_text())
            data=(ROOT/'build/native/h750/dart_h750.bin').read_bytes()
            if manifest['target']!='STM32H750VBT6' or hashlib.sha256(data).hexdigest()!=manifest['sha256']:raise RuntimeError('Candidate hash/target mismatch')
            if not 256<len(data)<=131072:raise RuntimeError('Candidate size')
            data=data+b'\xff'*(131072-len(data))
        stack=int.from_bytes(data[:4],'little');reset=int.from_bytes(data[4:8],'little')
        if not 0x20000000<stack<=0x20020000 or stack%8 or not reset&1 or not 0x08000000<(reset&~1)<0x08020000:raise RuntimeError('Invalid vector table')
        candidate=out/'candidate_full128k.bin';candidate.write_bytes(data)
        run('erase_sector0',['-e','0'])
        run('program',['--skipErase','-d',str(candidate),'0x08000000','-v']);report['programmed']=True
        run('full_readback',['-u','0x08000000','0x20000',str(out/'readback.bin')])
        if (out/'readback.bin').read_bytes()!=data:raise RuntimeError('Full readback mismatch')
        report['readback_verified']=True;print('PASS:128KiB backup and programmed readback verified. Reboot without BOOT0.',out)
    finally:(out/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
if __name__=='__main__':main()
