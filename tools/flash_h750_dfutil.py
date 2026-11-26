"""WinUSB alternative for H750 ROM DFU, with two backups and full verification.

Uses the board's verified 128KiB nominal limit and fixed serial; never changes
option bytes, read protection, or TF contents. dfu-util erases addressed pages.
"""
from pathlib import Path
from datetime import datetime
import argparse,hashlib,json,subprocess
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--dfu-util',required=True);p.add_argument('--program',action='store_true');p.add_argument('--restore',type=Path)
    # 明确指定构建目录，避免独立候选版本误烧成build/native中的旧固件。
    p.add_argument('--build-dir',type=Path,default=ROOT/'build/native');a=p.parse_args()
    if a.program and a.restore:p.error('Choose program or restore')
    out=ROOT.parent/'固件备份'/('h750_'+datetime.now().strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True)
    print('BACKUP_DIR:',out,flush=True)
    report={'target':'STM32H750VBT6','serial':'200364500000','flash_size':131072,'backup_verified':False,'programmed':False,'readback_verified':False,'backend':'dfu-util/WinUSB'}
    def run(name,args):
        cmd=[a.dfu_util,'-d','0483:df11','-S','200364500000','-a','0',*args]
        r=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,errors='replace',timeout=90)
        (out/(name+'.txt')).write_text(r.stdout,encoding='utf-8')
        print(name,'exit',r.returncode,flush=True)
        if r.returncode:raise RuntimeError(name+' failed; see '+str(out))
    def read(name):run(name,['-s','0x08000000:131072','-U',str(out/(name+'.bin'))]);return (out/(name+'.bin')).read_bytes()
    try:
        first=read('original');second=read('original_again')
        if len(first)!=131072 or first!=second:raise RuntimeError('Backup comparison failed')
        report['backup_verified']=True;report['original_sha256']=hashlib.sha256(first).hexdigest()
        if not a.program and not a.restore:print('Backup verified:',out);return
        if a.restore:
            data=a.restore.read_bytes()
            if len(data)!=131072:raise RuntimeError('Restore must be exactly128KiB')
        else:
            artifact_dir=a.build_dir.resolve()/'h750'
            m=json.loads((artifact_dir/'manifest.json').read_text());data=(artifact_dir/'dart_h750.bin').read_bytes()
            if m['target']!='STM32H750VBT6' or not 256<len(data)<=131072 or m['flash_bytes']!=len(data) or hashlib.sha256(data).hexdigest()!=m['sha256']:raise RuntimeError('Candidate mismatch')
            report['runtime_version']=m['version'];report['candidate_source']=str(artifact_dir/'dart_h750.bin')
            report['candidate_binary_bytes']=len(data);report['candidate_binary_sha256']=m['sha256']
            data+=b'\xff'*(131072-len(data))
        stack=int.from_bytes(data[:4],'little');pc=int.from_bytes(data[4:8],'little')
        if not 0x20000000<stack<=0x20020000 or stack%8 or not pc&1 or not 0x08000000<(pc&~1)<0x08020000:raise RuntimeError('Vector table')
        candidate=out/'candidate_full128k.bin';candidate.write_bytes(data)
        report['candidate_full128k_sha256']=hashlib.sha256(data).hexdigest()
        report['program_attempted']=True
        run('program',['-s','0x08000000','-D',str(candidate)]);report['programmed']=True
        after=read('readback')
        if after!=data:raise RuntimeError('Full128KiB readback mismatch')
        report['readback_verified']=True
        report['readback_sha256']=hashlib.sha256(after).hexdigest()
        print('PASS: complete backup, flash and128KiB verification. Reboot without BOOT.',out)
    finally:(out/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
if __name__=='__main__':main()
