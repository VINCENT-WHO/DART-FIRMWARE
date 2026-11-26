"""Package the separated program, documentation and firmware-recovery folders."""
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
import hashlib, json

ROOT=Path(__file__).resolve().parents[1]
PARENT=ROOT.parent
DOCS=PARENT/'说明文档'
BACKUPS=PARENT/'固件备份'
OUT=PARENT/'交付包'
OUT.mkdir(exist_ok=True)
release={}
for target in ['f103','h750']:
    manifest=ROOT/'build'/target/'manifest.json'
    if manifest.exists():
        m=json.loads(manifest.read_text(encoding='utf-8'))
        data=(manifest.parent/('dart_'+target+'.bin')).read_bytes()
        if hashlib.sha256(data).hexdigest()!=m['sha256']:raise SystemExit('Legacy build manifest mismatch: '+target)
        release[target]=m
    native=ROOT/'build/native'/target/('dart_'+target+'.bin')
    if native.exists():
        release[target+'_cmake']={'sha256':hashlib.sha256(native.read_bytes()).hexdigest(),'bytes':native.stat().st_size}
DOCS.mkdir(exist_ok=True)
(DOCS/'release.json').write_text(json.dumps(release,ensure_ascii=False,indent=2),encoding='utf-8')
archive=OUT/'飞镖程序与说明.zip'
with ZipFile(archive,'w',ZIP_DEFLATED,compresslevel=6) as z:
    for folder,label in [(ROOT,'dart_firmware'),(DOCS,'说明文档'),(BACKUPS,'固件备份')]:
        if not folder.exists():continue
        for path in folder.rglob('*'):
            if not path.is_file():continue
            rel=path.relative_to(folder)
            if any(part in rel.parts for part in ['.git','__pycache__','recordings','.venv']):continue
            if folder==ROOT and rel.parts[0]=='build':
                if path.suffix not in ['.elf','.bin','.hex','.map'] and path.name!='manifest.json':continue
                if any(part.endswith('.dir') for part in rel.parts) or 'CMakeFiles' in rel.parts:continue
            z.write(path,Path(label)/rel)
    workspace=PARENT/'飞镖开发.code-workspace'
    if workspace.exists():z.write(workspace,workspace.name)
print(str(archive))
print('SHA256',hashlib.sha256(archive.read_bytes()).hexdigest())
