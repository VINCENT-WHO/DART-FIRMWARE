"""Build both real MCU targets with GNU Arm; no board is accessed."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import argparse,hashlib,json,os,shutil,subprocess,sys
ROOT=Path(__file__).resolve().parents[1]
V=ROOT/'vendor'

def compiler_bin():
    if os.environ.get('ARM_GCC_BIN'):return Path(os.environ['ARM_GCC_BIN'])
    found=shutil.which('arm-none-eabi-gcc')
    if found:return Path(found).parent
    local=ROOT.parent/'work/toolchain'
    if (local/'bin/arm-none-eabi-gcc.exe').exists():return local/'bin'
    candidates=list(local.glob('*/bin/arm-none-eabi-gcc.exe'))
    if len(candidates)==1:return candidates[0].parent
    raise SystemExit('Set ARM_GCC_BIN to the Arm GNU toolchain bin directory.')

def build(target,tc):
    out=ROOT/'build'/target;out.mkdir(parents=True,exist_ok=True)
    common=[ROOT/'common/protocol.c']
    if target=='f103':
        sources=[ROOT/'f103/main.c',*common,V/'cmsis-device-f1/Source/Templates/system_stm32f1xx.c',V/'cmsis-device-f1/Source/Templates/gcc/startup_stm32f103xb.s']
        includes=[ROOT/'common',V/'cmsis-device-f1/Include',V/'CMSIS_5/Core/Include']
        flags=['-mcpu=cortex-m3','-mthumb','-mfloat-abi=soft','-DSTM32F103xB','-DHSE_VALUE=12000000U']
    else:
        h=ROOT/'h750';hal=V/'stm32h7xx-hal-driver';usb=V/'usb-device';fat=V/'fatfs'
        sources=[h/n for n in ['main.c','board.c','camera.c','camera_test.c','diskio_sd.c','sd_trace.c','usbd_conf.c','usb_tf.c']]
        sources += [*common,ROOT/'common/avi.c',ROOT/'common/auto_record.c',V/'ov5640/ov5640.c',V/'ov5640/ov5640_reg.c',fat/'source/ff.c',V/'cmsis-device-h7/Source/Templates/system_stm32h7xx.c',V/'cmsis-device-h7/Source/Templates/gcc/startup_stm32h750xx.s']
        sources += [hal/'Src'/('stm32h7xx_'+n+'.c') for n in ['hal','hal_cortex','hal_dma','hal_dma_ex','hal_pcd','hal_pcd_ex','ll_usb','hal_dcmi','hal_exti','hal_flash','hal_flash_ex','hal_gpio','hal_pwr','hal_pwr_ex','hal_rcc','hal_rcc_ex','hal_sd','hal_sd_ex','hal_uart','hal_uart_ex','ll_sdmmc']]
        sources += [usb/'Core/Src'/n for n in ['usbd_core.c','usbd_ctlreq.c','usbd_ioreq.c']]
        sources += [usb/'Class/MSC/Src'/n for n in ['usbd_msc.c','usbd_msc_bot.c','usbd_msc_scsi.c','usbd_msc_data.c']]
        includes=[ROOT/'common',h,V/'ov5640',V/'cmsis-device-h7/Include',hal/'Inc',V/'CMSIS_5/Core/Include',fat/'source',usb/'Core/Inc',usb/'Class/MSC/Inc']
        flags=['-mcpu=cortex-m7','-mthumb','-mfloat-abi=soft','-DSTM32H750xx','-DUSE_HAL_DRIVER','-DHSE_VALUE=12000000U']
    flags += ['-O2','-g3','-ffunction-sections','-fdata-sections','-fno-common','-std=gnu11','-Wall','-Wextra','-Werror','-Wno-misleading-indentation']
    flags += ['-ffile-prefix-map='+str(ROOT)+'=/dart_firmware','-fdebug-prefix-map='+str(ROOT)+'=/dart_firmware']
    flags += ['-I'+str(p) for p in includes]
    suffix='.exe' if os.name=='nt' else ''
    cc=str(tc/('arm-none-eabi-gcc'+suffix))
    def compile_one(src):
        if not src.exists():raise FileNotFoundError(src)
        obj=out/(src.stem+'_'+hashlib.sha256(str(src).encode()).hexdigest()[:8]+'.o')
        cmd=[cc,*flags,'-c',str(src),'-o',str(obj)]
        p=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,errors='replace')
        if p.returncode:raise RuntimeError(str(src)+'\n'+p.stdout)
        return obj
    with ThreadPoolExecutor(max_workers=8) as pool:objects=list(pool.map(compile_one,sources))
    elf=out/('dart_'+target+'.elf');cpu=flags[:3]
    wrap=[]
    if target=='h750':
        wrap=['-Wl,--wrap=SDMMC_Cmd'+n for n in ['GoIdleState','OperCond','AppCommand','AppOperCommand','SendCID','SetRelAdd','SendCSD','SelDesel','BlockLength','StatusRegister','SendSCR','BusWidth','SendStatus','ReadSingleBlock']]
    cmd=[cc,*cpu,'-nostartfiles','--specs=nano.specs','--specs=nosys.specs','-T'+str(ROOT/target/'linker.ld'),'-Wl,--gc-sections','-Wl,-Map='+str(elf.with_suffix('.map')),*wrap,*map(str,objects),'-lc','-lgcc','-o',str(elf)]
    subprocess.run(cmd,check=True)
    for fmt,ext in [('binary','bin'),('ihex','hex')]:subprocess.run([str(tc/('arm-none-eabi-objcopy'+suffix)),'-O',fmt,elf.name,elf.with_suffix('.'+ext).name],check=True,cwd=out)
    sizes=subprocess.check_output([str(tc/('arm-none-eabi-size'+suffix)),elf.name],text=True,cwd=out)
    print(sizes,flush=True)
    data=elf.with_suffix('.bin').read_bytes();limit=65536 if target=='f103' else 131072
    if len(data)>limit:raise RuntimeError('Flash exceeds physical device')
    result={'name':'Dart Unified '+target.upper(),'version':'1.0.0','target':'STM32F103C8T6' if target=='f103' else 'STM32H750VBT6','flash_start':'0x08000000','flash_bytes':len(data),'flash_limit':limit,'sha256':hashlib.sha256(data).hexdigest(),'compiler':subprocess.check_output([cc,'--version'],text=True).splitlines()[0],'size_report':sizes,'validation':'compiled; hardware acceptance pending'}
    (out/'manifest.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('target',choices=['all','f103','h750'],nargs='?',default='all');a=p.parse_args();tc=compiler_bin()
    for name in (['f103','h750'] if a.target=='all' else [a.target]):print(json.dumps(build(name,tc),indent=2))
