"""电脑串口工具：发送文本命令；本身不烧录、不自动开启舵机。"""
import argparse,secrets,time
def main():
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:raise SystemExit('Install console dependency: python -m pip install pyserial==3.5')
    p=argparse.ArgumentParser();p.add_argument('--port');p.add_argument('--command');p.add_argument('--seconds',type=float,default=20);a=p.parse_args()
    port=a.port
    if not port:
        ports=[v.device for v in list_ports.comports() if (v.vid,v.pid,v.serial_number)==(0x2E3C,0x5021,'30BDDF8D6C43')]
        if len(ports)!=1:raise SystemExit('DartDAP port not unique/present; supply --port')
        port=ports[0]
    with serial.Serial(port,115200,timeout=.1,write_timeout=1) as connection:
        connection.write(b'\n');connection.flush()
        until=time.monotonic()+.3
        while time.monotonic()<until:connection.read(max(1,connection.in_waiting))
        token='HELLO'+secrets.token_hex(3).upper();connection.write(('PING '+token+'\n').encode())
        deadline=time.monotonic()+2;ready=False
        while time.monotonic()<deadline:
            line=connection.readline().decode('ascii','replace').strip()
            if line:print(line,flush=True)
            if line=='PONG '+token:ready=True;break
        if not ready:raise SystemExit('Handshake failed; no action command sent')
        if not a.command:raise SystemExit('Connected. Use --command STATUS, or --command "H7 START A10001 10"')
        if '\n' in a.command or '\r' in a.command:raise SystemExit('One command per invocation')
        connection.write((a.command+'\n').encode('ascii'));connection.flush()
        deadline=time.monotonic()+a.seconds
        while time.monotonic()<deadline:
            line=connection.readline().decode('ascii','replace').strip()
            if line:print(line,flush=True)
            # 状态输出里的“ERR cam=0...”是错误计数，不代表命令执行失败。
            command_error=line.startswith(('ERR ', 'H7: ERR ')) and not line.startswith(('ERR cam=', 'H7: ERR cam='))
            if 'SESSION END' in line or command_error:break
if __name__=='__main__':main()
