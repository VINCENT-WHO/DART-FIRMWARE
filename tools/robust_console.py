"""串口诊断工具：进程内多次握手重试，碰到一次干净 PONG 就发命令。
用于物理链路偶发丢字节（~10-50%）时仍能完成一次命令交互。"""
import sys, time, secrets

def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: robust_console.py <command> [seconds] [tries]")
    command = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 15
    tries = int(sys.argv[3]) if len(sys.argv) > 3 else 25
    import serial
    from serial.tools import list_ports
    ports = [v.device for v in list_ports.comports()
             if (v.vid, v.pid, v.serial_number) == (0x2E3C, 0x5021, '30BDDF8D6C43')]
    if len(ports) != 1:
        raise SystemExit(f"DartDAP port not unique/present: {ports}")
    with serial.Serial(ports[0], 115200, timeout=0.1, write_timeout=1) as c:
        c.write(b'\n'); c.flush(); time.sleep(0.3)
        while c.in_waiting: c.read(c.in_waiting)
        for attempt in range(tries):
            token = 'HELLO' + secrets.token_hex(3).upper()
            c.write(('PING ' + token + '\n').encode()); c.flush()
            deadline = time.monotonic() + 1.0
            got = []
            while time.monotonic() < deadline:
                line = c.readline().decode('ascii', 'replace').strip()
                if line: got.append(line)
            if 'PONG ' + token in got:
                print(f"[handshake OK on attempt {attempt + 1}]")
                c.write((command + '\n').encode('ascii')); c.flush()
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    line = c.readline().decode('ascii', 'replace').strip()
                    if line:
                        print(line, flush=True)
                        if ('SESSION END' in line or
                            (line.startswith('ERR ') and not line.startswith('ERR cam=')) or
                            (line.startswith('H7: ERR ') and not line.startswith('H7: ERR cam='))):
                            return 0
                return 0
            else:
                # 打印残缺响应供观察丢字节模式
                frag = ' | '.join(got)[:60] if got else '(no response)'
                print(f"[attempt {attempt + 1} fail: {frag}]", flush=True)
                time.sleep(0.4)
        raise SystemExit(f"handshake failed after {tries} attempts")

if __name__ == '__main__':
    main()
