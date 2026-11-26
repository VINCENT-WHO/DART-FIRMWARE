"""Versioned COBS/CRC16 wire and on-card record decoder (stdlib only)."""
import struct
MAX_PAYLOAD=80
def crc16(data):
    c=0xffff
    for b in data:
        c^=b<<8
        for _ in range(8):c=((c<<1)^0x1021 if c&0x8000 else c<<1)&0xffff
    return c
def encode(kind,seq,payload):
    if len(payload)>MAX_PAYLOAD:raise ValueError('Payload too long')
    raw=struct.pack('<BBHI',1,kind,len(payload),seq)+payload
    raw+=struct.pack('<H',crc16(raw))
    out=bytearray([0]);code_at=0;code=1
    for b in raw:
        if b:out.append(b);code+=1
        else:out[code_at]=code;code_at=len(out);out.append(0);code=1
    out[code_at]=code;out.append(0);return bytes(out)
def decode(frame):
    if frame.endswith(b'\0'):frame=frame[:-1]
    raw=bytearray();i=0
    while i<len(frame):
        code=frame[i];i+=1
        if not code or i+code-1>len(frame):raise ValueError('COBS bounds')
        raw.extend(frame[i:i+code-1]);i+=code-1
        if code!=255 and i<len(frame):raw.append(0)
    if len(raw)<10:raise ValueError('Short frame')
    version,kind,n,seq=struct.unpack_from('<BBHI',raw)
    if version!=1 or n>MAX_PAYLOAD or len(raw)!=n+10:raise ValueError('Header/length')
    if crc16(raw[:-2])!=struct.unpack_from('<H',raw,len(raw)-2)[0]:raise ValueError('CRC16')
    return kind,seq,bytes(raw[8:-2])
def records(data):
    buffer=bytearray();errors=0;result=[];discard=False
    for b in data:
        if b:
            if not discard:
                buffer.append(b)
                if len(buffer)>96:discard=True;buffer.clear()
        else:
            if discard:errors+=1;discard=False;continue
            if buffer:
                try:
                    if len(buffer)>96:raise ValueError('Oversized')
                    result.append(decode(bytes(buffer)))
                except ValueError:errors+=1
                buffer.clear()
    return result,errors,bool(buffer) or discard
def signed_delta(a,b):return ((a-b+2**31)%2**32)-2**31
