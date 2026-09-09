function msg = tcpclientReadFrame(t)
%tcpclientReadFrame 从 tcpclient 读一整帧（[4 字节大端长度][JSON]）
    hdr = read(t, 4);
    len = double(hdr(1))*2^24 + double(hdr(2))*2^16 + double(hdr(3))*2^8 + double(hdr(4));
    if len < 1 || len > 1e7, error('tcpclientReadFrame:frame', 'invalid frame length %d', len); end
    body = read(t, len);
    msg = jsondecode(char(body));
end
