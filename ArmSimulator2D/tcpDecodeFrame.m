function msg = tcpDecodeFrame(bytes)
%tcpDecodeFrame 解码一帧：[4 字节大端长度][JSON 体]（UTF-8）
%   msg = tcpDecodeFrame(bytes)  % bytes 为 uint8 向量（可为流数据，取帧头定义的长度）
%   msg = 解码后的 struct（含 .v .type .seq .command_id .data）
    if numel(bytes) < 4, error('tcpDecodeFrame:short', '帧头不足'); end
    len = double(bytes(1))*2^24 + double(bytes(2))*2^16 + ...
          double(bytes(3))*2^8 + double(bytes(4));
    if numel(bytes) < 4 + len, error('tcpDecodeFrame:short', '帧体不足'); end
    msg = jsondecode(char(bytes(5:4+len)));
end
