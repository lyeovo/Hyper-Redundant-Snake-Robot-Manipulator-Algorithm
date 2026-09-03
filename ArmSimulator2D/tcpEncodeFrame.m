function bytes = tcpEncodeFrame(type, data, opts)
%tcpEncodeFrame 编码一帧：[4 字节大端长度][JSON 体]（UTF-8）
%   bytes = tcpEncodeFrame(type, data, opts)
%   type : 消息类型 'HELLO'|'HELLO_ACK'|'CMD'|'CMD_ACK'|'STATE'|'DONE'|'PING'|'PONG'|'ERROR'
%   data : struct 载荷（jsonencode 序列化）
%   opts : .seq(默认 0) .command_id(默认 '')
%   消息体 = {"v":1,"type":...,"seq":...,"command_id":...,"data":{...}}
    if nargin < 3, opts = struct(); end
    if ~isfield(opts,'seq'), opts.seq = 0; end
    if ~isfield(opts,'command_id'), opts.command_id = ''; end
    msg = struct('v', 1, 'type', type, 'seq', opts.seq, ...
        'command_id', opts.command_id, 'data', data);
    body = uint8(jsonencode(msg));
    bytes = [be32(numel(body)) body];   % 大端 int32 长度前缀
end

function b = be32(v)
    b = uint8([floor(v/2^24), floor(mod(v,2^24)/2^16), ...
               floor(mod(v,2^16)/2^8), mod(v,2^8)]);
end
