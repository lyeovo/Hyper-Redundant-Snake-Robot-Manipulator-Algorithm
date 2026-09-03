function tcpJavaWriteFrame(dos, bytes)
%tcpJavaWriteFrame 向 Java DataOutputStream 写一帧（TCP 服务器侧共用）
%   tcpJavaWriteFrame(dos, bytes)
%   dos   : java.io.DataOutputStream
%   bytes : 完整帧 [4 字节大端长度][JSON]（由 tcpEncodeFrame 生成）
%
%   历史：本函数此前在 tcpMotionServer.m 与 tcpStateStreamServer.m 中各定义了一份
%   （逐字相同），现提取为公共函数。
    jb = typecast(uint8(bytes), 'int8');   % Java 需要 int8
    dos.write(jb, 0, numel(jb));
end
