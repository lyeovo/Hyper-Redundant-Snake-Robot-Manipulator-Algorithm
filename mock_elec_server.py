"""mock_elec_server.py — 电控 mock TCP 服务器（本地回环仿真用，快速启动）
=======================================================================
接收 MOTOR_CMD(26参) 帧 → 应用(trigger 驱动：读帧注册目标/驱动帧累计绝对角) → 回发 STATE{t,joints[6](rad),gripper,obj,obj_frame,obj_absent}
帧格式与本项目 tcpEncodeFrame 一致：[4 字节大端长度][UTF-8 JSON]，JSON 体 {"v","type","seq","command_id","data"}。
支持多客户端：runTaskLoop 每任务会重连一次；服务器 accept 循环处理，直到 Ctrl+C。
用法：python mock_elec_server.py [port] [bind] [once]
      once = 只服务一个连接，客户端断开后自动退出（供自动化测试用，避免残留进程占端口）
"""
import socket
import struct
import json
import sys
import os
import tempfile

# 参数解析：位置参数 [port] [bind]，'once' 可出现在任意位置（不占位置）
ONCE = any(a.lower() == 'once' for a in sys.argv[1:])
_POS = [a for a in sys.argv[1:] if a.lower() != 'once']
PORT = int(_POS[0]) if len(_POS) > 0 else 9100
# 绑定地址：默认 0.0.0.0（所有网卡，局域网远程客户端可连）；可传 127.0.0.1 仅本机
BIND = _POS[1] if len(_POS) > 1 else '0.0.0.0'
N = 6
READY = os.path.join(tempfile.gettempdir(), 'elec_mock_ready.txt')   # 已绑定(等待连接)
CONN = os.path.join(tempfile.gettempdir(), 'elec_mock_conn.txt')     # 已有客户端连接


def _serve(conn: socket.socket) -> None:
    """服务单个连接：读 MOTOR_CMD 帧 → 应用(trigger 驱动) → 回 STATE；客户端断开即返回。
    trigger 语义：0=读入 angle_deg 到目标寄存器(不运动)；1=电机驱动到寄存器目标(累计运动)。
    run=0 时客户端处于“暂停”→ 本侧保持当前姿态（不驱动累计），但仍回 STATE。
    安装方向：电机本身固定，机械部分 2/4/6 反装；算法侧已区分取反 direction(偶数反装→空间正向)。
    dSPACE 对每台电机【一视同仁】motor_abs += sign(dir)*angle；空间几何角 = mount_sign .* motor_abs
    （机械反装把电机反向转成空间正向）。init 为初始【真实电机角度】= ms.*q_model(1)，作累计基准。
    """
    motor_abs = [0.0] * N       # dSPACE 侧：每台电机累计位（对每台电机等处理）
    target = [0.0] * N            # 每电机的目标寄存器（trigger=0 读入）
    mount = [1.0] * N             # 各电机安装方向符号
    gr = 0
    seq = 0
    buf = b''
    while True:
        chunk = conn.recv(4096)
        if not chunk:
            return                                  # 客户端断开
        buf += chunk
        while len(buf) >= 4:
            n = struct.unpack('>I', buf[:4])[0]     # 大端长度 = tcpEncodeFrame.be32
            if len(buf) < 4 + n:
                break                               # 半包，等更多
            body, buf = buf[4:4 + n], buf[4 + n:]
            msg = json.loads(body)
            if msg.get('type') == 'MOTOR_CMD':
                d = msg['data']
                seq += 1
                run = int(d.get('run', 1))
                msig = d.get('mount_sign')
                if msig and len(msig) >= N:
                    mount = [float(x) if x != 0 else 1.0 for x in msig[:N]]
                else:
                    mount = [1.0 if (j % 2 == 0) else -1.0 for j in range(N)]  # 缺省交替
                init = d.get('init')
                if init:
                    motor_abs = [float(x) for x in init]   # 首发帧：初始真实电机角(度)作累计基准
                    target = [0.0] * N
                for j, m in enumerate(d.get('motors', [])):
                    if j >= N:
                        break
                    trg = int(m.get('trigger', 0))
                    sgn = 1 if m.get('direction', 0) != 0 else -1
                    if trg == 0:                        # 读相：只入寄存器，不动
                        target[j] = sgn * float(m.get('angle_deg', 0.0))
                    elif trg == 1 and run != 0:         # 驱动相：累计（direction 已含算法取反补偿）
                        motor_abs[j] += target[j]
                sv = d.get('servo', {})
                gr = int(sv.get('enable', 0))
                # 电控对每台电机等价(motor_abs += sign(dir)*angle)；direction 已由算法对偶数取反。
                # 空间几何角 = 安装符号 .* 电机累计位（机械反装把电机反向转成空间正向）。
                joints = [mount[j] * motor_abs[j] for j in range(N)]
                st = {'t': seq * 0.1,
                      'joints': [round(x * 3.141592653589793 / 180, 6) for x in joints],
                      'gripper': gr, 'obj': [], 'obj_frame': 'base', 'obj_absent': True}
                out = json.dumps({'v': 1, 'type': 'STATE', 'seq': seq,
                                  'command_id': '', 'data': st}, separators=(',', ':')).encode()
                conn.sendall(struct.pack('>I', len(out)) + out)
            elif msg.get('type') == 'BYE':
                return


def main():
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((BIND, PORT))
    srv.listen(5)
    with open(READY, 'w') as f:
        f.write(str(PORT))                          # 就绪握手：绑定成功即写标志文件
    print(f'[elec] listen {PORT}', flush=True)
    try:
        while True:
            conn, addr = srv.accept()
            print(f'[elec] client {addr} connected', flush=True)
            with open(CONN, 'w') as f:
                f.write(str(PORT))                          # 已连接标志（runTaskLoop 正下发）
            try:
                _serve(conn)
            except Exception as e:
                print(f'[elec] serve err: {e}', flush=True)
            finally:
                conn.close()
                try:
                    os.remove(CONN)
                except OSError:
                    pass
            # 继续接受下一个客户端
            if ONCE:
                break                               # 测试用：单连接后退出，不留残留进程
    except KeyboardInterrupt:
        pass
    finally:
        srv.close()
        try:
            os.remove(READY)
        except OSError:
            pass
        print('[elec] done', flush=True)


if __name__ == '__main__':
    main()
