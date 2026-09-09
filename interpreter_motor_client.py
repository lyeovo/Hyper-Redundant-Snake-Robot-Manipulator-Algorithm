r"""
interpreter_motor_client.py — ControlDesk 内置 Interpreter 客户端：接收 MOTOR_CMD(26参)帧 → 写 dSPACE 变量
=========================================================================================================
运行在【ControlDesk 的 Interpreter (内置 Python)】里，作为【TCP 客户端】连接项目侧服务器（26 参流）。
帧协议与本项目 tcpEncodeFrame 一致：[4 字节大端长度][UTF-8 JSON]，JSON 体 {"v","type","seq","command_id","data"}。
收到 MOTOR_CMD 后，把 26 参写入 ControlDesk 变量。

26 参 = 6 电机 × {enable, angle_deg, direction, trigger} = 24 + 1 舵机 × {angle_deg, enable} = 2。
trigger 驱动语义（由电控侧执行；本客户端只透传写变量）：
  · trigger=0 -> 电机读入 angle_deg 到目标寄存器（不运动）
  · trigger=1 -> 电机驱动到寄存器目标（执行运动）
每帧顶层 run(0/1)：服务端控制的运行参数。run=0 时本端【暂停】——不写驱动相关变量（保持当前位姿），
但保持 TCP 连接并继续收帧；run=1 时【恢复】写变量。断线才自动重连。

变量路径（按实际变量浏览器）：
  VAR_ROOT = 'Platform()://Model Root'
  VALUE_NODE = '/Value'   ← 真实路径需在变量名后加 /Value（如 .../angle_1/Value）；不需要时设为 ''
  变量名：enable_<j> / angle_<j> / direction_<j> / trigger_<j>（j 从 MOTOR_BASE 起，默认 1..6）
          / servo_angle_1 / servo_enable_1

★ 关键（依据 dSPACE FAQ098 "Use a Python Thread"）：
  1) 不要把接收循环跑在 UI 线程里做"阻塞 recv"（会整个界面卡死、无法点击）。
  2) 【推荐】start_client()：后台线程版（dSPACE FAQ098 官方建议用 Python 线程），调用方立即返回。
     跨线程写 COM 用 marshal（CoMarshalInterThreadInterfaceInStream / CoGetInterfaceAndReleaseStream）。
  3) 【关键】减少 tool automation 调用：只写"值变了"的变量（值未变则跳过）。
     dSPACE FAQ098 明确指出"大量 automation 调用"会让 ControlDesk 卡死/栈溢出；本客户端据此优化。
  4) run_pumped()：备用——在 UI 线程里收帧+写变量+每轮 PumpWaitingMessages() 泵消息。不返回，Ctrl+C 退出。

用法（Interpreter 内粘贴/exec 本文件）：
    cli = start_client(log_path=LOG_PATH_DEFAULT)
        # 推荐：立即返回；界面保持可点击
        # log_path: 日志写到【Interpreter 所在机器】上；默认 LOG_PATH_DEFAULT(该机临时目录)，
        #           也可给该机上的任意路径，例如 log_path=r'C:\\Temp\\motor_log.txt'
    cli.connected / cli.frames / cli.errors ; stats() ; cli.stop()

    readback()                  # 【主线程】回读全部变量当前值(打印+写日志) —— 确认"是否生效"
    probe()                     # 诊断变量路径可读写
    diag()                      # 诊断输出通道(哪条流能显示)
    set_output('stderr'|'stdout'|'both'|'auto')
    reset_cache()               # 变量被外部改动/模型重启后，清缓存强制全写

★ 如何确认变量是否生效：
    1) 写入是否发生：日志里 "W angle_1=..."（start_client(log_path=...) 产生）；
    2) 是否真的生效：在主线程调用 readback()（读 dSPACE 真实值，实测可用）；
    3) 注意：后台线程内的周期性回读(readback_s)可能因跨线程读 COM 子对象而返回 None，
       这是已知限制 —— 请用主线程 readback()/probe() 验证，不要据此判断写入失败。

直接运行/粘贴到 __main__ 时默认 start_client()（线程版，不阻塞）：
    python interpreter_motor_client.py [host] [port]            # 默认线程版
    python interpreter_motor_client.py [host] [port] --wait     # 阻塞等待(独立脚本)
    python interpreter_motor_client.py [host] [port] --pumped   # 单线程+消息泵(备用)
"""
import os
import socket
import struct
import json
import sys
import tempfile
import threading
import time

# 日志默认写到【Interpreter 所在机器】的临时目录（不要用开发机的 D:\... 路径，那边不存在）。
LOG_PATH_DEFAULT = os.path.join(tempfile.gettempdir(), 'motor_log.txt')

# 在【模块加载时(主线程)】把可用的输出流抓下来：ControlDesk 的 stdout/stderr 可能是线程局部的，
# 后台线程直接用 sys.stdout 可能拿不到控制台流 —— 抓下来的这些流在后台线程里仍然可用。
_MAIN_STREAMS = []
for _nm in ('stderr', '__stderr__', 'stdout', '__stdout__'):
    _st = getattr(sys, _nm, None)
    if _st is not None:
        _MAIN_STREAMS.append((_nm, _st))

HOST = '10.84.160.80'
PORT = 9101          # 与视觉 GUI「算法推帧端口(Interpreter连此)」一致
VAR_ROOT = 'Platform()://Model Root'
VALUE_NODE = '/Value'   # 真实变量路径后缀（如 .../angle_1/Value）；若不需要设 ''
NMOTOR = 6          # 电机数
MOTOR_BASE = 1      # 电机编号起始：1 → 变量 enable_1..enable_6 / angle_1..angle_6 ...（设 0 则为 _0.._5）
SERVO_SUFFIX = '_1'  # 舵机变量后缀：servo_angle_1 / servo_enable_1

# 记录每种变量名实际可用的路径形态（带/不带 VALUE_NODE），避免每次都试错
_path_cache = {}


def _app():
    """解析 ControlDesk 的 Application 对象（粘贴运行时为全局；import 时从 __main__/builtins 兜底）。"""
    try:
        return Application            # noqa: F821  (Interpreter 注入的全局)
    except NameError:
        pass
    import builtins
    app = getattr(builtins, 'Application', None)
    if app is None:
        import __main__
        app = getattr(__main__, 'Application', None)
    if app is None:
        raise RuntimeError('未找到 ControlDesk Application 对象：请在 ControlDesk 的 Interpreter 中运行')
    return app


def _ensure_dynamic(obj):
    """确保对象是 win32com 动态对象（支持 obj['path'] 下标访问）。
    跨线程 marshal 回来的常是原始 PyIDispatch（不支持下标，报 'not subscriptable'），需重新包装。
    """
    if obj is None:
        return obj
    try:
        if hasattr(obj, '__getitem__'):
            return obj
    except Exception:
        pass
    try:
        import win32com.client
        return win32com.client.Dispatch(obj)
    except Exception:
        return obj


def _platform_vars():
    """自动找到包含 VAR_ROOT 变量的平台 Variables 对象（不写死下标，防 IndexError）。
    返回值保证是 win32com 动态对象（可 V[path] 下标访问）。"""
    platforms = _app().ActiveExperiment.Platforms
    try:
        n = len(platforms)
    except Exception:
        n = 1
    for i in range(n):
        try:
            v = _ensure_dynamic(platforms[i].ActiveVariableDescription.Variables)
            _ = v[f'{VAR_ROOT}/enable_{MOTOR_BASE}{VALUE_NODE}']   # 虚位探测
            return v
        except Exception:
            continue
    try:
        return _ensure_dynamic(platforms[0].ActiveVariableDescription.Variables)
    except Exception:
        raise RuntimeError('无法确定 dSPACE 平台/变量容器：请检查 ActiveExperiment 是否已加载模型')


def _candidate_paths(name):
    """某变量名可能的完整路径（先带 VALUE_NODE，再回退不带）。"""
    base = f'{VAR_ROOT}/{name}'
    if VALUE_NODE:
        return [base + VALUE_NODE, base]
    return [base]


# 记录每种变量名实际可用的路径形态（带/不带 VALUE_NODE），避免每次都试错
_path_cache = {}
# 记录每个变量上一次写入的值：值未变则跳过写（大幅减少 tool automation 调用，
# 避免「大量 automation 调用刷爆 UI 线程 → ControlDesk 冻结/栈溢出」，见 dSPACE FAQ098）。
_last_written = {}
_write_stats = {'writes': 0, 'skipped': 0, 'fails': 0}
# 写日志文件：UI 冻结时可在外部(记事本)查看"到底写了什么/读回什么"
_log_path = None


def set_log(path):
    """开启写日志：所有写变量/回读都会追加到该文件。
    path 必须是【Interpreter 所在机器】上的可写路径；若不可写则回退到 LOG_PATH_DEFAULT。
    """
    global _log_path
    if not path:
        _log_path = None
        return None
    try:
        with open(path, 'a', encoding='utf-8'):
            pass
    except OSError:
        path = LOG_PATH_DEFAULT                     # 给的路径不可写 → 回退默认(该机临时目录)
    _log_path = path
    _log(f'--- log started (root={VAR_ROOT}, value_node={VALUE_NODE!r}) ---')
    return path


def _log(msg):
    if not _log_path:
        return
    try:
        with open(_log_path, 'a', encoding='utf-8') as f:
            f.write(f'{time.strftime("%H:%M:%S")} {msg}\n')
    except OSError:
        pass


# 输出通道：'auto'(默认，先 stderr 再 stdout) | 'stderr' | 'stdout' | 'both'
_out_channel = 'auto'


def set_output(channel):
    """选择客户端输出通道：'auto' | 'stderr' | 'stdout' | 'both'。
    若 ControlDesk 只显示 stderr（traceback 可见、print 不可见），用 'stderr'。
    """
    global _out_channel
    _out_channel = str(channel).lower()
    return _out_channel


def _emit(msg, **kwargs):
    """后台线程输出：优先写"模块加载时抓下的主线程流"，再回退当前 sys 流；绝不抛，并始终写日志。
    （ControlDesk 的 stdout/stderr 可能是线程局部的，线程里直接 sys.stdout 可能无效。）
    """
    line = str(msg) + '\n'
    streams = [st for _nm, st in _MAIN_STREAMS]        # 主线程抓下的流优先
    if _out_channel in ('auto', 'both', 'stderr'):
        streams += [getattr(sys, 'stderr', None), getattr(sys, '__stderr__', None)]
    if _out_channel in ('auto', 'both', 'stdout'):
        streams += [getattr(sys, 'stdout', None), getattr(sys, '__stdout__', None)]
    seen = set()
    for st in streams:
        if st is None or id(st) in seen:
            continue
        seen.add(id(st))
        try:
            st.write(line)
            st.flush()
            break
        except Exception:
            continue
    _log(msg)


def diag():
    """诊断输出通道：向各候选流各写一行，看哪一行出现在 Interpreter 里。"""
    for nm, st in _MAIN_STREAMS:
        try:
            st.write(f'[diag] captured main-thread {nm} OK\n')
            st.flush()
        except Exception as e:
            pass
    for nm in ('stderr', '__stderr__', 'stdout', '__stdout__'):
        st = getattr(sys, nm, None)
        try:
            st.write(f'[diag] current {nm} OK\n')
            st.flush()
        except Exception:
            pass


def readback(V=None, nmotor=NMOTOR, verbose=True):
    """通过 COM 读出所有目标变量的当前值（确认写入是否真的生效）。
    返回 dict{name: value}；同时打印并写入日志文件。UI 冻结时看日志文件即可。"""
    if V is None:
        V = _platform_vars()
    names = []
    for j in range(MOTOR_BASE, MOTOR_BASE + nmotor):
        names += [f'enable_{j}', f'angle_{j}', f'direction_{j}', f'trigger_{j}']
    names += [f'servo_angle{SERVO_SUFFIX}', f'servo_enable{SERVO_SUFFIX}']
    out = {}
    for nm in names:
        for path in _candidate_paths(nm):
            try:
                out[nm] = V[path].ValueConverted
                break
            except Exception:
                continue
    line = ' '.join(f'{k}={v}' for k, v in out.items())
    if verbose:
        print(f'[readback] {line}', flush=True)
    _log(f'READBACK {line}')
    return out


def _set_var(V, name, value, errlog):
    """写一个变量：值未变则跳过；否则按缓存/候选路径写。返回是否成功。"""
    if name in _last_written and _last_written[name] == value:
        _write_stats['skipped'] += 1
        return True                                   # 值未变：不产生 COM 调用
    cached = _path_cache.get(name)
    paths = [cached] if cached else _candidate_paths(name)
    last_err = None
    for path in paths:
        try:
            V[path].ValueConverted = value
            _path_cache[name] = path
            _last_written[name] = value
            _write_stats['writes'] += 1
            _log(f'W {name}={value}')
            return True
        except Exception as e:
            last_err = e
    _write_stats['fails'] += 1
    errlog.append(f'write {name} = {value} failed: {last_err}')
    _log(f'! FAIL {name}={value} : {last_err}')
    return False


def _apply(d, V, errlog):
    """把一帧 MOTOR_CMD 的 26 参写到 dSPACE 变量。单变量写失败仅记录，不抛异常、不中断连接。
    电机编号按 MOTOR_BASE 起（默认 1 → enable_1..enable_6）。"""
    for j, m in enumerate(d.get('motors', []), start=MOTOR_BASE):
        if j >= MOTOR_BASE + NMOTOR:
            break
        _set_var(V, f'enable_{j}', m.get('enable', 0), errlog)
        _set_var(V, f'angle_{j}', m.get('angle_deg', 0.0), errlog)
        _set_var(V, f'direction_{j}', m.get('direction', 0), errlog)
        _set_var(V, f'trigger_{j}', m.get('trigger', 0), errlog)
    sv = d.get('servo', {})
    _set_var(V, f'servo_angle{SERVO_SUFFIX}', sv.get('angle_deg', 0.0), errlog)
    _set_var(V, f'servo_enable{SERVO_SUFFIX}', sv.get('enable', 0), errlog)


def _consume(buf, V, errlog, write=True):
    """从 buf 解析出所有完整帧并（可选）写变量，返回 (剩余buf, 本次处理帧数)。
    任何单帧异常都被吞掉并记录，绝不向外抛（否则会中断收帧循环、导致连接被 reset）。
    """
    nframes = 0
    while len(buf) >= 4:
        n = struct.unpack('>I', buf[:4])[0]          # 大端长度 = tcpEncodeFrame.be32
        if len(buf) < 4 + n:
            break                                     # 半包，继续等
        body, buf = buf[4:4 + n], buf[4 + n:]
        try:
            msg = json.loads(body)                    # {"v","type","seq","command_id","data"}
            if not isinstance(msg, dict):
                continue
            if msg.get('type') == 'MOTOR_CMD':
                d = msg.get('data')
                if not isinstance(d, dict):
                    continue
                if int(d.get('run', 1)):              # run=0 暂停：不写驱动变量
                    if write:
                        _apply(d, V, errlog)
                    nframes += 1
        except Exception as e:                        # 单帧异常不致命：记录后跳过
            errlog.append(f'frame handling failed: {e}')
            continue
    return buf, nframes


def stats():
    """写变量统计：writes=实际 COM 调用次数，skipped=因值未变而跳过的次数，fails=失败次数。
    用于验证「减少 automation 调用」是否生效（skipped 应远大于 0）。"""
    return dict(_write_stats)


def reset_cache():
    """清空值/路径缓存，强制下次全部重写（变量被外部改动或模型重启后调用）。"""
    _last_written.clear()
    _path_cache.clear()
    _write_stats.update(writes=0, skipped=0, fails=0)


def probe(nmotor=NMOTOR):
    """诊断：检查所有目标变量路径是否可读写（非破坏性：读出原值再写回）。返回 (ok, fail)。"""
    V = _platform_vars()
    names = []
    for j in range(MOTOR_BASE, MOTOR_BASE + nmotor):
        names += [f'enable_{j}', f'angle_{j}', f'direction_{j}', f'trigger_{j}']
    names += [f'servo_angle{SERVO_SUFFIX}', f'servo_enable{SERVO_SUFFIX}']
    ok = fail = 0
    for nm in names:
        done = False
        for path in _candidate_paths(nm):
            try:
                val = V[path].ValueConverted
                V[path].ValueConverted = val        # 写回原值，验证可写
                _path_cache[nm] = path
                print(f'  [OK]   {nm} -> {path}  (= {val})')
                ok += 1
                done = True
                break
            except Exception as e:
                last = e
        if not done:
            print(f'  [FAIL] {nm} : {last}')
            fail += 1
    print(f'probe: ok={ok} fail={fail}')
    return ok, fail


class MotorClient:
    """后台线程版客户端：start() 立即返回，收帧/写变量在守护线程里做，UI 不卡。
    COM 跨线程：主线程 marshal Variables → 后台线程 unmarshal 使用。
    write=False 时只收帧/解析/计数、不写 dSPACE 变量（诊断用：判断卡顿是否来自 COM 写入）。
    """

    def __init__(self, host=HOST, port=PORT, write=True, log_path=None, readback_s=0.0,
                 readback_print=True):
        self.host = host
        self.port = port
        self._write = write
        self._log_path = log_path
        self._readback_s = float(readback_s)
        self._readback_print = bool(readback_print)
        self._last_readback = 0.0
        self._stop = threading.Event()
        self._errlog = []
        self._lock = threading.Lock()
        self._connected = False
        self._frames = 0
        self._stream = None
        self._th = threading.Thread(target=self._run, name='motor-client', daemon=True)

    # ---- 对外状态 ----
    @property
    def connected(self):
        return self._connected

    @property
    def frames(self):
        return self._frames

    @property
    def errors(self):
        return list(self._errlog)

    def start(self):
        # 开启写日志（UI 冻结时可在外部记事本查看写了什么/读回什么）
        if self._log_path:
            try:
                set_log(self._log_path)
            except Exception as e:
                self._errlog.append(f'set_log failed: {e}')
        # 在主线程(COM 已初始化)取 Variables 并 marshal 成可跨线程使用的流
        try:
            import pythoncom
            V = _platform_vars()
            self._stream = pythoncom.CoMarshalInterThreadInterfaceInStream(pythoncom.IID_IDispatch, V)
        except Exception as e:
            self._errlog.append(f'marshal vars failed: {e}')
            self._stream = None
        self._th.start()
        return self

    def stop(self):
        self._stop.set()

    def is_alive(self):
        return self._th.is_alive()

    # ---- 后台线程 ----
    def _run(self):
        co_ready = False
        try:
            import pythoncom
            pythoncom.CoInitialize()          # 后台线程必须自己初始化 COM 单元
            co_ready = True
        except Exception as e:
            self._errlog.append(f'CoInitialize failed: {e}')
            _emit(f'[client] CoInitialize failed: {e}', flush=True)
        try:
            V = None
            if self._stream is not None:
                try:
                    import pythoncom
                    raw = pythoncom.CoGetInterfaceAndReleaseStream(self._stream, pythoncom.IID_IDispatch)
                    V = _ensure_dynamic(raw)   # 重新包成动态对象，恢复 V[path] 下标访问
                except Exception as e:
                    self._errlog.append(f'unmarshal vars failed: {e}')
            if V is None:
                try:
                    V = _platform_vars()      # 兜底（跨线程可能报 wrong-thread）
                except Exception as e:
                    self._errlog.append(f'platform vars failed: {e}')
                    _emit(f'[client] platform vars failed: {e}', flush=True)
                    return
            self._serve_loop(V)
        finally:
            if co_ready:
                try:
                    import pythoncom
                    pythoncom.CoUninitialize()
                except Exception:
                    pass

    def _serve_loop(self, V):
        buf = b''
        while not self._stop.is_set():
            try:
                s = socket.create_connection((self.host, self.port), timeout=5)
            except OSError as e:
                self._errlog.append(f'connect {self.host}:{self.port} failed: {e}')
                _emit(f'[client] connect {self.host}:{self.port} failed: {e}', flush=True)
                if self._stop.wait(1.0):
                    break
                continue
            s.settimeout(1.0)                     # 让 stop() 能被及时响应
            self._connected = True
            buf = b''
            _emit(f'[client] connected {self.host}:{self.port}', flush=True)
            try:
                while not self._stop.is_set():
                    try:
                        chunk = s.recv(4096)
                    except socket.timeout:
                        # 空闲时按需回读变量当前值（确认写入生效；UI 冻结时看日志文件）
                        self._maybe_readback(V)
                        continue
                    except OSError as e:
                        self._errlog.append(f'recv: {e}')
                        break
                    if not chunk:
                        break
                    buf += chunk
                    buf = self._drain(buf, V)
                    self._maybe_readback(V)
            finally:
                try:
                    s.close()
                except OSError:
                    pass
                self._connected = False
                _emit('[client] disconnected', flush=True)
            if self._stop.is_set():
                break
            _emit('[client] reconnecting...', flush=True)
        _emit('[client] stopped', flush=True)

    def _drain(self, buf, V):
        """从 buf 解析出所有完整帧并写变量，返回未消费的余包。"""
        with self._lock:
            buf, n = _consume(buf, V, self._errlog, write=self._write)
        self._frames += n
        return buf

    def _maybe_readback(self, V):
        """按 readback_s 周期回读变量当前值。注意：后台线程读 COM 子对象可能拿不到值(全 None)，
        此时给出一次提示——验证请改在主线程手动调用 readback()。"""
        if self._readback_s <= 0:
            return
        now = time.time()
        if now - self._last_readback < self._readback_s:
            return
        self._last_readback = now
        try:
            with self._lock:
                rb = readback(V, verbose=False)
            vals = list(rb.values())
            if not any(v is not None for v in vals):
                # 线程内读不到（marshal 代理读子对象不可靠）——提示一次，改用主线程 readback()
                if not getattr(self, '_rb_warned', False):
                    self._rb_warned = True
                    _emit('[client] 线程内回读拿不到值(全 None)：请在主线程手动 readback() 确认；'
                          '写入本身以日志 W 行为准', flush=True)
                return
            if self._readback_print:
                ang = [rb.get(f'angle_{j}') for j in range(MOTOR_BASE, MOTOR_BASE + NMOTOR)]
                trg = [rb.get(f'trigger_{j}') for j in range(MOTOR_BASE, MOTOR_BASE + NMOTOR)]
                en = [rb.get(f'enable_{j}') for j in range(MOTOR_BASE, MOTOR_BASE + NMOTOR)]
                _emit(f'[readback] frames={self._frames} angle={ang} trigger={trg} enable={en} '
                      f'servo={rb.get("servo_angle" + SERVO_SUFFIX)}', flush=True)
        except Exception as e:
            self._errlog.append(f'readback failed: {e}')


def start_client(host=HOST, port=PORT, write=True, log_path=None, readback_s=0.0,
                 readback_print=True):
    """【后台线程版·推荐】立即返回(调用方不阻塞)，收帧/写变量在守护线程里做。
    log_path       : 若给出文件路径，则所有写变量/回读都追加到该文件（UI 冻结时可在外部查看）。
    readback_s     : >0 时每隔该秒数回读一次全部变量，确认写入是否真的生效。
    readback_print : 回读结果是否 print 到 Interpreter 控制台（界面不能动但输出仍会显示）。
    write=False    : 只收帧不写变量（诊断用）。
    """
    return MotorClient(host, port, write=write, log_path=log_path, readback_s=readback_s,
                       readback_print=readback_print).start()


def run_pumped(host=HOST, port=PORT, write=True):
    """【备用】单线程 + 消息泵：在 UI 线程里收帧 → 写变量 → 每轮 PumpWaitingMessages() 泵消息。
    本调用不返回，Ctrl+C 退出。write=False：只收帧不写变量（诊断用）。
    """
    import select
    try:
        import pythoncom
    except Exception as e:
        _emit(f'[client] 需要 pythoncom(pywin32)：{e}', flush=True)
        return
    V = _platform_vars()
    errlog = []
    buf = b''
    s = None
    frames = 0
    _emit('[client] run_pumped started (UI 消息泵模式)', flush=True)
    try:
        while True:
            if s is None:
                try:
                    s = socket.create_connection((host, port), timeout=5)
                    s.setblocking(False)
                    _emit(f'[client] connected {host}:{port}', flush=True)
                except OSError as e:
                    errlog.append(f'connect {host}:{port} failed: {e}')
                    pythoncom.PumpWaitingMessages()
                    time.sleep(0.5)
                    continue
            try:
                r, _, _ = select.select([s], [], [], 0.02)   # 短超时：保证高频泵消息
            except (OSError, ValueError):
                r = []
            if r:
                try:
                    chunk = s.recv(4096)
                except (BlockingIOError, OSError):
                    chunk = b''
                if not chunk:
                    try:
                        s.close()
                    except OSError:
                        pass
                    s = None
                    _emit('[client] disconnected; reconnecting', flush=True)
                else:
                    buf += chunk
                    buf, n = _consume(buf, V, errlog, write=write)
                    frames += n
            pythoncom.PumpWaitingMessages()                  # ★ 关键：泵消息，保持 UI 响应
    except KeyboardInterrupt:
        _emit(f'[client] stopped (frames={frames}, errors={len(errlog)})', flush=True)
    except Exception as e:
        # 兜底：任何意外异常都不让循环崩掉（否则连接被 reset、服务端报 Connection reset by peer）
        errlog.append(f'loop fatal: {e}')
        _emit(f'[client] loop fatal: {e}', flush=True)
    finally:
        try:
            if s is not None:
                s.close()
        except OSError:
            pass


def receive_loop(host=HOST, port=PORT):
    """【后台线程版·阻塞等待】启动后台线程并等待，Ctrl+C 退出。
    注意：写 COM 会被 marshal 回 UI 线程，某些 ControlDesk 环境会冻结界面；优先用 run_pumped()。
    """
    cli = start_client(host, port)
    try:
        while cli.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        cli.stop()
    return cli


if __name__ == '__main__':
    # 默认：后台线程版 start_client()（dSPACE FAQ098 官方建议用 Python 线程），立即返回、不阻塞 UI。
    # 其它模式：--wait 用阻塞等待(独立脚本)；--pumped 用单线程+消息泵(备用)。
    import sys
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    host = args[0] if len(args) > 0 else HOST
    port = int(args[1]) if len(args) > 1 else PORT
    if '--pumped' in sys.argv:
        run_pumped(host, port)
    elif '--wait' in sys.argv:
        receive_loop(host, port)
    else:
        cli = start_client(host, port)
        _emit('[client] started (thread mode). 查看: cli.connected / cli.frames / cli.errors / stats() / cli.stop()',
              flush=True)
