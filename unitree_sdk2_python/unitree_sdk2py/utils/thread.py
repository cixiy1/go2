import sys
import os
import time
import errno
import ctypes
import struct
import threading

from .future import Future

# timerfd 是 Linux 专有接口。原实现在 import 阶段就 dlsym 失败，
# 导致整个 unitree_sdk2py.utils.thread 无法导入 —— macOS 上任何用到
# RecurrentThread 的程序（包括 unitree_mujoco 的 bridge）都起不来。
# 这里做成可选：Linux 仍走 timerfd 精确定时，其他平台降级到软件定时。
try:
    from .timerfd import *
    _HAS_TIMERFD = True
except (ImportError, AttributeError, OSError):
    _HAS_TIMERFD = False

class Thread(Future):
    def __init__(self, target = None, name = None, args = (), kwargs = None):
        super().__init__()
        self.__target = target
        self.__args = args
        self.__kwargs = {} if kwargs is None else kwargs
        self.__thread = threading.Thread(target=self.__ThreadFunc, name=name, daemon=True)

    def Start(self):
        return self.__thread.start()
    
    def GetId(self):
        return self.__thread.ident
    
    def GetNativeId(self):
        return self.__thread.native_id

    def __ThreadFunc(self):
        value = None
        try:
            value = self.__target(*self.__args, **self.__kwargs)
            self.Ready(value)
        except:
            info = sys.exc_info() 
            self.Fail(f"[Thread] target func raise exception: name={info[0].__name__}, args={str(info[1].args)}")

class RecurrentThread(Thread):
    def __init__(self, interval: float = 1.0, target = None, name = None, args = (), kwargs = None):
        self.__quit = False
        self.__inter = interval
        self.__loopTarget = target
        self.__loopArgs = args
        self.__loopKwargs = {} if kwargs is None else kwargs

        if interval is None or interval <= 0.0:
            super().__init__(target=self.__LoopFunc_0, name=name)
        else:
            super().__init__(target=self.__LoopFunc, name=name)

    def Wait(self, timeout: float = None):
        self.__quit = True
        super().Wait(timeout)

    def __LoopFunc(self):
        if not _HAS_TIMERFD:
            self.__LoopFuncPortable()
            return

        # clock type CLOCK_MONOTONIC = 1
        tfd = timerfd_create(1, 0)
        spec = itimerspec.from_seconds(self.__inter, self.__inter)
        timerfd_settime(tfd, 0, ctypes.byref(spec), None)

        while not self.__quit:
            try:
                self.__loopTarget(*self.__loopArgs, **self.__loopKwargs)
            except:
                info = sys.exc_info()
                print(f"[RecurrentThread] target func raise exception: name={info[0].__name__}, args={str(info[1].args)}")

            try:
                buf = os.read(tfd, 8)
                # print(struct.unpack("Q", buf)[0])
            except OSError as e:
                if e.errno != errno.EAGAIN:
                    raise e

        os.close(tfd)

    def __LoopFuncPortable(self):
        """非 Linux 平台的定拍循环。

        用绝对时间累加而不是「执行完再 sleep(interval)」，避免周期随执行
        耗时漂移。落后超过一个周期就重设基准，防止追帧风暴。

        末尾一小段用忙等而不是 sleep：macOS / Windows 的 sleep 精度只有
        1~5 ms，200 Hz 的周期就是 5 ms —— 全交给 sleep 的话实际周期会在
        2~10 ms 之间乱跳。这条回路直接算关节力矩，周期一抖，kp·(q-q) 和
        kd·dq 就跟着抖，真机上听得到电机啸叫、看得见腿在颤。忙等只占
        最后 1 ms，CPU 代价可以接受。
        """
        next_t = time.perf_counter()
        while not self.__quit:
            try:
                self.__loopTarget(*self.__loopArgs, **self.__loopKwargs)
            except:
                info = sys.exc_info()
                print(f"[RecurrentThread] target func raise exception: name={info[0].__name__}, args={str(info[1].args)}")

            next_t += self.__inter
            delay = next_t - time.perf_counter()
            if delay > 0.0015:
                # 预留 1 ms 给忙等，剩下的交给 sleep
                time.sleep(delay - 0.001)
            while time.perf_counter() < next_t:
                pass
            if time.perf_counter() - next_t > self.__inter:
                next_t = time.perf_counter()
    
    def __LoopFunc_0(self):
        while not self.__quit:
            try:
                self.__loopTarget(*self.__args, **self.__kwargs)
            except:
                info = sys.exc_info() 
                print(f"[RecurrentThread] target func raise exception: name={info[0].__name__}, args={str(info[1].args)}")

