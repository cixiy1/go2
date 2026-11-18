"""没有视野时的感知源：只有机身遥测。

用它有两个场合：真机还没装相机，以及对照实验（同样的 V 层接口、同样的 L 层提示词，
只把视野拿掉，看 L 层的决策差多少）。

`origin` 会写成 `proprio`，`Percept.text()` 里也会明说「只有本体数据，看不到外部
环境」。这一条必须保留：模型看到一个没有视野的输入，至少会谨慎；如果装配得像
有视野、只是全部为空，它会得出「四周空旷」的结论，然后照常往前走。
"""

from __future__ import annotations

import math
import time

from quadruped.percept.types import Ground, Percept, SelfState


class ProprioPercept:
    name = "proprio"

    def __init__(self, backend, *, rate: float = 20.0):
        self.backend = backend
        self.period = 1.0 / rate
        self._last_pos = None
        self._last_t = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._stamp = 0.0

    def close(self) -> None:
        pass

    # 从两次位置差分估速度。控制端的 `status()` 里没有线速度，只能这么算。
    #
    # 用「机体系」再转：世界系的 dx 直接当 vx 会忽略航向，机器狗横着走时会被当成
    # 前进。这两个量差的不是精度，是方向。
    def _velocity(self, x: float, y: float, yaw: float, now: float) -> tuple[float, float]:
        if self._last_pos is None:
            self._last_pos = (x, y)
            self._last_t = now
            return 0.0, 0.0
        dt = now - self._last_t
        if dt < 1e-3:
            return self._vx, self._vy
        dx = (x - self._last_pos[0]) / dt
        dy = (y - self._last_pos[1]) / dt
        self._last_pos = (x, y)
        self._last_t = now
        # 低通，差分噪声大。时间常数 0.3 s，比策略的加速度快一点就够。
        a = math.exp(-dt / 0.3)
        c, s = math.cos(yaw), math.sin(yaw)
        vx = dx * c + dy * s
        vy = -dx * s + dy * c
        self._vx = a * self._vx + (1 - a) * vx
        self._vy = a * self._vy + (1 - a) * vy
        return self._vx, self._vy

    def read(self) -> Percept | None:
        b = self.backend
        try:
            pos = b.pos
            rpy = b.rpy
            height = b.height
        except Exception:                                   # noqa: BLE001
            return None
        if pos is None or rpy is None:
            return None

        now = time.monotonic()
        x, y = float(pos[0]), float(pos[1])
        yaw = float(rpy[2])
        vx, vy = self._velocity(x, y, yaw, now)
        self._stamp = now

        return Percept(
            stamp=now,
            origin=self.name,
            ego=SelfState(
                x=x, y=y,
                z=float(height) if height is not None else float(pos[2]),
                yaw=yaw, pitch=float(rpy[1]), roll=float(rpy[0]),
                vx=vx, vy=vy,
            ),
            rays=(),
            ground=Ground(),
            range_max=0.0,
        )
