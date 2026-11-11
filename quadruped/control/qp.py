"""OSQP 的薄封装：结构固定的稠密 QP。

    min 0.5·xᵀ·H·x + gᵀ·x      s.t.  l ≤ A·x ≤ u

用法是「每帧 H / g / A / l / u 全换，结构不变」。看着简单，OSQP 的 Python
接口有两个坑，两个都是静默的，不会报错，只会给出错的解 —— 所以单独封一层。

## 坑一：`update(P=obj)` 在原地改完之后不生效

    self._P.data[:] = H.ravel()      # 改的是同一个对象
    solver.update(P=self._P)         # ← 什么也没发生

osqp-python 拿新矩阵和缓存的那份比，同一个对象永远「相等」，于是判定
「没变」直接跳过。表现是求解器一直用第一帧的 Hessian：WBC 里表现为力矩
限幅被违反 7 N·m（硬约束、状态还报 solved），MPC 里更隐蔽 —— 力几乎一样，
悬停测试照样过。所以这里走 `Px` / `Ax` 数值接口，永远不传矩阵对象。

## 坑二：`P` 只存上三角，`Px` 的长度是 n(n+1)/2

顺序是**上三角的列优先**：(0,0), (0,1), (1,1), (0,2), (1,2), (2,2)…
传全矩阵会报 "new number of elements out of bounds"；顺序算错不会报错，
只会解出一个奇怪的 x。`A` 没有这个限制，全稠密，顺序就是列优先。
"""

from __future__ import annotations

import numpy as np
import osqp
import scipy.sparse as sp


class DenseQP:
    """结构固定的稠密 QP。第一次 `solve()` 建求解器，之后只喂数值。"""

    def __init__(self, nx: int, nrow: int, *, eps_abs: float = 1e-6,
                 eps_rel: float = 1e-6, max_iter: int = 8000,
                 polish: bool = False):
        self.nx = int(nx)
        self.nrow = int(nrow)
        self.eps_abs = float(eps_abs)
        self.eps_rel = float(eps_rel)
        self.max_iter = int(max_iter)
        self.polish = bool(polish)

        # P 的上三角，列优先。`_px_idx` 是这些位置在 H（行优先展开）里的下标。
        ij = np.array([(i, j) for j in range(self.nx) for i in range(j + 1)],
                      dtype=int)
        self._px_idx = ij[:, 0] * self.nx + ij[:, 1]
        # 模板矩阵用**全 1**：带 0 的话 csc 会把位置丢掉，结构就跟 Px 对不上了
        self._P = sp.triu(sp.csc_matrix(np.ones((self.nx, self.nx))),
                          format="csc")
        self._A = sp.csc_matrix(np.ones((self.nrow, self.nx)))
        if self._P.data.size != self._px_idx.size:
            raise RuntimeError("P 的稀疏结构与 Px 下标对不上")
        if self._A.data.size != self.nrow * self.nx:
            raise RuntimeError("A 的稀疏结构不是全稠密")

        self._solver = None
        self.last_info = {}

    @property
    def ready(self) -> bool:
        return self._solver is not None

    def solve(self, H, g, A, l, u):
        # OSQP 里 H 必须对称；这里兜一下，省得调用方忘了对称化
        H = 0.5 * (H + H.T)
        Px = np.ascontiguousarray(H.ravel()[self._px_idx])
        g = np.ascontiguousarray(g, dtype=float)
        l = np.ascontiguousarray(l, dtype=float)
        u = np.ascontiguousarray(u, dtype=float)
        # `A=None` 表示「跟上一帧一样」，省掉一次 (nrow × nx) 的拷贝。
        # 结构固定、数值也固定的场合（MPC 的约束矩阵就是这样）用它。
        Ax = None if A is None else np.ascontiguousarray(A.ravel(order="F"))

        if self._solver is None:
            if Ax is None:
                raise ValueError("第一次 solve() 必须给 A")
            self._P.data[:] = Px
            self._A.data[:] = Ax
            self._solver = osqp.OSQP()
            self._solver.setup(P=self._P, q=g, A=self._A, l=l, u=u,
                               verbose=False, eps_abs=self.eps_abs,
                               eps_rel=self.eps_rel, max_iter=self.max_iter,
                               warm_start=True, polish=self.polish)
        elif Ax is None:
            self._solver.update(Px=Px, q=g, l=l, u=u)
        else:
            self._solver.update(Px=Px, Ax=Ax, q=g, l=l, u=u)

        res = self._solver.solve()
        self.last_info = dict(status=res.info.status,
                              status_val=int(res.info.status_val),
                              obj=float(res.info.obj_val),
                              iter=int(res.info.iter),
                              pri_res=float(res.info.prim_res),
                              dua_res=float(res.info.dual_res),
                              run_ms=1e3 * float(res.info.run_time))
        return np.asarray(res.x, dtype=float), self.last_info
