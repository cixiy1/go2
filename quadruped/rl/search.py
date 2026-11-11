"""CMA-ES 搜步态参数。

搜的是 `gait.py` 里的 8 维向量，目标函数是 `env.py` 的 score（速度、姿态、
力矩、高度四项加权）。无梯度，不要求可微，适合 8 维这种小空间。

    # 单个 episode 自检
    python -m quadruped.rl.env --vx 0.25 --seconds 6

    # 搜 40 代，每代 12 个样本，平地 0.25 m/s
    python -m quadruped.rl.search --gens 40 --pop 12 --vx 0.25

    # 只在 5° 横坡上搜
    python -m quadruped.rl.search --gens 40 --slope 5

结果写到 `quadruped/rl/out/`，同时打印最优参数与指标。**搜完必须回
`test/gait_speed.py` 复核** —— 这个环境没有机身层 wrench 分配与 MPC/WBC，
只是同一套足端数学的简化被控对象。
"""
from __future__ import annotations

import argparse
import json
import os
import time
from dataclasses import asdict

import numpy as np

from .env import GaitEnv
from .gait import BASELINE, BOUNDS, FIELDS, GaitSpec

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")


# 标准 CMA-ES（Hansen 2016），n 步长与协方差自适应。
#
# 不引第三方依赖：`cma` 包体量不大，但训练环境要能离线复现，自己写 70 行
# 更可控，也便于后面加约束（比如把 period 锁死）。
class CMAES:
    def __init__(self, n: int, x0, sigma: float = 0.25,
                 popsize: int | None = None, seed: int = 0):
        self.n = n
        self.rng = np.random.default_rng(seed)
        self.mean = np.asarray(x0, dtype=float).copy()
        self.sigma = float(sigma)
        self.lam = int(popsize or 4 + int(3 * np.log(n)))
        self.mu = self.lam // 2

        w = np.log(self.mu + 0.5) - np.log(np.arange(1, self.mu + 1))
        self.w = w / w.sum()
        self.mueff = 1.0 / float((self.w ** 2).sum())

        n_ = float(n)
        self.cc = (4 + self.mueff / n_) / (n_ + 4 + 2 * self.mueff / n_)
        self.cs = (self.mueff + 2) / (n_ + self.mueff + 5)
        self.c1 = 2 / ((n_ + 1.3) ** 2 + self.mueff)
        self.cmu = min(1 - self.c1,
                       2 * (self.mueff - 2 + 1 / self.mueff) / ((n_ + 2) ** 2 + self.mueff))
        self.damps = 1 + 2 * max(0.0, np.sqrt((self.mueff - 1) / (n_ + 1)) - 1) + self.cs
        self.chiN = np.sqrt(n_) * (1 - 1 / (4 * n_) + 1 / (21 * n_ ** 2))

        self.C = np.eye(n)
        self.pc = np.zeros(n)
        self.ps = np.zeros(n)
        self.gen = 0
        self._eig = (np.ones(n), np.eye(n))

    # 采样一代：返回 (候选, 步长)，候选是 [0,1]^n 里被夹住的值。
    def ask(self):
        d, B = self._eig
        z = self.rng.standard_normal((self.lam, self.n))
        y = z @ (B * d).T
        x = self.mean + self.sigma * y
        return np.clip(x, 0.0, 1.0), y

    # 用这一代的适应度更新分布（最大化）。
    def tell(self, xs, fs):
        order = np.argsort(-np.asarray(fs))
        xs = np.asarray(xs)[order]
        x_old = self.mean.copy()

        sel = xs[:self.mu]
        self.mean = self.w @ sel

        y_w = (self.mean - x_old) / max(self.sigma, 1e-12)
        # _eig[0] 已经是 sqrt(特征值)，invsqrtC = B · diag(1/d) · Bᵀ。
        # 特征值退化到 0 时 1/d 会溢出，钳在 1e-6 上。
        d, B = self._eig
        invsqrtC = (B / np.maximum(d, 1e-6)) @ B.T

        self.ps = ((1 - self.cs) * self.ps
                   + np.sqrt(self.cs * (2 - self.cs) * self.mueff) * (invsqrtC @ y_w))
        hsig = (np.linalg.norm(self.ps)
                / np.sqrt(1 - (1 - self.cs) ** (2 * (self.gen + 1)))
                / self.chiN) < (1.4 + 2 / (self.n + 1))
        self.pc = ((1 - self.cc) * self.pc
                   + hsig * np.sqrt(self.cc * (2 - self.cc) * self.mueff) * y_w)

        artmp = (sel - x_old) / max(self.sigma, 1e-12)
        self.C = ((1 - self.c1 - self.cmu) * self.C
                  + self.c1 * (np.outer(self.pc, self.pc)
                               + (1 - hsig) * self.cc * (2 - self.cc) * self.C)
                  + self.cmu * (artmp.T * self.w) @ artmp)
        self.sigma *= np.exp((self.cs / self.damps)
                             * (np.linalg.norm(self.ps) / self.chiN - 1))
        self.sigma = float(np.clip(self.sigma, 1e-3, 0.5))
        self.gen += 1

        self.C = (self.C + self.C.T) / 2
        evals, evecs = np.linalg.eigh(self.C)
        evals = np.maximum(evals, 1e-12)
        self._eig = (np.sqrt(evals), evecs)


# 多速度合并：一套步态要覆盖一个速度窗口，不能只对单个指令调优。
#
# 展示用的指标取中间那速，评分取各速平均，姿态与力矩取最差 —— 宁可保守，
# 也不让优化器挑一个「只在某一速好看」的解。
def _combine(runs: list[dict]) -> dict:
    if len(runs) == 1:
        return runs[0]
    m = dict(runs[len(runs) // 2])
    m["score"] = float(np.mean([r["score"] for r in runs]))
    m["fell"] = any(r["fell"] for r in runs)
    m["vx"] = float(np.mean([r["vx"] for r in runs]))
    m["vx_ratio"] = float(np.mean([r["vx_ratio"] for r in runs]))
    m["roll_pp"] = max(r["roll_pp"] for r in runs)
    m["pitch_pp"] = max(r["pitch_pp"] for r in runs)
    m["tau_p95"] = max(r["tau_p95"] for r in runs)
    m["per_vx"] = [{k: r[k] for k in ("slope", "vx_cmd", "vx", "vx_ratio",
                                     "roll_pp", "fell", "score")
                    if k in r} for r in runs]
    return m


# 评估一个归一化向量：映射成物理参数 → 跑一个或多个速度的 episode → 汇总。
def evaluate(x, cfg: dict) -> dict:
    spec = GaitSpec.from_vector(x, bounds=cfg.get("bounds"))
    if cfg.get("lock"):
        # 锁死某几维：用于验证「某个量是不是必须自由」。典型是 stride_gain ——
        # 它大于 1 就是「脚比机身跑得快」，落地速度必然对不上，控制器自带的
        # `_check_gait` 会判不合格（bio 那档也是这么不合格的）。
        spec = GaitSpec(**{**spec.as_dict(), **cfg["lock"]})
    vxs = cfg.get("vx_list") or [cfg["vx"]]
    slopes = cfg.get("slope_list") or [cfg["slope"]]
    runs = []
    for slope in slopes:
        for vx in vxs:
            env = GaitEnv(vx=float(vx), seconds=cfg["seconds"],
                          slope_deg=float(slope), seed=cfg.get("seed", 0),
                          motor_bw=cfg.get("motor_bw", 0.0),
                          kp=cfg.get("kp", 100.0), kd=cfg.get("kd", 1.5),
                          w_speed=cfg.get("w_speed", 0.45),
                          w_stab=cfg.get("w_stab", 0.30),
                          w_yaw=cfg.get("w_yaw", 0.0),
                          push=cfg.get("push", 0.0),
                          rough=cfg.get("rough", 0.0),
                          rough_seed=cfg.get("rough_seed", 0),
                          slip_asym=cfg.get("slip_asym", 0.0))
            r = env.run(spec)
            r["slope"] = float(slope)
            runs.append(r)
    m = _combine(runs)
    m["vector"] = [float(v) for v in x]
    return m


# 并行评估。每个进程自己建 env，避免共享 MjData。
def _eval_worker(args):
    x, cfg = args
    return evaluate(x, cfg)


def _pool_map(fn, items, workers):
    if workers and workers > 1:
        import multiprocessing as mp
        with mp.get_context("spawn").Pool(workers) as pool:
            return pool.map(fn, items)
    return [fn(it) for it in items]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--gens", type=int, default=30)
    ap.add_argument("--pop", type=int, default=12)
    ap.add_argument("--vx", type=float, default=0.25, help="指令速度 m/s")
    ap.add_argument("--vx-list", default=None,
                    help="多速度联合评估，如 0.15,0.25,0.35（给了就覆盖 --vx）")
    ap.add_argument("--seconds", type=float, default=6.0, help="单个 episode 时长")
    ap.add_argument("--slope", type=float, default=0.0, help="横坡角度 deg")
    ap.add_argument("--slope-list", default=None,
                    help="多坡度联合评估，如 0,5（给了就覆盖 --slope，与 --vx-list 交叉组合）")
    ap.add_argument("--sigma", type=float, default=0.20, help="初始步长（归一化空间）")
    ap.add_argument("--kd", type=float, default=1.5,
                    help="搜索时用的微分增益，必须与部署侧一致")
    ap.add_argument("--kp", type=float, default=100.0)
    ap.add_argument("--motor-bw", type=float, default=0.0,
                    help="力矩一阶滞后带宽 Hz；0 = 理想（当帧到位）。"
                         "40 接近真机，开它会逼优化器留阻尼余量")
    ap.add_argument("--w-speed", type=float, default=0.45, help="评分权重：速度")
    ap.add_argument("--w-stab", type=float, default=0.30, help="评分权重：姿态")
    ap.add_argument("--w-yaw", type=float, default=0.0,
                    help="评分权重：偏航保持（0 = 关，仿真里 yaw 恒为 0 时无意义）")
    ap.add_argument("--push", type=float, default=0.0,
                    help="中段施加的侧向+偏航冲量 N·s；配合 --w-yaw 才算得出偏航稳定")
    ap.add_argument("--rough", type=float, default=0.0,
                    help="地面起伏高度 m（0.035 实测能区分抬脚高低：40mm 掉速 28%%，80mm 掉 18%%）")
    ap.add_argument("--slip-asym", type=float, default=0.0,
                    help="每次落地的随机偏航冲量 N·m·s（1.5 时 yaw 漂 90°，2 N·s 侧推只有 2~6°）")
    ap.add_argument("--workers", type=int, default=0, help=">1 开多进程")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--init", default="baseline", choices=("baseline", "random"))
    ap.add_argument("--resume", nargs="?", const="__best__", default=None, metavar="PATH",
                    help="从已有结果热启动；不给路径则读 out/gait_best.json")
    ap.add_argument("--tag", default=None, help="输出文件名后缀")
    ap.add_argument("--promote", action="store_true",
                    help="把最优参数写成 out/gait_best.json，供 make_gait('search') 读取")
    ap.add_argument("--lock", action="append", default=None, metavar="K=V",
                    help="锁死某一维，如 --lock stride_gain=1.0（可重复）")
    ap.add_argument("--bound", action="append", default=None, metavar="K=LO,HI",
                    help="临时收窄某一维的搜索范围，如 --bound duty=0.45,0.62（可重复）")
    a = ap.parse_args(argv)

    lock = {}
    for item in (a.lock or []):
        k, _, v = item.partition("=")
        if k not in FIELDS:
            ap.error(f"--lock 的键不认识：{k}，只有 {FIELDS}")
        lock[k] = float(v)

    vx_list = ([float(v) for v in a.vx_list.split(",")]
               if a.vx_list else None)
    slope_list = ([float(v) for v in a.slope_list.split(",")]
                  if a.slope_list else None)
    bounds = {}
    for item in (a.bound or []):
        k, _, rng = item.partition("=")
        if k not in FIELDS:
            ap.error(f"--bound 的键不认识：{k}，只有 {FIELDS}")
        lo_s, _, hi_s = rng.partition(",")
        bounds[k] = (float(lo_s), float(hi_s))
    cfg = dict(vx=a.vx, seconds=a.seconds, slope=a.slope, seed=a.seed,
               vx_list=vx_list, slope_list=slope_list, lock=lock,
               bounds=bounds or None,
               kp=a.kp, kd=a.kd, motor_bw=a.motor_bw,
               w_speed=a.w_speed, w_stab=a.w_stab, w_yaw=a.w_yaw, push=a.push,
               rough=a.rough, rough_seed=a.seed, slip_asym=a.slip_asym)

    # 热启动：从已有最优继续搜，而不是每次都退回 BASELINE。
    # CMA-ES 的协方差矩阵不跨进程保存，所以这里是「重启式热启动」——
    # 均值落在旧最优上，步长给一个小值，靠 --sigma 控制搜索半径。
    if a.resume:
        path = (os.path.join(OUT_DIR, "gait_best.json") if a.resume == "__best__"
                else a.resume)
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        sd = d.get("spec") or d
        x0 = GaitSpec(**{k: float(v) for k, v in sd.items() if k in FIELDS}).to_vector()
        print(f"热启动自 {path}（score {d.get('score', float('nan')):.4f}）", flush=True)
    elif a.init == "baseline":
        x0 = GaitSpec(**BASELINE).to_vector()
    else:
        x0 = np.random.default_rng(a.seed).random(len(FIELDS))
    es = CMAES(len(FIELDS), x0, sigma=a.sigma, popsize=a.pop, seed=a.seed)

    os.makedirs(OUT_DIR, exist_ok=True)
    t0 = time.time()
    best = None
    history = []

    for g in range(a.gens):
        xs, _ = es.ask()
        res = _pool_map(_eval_worker, [(x, cfg) for x in xs], a.workers)
        fs = [r["score"] for r in res]
        es.tell(xs, fs)

        i = int(np.argmax(fs))
        if best is None or res[i]["score"] > best["score"]:
            best = res[i]
        history.append(dict(gen=g, best=float(max(fs)),
                            mean=float(np.mean(fs)), sigma=es.sigma))
        print(f"gen {g:3d}  best {max(fs):.4f}  mean {np.mean(fs):.4f}  "
              f"sigma {es.sigma:.3f}  vx {res[i]['vx']:+.3f}  "
              f"roll {res[i]['roll_pp']:.3f}  fell {res[i]['fell']}  "
              f"({time.time() - t0:.0f}s)", flush=True)

    # 落盘用的是「真正被评估的那套参数」：锁死的维要按锁值写，
    # 否则文件里记的是搜索空间里的原始值，和实际跑出来的不是一套。
    spec = GaitSpec.from_vector(best["vector"], bounds=bounds or None)
    if lock:
        spec = GaitSpec(**{**spec.as_dict(), **lock})
    out = dict(cfg=cfg, spec=spec.as_dict(), best=best, history=history,
               gens=a.gens, pop=a.pop, seconds_used=round(time.time() - t0, 1))
    tag = "mix" if (slope_list and len(slope_list) > 1) else (
        "slope" if (slope_list or a.slope) else "flat")
    name = f"gait_{tag}_{a.vx:.2f}_{a.tag or int(time.time())}.json"
    path = os.path.join(OUT_DIR, name)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)

    print("\n=== 最优 ===")
    for k in FIELDS:
        print(f"  {k:14s} {getattr(spec, k):.4f}")
    print(f"  {'score':14s} {best['score']:.4f}")
    print(f"  {'vx / dx':14s} {best['vx']:.3f} m/s / {best['dx']:.2f} m")
    print(f"  {'roll/pitch pp':14s} {best['roll_pp']:.3f} / {best['pitch_pp']:.3f} rad")
    if best.get("per_vx"):
        print("\n=== 分项 ===")
        for r in best["per_vx"]:
            print(f"  slope {r.get('slope', 0):>4.1f}°  vx {r['vx_cmd']:.2f} -> "
                  f"{r['vx']:+.3f} ({r['vx_ratio'] * 100:3.0f}%)  "
                  f"roll {r['roll_pp'] * 57.3:5.1f}°  "
                  f"{'摔' if r['fell'] else '稳'}  score {r['score']:.3f}")
    print("\n=== 贴进 make_gait() ===")
    print("        return _gp(" + ",\n                  ".join(
        f"{k}={getattr(spec, k):.4f}" for k in FIELDS) + ',\n                  name="search")')
    print(f"\n写入 {path}")
    if a.promote:
        best_path = os.path.join(OUT_DIR, "gait_best.json")
        with open(best_path, "w", encoding="utf-8") as f:
            json.dump(dict(spec=spec.as_dict(), source=os.path.basename(path),
                           cfg=cfg, score=best["score"]), f,
                      ensure_ascii=False, indent=2)
        print(f"已提升为 make_gait('search') 的来源：{best_path}")
    print("下一步：test/gait_speed.py --style search 在 DDS 仿真里复核")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
