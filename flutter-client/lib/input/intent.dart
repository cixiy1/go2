import 'package:flutter/foundation.dart';

import '../core/settings.dart';

// 控制意图：所有输入源（触摸摇杆、手柄、键盘）都产出这一个结构。
//
// 这是整个 App 的抽象层。摇杆和手柄的区别只到「谁产生 -1~1 的数」为止，
// 往下一律是「把意图按 20 Hz 转发给机器狗」。加一种输入（比如手机重力感应）
// 不需要碰传输、模式和界面，只写一个产出 ControlIntent 的源。
class ControlIntent {
  const ControlIntent({
    this.vx = 0,
    this.vy = 0,
    this.vyaw = 0,
    this.source = '-',
  });

  final double vx; // 前后，正为前
  final double vy; // 左右，正为左
  final double vyaw; // 转向，正为逆时针
  final String source;

  static const zero = ControlIntent();

  bool get isZero => vx.abs() < 1e-3 && vy.abs() < 1e-3 && vyaw.abs() < 1e-3;

  double get magnitude {
    final m = vx.abs() > vy.abs() ? vx.abs() : vy.abs();
    return m > vyaw.abs() ? m : vyaw.abs();
  }

  @override
  String toString() =>
      'ControlIntent(vx: ${vx.toStringAsFixed(2)}, vy: ${vy.toStringAsFixed(2)}, '
      'vyaw: ${vyaw.toStringAsFixed(2)}, $source)';
}

// 多路输入的合并器。
//
// 规则只有一条：**幅度最大的那一路赢**。不做加权求和 —— 手柄回中不准时会有
// 0.05 的漂移，加上摇杆的 0.5，狗会以 0.55 走；用户看到的是「我明明只推了
// 一半」。幅度赢家制下，漂移永远输给真的在动的那一路。
class InputRouter {
  InputRouter(this.settings);

  final AppSettings settings;

  final ValueNotifier<ControlIntent> touch = ValueNotifier(ControlIntent.zero);
  final ValueNotifier<ControlIntent> gamepad = ValueNotifier(
    ControlIntent.zero,
  );
  final ValueNotifier<ControlIntent> keyboard = ValueNotifier(
    ControlIntent.zero,
  );

  final ValueNotifier<ControlIntent> merged = ValueNotifier(ControlIntent.zero);

  void attach() {
    touch.addListener(_merge);
    gamepad.addListener(_merge);
    keyboard.addListener(_merge);
  }

  void dispose() {
    touch.removeListener(_merge);
    gamepad.removeListener(_merge);
    keyboard.removeListener(_merge);
    touch.dispose();
    gamepad.dispose();
    keyboard.dispose();
    merged.dispose();
  }

  void _merge() {
    final candidates = [gamepad.value, keyboard.value, touch.value];
    var best = ControlIntent.zero;
    // 手柄优先：它是最精确的一路，而且物理回中比手指离开屏幕可靠。
    for (final c in candidates) {
      if (c.magnitude > best.magnitude + 1e-3) best = c;
    }
    merged.value = best;
  }

  // 手柄/键盘给的是 -1~1 的原始值，这里统一过一遍死区和曲线。
  ControlIntent shaped(double vx, double vy, double vyaw, String source) {
    final s = settings;
    return ControlIntent(
      vx: s.shape(vx),
      vy: s.invertVy ? -s.shape(vy) : s.shape(vy),
      vyaw: -s.shape(vyaw), // 手柄右推 = 顺时针，机器人 vyaw 正为逆时针
      source: source,
    );
  }
}
