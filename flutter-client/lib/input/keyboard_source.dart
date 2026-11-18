import 'dart:async';

import 'package:flutter/services.dart';

import 'gamepad_source.dart' show PadAction;
import 'intent.dart';

// 键盘遥控。桌面和网页上最省事的一套：WASD 平移、Q/E 转向，空格跳，Esc 急停。
//
// 存在的意义不只是「没手柄时能用」——调试时它是唯一能在不碰手机的情况下
// 反复触发一条指令的输入源。
class KeyboardSource {
  KeyboardSource(this.router);

  final InputRouter router;
  final Set<LogicalKeyboardKey> _keys = {};
  final StreamController<PadAction> _actions =
      StreamController<PadAction>.broadcast();
  Stream<PadAction> get actions => _actions.stream;

  // 不能写成 const 集合：LogicalKeyboardKey 重载了 ==，Dart 不允许它出现在
  // 常量集合里。改成 final 的静态集合，只在首次访问时建一次。
  static final _up = {LogicalKeyboardKey.keyW, LogicalKeyboardKey.arrowUp};
  static final _down = {LogicalKeyboardKey.keyS, LogicalKeyboardKey.arrowDown};
  static final _left = {LogicalKeyboardKey.keyA, LogicalKeyboardKey.arrowLeft};
  static final _right = {
    LogicalKeyboardKey.keyD,
    LogicalKeyboardKey.arrowRight,
  };
  static final _ccw = {LogicalKeyboardKey.keyQ};
  static final _cw = {LogicalKeyboardKey.keyE};

  bool _attached = false;

  void attach() {
    if (_attached) return;
    HardwareKeyboard.instance.addHandler(_onKey);
    _attached = true;
  }

  void detach() {
    if (!_attached) return;
    HardwareKeyboard.instance.removeHandler(_onKey);
    _attached = false;
    _keys.clear();
    router.keyboard.value = ControlIntent.zero;
  }

  bool _onKey(KeyEvent event) {
    final key = event.logicalKey;
    final known =
        _up.contains(key) ||
        _down.contains(key) ||
        _left.contains(key) ||
        _right.contains(key) ||
        _ccw.contains(key) ||
        _cw.contains(key) ||
        key == LogicalKeyboardKey.space ||
        key == LogicalKeyboardKey.escape;
    if (!known) return false;

    if (event is KeyDownEvent) {
      _keys.add(key);
      // 空格和 Esc 是「一次性动作」，按下就发，不等抬起。
      if (key == LogicalKeyboardKey.space) _actions.add(PadAction.jump);
      if (key == LogicalKeyboardKey.escape) _actions.add(PadAction.stop);
      _emit();
      return true;
    }
    if (event is KeyUpEvent) {
      _keys.remove(key);
      _emit();
      return true;
    }
    return false;
  }

  void _emit() {
    double axis(Set<LogicalKeyboardKey> pos, Set<LogicalKeyboardKey> neg) {
      final p = _keys.any(pos.contains) ? 1.0 : 0.0;
      final n = _keys.any(neg.contains) ? 1.0 : 0.0;
      return p - n;
    }

    router.keyboard.value = ControlIntent(
      vx: axis(_up, _down),
      vy: axis(_left, _right), // 机器人 vy 正为左，A 键就是 +1
      vyaw: axis(_ccw, _cw),
      source: '键盘',
    );
  }

  void dispose() {
    detach();
    _actions.close();
  }
}
