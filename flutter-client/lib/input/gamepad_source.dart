import 'dart:async';
import 'dart:ui' show Offset;

import 'package:flutter/foundation.dart';
import 'package:gamepads/gamepads.dart';

import '../core/settings.dart';
import 'intent.dart';

// 手柄按键 → App 动作。手柄只负责「产生按下事件」，具体做什么由 App 决定：
// 这样换一个手柄、换一套映射，都不用碰传输层。
enum PadAction { arm, stop, jump, stand, crouch, gait, speedUp, speedDown }

// 物理手柄。`gamepads` 包把六个平台的差异抹平成归一化轴/键：
// 左摇杆 = 平移，右摇杆横向 = 转向。
//
// 只用归一化事件、不用原始事件：原始键名在 macOS 上是 `a.circle`、Android 是
// `KEYCODE_BUTTON_A`、Web 是 `button 0`，直接写死等于只支持一个平台。
class GamepadSource {
  GamepadSource(this.router, this.settings);

  final InputRouter router;
  final AppSettings settings;

  final ValueNotifier<List<String>> pads = ValueNotifier(const []);
  final ValueNotifier<String?> lastEvent = ValueNotifier(null);
  final ValueNotifier<Map<String, double>> axes = ValueNotifier(const {});

  //: 按下的键，键名用 `GamepadButton.name`（a / b / dpadUp / start …）。
  //: 手柄页要画「哪个键亮着」，只有轴值和速度意图不够 —— 起立、换步态这些
  //: 动作不出现在 vx/vy/vyaw 里。
  final ValueNotifier<Set<String>> pressed = ValueNotifier(const {});

  //: 两个摇杆的归一化位置（x 右正、y 下正）。屏幕坐标系，和 Joystick 一致。
  final ValueNotifier<Offset> leftStick = ValueNotifier(Offset.zero);
  final ValueNotifier<Offset> rightStick = ValueNotifier(Offset.zero);
  final ValueNotifier<String> status = ValueNotifier('未检测到手柄');

  final StreamController<PadAction> _actions =
      StreamController<PadAction>.broadcast();
  Stream<PadAction> get actions => _actions.stream;

  final List<StreamSubscription<dynamic>> _subs = [];
  final Map<GamepadButton, bool> _down = {};
  final Map<String, double> _axisValues = {};

  double _lx = 0;
  double _ly = 0;
  double _rx = 0;
  double _rY = 0;

  bool get connected => pads.value.isNotEmpty;

  Future<void> start() async {
    try {
      final list = await Gamepads.list();
      _setPads(list.map((g) => g.name).toList());
      _subs.add(Gamepads.normalizedEvents.listen(_onNormalized));
      _subs.add(
        Gamepads.onConnected.listen((e) {
          _setPads([...pads.value, e.name]);
          status.value = '${e.name} 已接入';
        }),
      );
      _subs.add(
        Gamepads.onDisconnected.listen((e) {
          _setPads(pads.value.where((n) => n != e.name).toList());
          _reset();
          status.value = '${e.name} 已断开';
        }),
      );
    } catch (e) {
      // Web 上要用户先按一下手柄才拿得到（浏览器要求用户手势）。
      status.value = '手柄不可用：$e';
    }
  }

  void _setPads(List<String> names) {
    pads.value = names;
    if (names.isNotEmpty) status.value = names.join('、');
  }

  void _reset() {
    _lx = _ly = _rx = _rY = 0;
    _down.clear();
    _axisValues.clear();
    axes.value = const {};
    pressed.value = const {};
    leftStick.value = Offset.zero;
    rightStick.value = Offset.zero;
    router.gamepad.value = ControlIntent.zero;
  }

  void _onNormalized(NormalizedGamepadEvent e) {
    lastEvent.value = e.toString();
    if (e.axis != null) {
      _axisValues[e.axis!.name] = e.value;
      axes.value = Map.unmodifiable(_axisValues);
      switch (e.axis!) {
        case GamepadAxis.leftStickX:
          _lx = e.value;
        case GamepadAxis.leftStickY:
          _ly = e.value;
        case GamepadAxis.rightStickX:
          _rx = e.value;
        case GamepadAxis.rightStickY:
          _rY = e.value;
        default:
          break;
      }
      leftStick.value = Offset(_lx, _ly);
      rightStick.value = Offset(_rx, _rY);
      // 左摇杆上下 = 前后；左右 = 横移（机器人 vy 正为左，所以取负）。
      router.gamepad.value = router.shaped(_ly, -_lx, _rx, '手柄');
      return;
    }
    final button = e.button;
    if (button == null) return;
    final down = e.value > 0.5;
    final was = _down[button] ?? false;
    _down[button] = down;
    // 按钮状态每次都同步给 UI，但动作只在「抬起→按下」那一刻触发一次。
    final next = <String>{
      for (final entry in _down.entries)
        if (entry.value) entry.key.name,
    };
    pressed.value = Set.unmodifiable(next);
    if (!down || was) return;
    final action = switch (button) {
      GamepadButton.start => PadAction.arm,
      GamepadButton.b => PadAction.stop,
      GamepadButton.a => PadAction.jump,
      GamepadButton.x => PadAction.crouch,
      GamepadButton.y => PadAction.gait,
      GamepadButton.dpadUp => PadAction.stand,
      GamepadButton.dpadRight => PadAction.speedUp,
      GamepadButton.dpadLeft => PadAction.speedDown,
      _ => null,
    };
    if (action != null) _actions.add(action);
  }

  // 手柄马达震动，用来确认「这一下真的按到了」。没有马达就静默失败。
  Future<void> buzz() async {
    try {
      final list = await Gamepads.list();
      if (list.isEmpty) return;
      final pad = list.first;
      if (!await pad.hasRumble()) return;
      await pad.rumble(
        lowFrequency: 0.4,
        highFrequency: 0.2,
        duration: const Duration(milliseconds: 120),
      );
    } catch (_) {
      // 震动是锦上添花，失败不提示。
    }
  }

  void dispose() {
    for (final s in _subs) {
      s.cancel();
    }
    _subs.clear();
    _actions.close();
    pads.dispose();
    lastEvent.dispose();
    axes.dispose();
    status.dispose();
  }
}
