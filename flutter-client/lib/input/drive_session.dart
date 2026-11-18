import 'dart:async';

import 'package:flutter/foundation.dart';

import '../core/gateway_client.dart';
import '../core/settings.dart';
import 'intent.dart';

// 把控制意图变成真正的速度指令，按固定节拍发给网关。
//
// 这里是整个客户端唯一「有状态、会持续动作」的地方，所以三条纪律都落在这个
// 文件里：
//
//   1. **使能**（armed）默认关。摇杆随便推，没使能就一个字节都不发。手机放兜里
//      蹭到屏幕不该让狗走。
//   2. 使能的前提是：连着网关 + 手动模式 + 持有操作权。任何一条断了自动解除，
//      并立刻补一帧零速 —— 不补的话，网关要等 250 ms 的设定值超时才停。
//   3. 急停发两件事：零速（立刻）+ `stop` 命令（机器人侧停步）。只发零速时，
//      机器人还在「走」的状态里，遇到坡会原地踏步。
class DriveSession {
  DriveSession(this.client, this.settings, this.router);

  final GatewayClient client;
  final AppSettings settings;
  final InputRouter router;

  final ValueNotifier<bool> armed = ValueNotifier(false);
  final ValueNotifier<bool> sending = ValueNotifier(false);
  final ValueNotifier<String> source = ValueNotifier('-');

  Timer? _timer;
  double _vx = 0;
  double _vy = 0;
  double _vyaw = 0;
  bool _wasSending = false;
  int _rateHz = 20;

  void start() {
    _timer?.cancel();
    _timer = Timer.periodic(
      Duration(microseconds: 1000000 ~/ _rateHz),
      (_) => _tick(),
    );
    client.state.addListener(_guard);
  }

  void setRate(int hz) {
    _rateHz = hz.clamp(5, 50);
    if (_timer != null) start();
  }

  // 使能开关。打开之前先确认四个前提都在，否则打开也会在下一拍被关掉，
  // 用户看到的是「开关自己弹回去了」。
  Future<bool> setArmed(bool value) async {
    if (!value) {
      armed.value = false;
      _flushZero();
      return false;
    }
    final st = client.state.value;
    if (!client.connected) {
      client.error.value = '还没连上网关';
      return false;
    }
    if (st == null) return false;
    if (st.mode != 'manual') {
      client.error.value = '现在是 ${st.mode} 模式，先切到手动遥控';
      return false;
    }
    if (!st.lease.mine) {
      final out = await client.lease(true);
      if (out['ok'] != true) {
        client.error.value = (out['msg'] ?? '取不到操作权').toString();
        return false;
      }
    }
    armed.value = true;
    return true;
  }

  // 状态一变就检查前提。丢了操作权、模式被切走、链路断了 —— 一律解除使能。
  void _guard() {
    if (!armed.value) return;
    final st = client.state.value;
    final ok = st != null && st.mode == 'manual' && st.lease.mine;
    if (!ok) {
      armed.value = false;
      _flushZero();
      client.notice.value = '已解除使能（模式或操作权变了）';
    }
  }

  void _tick() {
    final st = client.state.value;
    final ready =
        armed.value &&
        client.connected &&
        st != null &&
        st.mode == 'manual' &&
        st.lease.mine;

    if (!ready) {
      if (_wasSending) _flushZero();
      return;
    }

    final i = router.merged.value;
    final lim = st.limits;
    final scale = settings.speedScale;
    final vx = (i.vx * lim.vx * scale).clamp(-lim.vx, lim.vx);
    final vy = (i.vy * lim.vy * scale).clamp(-lim.vy, lim.vy);
    final vyaw = (i.vyaw * lim.vyaw * scale).clamp(-lim.vyaw, lim.vyaw);

    final changed =
        (vx - _vx).abs() > 0.005 ||
        (vy - _vy).abs() > 0.005 ||
        (vyaw - _vyaw).abs() > 0.005;
    if (!changed && _wasSending) return;

    _vx = vx;
    _vy = vy;
    _vyaw = vyaw;
    source.value = i.source;
    client.setpoint(vx, vy, vyaw, source: i.source);
    _wasSending = true;
    sending.value = vx != 0 || vy != 0 || vyaw != 0;
  }

  void _flushZero() {
    if (!client.connected) {
      _wasSending = false;
      sending.value = false;
      return;
    }
    _vx = _vy = _vyaw = 0;
    client.setpoint(0, 0, 0, source: 'idle');
    _wasSending = true;
    sending.value = false;
  }

  // 急停。零速先发（不等任何往返），再让机器人侧停步。
  Future<void> emergencyStop() async {
    armed.value = false;
    _flushZero();
    if (client.connected) {
      await client.command('stop');
    }
  }

  void dispose() {
    _timer?.cancel();
    client.state.removeListener(_guard);
    armed.dispose();
    sending.dispose();
    source.dispose();
  }
}
