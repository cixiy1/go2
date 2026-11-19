import 'dart:async';

import 'package:flutter/foundation.dart';

import '../core/gateway_client.dart';
import '../core/models.dart';
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

  //: 最近一次「使能失败/被解除」的原因。常驻在读数下方，直到下次成功。
  //: 只塞 snackbar 的话，3 秒就没了 —— 用户回头再看只见到读数不动。
  final ValueNotifier<String?> blockReason = ValueNotifier(null);
  final ValueNotifier<bool> sending = ValueNotifier(false);
  final ValueNotifier<String> source = ValueNotifier('-');

  Timer? _timer;
  double _vx = 0;
  double _vy = 0;
  double _vyaw = 0;
  bool _wasSending = false;
  // 推了杆却发不出去时只提示一次；杆回中后重置，下次再推还提示。
  bool _hinted = false;
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
      blockReason.value = '还没连上网关';
      client.error.value = '还没连上网关';
      return false;
    }
    if (st == null) {
      blockReason.value = '还没收到网关状态';
      return false;
    }
    if (st.mode != 'manual') {
      blockReason.value = '现在是 ${st.mode} 模式，摇杆不下发';
      client.error.value = '现在是 ${st.mode} 模式，先切到手动遥控';
      return false;
    }
    if (!st.lease.mine) {
      final out = await client.lease(true);
      if (out['ok'] != true) {
        final msg = (out['msg'] ?? '取不到操作权').toString();
        blockReason.value = msg;
        client.error.value = msg;
        return false;
      }
    }
    blockReason.value = null;
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
      blockReason.value = '使能已解除：模式或操作权变了';
      client.notice.value = '已解除使能（模式或操作权变了）';
    }
  }

  // 杆子推了但一个字节都发不出去时，把原因说一次。
  //
  // 不说的后果是真实的：界面上只有读数不动，用户以为遥控坏了，去查网关、
  // 查网络，其实只是没点使能、或者操作权在别的标签页手里。
  void _hintIfIgnored(GatewayState? st) {
    final i = router.merged.value;
    final pushed =
        i.vx.abs() > 0.02 || i.vy.abs() > 0.02 || i.vyaw.abs() > 0.02;
    if (!pushed) {
      _hinted = false;
      return;
    }
    if (_hinted) return;
    _hinted = true;
    if (!client.connected) {
      client.notice.value = '还没连上网关';
    } else if (st != null && st.mode != 'manual') {
      client.notice.value = '现在是 ${st.mode} 模式，摇杆不下发';
    } else if (st != null && !st.lease.mine) {
      client.notice.value = '没有操作权，摇杆不下发 —— 点「取得控制」';
    } else {
      client.notice.value = '先点「使能」，摇杆才会下发';
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
      _hintIfIgnored(st);
      return;
    }
    _hinted = false;

    final i = router.merged.value;
    final lim = st.limits;
    final scale = settings.speedScale;
    final vx = (i.vx * lim.vx * scale).clamp(-lim.vx, lim.vx);
    final vy = (i.vy * lim.vy * scale).clamp(-lim.vy, lim.vy);
    final vyaw = (i.vyaw * lim.vyaw * scale).clamp(-lim.vyaw, lim.vyaw);

    // 只要不是「一直全零」就每一拍都发。
    //
    // 原来这里有个「值没变就不重发」，看着省带宽，实际是致命的：杆推住不动
    // 时只发出去一帧，网关 250 ms 后判「设定值过期 → 停车」，表现就是
    // 「动一下、停、再动一下」。20 Hz 持续刷新，一帧几十字节，省这点没意义。
    final zero = vx == 0 && vy == 0 && vyaw == 0;
    if (zero && _vx == 0 && _vy == 0 && _vyaw == 0) {
      if (_wasSending) {
        sending.value = false;
        _wasSending = false;
      }
      return;
    }

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
