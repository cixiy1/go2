import 'dart:async';

import 'package:flutter/widgets.dart';

import 'core/gateway_client.dart';
import 'core/settings.dart';
import 'input/drive_session.dart';
import 'input/gamepad_source.dart';
import 'input/intent.dart';
import 'input/keyboard_source.dart';

// 全局装配：设置、连接、输入三路、驱动会话。
//
// 没有引入状态管理库。理由是这一屏的状态本来就分两类 —— 10 Hz 的遥测和
// 「用户点了一下」—— 前者用 ValueNotifier 精确到字段，后者用 ChangeNotifier
// 通知一层。再多一层框架，只会让人搞不清到底谁在重建。
class AppState extends ChangeNotifier {
  AppState(this.settings);

  final AppSettings settings;
  final GatewayClient client = GatewayClient();
  final ValueNotifier<int> tab = ValueNotifier(0);

  late final InputRouter router;
  late final GamepadSource gamepad;
  late final KeyboardSource keyboard;
  late final DriveSession drive;

  bool _ready = false;
  bool get ready => _ready;

  Future<void> init() async {
    await settings.load();
    final autoConnect = _applyDeepLink();
    router = InputRouter(settings)..attach();
    gamepad = GamepadSource(router, settings);
    keyboard = KeyboardSource(router);
    drive = DriveSession(client, settings, router)..start();
    gamepad.actions.listen(_onPadAction);
    keyboard.actions.listen(_onPadAction);
    await gamepad.start();
    _ready = true;
    notifyListeners();
    if (autoConnect) {
      // 深链接带了地址就连：网页版刷新、手机扫码打开都属于这一类。
      // 不 await —— 连不上时第一帧就该出来，而不是白屏六秒。
      unawaited(connect());
    }
  }

  // 深链接：`?host=192.168.123.1&port=8780&t=口令&tab=vla`。
  //
  // 落地页与连接页之间的那段手工输入，在「刷新一下」和「发给别人试试」这两个
  // 场景里最烦。参数只覆盖设置里的同名字段，没给的保持原样。
  bool _applyDeepLink() {
    final q = Uri.base.queryParameters;
    if (q.isEmpty) return false;
    final host = q['host'];
    if (host != null && host.trim().isNotEmpty) settings.host = host.trim();
    final port = int.tryParse(q['port'] ?? '');
    if (port != null) settings.port = port;
    if (q.containsKey('t')) settings.token = q['t'] ?? '';
    // 遥控页背景也能用链接指定：`bg=camera|radar|none`、`bgsize=main|small`。
    // 大屏挂墙上看画面时不用每次手点。
    final bg = q['bg'];
    if (bg == 'camera' || bg == 'radar' || bg == 'none') {
      settings.bgSource = bg!;
    }
    final size = q['bgsize'];
    if (size == 'main' || size == 'small') {
      settings.bgSize = size!;
    }
    final want = q['tab'];
    if (want != null) {
      tab.value = switch (want) {
        'drive' => 1,
        'vla' => 2,
        'gamepad' || 'pad' => 3,
        'settings' => 4,
        _ => 0,
      };
    }
    unawaited(settings.save());
    return host != null && host.trim().isNotEmpty;
  }

  Future<void> connect() async {
    await client.connect(settings.uri, name: settings.name);
  }

  Future<void> disconnect() async {
    await drive.setArmed(false);
    await client.disconnect();
    notifyListeners();
  }

  void go(int index) {
    tab.value = index;
  }

  // 发命令。所有按钮都走这里，好处是「要不要先取操作权」只写一遍：
  // 网关对每条命令都要求租约，界面不该让用户先手动点一次「取得控制」。
  Future<Map<String, dynamic>> send(
    String cmd, [
    Map<String, dynamic> args = const {},
  ]) async {
    if (!client.connected) {
      client.error.value = '还没连上网关';
      return {'ok': false, 'msg': '未连接'};
    }
    final st = client.state.value;
    if (st == null || !st.lease.mine) {
      final lease = await client.lease(true);
      if (lease['ok'] != true) {
        client.error.value = (lease['msg'] ?? '取不到操作权').toString();
        return lease;
      }
    }
    final out = await client.command(cmd, args);
    if (out['ok'] != true) {
      client.error.value = (out['msg'] ?? '$cmd 失败').toString();
    }
    return out;
  }

  // 急停。手动模式下是「零速 + stop」；VLA 模式下 App 根本没有 lowcmd 的写口，
  // 唯一有效的手段是把 VLA 停掉（SIGINT → 蹲下 → 阻尼 → 交还）。
  // 这条分叉必须写在 App 里：用户按急停时不会去想「现在是哪个模式在控腿」。
  Future<void> emergencyStop() async {
    final st = client.state.value;
    if (st?.mode == 'vla') {
      final out = await client.vlaAction('stop');
      if (out['ok'] != true) {
        client.error.value = (out['msg'] ?? '停 VLA 失败').toString();
      } else {
        client.notice.value = '正在停止 VLA（先蹲下再交还控制权）';
      }
      return;
    }
    await drive.emergencyStop();
  }

  // 换高层控制端。代价是服务端重启：狗会先蹲下再站起，操作权会掉。
  // 所以先把使能关掉，别让最后一帧速度跟着旧控制器走。
  Future<Map<String, dynamic>> switchController(String kind) async {
    if (!client.connected) return {'ok': false, 'msg': '还没连上网关'};
    await drive.setArmed(false);
    final out = await client.request('controller', {'kind': kind});
    if (out['ok'] != true) {
      client.error.value = (out['msg'] ?? '换不了控制端').toString();
    }
    return out;
  }

  // 切模式。网关是异步切（park / 起 VLA 要几秒），这里只负责把请求递出去，
  // 进度靠状态帧里的 mode=switching + note 显示。
  Future<Map<String, dynamic>> setMode(String mode) async {
    if (!client.connected) {
      return {'ok': false, 'msg': '还没连上网关'};
    }
    if (mode != 'manual') {
      // 切走之前先解除使能，否则最后一帧速度会把狗推着走。
      await drive.setArmed(false);
    }
    return client.setMode(mode);
  }

  // 手柄和键盘都是「产生动作」，动作到行为的映射只写在这一处 —— 界面上的
  // 按钮走的是同一批方法。
  Future<void> _onPadAction(PadAction action) async {
    switch (action) {
      case PadAction.arm:
        await drive.setArmed(!drive.armed.value);
        await gamepad.buzz();
      case PadAction.stop:
        await emergencyStop();
      case PadAction.jump:
        await client.command('jump');
      case PadAction.stand:
        await client.command('stand');
      case PadAction.crouch:
        await client.command('crouch', {'depth': 0.08});
      case PadAction.gait:
        await client.command('gait');
      case PadAction.speedUp:
        settings.speedScale = (settings.speedScale + 0.15).clamp(0.15, 1.0);
        await settings.save();
      case PadAction.speedDown:
        settings.speedScale = (settings.speedScale - 0.15).clamp(0.15, 1.0);
        await settings.save();
    }
  }

  @override
  void dispose() {
    drive.dispose();
    keyboard.dispose();
    gamepad.dispose();
    router.dispose();
    client.disconnect();
    super.dispose();
  }
}

// 让任意页面拿到 AppState，又不把它塞进构造函数。
class AppScope extends InheritedNotifier<AppState> {
  const AppScope({super.key, required AppState state, required super.child})
    : super(notifier: state);

  static AppState of(BuildContext context) {
    final scope = context.dependOnInheritedWidgetOfExactType<AppScope>();
    assert(scope != null, 'AppScope 不在树上');
    return scope!.notifier!;
  }

  // 只要方法、不要重建时用这个。
  static AppState read(BuildContext context) {
    final scope = context.getInheritedWidgetOfExactType<AppScope>();
    assert(scope != null, 'AppScope 不在树上');
    return scope!.notifier!;
  }
}
