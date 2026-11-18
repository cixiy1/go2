import 'dart:io' show Platform;

import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';

// 连接参数与手感参数。全部落在 SharedPreferences 里：换台设备不至于每次重填
// 地址，手柄死区也不该每次重新调。
//
// 默认值是按「第一次打开就能连」定的：本机回环 + 仿真。真机上改一次地址，
// 之后一直用。
class AppSettings extends ChangeNotifier {
  static const _kHost = 'host';
  static const _kPort = 'port';
  static const _kToken = 'token';
  static const _kName = 'name';
  static const _kDeadzone = 'deadzone';
  static const _kExpo = 'expo';
  static const _kInvert = 'invert_vy';
  static const _kSpeed = 'speed_scale';
  static const _kCam = 'camera_on';
  static const _kBgSource = 'bg_source';
  static const _kBgSize = 'bg_size';

  String host = '127.0.0.1';
  int port = 8780;
  String token = '';
  String name = 'Flutter';
  double deadzone = 0.12;
  double expo = 0.25;
  bool invertVy = false;
  double speedScale = 0.5;
  bool cameraOn = true;

  // 遥控页的背景：'camera' / 'radar' / 'none'；尺寸 'main'（铺满摇杆区）/
  // 'small'（右上角小窗）。默认相机 + 主视图 —— 遥控时先看到路，再谈别的。
  String bgSource = 'camera';
  String bgSize = 'main';

  // 设备名会出现在「谁在控制」的提示里。默认取平台，用户可改。
  static String defaultName() {
    if (kIsWeb) return '网页';
    try {
      if (Platform.isAndroid) return '安卓';
      if (Platform.isIOS) return 'iPhone';
      if (Platform.isMacOS) return 'Mac';
      if (Platform.isWindows) return 'Windows';
      if (Platform.isLinux) return 'Linux';
    } catch (_) {
      // 有些环境没有 dart:io（web 走上面的分支，这里只兜底）。
    }
    return 'App';
  }

  Future<void> load() async {
    final p = await SharedPreferences.getInstance();
    host = p.getString(_kHost) ?? host;
    port = p.getInt(_kPort) ?? port;
    token = p.getString(_kToken) ?? token;
    name = p.getString(_kName) ?? defaultName();
    deadzone = p.getDouble(_kDeadzone) ?? deadzone;
    expo = p.getDouble(_kExpo) ?? expo;
    invertVy = p.getBool(_kInvert) ?? invertVy;
    speedScale = p.getDouble(_kSpeed) ?? speedScale;
    cameraOn = p.getBool(_kCam) ?? cameraOn;
    bgSource = p.getString(_kBgSource) ?? bgSource;
    bgSize = p.getString(_kBgSize) ?? bgSize;
    notifyListeners();
  }

  Future<void> save() async {
    final p = await SharedPreferences.getInstance();
    await p.setString(_kHost, host);
    await p.setInt(_kPort, port);
    await p.setString(_kToken, token);
    await p.setString(_kName, name);
    await p.setDouble(_kDeadzone, deadzone);
    await p.setDouble(_kExpo, expo);
    await p.setBool(_kInvert, invertVy);
    await p.setDouble(_kSpeed, speedScale);
    await p.setBool(_kCam, cameraOn);
    await p.setString(_kBgSource, bgSource);
    await p.setString(_kBgSize, bgSize);
    notifyListeners();
  }

  Uri get uri {
    final host = this.host.trim().isEmpty ? '127.0.0.1' : this.host.trim();
    final t = token.trim();
    return Uri(
      scheme: 'ws',
      host: host,
      port: port,
      path: '/',
      queryParameters: t.isEmpty ? null : {'t': t},
    );
  }

  // 摇杆/手柄的原始值 → 有效值。死区之外做一次重标定，再按指数曲线压一下，
  // 让小幅推杆能精细控制 —— 摇杆线性映射时，前 20% 行程就已经跑到最大速度的
  // 三分之一，机器狗在窄地方根本走不动。
  double shape(double raw) {
    final a = raw.abs();
    if (a <= deadzone) return 0;
    final norm = ((a - deadzone) / (1 - deadzone)).clamp(0.0, 1.0);
    final curved = expo <= 0 ? norm : norm * ((1 - expo) + expo * norm * norm);
    return raw.isNegative ? -curved : curved;
  }
}
