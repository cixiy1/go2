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
  static const _kGlass = 'glass';

  // 手机和桌面默认地址不一样：网关跑在这台 Mac 上，桌面直接走回环，
  // 手机必须走局域网。给手机一个 127.0.0.1 等于装完就连不上，还看不出
  // 是填错了还是网关没起 —— 所以按平台给不同的默认值。
  //
  // 这个 IP 是 Mac 在 Yukiho 热点里的地址，DHCP 分配，重连后可能变。
  // 变了就在连接页改一次，改完会记住。
  String host = defaultHost();
  int port = 8780;
  String token = defaultToken();
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

  // 毛玻璃。侧栏、顶栏、手机底栏是半透明 + 背景模糊（对齐 FlClash 的
  // LiquidGlass 观感）。低端设备上 BackdropFilter 每帧采样背景，卡的话关掉 ——
  // 关掉后这些栏退化成纯填充，布局不变。
  bool glass = true;

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

  //! 网关所在机器的局域网地址（Mac 连 Yukiho 热点时拿到的）。
  //!
  //! 手机必须用这个；桌面用 127.0.0.1 就够，走回环少一层。
  static const lanHost = '10.216.47.152';

  static String defaultHost() {
    if (kIsWeb) return 'localhost';
    try {
      if (Platform.isAndroid || Platform.isIOS) return lanHost;
    } catch (_) {}
    return '127.0.0.1';
  }

  //! 网关的默认口令。`start_real.sh` 和网关的 `--token` 都是这个值。
  //!
  //! 上公网前必须改掉 —— 它只是给局域网调试省一次输入。
  static String defaultToken() => 'demo';

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
    glass = p.getBool(_kGlass) ?? glass;
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
    await p.setBool(_kGlass, glass);
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
