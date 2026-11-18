import 'package:flutter/material.dart';

import 'app_state.dart';
import 'core/settings.dart';
import 'theme.dart';
import 'ui/app_shell.dart';

// 入口。初始化顺序有要求：设置要先从磁盘读出来，AppShell 第一帧才拿得到
// 正确的地址和手感参数。所以 `init()` 在 runApp 之前 await —— 这一步只读
// SharedPreferences，几毫秒。
Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  final settings = AppSettings();
  final app = AppState(settings);
  await app.init();
  runApp(Go2RemoteApp(app: app));
}

class Go2RemoteApp extends StatelessWidget {
  const Go2RemoteApp({super.key, required this.app});
  final AppState app;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Go2 遥控台',
      debugShowCheckedModeBanner: false,
      theme: buildTheme(),
      home: AppScope(state: app, child: const AppShell()),
    );
  }
}
