import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:go2_remote/app_state.dart';
import 'package:go2_remote/core/models.dart';
import 'package:go2_remote/core/settings.dart';
import 'package:go2_remote/main.dart';
import 'package:shared_preferences/shared_preferences.dart';

// 整机冒烟：把真的 AppShell 和五个页面按几种屏幕尺寸各渲染一遍。
//
// 单元测试只覆盖解析和摇杆，页面里的布局错误（Column 溢出、给 Expanded
// 无限高度）不会被 `flutter analyze` 发现，只会在真机上变成黄黑条纹或者
// 一片灰。这里把「一屏塞不下」这件事变成断言。
void main() {
  // 一份「什么都报」的状态帧和 VLA 帧：字最长的分支全都走到。
  GatewayState fullState() => GatewayState.fromJson({
        'mode': 'manual',
        'note': '正在切换',
        'link': {
          'ok': true,
          'preset': 'real',
          'target': '192.168.123.1:8780',
          'rtt_ms': 12.4,
          'controller': 'ours',
          'available': ['official', 'ours', 'policy'],
        },
        'robot': {
          'pos': [1.234, -2.345],
          'rpy': [0.1, -0.2, 0.3],
          'height': 0.301,
          'gait': '机械（标准小跑，同官方灵动）',
          'busy': false,
          'moving': true,
          'loop': '200 Hz',
        },
        'input': {
          'vx': 0.42,
          'vy': -0.12,
          'vyaw': 0.3,
          'age_ms': 18,
          'source': '摇杆',
          'driving': true,
        },
        'lease': {'holder': 3, 'mine': true, 'clients': 2},
        'vla': {'running': true, 'pid': 91, 'uptime': 42.5, 'log': 'ok'},
        'cam': {'running': true, 'pid': 92, 'note': ''},
        'limits': {
          'vx': 1.0,
          'vy': 0.3,
          'vyaw': 4.0,
          'crouch': [0.02, 0.18],
        },
        'server': {'uptime': 123.4},
      });

  VlaFrame fullVla() => VlaFrame.fromJson({
        'seq': 42,
        't': 128.0,
        'scene': 'maze',
        'source': 'sim+vision',
        'auto': true,
        'cmd': {'vx': 0.25, 'vy': 0.01, 'vyaw': -0.15},
        'gait': 'trot',
        'front_gap': 0.72,
        'sectors': [
          for (var i = 0; i < 8; i++)
            {'name': '方位$i', 'gap': 0.6 + i * 0.2, 'top': 0.3},
        ],
        'scene_text': '左：空地，能，远\n中：墙，不能，近\n右：空地，能，中',
        'scene_age': 5.2,
        'thought': '任务已走 1.1 米还差 0.9 米；正前一米有墙，先左转绕开，再往前走两米，'
            '走够之后停下来跟人说一句我到了。',
        'reply': '正前有墙，我先左转绕开',
        'reply_seq': 7,
        'emit_thought': true,
        'goal': '正前（换 49 次，受阻 3 次）',
        'turns': 14,
        'turn_left_deg': 22.5,
        'thoughts': 31,
        'gen_s': 1.42,
        'think_rate': 0.62,
        'guard': {'trips': 4, 'last': '正前 0.55 m，vx 压到 0.12'},
        'memory': {'cells': 137, 'travelled': 21.4},
        'pos': [3.04, -0.21],
        'yaw_deg': 168.0,
        'tilt_deg': 6.2,
        'policy_hz': 49.8,
        'reflex_hz': 19.9,
        'view_fps': 5.0,
        'vision': {'frames': 640, 'dropped': 3, 'views': 96, 'gen_s': 2.4},
        'driver': {'direction': '正前', 'changes': 49, 'blocked': 3},
        'alert': true,
      });

  // 每屏幕一份（宽 × 高，物理像素按 dpr=1 算）。
  const sizes = <(String, double, double)>[
    ('桌面 1440x900', 1440, 900),
    ('窄桌面 1100x700', 1100, 700),
    ('平板 820x1180', 820, 1180),
    ('手机 414x896', 414, 896),
    ('小屏手机 360x640', 360, 640),
  ];

  for (final (name, w, h) in sizes) {
    testWidgets('整机渲染 · $name', (tester) async {
      SharedPreferences.setMockInitialValues({});
      tester.view.physicalSize = Size(w, h);
      tester.view.devicePixelRatio = 1.0;
      addTearDown(tester.view.reset);

      final app = AppState(AppSettings());
      // `init()` 里有 SharedPreferences 和手柄插件，都是真异步。testWidgets 的
      // 假时钟不会推进它们 —— 直接 await 会一直等到超时（表现为这条测试挂 8 分钟）。
      await tester.runAsync(app.init);
      app.client.state.value = fullState();
      app.client.vla.value = fullVla();

      await tester.pumpWidget(Go2RemoteApp(app: app));
      await tester.pump();

      // 逐页收集错误，并把完整诊断打出来。`takeException()` 只给最后一条、
      // 也没有「是哪个控件」—— 溢出的定位全在 `FlutterErrorDetails` 的诊断树
      // 里（The relevant error-causing widget was: …）。
      final problems = <String>[];
      for (var i = 0; i < 5; i++) {
        final box = <FlutterErrorDetails>[];
        final old = FlutterError.onError;
        FlutterError.onError = box.add;
        app.go(i);
        await tester.pump();
        await tester.pump(const Duration(milliseconds: 32));
        FlutterError.onError = old;
        tester.takeException();
        for (final d in box) {
          problems.add('第 $i 页 @ $name：${d.exception}');
          // ignore: avoid_print
          print('--- 第 $i 页 @ $name ---\n$d');
        }
      }
      expect(problems, isEmpty, reason: problems.join('\n'));

      await tester.pumpWidget(const SizedBox.shrink());
      app.dispose();
    });
  }
}
