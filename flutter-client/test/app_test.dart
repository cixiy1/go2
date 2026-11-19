import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:go2_remote/core/models.dart';
import 'package:go2_remote/input/intent.dart';
import 'package:go2_remote/core/settings.dart';
import 'package:go2_remote/ui/widgets/glass.dart';
import 'package:go2_remote/ui/widgets/joystick.dart';

// 客户端最容易被悄悄改坏的两块：网关报文的解析、摇杆到意图的那一步符号。
// 这两个都不是「看起来对不对」能发现的 —— 方向反了狗会朝人走过来。
void main() {
  group('报文解析', () {
    test('半份状态帧不抛异常', () {
      // 切模式中间态：机器人已经放下、VLA 还没起来，字段是缺的。
      final st = GatewayState.fromJson({'mode': 'switching', 'robot': {}});
      expect(st.mode, 'switching');
      expect(st.robot.height, isNull);
      expect(st.lease.mine, isFalse);
      expect(st.limits.vx, 1.0);
    });

    test('VLA 帧的字段映射', () {
      final f = VlaFrame.fromJson({
        'seq': 12,
        't': 3.5,
        'cmd': {'vx': 0.25, 'vy': 0.0, 'vyaw': -0.1},
        'gait': 'trot',
        'front_gap': 1.42,
        'sectors': [
          {'name': '正前', 'gap': 1.42, 'top': 0.0},
          {'name': '左前', 'gap': 2.9, 'top': 0.0},
        ],
        'thought': '左边空着',
        'reply': '我往左走',
        'reply_seq': 3,
        'guard': {'trips': 1, 'last': '正前 0.42'},
        'memory': {'cells': 7, 'travelled': 2.5},
        'pos': [1.25, -0.5],
        'policy_hz': 49.2,
        'vision': {'frames': 120, 'dropped': 2, 'views': 9, 'gen_s': 1.1},
        'alert': true,
      });
      expect(f.hasData, isTrue);
      expect(f.vx, 0.25);
      expect(f.gait, 'trot');
      expect(f.sectors.length, 2);
      expect(f.sectors.first.name, '正前');
      expect(f.replySeq, 3);
      expect(f.guardTrips, 1);
      expect(f.memoryMetres, 2.5);
      expect(f.pos![0], 1.25);
      expect(f.policyHz, 49.2);
      expect(f.vision.dropped, 2);
      expect(f.alert, isTrue);
    });

    test('链路信息带上控制端与可选列表', () {
      final st = GatewayState.fromJson({
        'mode': 'manual',
        'link': {
          'ok': true,
          'controller': 'ours',
          'available': ['ours', 'policy'],
        },
      });
      expect(st.link.controller, 'ours');
      expect(st.link.available, ['ours', 'policy']);
    });

    test('老版本服务端没报控制端时不炸', () {
      final st = GatewayState.fromJson({'mode': 'manual', 'link': {'ok': true}});
      expect(st.link.controller, isNull);
      expect(st.link.available, isEmpty);
    });

    test('清空 VLA 后的空帧 hasData 为 false', () {
      final f = VlaFrame.fromJson({});
      expect(f.hasData, isFalse);
      expect(f.sectors, isEmpty);
    });
  });

  group('JPEG 宽高', () {
    test('从 SOF0 段读出 320x240', () {
      // FFD8 + APP0(空) + SOF0(h=240,w=320)
      final bytes = <int>[
        0xFF, 0xD8,
        0xFF, 0xE0, 0x00, 0x04, 0x00, 0x00,
        0xFF, 0xC0, 0x00, 0x11, 0x08, 0x00, 0xF0, 0x01, 0x40, 0x03,
        0x00, 0x00, 0x00, 0x00,
      ];
      final size = jpegSize(Uint8List.fromList(bytes));
      expect(size, isNotNull);
      expect(size!.$1, 320);
      expect(size.$2, 240);
      expect(jpegSize(Uint8List.fromList([0, 1, 2])), isNull);
    });

    test('CameraFrame 带上比例，首字节被剥掉', () {
      final raw = Uint8List.fromList(<int>[
        2,
        0xFF, 0xD8,
        0xFF, 0xC0, 0x00, 0x11, 0x08, 0x00, 0xF0, 0x01, 0x40, 0x03,
      ]);
      final f = CameraFrame.parse(raw);
      expect(f.isCamera, isTrue);
      expect(f.aspect, closeTo(320 / 240, 1e-9));
      expect(f.bytes[0], 0xFF);
    });

    test('帧数据是独立副本，不是原始消息的视图', () {
      // Web 上 CanvasKit 走 blob URL 解码时按整个 buffer 取数据；如果这里
      // 返回 sublistView，JPEG 头前面会多出那个来源字节，直接解码失败。
      // 所以断言：offset 必须为 0、长度正好少一个字节。
      final raw = Uint8List.fromList(<int>[
        2, 0xFF, 0xD8, 0xFF, 0xC0, 0x00, 0x11, 0x08, 0x00, 0xF0, 0x01, 0x40,
      ]);
      final f = CameraFrame.parse(raw);
      expect(f.bytes.offsetInBytes, 0);
      expect(f.bytes.lengthInBytes, raw.length - 1);
      expect(f.bytes.buffer.lengthInBytes, raw.length - 1);
    });
  });

  group('手感曲线', () {
    test('死区内归零', () {
      final s = AppSettings()..deadzone = 0.12;
      expect(s.shape(0.1), 0);
      expect(s.shape(-0.1), 0);
      expect(s.shape(0.05), 0);
    });

    test('满杆仍然是 1', () {
      final s = AppSettings()..deadzone = 0.12..expo = 0.3;
      expect(s.shape(1.0), closeTo(1.0, 1e-9));
      expect(s.shape(-1.0), closeTo(-1.0, 1e-9));
    });

    test('曲线压低小杆量', () {
      final s = AppSettings()..deadzone = 0.0..expo = 0.5;
      expect(s.shape(0.3), lessThan(0.3));
      expect(s.shape(0.3), greaterThan(0));
    });
  });

  group('玻璃层', () {
    // 关掉毛玻璃要退化成纯填充，不能连内容一起没掉 —— 设置里那个开关
    // 是给低端设备留的，不能变成「一关就白屏」。
    testWidgets('关掉时仍然渲染内容', (tester) async {
      await tester.pumpWidget(MaterialApp(
        home: Center(
          child: SizedBox(
            width: 200,
            height: 80,
            child: Glass(
              enabled: false,
              child: const Text('面板'),
            ),
          ),
        ),
      ));
      expect(find.text('面板'), findsOneWidget);
      expect(tester.takeException(), isNull);
    });

    testWidgets('开着时也不抛异常', (tester) async {
      await tester.pumpWidget(MaterialApp(
        home: Center(
          child: SizedBox(
            width: 200,
            height: 80,
            child: Glass(child: const Text('面板2')),
          ),
        ),
      ));
      expect(find.text('面板2'), findsOneWidget);
      expect(tester.takeException(), isNull);
    });
  });

  group('摇杆', () {
    // 右摇杆改全向后，纵向和左手那路都会给 vx。取大而不是相加：
    // 相加会到 2.0 超出行程，取小则让推得更狠的那只手失效。
    test('同一根轴两路取绝对值大的', () {
      expect(pickAxis(0.8, 0.3), 0.8);
      expect(pickAxis(0.3, -0.8), -0.8);
      expect(pickAxis(0.0, 0.0), 0.0);
      expect(pickAxis(-0.4, 0.4), -0.4); // 相等时保留前一路，行为确定
    });

    testWidgets('拖动产生 -1~1 的偏移，松手归零', (tester) async {
      final values = <Offset>[];
      await tester.pumpWidget(MaterialApp(
        home: Center(
          child: Joystick(size: 160, onChanged: values.add),
        ),
      ));
      final center = tester.getCenter(find.byType(Joystick));
      final gesture = await tester.startGesture(center);
      await gesture.moveBy(const Offset(60, 0));
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(values, isNotEmpty);
      final pushed = values.firstWhere((v) => v.dx > 0.1);
      expect(pushed.dx, greaterThan(0));
      expect(pushed.dy.abs(), lessThan(0.01));
      expect(values.last, Offset.zero);
    });

    testWidgets('lockY 只输出横向', (tester) async {
      final values = <Offset>[];
      await tester.pumpWidget(MaterialApp(
        home: Center(
          child: Joystick(size: 160, lockY: true, onChanged: values.add),
        ),
      ));
      final center = tester.getCenter(find.byType(Joystick));
      final gesture = await tester.startGesture(center);
      await gesture.moveBy(const Offset(40, 40));
      await tester.pump();
      await gesture.up();
      await tester.pump();
      expect(values.any((v) => v.dx > 0.1 && v.dy == 0), isTrue);
    });
  });
}
