import 'package:flutter/material.dart';

import '../app_state.dart';
import '../theme.dart';
import 'widgets/pad_diagram.dart';
import 'widgets/panel.dart';

// 手柄页：确认「按下去了没有」。手柄接入在各平台上差异很大，出问题时第一件事
// 是看有没有事件进来，而不是怀疑映射写错了。
class GamepadPage extends StatelessWidget {
  const GamepadPage({super.key});

  @override
  Widget build(BuildContext context) {
    final app = AppScope.of(context);
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Center(
        child: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 860),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              _status(app),
              const SizedBox(height: 12),
              _live(app),
              const SizedBox(height: 12),
              _mapping(),
              const SizedBox(height: 12),
              _tuning(app),
            ],
          ),
        ),
      ),
    );
  }

  Widget _status(AppState app) {
    return Panel(
      title: '连接状态',
      trailing: ValueListenableBuilder<List<String>>(
        valueListenable: app.gamepad.pads,
        builder: (context, pads, _) => StatusChip(
          text: pads.isEmpty ? '未连接' : '${pads.length} 个',
          color: pads.isEmpty ? C.faint : C.accent,
          dense: true,
        ),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          ValueListenableBuilder<String>(
            valueListenable: app.gamepad.status,
            builder: (context, text, _) =>
                Text(text, style: const TextStyle(fontSize: 13, color: C.text)),
          ),
          const SizedBox(height: 8),
          const Text(
            '蓝牙或 USB 连上手柄后按任意键。网页版要先按一下手柄，浏览器才会把它'
            '交给页面（这是浏览器的安全限制，不是 App 的问题）。',
            style: TextStyle(fontSize: 11.5, color: C.faint, height: 1.55),
          ),
        ],
      ),
    );
  }

  Widget _live(AppState app) {
    return Panel(
      title: '实时输入',
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          // 手柄俯视图。数字（vx/vy/vyaw）只反映「平移和转向」，看不出是哪个
          // 键按下的 —— 起立、换步态、跳跃都不在这三个数里。图能直接回答
          // 「我这一下到底按到没有」。
          Center(
            child: ConstrainedBox(
              constraints:
                  const BoxConstraints(maxWidth: padDiagramMaxWidth),
              child: ListenableBuilder(
                listenable: Listenable.merge([
                  app.gamepad.pressed,
                  app.gamepad.leftStick,
                  app.gamepad.rightStick,
                  app.gamepad.pads,
                ]),
                builder: (context, _) => PadDiagram(
                  pressed: app.gamepad.pressed.value,
                  left: app.gamepad.leftStick.value,
                  right: app.gamepad.rightStick.value,
                  connected: app.gamepad.connected,
                ),
              ),
            ),
          ),
          const SizedBox(height: 4),
          const Center(
            child: Text(
              '按任意键，「它」就会亮',
              style: TextStyle(fontSize: 10.5, color: C.faint),
            ),
          ),
          const SizedBox(height: 14),
          ValueListenableBuilder(
            valueListenable: app.router.gamepad,
            builder: (context, intent, _) => Row(
              children: [
                Expanded(
                  child: MetricTile(
                    label: '前后 vx',
                    value: intent.vx.toStringAsFixed(2),
                    color: intent.vx.abs() > 0.01 ? C.accent : C.dim,
                  ),
                ),
                Expanded(
                  child: MetricTile(
                    label: '横移 vy',
                    value: intent.vy.toStringAsFixed(2),
                    color: intent.vy.abs() > 0.01 ? C.accent : C.dim,
                  ),
                ),
                Expanded(
                  child: MetricTile(
                    label: '转向 vyaw',
                    value: intent.vyaw.toStringAsFixed(2),
                    color: intent.vyaw.abs() > 0.01 ? C.accent : C.dim,
                  ),
                ),
              ],
            ),
          ),
          const SizedBox(height: 12),
          ValueListenableBuilder<Map<String, double>>(
            valueListenable: app.gamepad.axes,
            builder: (context, axes, _) {
              if (axes.isEmpty) {
                return const Text(
                  '还没收到轴事件 —— 推一下摇杆',
                  style: TextStyle(fontSize: 11.5, color: C.faint),
                );
              }
              return Column(
                children: [
                  for (final e in axes.entries)
                    if (e.key.startsWith('left') || e.key.startsWith('right'))
                      Padding(
                        padding: const EdgeInsets.symmetric(vertical: 3),
                        child: Row(
                          children: [
                            SizedBox(
                              width: 96,
                              child: Text(
                                e.key,
                                style: const TextStyle(
                                  fontSize: 10.5,
                                  color: C.faint,
                                ),
                              ),
                            ),
                            Expanded(child: _AxisBar(value: e.value)),
                            SizedBox(
                              width: 46,
                              child: Text(
                                e.value.toStringAsFixed(2),
                                textAlign: TextAlign.right,
                                style: numStyle.copyWith(
                                  fontSize: 11,
                                  color: C.dim,
                                ),
                              ),
                            ),
                          ],
                        ),
                      ),
                ],
              );
            },
          ),
          const SizedBox(height: 10),
          ValueListenableBuilder<String?>(
            valueListenable: app.gamepad.lastEvent,
            builder: (context, text, _) => Text(
              text ?? '最近事件：—',
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
              style: const TextStyle(
                fontSize: 10.5,
                color: C.faint,
                fontFamily: 'monospace',
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _mapping() {
    const rows = [
      ('左摇杆', '平移（前后 / 左右）'),
      ('右摇杆横向', '转向'),
      ('A', '跳跃'),
      ('B', '急停'),
      ('X', '蹲下'),
      ('Y', '换步态'),
      ('Start', '使能开关'),
      ('十字键 ↑', '起立'),
      ('十字键 ← / →', '速度档减 / 加'),
      ('空格 / Esc', '跳跃 / 急停'),
    ];
    return Panel(
      title: '映射',
      child: Column(
        children: [
          for (final r in rows) KeyValue(r.$1, r.$2, mono: false),
          const SizedBox(height: 6),
          const Text(
            '映射固定写在 App 里，改一份就够：所有平台共用同一套归一化按键名。',
            style: TextStyle(fontSize: 11, color: C.faint),
          ),
        ],
      ),
    );
  }

  Widget _tuning(AppState app) {
    return ListenableBuilder(
      listenable: app.settings,
      builder: (context, _) {
        final s = app.settings;
        return Panel(
          title: '手感',
          child: Column(
            children: [
              _slider(
                label: '死区',
                value: s.deadzone,
                min: 0,
                max: 0.4,
                format: (v) => v.toStringAsFixed(2),
                onChanged: (v) {
                  s.deadzone = v;
                  s.save();
                },
                hint: '摇杆回中不准时调大。太大则小幅推杆没反应',
              ),
              _slider(
                label: '曲线',
                value: s.expo,
                min: 0,
                max: 0.9,
                format: (v) => v.toStringAsFixed(2),
                onChanged: (v) {
                  s.expo = v;
                  s.save();
                },
                hint: '越大，小幅推杆越精细（机器狗在窄地方更好使）',
              ),
              SwitchListTile(
                contentPadding: EdgeInsets.zero,
                dense: true,
                activeThumbColor: C.accent,
                title: const Text('反转横移', style: TextStyle(fontSize: 12.5)),
                subtitle: const Text(
                  '推左往右走的时候打开',
                  style: TextStyle(fontSize: 11, color: C.faint),
                ),
                value: s.invertVy,
                onChanged: (v) {
                  s.invertVy = v;
                  s.save();
                },
              ),
            ],
          ),
        );
      },
    );
  }

  Widget _slider({
    required String label,
    required double value,
    required double min,
    required double max,
    required String Function(double) format,
    required ValueChanged<double> onChanged,
    String? hint,
  }) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Expanded(
                child: Text(
                  label,
                  style: const TextStyle(fontSize: 12.5, color: C.text),
                ),
              ),
              Text(
                format(value),
                style: numStyle.copyWith(fontSize: 12, color: C.accent),
              ),
            ],
          ),
          Slider(value: value, min: min, max: max, onChanged: onChanged),
          if (hint != null)
            Padding(
              padding: const EdgeInsets.only(bottom: 6),
              child: Text(
                hint,
                style: const TextStyle(fontSize: 10.5, color: C.faint),
              ),
            ),
        ],
      ),
    );
  }
}

class _AxisBar extends StatelessWidget {
  const _AxisBar({required this.value});
  final double value;

  @override
  Widget build(BuildContext context) {
    // 从中线往两边长：左半格是负值、右半格是正值。用 Align + FractionallySizedBox
    // 从边缘长会看起来像「单向进度条」，读不出符号。
    Widget half({required bool right}) {
      final v = right ? value.clamp(0.0, 1.0) : (-value).clamp(0.0, 1.0);
      return Expanded(
        child: Align(
          alignment: right ? Alignment.centerLeft : Alignment.centerRight,
          child: FractionallySizedBox(
            widthFactor: v,
            child: Container(
              height: 8,
              decoration: BoxDecoration(
                color: right ? C.accent : C.warn,
                borderRadius: BorderRadius.circular(R.card),
              ),
            ),
          ),
        ),
      );
    }

    return SizedBox(
      height: 8,
      child: Stack(
        children: [
          Positioned.fill(
            child: Container(
              decoration: BoxDecoration(
                color: C.surfaceHi,
                borderRadius: BorderRadius.circular(R.card),
              ),
            ),
          ),
          Positioned.fill(
            child: Row(
              children: [
                half(right: false),
                const SizedBox(width: 2),
                half(right: true),
              ],
            ),
          ),
        ],
      ),
    );
  }
}
