import 'dart:math' as math;

import 'package:flutter/material.dart';

import '../../theme.dart';

// 虚拟摇杆。输出的是屏幕坐标系的偏移（x 右正、y 下正，范围 -1~1），
// 由调用方翻译成 vx/vy —— 摇杆自己不该知道「机器人的前方」是哪个方向。
class Joystick extends StatefulWidget {
  const Joystick({
    super.key,
    required this.onChanged,
    this.label = '',
    this.color = C.accent,
    this.lockX = false,
    this.lockY = false,
    this.size = 150,
  });

  final ValueChanged<Offset> onChanged;
  final String label;
  final Color color;
  final bool lockX;
  final bool lockY;
  final double size;

  @override
  State<Joystick> createState() => _JoystickState();
}

class _JoystickState extends State<Joystick> {
  Offset _value = Offset.zero;
  bool _active = false;

  void _update(Offset local, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final radius = size.shortestSide / 2 - 8;
    var delta = local - center;
    if (delta.distance > radius) {
      delta = delta / delta.distance * radius;
    }
    var v = Offset(delta.dx / radius, delta.dy / radius);
    if (widget.lockX) v = Offset(0, v.dy);
    if (widget.lockY) v = Offset(v.dx, 0);
    setState(() => _value = v);
    widget.onChanged(v);
  }

  void _release() {
    setState(() {
      _value = Offset.zero;
      _active = false;
    });
    widget.onChanged(Offset.zero);
  }

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: widget.size,
      height: widget.size,
      child: LayoutBuilder(
        builder: (context, box) {
          final size = Size(box.maxWidth, box.maxHeight);
          return GestureDetector(
            behavior: HitTestBehavior.opaque,
            onPanDown: (d) {
              _active = true;
              _update(d.localPosition, size);
            },
            onPanUpdate: (d) => _update(d.localPosition, size),
            onPanEnd: (_) => _release(),
            onPanCancel: _release,
            child: Stack(
              alignment: Alignment.center,
              children: [
                CustomPaint(
                  size: size,
                  painter: _StickPainter(
                    value: _value,
                    color: widget.color,
                    active: _active,
                    label: widget.label,
                  ),
                ),
                if (widget.label.isNotEmpty)
                  Positioned(
                    left: 0,
                    right: 0,
                    bottom: 6,
                    // 标签长度不可控（「转向 / 前后」这种），Text 会按内容撑开
                    // 直接画到圆外。给宽度上限 + 缩小，任何标签都不会溢出。
                    child: Center(
                      child: SizedBox(
                        width: widget.size * 0.78,
                        child: FittedBox(
                          fit: BoxFit.scaleDown,
                          child: Text(
                            widget.label,
                            maxLines: 1,
                            style: const TextStyle(
                              fontSize: 10,
                              color: C.faint,
                              letterSpacing: 0.5,
                            ),
                          ),
                        ),
                      ),
                    ),
                  ),
              ],
            ),
          );
        },
      ),
    );
  }
}

class _StickPainter extends CustomPainter {
  _StickPainter({
    required this.value,
    required this.color,
    required this.active,
    required this.label,
  });
  final Offset value;
  final Color color;
  final bool active;
  final String label;

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final radius = size.shortestSide / 2 - 8;

    final base = Paint()
      ..style = PaintingStyle.fill
      ..color = C.surfaceHi.withValues(alpha: active ? 0.9 : 0.55);
    canvas.drawCircle(center, radius, base);

    final ring = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1
      ..color = C.line;
    canvas.drawCircle(center, radius, ring);
    canvas.drawCircle(
      center,
      radius * 0.62,
      ring..color = C.line.withValues(alpha: 0.6),
    );

    // 十字准线：告诉手指「回中在哪」。
    final axis = Paint()
      ..strokeWidth = 1
      ..color = C.faint.withValues(alpha: 0.5);
    canvas.drawLine(
      center - Offset(radius * 0.85, 0),
      center + Offset(radius * 0.85, 0),
      axis,
    );
    canvas.drawLine(
      center - Offset(0, radius * 0.85),
      center + Offset(0, radius * 0.85),
      axis,
    );

    final knob = center + Offset(value.dx * radius, value.dy * radius);
    final glow = Paint()
      ..color = color.withValues(alpha: active ? 0.35 : 0.10)
      ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 12);
    canvas.drawCircle(knob, radius * 0.30, glow);
    canvas.drawCircle(
      knob,
      radius * 0.25,
      Paint()..color = active ? color : color.withValues(alpha: 0.45),
    );
    canvas.drawCircle(
      knob,
      radius * 0.25,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2
        ..color = Colors.white.withValues(alpha: active ? 0.55 : 0.2),
    );
    canvas.drawCircle(knob, radius * 0.08, Paint()
      ..color = C.bg.withValues(alpha: active ? 0.75 : 0.4));

    // 分量刻度：拖动时在边缘画一小段，能看出现在推了多少。
    if (active && !value.offsetZero) {
      final dir = value.distance == 0
          ? const Offset(0, -1)
          : value / value.distance;
      final mark = Paint()
        ..strokeWidth = 3
        ..strokeCap = StrokeCap.round
        ..color = color.withValues(alpha: 0.8);
      canvas.drawLine(
        center + dir * radius * 0.86,
        center + dir * radius,
        mark,
      );
    }
  }

  @override
  bool shouldRepaint(_StickPainter old) =>
      old.value != value || old.active != active;
}

extension on Offset {
  bool get offsetZero => dx.abs() < 1e-4 && dy.abs() < 1e-4;
}

// 一个横向档位条：速度倍率、死区这类「0~1 之间选一个数」的地方用它。
class StepBar extends StatelessWidget {
  const StepBar({
    super.key,
    required this.values,
    required this.value,
    required this.onChanged,
    required this.formatter,
    this.color = C.accent,
  });

  final List<double> values;
  final double value;
  final ValueChanged<double> onChanged;
  final String Function(double) formatter;
  final Color color;

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        for (final v in values)
          Expanded(
            child: Padding(
              padding: const EdgeInsets.only(right: 6),
              child: InkWell(
                borderRadius: BorderRadius.circular(R.ctl),
                onTap: () => onChanged(v),
                child: Container(
                  height: 34,
                  alignment: Alignment.center,
                  decoration: BoxDecoration(
                    color: (v - value).abs() < 1e-6
                        ? color.withValues(alpha: 0.16)
                        : C.surfaceHi,
                    borderRadius: BorderRadius.circular(R.ctl),
                    border: Border.all(
                      color: (v - value).abs() < 1e-6
                          ? color.withValues(alpha: 0.55)
                          : C.line,
                    ),
                  ),
                  child: Text(
                    formatter(v),
                    style: TextStyle(
                      fontSize: 12,
                      fontWeight: FontWeight.w600,
                      color: (v - value).abs() < 1e-6 ? color : C.dim,
                    ),
                  ),
                ),
              ),
            ),
          ),
      ],
    );
  }
}

// 遥测条里那几个小竖条（回路频率之类的历史）。放不下就用它占位。
class MiniBars extends StatelessWidget {
  const MiniBars({
    super.key,
    required this.values,
    required this.max,
    this.color = C.accent,
    this.height = 28,
  });
  final List<double> values;
  final double max;
  final Color color;
  final double height;

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      height: height,
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.end,
        children: [
          for (final v in values)
            Expanded(
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 1),
                child: Container(
                  height: math.max(2, height * (v / max).clamp(0.0, 1.0)),
                  decoration: BoxDecoration(
                    color: color.withValues(alpha: 0.55),
                    borderRadius: BorderRadius.circular(1.5),
                  ),
                ),
              ),
            ),
        ],
      ),
    );
  }
}
