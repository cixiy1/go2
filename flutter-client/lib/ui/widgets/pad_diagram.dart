import 'dart:ui' as ui;

import 'package:flutter/material.dart';

import '../../theme.dart';

// 手柄俯视图。拟实物：机身按上亮下暗打光、中间有上壳/下壳的分缝、按键是凸起
// 的圆钮嵌在凹陷底座里、摇杆有凹槽和橡胶纹。
//
// 画矢量而不是贴图，理由和 gamepadviewer / input-overlay 那类项目不同：它们
// 要的是「换皮肤方便」（一张底图 + 每键一个图层），我们要的是「跟着主题色走、
// 任意尺寸不虚、不额外带 asset」。
//
// 几何按 Xbox 手柄正面实拍比例定：机身宽 15.3 cm、高 10.5 cm（含握把），
// 四键中心到中心 2.2 cm，摇杆直径 1.9 cm。换算到 1000 宽的画布上就是下面
// 那几个数，不是凭感觉摆的。
class PadDiagram extends StatelessWidget {
  const PadDiagram({
    super.key,
    required this.pressed,
    required this.left,
    required this.right,
    this.connected = true,
  });

  //: 按下的键名，与 `GamepadButton.name` 一致（a / b / dpadUp / start …）。
  final Set<String> pressed;

  //: 两个摇杆位置，-1~1，屏幕坐标系（y 下正）。
  final Offset left;
  final Offset right;

  final bool connected;

  @override
  Widget build(BuildContext context) {
    return AspectRatio(
      aspectRatio: padDiagramAspect,
      child: CustomPaint(
        painter: _PadPainter(
          pressed: pressed,
          left: left,
          right: right,
          connected: connected,
        ),
      ),
    );
  }
}

class _PadPainter extends CustomPainter {
  _PadPainter({
    required this.pressed,
    required this.left,
    required this.right,
    required this.connected,
  });

  final Set<String> pressed;
  final Offset left;
  final Offset right;
  final bool connected;

  // 未连接时压暗，但只压到「看得出这是只没通电的手柄」为止。
  double get _dim => connected ? 1.0 : 0.86;

  bool _on(String key) => pressed.contains(key);

  // 三档灰：机身 → 按键面 → 标签。全用一个灰就是一块肥皂。
  static const _shellTop = Color(0xFF323840);
  static const _shellBottom = Color(0xFF14171B);
  static const _keyFace = Color(0xFF3C424A);
  static const _keyRim = Color(0xFF596170);
  static const _keyWell = Color(0xFF101317);
  static const _labelIdle = Color(0xFFB4BCC7);

  @override
  void paint(Canvas canvas, Size size) {
    final s = size.width / 1000;
    canvas.save();
    canvas.scale(s, s);

    _body(canvas);
    _shoulders(canvas);
    _dpad(canvas);
    _faceButtons(canvas);
    _stick(canvas, const Offset(318, 288), left, on: _on('leftStick'));
    _stick(canvas, const Offset(678, 486), right, on: _on('rightStick'));
    _centerKeys(canvas);

    canvas.restore();
  }

  // -- 机身 ----------------------------------------------------------------

  Path _bodyPath() {
    // 决定「像不像手柄」的是三处：肩部外扩、腰（两条握把分开的位置）、
    // 握把末端向内勾的圆头。缺一处就会变成带两个圆角的矩形。
    final p = Path();
    p.moveTo(262, 78);
    p.cubicTo(180, 74, 124, 130, 112, 216);
    p.cubicTo(100, 320, 80, 466, 116, 578);
    p.cubicTo(146, 672, 238, 694, 306, 638);
    p.cubicTo(352, 600, 360, 508, 382, 446);
    p.cubicTo(402, 410, 442, 396, 500, 396);
    p.cubicTo(558, 396, 598, 410, 618, 446);
    p.cubicTo(640, 508, 648, 600, 694, 638);
    p.cubicTo(762, 694, 854, 672, 884, 578);
    p.cubicTo(920, 466, 900, 320, 888, 216);
    p.cubicTo(876, 130, 820, 74, 738, 78);
    p.cubicTo(682, 82, 600, 92, 500, 92);
    p.cubicTo(400, 92, 318, 82, 262, 78);
    p.close();
    return p;
  }

  void _body(Canvas canvas) {
    final body = _bodyPath();

    canvas.drawPath(
      body.shift(const Offset(0, 10)),
      Paint()
        ..color = Colors.black.withValues(alpha: 0.42 * _dim)
        ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 20),
    );

    // 机身主体：上亮下暗。光从上方来，握把根部自然落到最暗。
    canvas.drawPath(
      body,
      Paint()
        ..shader = ui.Gradient.linear(
          const Offset(500, 70),
          const Offset(500, 640),
          [
            _shellTop.withValues(alpha: _dim),
            const Color(0xFF23282E).withValues(alpha: _dim),
            _shellBottom.withValues(alpha: _dim),
          ],
          const [0.0, 0.52, 1.0],
        ),
    );

    // 上壳/下壳分缝：一条顺着机身弧线走的暗线。真机上这条线把正面分成
    // 「握把以上」和「握把」两段，少它整块就是平的。
    final seam = Path()
      ..moveTo(118, 380)
      ..cubicTo(220, 442, 340, 456, 500, 456)
      ..cubicTo(660, 456, 780, 442, 882, 380);
    canvas.drawPath(
      seam,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.4
        ..color = Colors.black.withValues(alpha: 0.42 * _dim),
    );
    canvas.drawPath(
      seam.shift(const Offset(0, 2.4)),
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.2
        ..color = Colors.white.withValues(alpha: 0.05 * _dim),
    );

    // 顶部高光：贴着实心边缘的一条窄弧
    canvas.drawPath(
      body,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.2
        ..shader = ui.Gradient.linear(
          const Offset(120, 110),
          const Offset(880, 110),
          [
            Colors.white.withValues(alpha: 0.02 * _dim),
            Colors.white.withValues(alpha: 0.20 * _dim),
            Colors.white.withValues(alpha: 0.02 * _dim),
          ],
          const [0.0, 0.5, 1.0],
        ),
    );
    canvas.drawPath(
      body,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.6
        ..color = C.line.withValues(alpha: _dim),
    );

  }

  // -- 肩键 ----------------------------------------------------------------

  void _shoulders(Canvas canvas) {
    // LB/RB 是从机身顶部伸出来的一块：做成顺着肩线倾斜的形状，前端圆角。
    _bumper(canvas, const Rect.fromLTRB(150, 56, 336, 116), _on('leftBumper'),
        'LB', left: true);
    _bumper(canvas, const Rect.fromLTRB(664, 56, 850, 116), _on('rightBumper'),
        'RB', left: false);
    // LT/RT 在肩键后面，正面只露一条边 —— 画成细条，按下去才亮。
    _trigger(canvas, const Rect.fromLTRB(200, 30, 318, 52), _on('leftTrigger'),
        'LT');
    _trigger(canvas, const Rect.fromLTRB(682, 30, 800, 52), _on('rightTrigger'),
        'RT');
  }

  void _bumper(Canvas canvas, Rect r, bool on, String label,
      {required bool left}) {
    // 左侧右高左低、右侧镜像，跟着机身肩线走。
    final p = Path();
    if (left) {
      p.moveTo(r.left, r.bottom);
      p.lineTo(r.left + 4, r.top + 16);
      p.quadraticBezierTo(r.left + 16, r.top, r.left + 52, r.top);
      p.lineTo(r.right - 48, r.top + 8);
      p.quadraticBezierTo(r.right - 4, r.top + 14, r.right - 2, r.bottom);
    } else {
      p.moveTo(r.right, r.bottom);
      p.lineTo(r.right - 4, r.top + 16);
      p.quadraticBezierTo(r.right - 16, r.top, r.right - 52, r.top);
      p.lineTo(r.left + 48, r.top + 8);
      p.quadraticBezierTo(r.left + 4, r.top + 14, r.left + 2, r.bottom);
    }
    p.close();
    _keyShell(canvas, p, on: on);
    _text(canvas, Offset(r.center.dx, r.center.dy + 6), label,
        size: 17,
        bold: true,
        color: (on ? Colors.white : _labelIdle).withValues(alpha: _dim));
  }

  void _trigger(Canvas canvas, Rect r, bool on, String label) {
    final rr = RRect.fromRectAndRadius(r, Radius.circular(r.height / 2));
    canvas.drawRRect(
      rr,
      Paint()..color = (on ? C.accent : _keyWell).withValues(alpha: _dim),
    );
    canvas.drawRRect(
      rr,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.4
        ..color = (on ? C.accentText : _keyRim).withValues(alpha: 0.85 * _dim),
    );
    _text(canvas, r.center, label, size: 14, bold: true,
        color: (on ? Colors.white : _labelIdle).withValues(alpha: 0.9 * _dim));
  }

  // 按键的通用外壳：凹陷底 + 面 + 描边 + 按下发光。
  void _keyShell(Canvas canvas, Path path, {required bool on}) {
    canvas.drawPath(
      path,
      Paint()..color = _keyWell.withValues(alpha: _dim),
    );
    final face = path.shift(const Offset(0, -1.6));
    canvas.drawPath(
      face,
      Paint()
        ..shader = ui.Gradient.linear(
          const Offset(500, 58),
          const Offset(500, 132),
          [
            (on ? C.accent : _keyFace).withValues(alpha: _dim),
            (on ? C.accent : _keyFace).withValues(alpha: 0.7 * _dim),
          ],
        ),
    );
    if (on) {
      canvas.drawPath(
        face,
        Paint()
          ..color = C.accent.withValues(alpha: 0.42)
          ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 12),
      );
    }
    canvas.drawPath(
      face,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.4
        ..color = (on ? C.accentText : _keyRim).withValues(alpha: _dim),
    );
  }

  // -- 十字键 --------------------------------------------------------------

  void _dpad(Canvas canvas) {
    const c = Offset(322, 476);
    const arm = 54.0;
    const half = 18.0;

    Path bar(bool horizontal, int dir) {
      final p = Path();
      if (horizontal) {
        p.addRRect(RRect.fromRectAndRadius(
          Rect.fromCenter(
              center: Offset(c.dx + dir * arm / 2, c.dy),
              width: arm,
              height: half * 2),
          const Radius.circular(6),
        ));
      } else {
        p.addRRect(RRect.fromRectAndRadius(
          Rect.fromCenter(
              center: Offset(c.dx, c.dy + dir * arm / 2),
              width: half * 2,
              height: arm),
          const Radius.circular(6),
        ));
      }
      return p;
    }

    final keys = {
      'dpadLeft': bar(true, -1),
      'dpadRight': bar(true, 1),
      'dpadUp': bar(false, -1),
      'dpadDown': bar(false, 1),
    };
    // 底座：比十字大一圈的暗色，让它看起来嵌在机身里
    canvas.drawCircle(
      c,
      arm * 0.82,
      Paint()..color = _keyWell.withValues(alpha: 0.5 * _dim),
    );
    for (final e in keys.entries) {
      final on = _on(e.key);
      canvas.drawPath(
        e.value,
        Paint()..color = (on ? C.accent : _keyFace).withValues(alpha: _dim),
      );
      canvas.drawPath(
        e.value,
        Paint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1.2
          ..color = (on ? C.accentText : _keyRim).withValues(alpha: 0.9 * _dim),
      );
    }
    // 中心圆盖住四条臂的接缝
    canvas.drawCircle(c, 13, Paint()..color = _shellTop.withValues(alpha: _dim));
    canvas.drawCircle(
      c,
      13,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.2
        ..color = _keyRim.withValues(alpha: 0.7 * _dim),
    );
  }

  // -- ABXY ----------------------------------------------------------------

  void _faceButtons(Canvas canvas) {
    const c = Offset(682, 288);
    const gap = 60.0;
    const r = 31.0;
    // Xbox 排布：Y 上、X 左、B 右、A 下。字母颜色沿用真机那四色。
    final items = <(String, Offset, String, Color)>[
      ('y', Offset(c.dx, c.dy - gap), 'Y', const Color(0xFFE8B33A)),
      ('x', Offset(c.dx - gap, c.dy), 'X', const Color(0xFF3A7BDE)),
      ('b', Offset(c.dx + gap, c.dy), 'B', const Color(0xFFE2523D)),
      ('a', Offset(c.dx, c.dy + gap), 'A', const Color(0xFF6BBE45)),
    ];
    for (final (key, pos, label, tint) in items) {
      final on = _on(key);
      // 底座凹陷：比按钮大 4px 的一圈暗色
      canvas.drawCircle(
        pos,
        r + 4,
        Paint()..color = _keyWell.withValues(alpha: _dim),
      );
      canvas.drawCircle(
        pos,
        r,
        Paint()
          ..shader = ui.Gradient.radial(
            pos.translate(-8, -10),
            r * 1.55,
            [
              (on ? C.accent : _keyFace).withValues(alpha: _dim),
              (on ? C.accent : const Color(0xFF2E333A)).withValues(alpha: _dim),
            ],
          ),
      );
      if (on) {
        canvas.drawCircle(
          pos,
          r,
          Paint()
            ..color = C.accent.withValues(alpha: 0.5)
            ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 13),
        );
      }
      canvas.drawCircle(
        pos,
        r,
        Paint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1.5
          ..color = (on ? C.accentText : _keyRim).withValues(alpha: _dim),
      );
      _text(canvas, pos, label, size: 23, bold: true,
          color: (on ? Colors.white : tint).withValues(alpha: _dim));
    }
  }

  // -- 摇杆 ----------------------------------------------------------------

  void _stick(Canvas canvas, Offset c, Offset value, {required bool on}) {
    const well = 76.0; // 凹槽
    const cap = 44.0; // 帽

    canvas.drawCircle(
      c,
      well,
      Paint()
        ..shader = ui.Gradient.radial(
          c.translate(0, -10),
          well * 1.2,
          [
            const Color(0xFF191D22).withValues(alpha: _dim),
            const Color(0xFF0D0F12).withValues(alpha: _dim),
          ],
        ),
    );
    canvas.drawCircle(
      c,
      well,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.6
        ..color = _keyRim.withValues(alpha: 0.55 * _dim),
    );

    // 帽跟输入走，行程 ±26，别贴到凹槽边
    final knob = c + Offset(value.dx * 26, value.dy * 26);
    canvas.drawCircle(
      knob.translate(0, 5),
      cap,
      Paint()
        ..color = Colors.black.withValues(alpha: 0.5 * _dim)
        ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 10),
    );
    canvas.drawCircle(
      knob,
      cap,
      Paint()
        ..shader = ui.Gradient.radial(
          knob.translate(-12, -14),
          cap * 1.6,
          [
            (on ? C.accent : const Color(0xFF4E555F)).withValues(alpha: _dim),
            (on ? C.accent : const Color(0xFF262A30)).withValues(alpha: _dim),
          ],
        ),
    );
    // 橡胶纹：两圈同心细线
    for (final rr in [cap - 8, cap - 16]) {
      canvas.drawCircle(
        knob,
        rr,
        Paint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1
          ..color = Colors.white.withValues(alpha: 0.06 * _dim),
      );
    }
    canvas.drawCircle(
      knob,
      cap,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.6
        ..color = (on ? C.accentText : _keyRim).withValues(alpha: _dim),
    );
  }

  // -- 中键 ----------------------------------------------------------------

  void _centerKeys(Canvas canvas) {
    // Xbox 键：正中偏上
    const g = Offset(500, 272);
    final onGuide = _on('guide');
    canvas.drawCircle(
      g,
      30,
      Paint()..color = (onGuide ? C.accent : _keyWell).withValues(alpha: _dim),
    );
    canvas.drawCircle(
      g,
      30,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.4
        ..color = (onGuide ? C.accentText : _keyRim).withValues(alpha: _dim),
    );
    final logo = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 3
      ..strokeCap = StrokeCap.round
      ..color = (onGuide ? Colors.white : _labelIdle).withValues(alpha: _dim);
    canvas.drawLine(g.translate(-8, -8), g.translate(8, 8), logo);
    canvas.drawLine(g.translate(8, -8), g.translate(-8, 8), logo);

    // View / Menu：guide 下方两侧。不写字 ——「⧉」「▷」在缺字体的环境里就是
    // 两个方块，直接画线条更稳。
    _smallKey(canvas, const Offset(446, 344), 'back', _on('back'),
        icon: _IconKind.view);
    _smallKey(canvas, const Offset(554, 344), 'start', _on('start'),
        icon: _IconKind.menu);
  }

  void _smallKey(Canvas canvas, Offset c, String key, bool on,
      {required _IconKind icon}) {
    final r = RRect.fromRectAndRadius(
      Rect.fromCenter(center: c, width: 34, height: 34),
      const Radius.circular(17),
    );
    canvas.drawRRect(
      r,
      Paint()..color = (on ? C.accent : _keyWell).withValues(alpha: _dim),
    );
    canvas.drawRRect(
      r,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.3
        ..color = (on ? C.accentText : _keyRim).withValues(alpha: _dim),
    );
    final ink = Paint()
      ..color = (on ? Colors.white : _labelIdle).withValues(alpha: _dim)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.8
      ..strokeCap = StrokeCap.round;
    if (icon == _IconKind.view) {
      canvas.drawRect(
          Rect.fromCenter(center: c.translate(-2.5, -2.5), width: 10, height: 10),
          ink);
      canvas.drawRect(
          Rect.fromCenter(center: c.translate(2.5, 2.5), width: 10, height: 10),
          ink);
    } else {
      for (var i = -1; i <= 1; i++) {
        canvas.drawLine(c.translate(-8, i * 6), c.translate(8, i * 6), ink);
      }
    }
  }

  // -- 文字 ----------------------------------------------------------------

  void _text(Canvas canvas, Offset c, String s,
      {required double size, Color color = Colors.white, bool bold = false}) {
    final tp = TextPainter(
      text: TextSpan(
        text: s,
        style: TextStyle(
          fontSize: size,
          height: 1,
          color: color,
          fontWeight: bold ? FontWeight.w700 : FontWeight.w500,
        ),
      ),
      textDirection: TextDirection.ltr,
    )..layout();
    tp.paint(canvas, c - Offset(tp.width / 2, tp.height / 2));
  }

  @override
  bool shouldRepaint(_PadPainter old) =>
      old.pressed != pressed ||
      old.left != left ||
      old.right != right ||
      old.connected != connected;
}

enum _IconKind { view, menu }

//: 画布比例与建议宽度，页面用 const 约束就好，别在构建里散落魔法数。
const padDiagramAspect = 1000 / 700;
const padDiagramMaxWidth = 580.0;
