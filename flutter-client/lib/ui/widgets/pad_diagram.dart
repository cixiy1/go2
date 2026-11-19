import 'package:flutter/material.dart';

import '../../theme.dart';

// 手柄俯视图：一张实物底图 + 一层按键高亮。
//
// 前几版是拿 CustomPainter 手画机身和按键，路走错了 —— 手绘矢量要做到「像一只
// 真手柄」需要的曲线精度远超这里能维护的量，怎么调都像示意图。底图用真实手柄
// 的正面图（`assets/pad/xbox.webp`，491x338 原尺寸，32 KB），形状、比例、材质
// 都是真的；高亮层只负责画「哪个键亮着」。
//
// 坐标不是估的：底图上用霍夫圆检测量出摇杆和 ABXY 的圆心半径，再除以图宽高
// 得到归一化值（见 `_keys` 表）。换底图时重跑一遍脚本、更新那张表就行。
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

  static const _asset = 'assets/pad/xbox.webp';

  @override
  Widget build(BuildContext context) {
    return AspectRatio(
      aspectRatio: padDiagramAspect,
      child: Stack(
        children: [
          Positioned.fill(
            child: Opacity(
              // 没连手柄时压暗，但保留能看清的轮廓 —— 全透明会让人以为图没加载。
              opacity: connected ? 1.0 : 0.55,
              child: Image.asset(_asset, fit: BoxFit.fill),
            ),
          ),
          Positioned.fill(
            child: CustomPaint(
              painter: _Overlay(
                pressed: pressed,
                left: left,
                right: right,
                connected: connected,
              ),
            ),
          ),
        ],
      ),
    );
  }
}

//: 按键在底图上的位置与半径，全部是「相对图宽/高的比例」。
//:
//: 实测来源：源图 491x338，HoughCircles 检出圆心的像素坐标 ÷ 图宽高。
//: 圆心 (x/491, y/338)，半径按图宽换算（r/491）。
class _KeyPos {
  const _KeyPos(this.key, this.x, this.y, this.r);
  final String key;
  final double x;
  final double y;
  final double r;
}

const _keys = <_KeyPos>[
  // 摇杆（实测帽半径 30px，右摇杆在下）
  _KeyPos('leftStick', 0.236, 0.314, 0.061),
  _KeyPos('rightStick', 0.644, 0.562, 0.061),
  // ABXY
  _KeyPos('y', 0.758, 0.213, 0.036),
  _KeyPos('x', 0.692, 0.320, 0.038),
  _KeyPos('b', 0.827, 0.314, 0.036),
  _KeyPos('a', 0.762, 0.420, 0.037),
  // 中键
  _KeyPos('back', 0.399, 0.189, 0.026),
  _KeyPos('start', 0.603, 0.189, 0.026),
  _KeyPos('guide', 0.511, 0.077, 0.032),
  // 肩键与扳机
  _KeyPos('leftBumper', 0.308, 0.062, 0.048),
  _KeyPos('rightBumper', 0.715, 0.062, 0.048),
  _KeyPos('leftTrigger', 0.262, 0.016, 0.038),
  _KeyPos('rightTrigger', 0.752, 0.016, 0.038),
];

// 十字键：中心 + 四个方向各偏移一点
const _dpadCenter = Offset(0.367, 0.544);
const _dpadOffset = 0.045;
const _dpadArmR = 0.030;

class _Overlay extends CustomPainter {
  _Overlay({
    required this.pressed,
    required this.left,
    required this.right,
    required this.connected,
  });

  final Set<String> pressed;
  final Offset left;
  final Offset right;
  final bool connected;

  bool _on(String k) => pressed.contains(k);

  @override
  void paint(Canvas canvas, Size size) {
    final w = size.width;
    final h = size.height;
    Offset at(double nx, double ny) => Offset(nx * w, ny * h);

    for (final k in _keys) {
      if (!_on(k.key)) continue;
      _glow(canvas, at(k.x, k.y), k.r * w, size);
    }

    const dirs = {
      'dpadUp': Offset(0, -1),
      'dpadDown': Offset(0, 1),
      'dpadLeft': Offset(-1, 0),
      'dpadRight': Offset(1, 0),
    };
    for (final e in dirs.entries) {
      if (!_on(e.key)) continue;
      final nx = _dpadCenter.dx + e.value.dx * _dpadOffset;
      final ny = _dpadCenter.dy + e.value.dy * _dpadOffset;
      _glow(canvas, at(nx, ny), _dpadArmR * w, size);
    }

    // 摇杆位置：一个小点跟在当前值后面，看得出「推了多少、往哪推」
    _knobDot(canvas, at(_keys[0].x, _keys[0].y), _keys[0].r * w, left);
    _knobDot(canvas, at(_keys[1].x, _keys[1].y), _keys[1].r * w, right);
  }

  // 按下的按键：半透明填充 + 描边 + 外发光。底图是实物照片，高亮必须够透，
  // 否则会把按键本身的字和质感盖掉 —— 那还不如直接画个圆。
  void _glow(Canvas canvas, Offset c, double r, Size size) {
    canvas.drawCircle(
      c,
      r * 1.25,
      Paint()
        ..color = C.accent.withValues(alpha: 0.45)
        ..maskFilter = MaskFilter.blur(BlurStyle.normal, r * 0.45),
    );
    canvas.drawCircle(c, r, Paint()..color = C.accent.withValues(alpha: 0.55));
    canvas.drawCircle(
      c,
      r,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = (size.width / 260).clamp(1.2, 3.0)
        ..color = Colors.white.withValues(alpha: 0.9),
    );
  }

  void _knobDot(Canvas canvas, Offset c, double r, Offset value) {
    final dot = c + Offset(value.dx * r * 0.45, value.dy * r * 0.45);
    // 底图上摇杆自带凹坑，这个点只是「现在推到哪」。做大了会像眼睛。
    canvas.drawCircle(
      dot,
      r * 0.13,
      Paint()..color = Colors.black.withValues(alpha: 0.3),
    );
    canvas.drawCircle(
      dot,
      r * 0.09,
      Paint()..color = Colors.white.withValues(alpha: 0.7),
    );
  }

  @override
  bool shouldRepaint(_Overlay old) =>
      old.pressed != pressed ||
      old.left != left ||
      old.right != right ||
      old.connected != connected;
}

//: 画布比例（底图 491x338）与页面建议宽度。
const padDiagramAspect = 491 / 338;
const padDiagramMaxWidth = 520.0;
