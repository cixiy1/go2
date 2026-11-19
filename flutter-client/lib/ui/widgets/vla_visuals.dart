import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter/material.dart';

import '../../core/models.dart';
import '../../theme.dart';

// 机载画面。VLA 把窗口渲染成 JPEG 写文件，网关按 mtime 变化推过来（5 Hz）。
// `gaplessPlayback` 是必须的：每帧都是新的 Image，不设它画面会在两帧之间闪黑。
class CameraPanel extends StatelessWidget {
  const CameraPanel({
    super.key,
    required this.frame,
    this.frontGap,
    this.alert = false,
    this.running = false,
    this.aspect,
    this.fit = BoxFit.contain,
  });

  final Uint8List? frame;
  final double? frontGap;
  final bool alert;
  final bool running;

  //: 没有画面/读不出尺寸时的兜底比例。有帧时一律用帧自己的比例 ——
  //: 真机是 16:9 鱼眼，按 4:3 显示会把两边切掉，而鱼眼的视野就在两边。
  final double? aspect;

  //: `contain` 显示整幅（看视野用），`cover` 铺满（当背景用）。
  final BoxFit fit;

  @override
  Widget build(BuildContext context) {
    final border = alert ? C.danger : C.line;
    return Container(
      decoration: BoxDecoration(
        color: const Color(0xFF060809),
        borderRadius: BorderRadius.circular(R.card),
        // 正常不画框（和卡片同一套规矩），只有安全闸介入时才亮红边。
        border: alert ? Border.all(color: border, width: 1.6) : null,
      ),
      clipBehavior: Clip.antiAlias,
      child: AspectRatio(
        aspectRatio: aspect ?? 4 / 3,
        child: Stack(
          fit: StackFit.expand,
          children: [
            if (frame != null)
              Image.memory(
                frame!,
                gaplessPlayback: true,
                fit: fit,
                filterQuality: FilterQuality.medium,
              )
            else
              _placeholder(context),
            // 顶部渐变让叠加的文字在亮画面上也读得清。
            const Positioned(
              top: 0,
              left: 0,
              right: 0,
              child: DecoratedBox(
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    begin: Alignment.topCenter,
                    end: Alignment.center,
                    colors: [Color(0xCC000000), Color(0x00000000)],
                  ),
                ),
                child: SizedBox(height: 46),
              ),
            ),
            Positioned(
              top: 8,
              left: 10,
              child: Row(
                children: [
                  Container(
                    width: 6,
                    height: 6,
                    decoration: BoxDecoration(
                      color: running ? C.danger : C.faint,
                      shape: BoxShape.circle,
                    ),
                  ),
                  const SizedBox(width: 6),
                  Text(
                    running ? '机载画面 · 直播' : '机载画面',
                    style: const TextStyle(
                      fontSize: 11,
                      color: Colors.white70,
                      letterSpacing: 0.4,
                    ),
                  ),
                ],
              ),
            ),
            if (frontGap != null)
              Positioned(
                top: 6,
                right: 8,
                child: Container(
                  padding: const EdgeInsets.symmetric(
                    horizontal: 8,
                    vertical: 4,
                  ),
                  decoration: BoxDecoration(
                    color: Colors.black.withValues(alpha: 0.55),
                    borderRadius: BorderRadius.circular(R.inner),
                  ),
                  child: Row(
                    children: [
                      const Icon(
                        Icons.straighten,
                        size: 12,
                        color: Colors.white70,
                      ),
                      const SizedBox(width: 4),
                      Text(
                        '前方 ${frontGap!.toStringAsFixed(2)} m',
                        style: numStyle.copyWith(
                          fontSize: 11,
                          color: frontGap! < 0.9 ? C.warn : Colors.white,
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            if (alert)
              Positioned(
                left: 0,
                right: 0,
                bottom: 0,
                child: Container(
                  color: C.danger.withValues(alpha: 0.22),
                  padding: const EdgeInsets.symmetric(vertical: 5),
                  child: const Text(
                    '安全闸触发 · 机器人正在自行避让',
                    textAlign: TextAlign.center,
                    style: TextStyle(
                      fontSize: 11,
                      color: C.danger,
                      fontWeight: FontWeight.w600,
                    ),
                  ),
                ),
              ),
          ],
        ),
      ),
    );
  }

  // 相机面板在横屏只剩 180 高，这段占位文字本身就有 70，会溢出。
  // 包一层 scaleDown：高度够时是原尺寸，不够时整体等比缩小。
  Widget _placeholder(BuildContext context) => Center(
    child: FittedBox(
      fit: BoxFit.scaleDown,
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
        const Icon(Icons.videocam_off_outlined, size: 26, color: C.faint),
        const SizedBox(height: 8),
        Text(
          running ? '等待第一帧…' : 'VLA 未运行',
          style: const TextStyle(fontSize: 12, color: C.faint),
        ),
        const SizedBox(height: 4),
        const Text(
          '画面要等策略接上之后才有',
          style: TextStyle(fontSize: 10.5, color: C.faint),
        ),
        ],
      ),
    ),
  );
}

// 八方位视野。数据来自 VLA 的 `sectors`：24 条射线归纳成 8 个方向，
// 每个方向取最近的一条。
//
// 为什么不用一条时间曲线：安全闸看的是「哪个方向多近」，雷达图一眼能看出
// 「左边 0.8 米、右边 2.7 米」，曲线要读三个数才反应过来。
class SectorRadar extends StatelessWidget {
  const SectorRadar({
    super.key,
    required this.sectors,
    this.maxRange = 3.0,
    this.frontGap,
    this.maxSide = 240,
    this.showValues = true,
  });

  final List<SectorGap> sectors;
  final double maxRange;
  final double? frontGap;

  //: 画多大。卡片里 240 够，铺成背景时要更大 —— 但仍旧是个上限，
  //: 别让它按宽度铺满再压到别的东西上。
  final double maxSide;

  //: 要不要把距离数字画上去。铺成背景时关掉 —— 那些数字会和大号速度读数
  //: 撞在同一块地方，两组数都读不出来。
  final bool showValues;

  Color _color(double gap) {
    if (gap < 0.6) return C.danger;
    if (gap < 1.2) return C.warn;
    if (gap < 2.0) return C.accent;
    return C.info;
  }

  @override
  Widget build(BuildContext context) {
    if (sectors.isEmpty) {
      return const Center(
        child: Text(
          '没有测距数据\n（--geom off 或雷达没数据）',
          textAlign: TextAlign.center,
          style: TextStyle(fontSize: 11.5, color: C.faint, height: 1.5),
        ),
      );
    }
    // 尺寸要自己兜住。Column 给非弹性子节点的主轴约束是无限的，LayoutBuilder
    // 拿到的 maxHeight 就是 infinity —— `min(w, h)` 会退化成「按宽度铺满」，
    // 画出来的圆比卡片还高，压在下面的内容上。所以这里给一个上限，并接受
    // 「卡片装不下就画小一点」。
    return LayoutBuilder(
      builder: (context, box) {
        final limit = box.maxHeight.isFinite ? box.maxHeight : 220.0;
        final side = math.min(math.min(box.maxWidth, limit), maxSide);
        return Center(
          child: SizedBox(
            width: side,
            height: side,
            child: CustomPaint(
              painter: _RadarPainter(
                sectors: sectors,
                maxRange: maxRange,
                colorOf: _color,
                showValues: showValues,
              ),
            ),
          ),
        );
      },
    );
  }
}

class _RadarPainter extends CustomPainter {
  _RadarPainter({
    required this.sectors,
    required this.maxRange,
    required this.colorOf,
    this.showValues = true,
  });
  final List<SectorGap> sectors;
  final double maxRange;
  final Color Function(double) colorOf;
  final bool showValues;

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final r = size.shortestSide / 2 - 14;

    // 三圈刻度：1 / 2 / 3 m。只有最外圈带标签，画满会吵。
    final ring = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1
      ..color = C.line;
    for (var i = 1; i <= 3; i++) {
      canvas.drawCircle(center, r * i / 3, ring);
    }

    // 八个方位：0 = 正前（屏幕上方），逆时针为正（屏幕左侧）。
    for (var i = 0; i < sectors.length; i++) {
      final s = sectors[i];
      final angle = i * math.pi / 4;
      final dir = Offset(-math.sin(angle), -math.cos(angle));
      final norm = (s.gap / maxRange).clamp(0.06, 1.0);
      final tip = center + dir * (r * norm);
      final color = colorOf(s.gap);

      canvas.drawLine(
        center + dir * (r * 0.16),
        tip,
        Paint()
          ..strokeWidth = 3
          ..strokeCap = StrokeCap.round
          ..color = color.withValues(alpha: 0.65),
      );
      canvas.drawCircle(tip, 4.5, Paint()..color = color);
      canvas.drawCircle(
        tip,
        7,
        Paint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1
          ..color = color.withValues(alpha: 0.35),
      );

      // 正前方那个方位单独标数字，它是安全闸真正用的量。
      if (i == 0 && showValues) {
        _text(
          canvas,
          s.gap.toStringAsFixed(2),
          center + dir * (r * norm + 22),
          color,
          11,
          bold: true,
        );
      }
    }

    // 机身：一个短圆角矩形，机头朝上。
    final body = RRect.fromRectAndRadius(
      Rect.fromCenter(center: center, width: r * 0.30, height: r * 0.42),
      Radius.circular(r * 0.08),
    );
    canvas.drawRRect(body, Paint()..color = C.surfaceHi);
    canvas.drawRRect(
      body,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1
        ..color = C.faint,
    );

    // 机头指示三角。
    final nose = Path()
      ..moveTo(center.dx, center.dy - r * 0.30)
      ..lineTo(center.dx - r * 0.06, center.dy - r * 0.20)
      ..lineTo(center.dx + r * 0.06, center.dy - r * 0.20)
      ..close();
    canvas.drawPath(nose, Paint()..color = C.accent);

    _text(
      canvas,
      '${maxRange.toStringAsFixed(0)} m',
      Offset(center.dx, center.dy + r + 6),
      C.faint,
      9.5,
    );
  }

  void _text(
    Canvas canvas,
    String text,
    Offset at,
    Color color,
    double size, {
    bool bold = false,
  }) {
    final tp = TextPainter(
      text: TextSpan(
        text: text,
        style: TextStyle(
          fontSize: size,
          color: color,
          fontWeight: bold ? FontWeight.w700 : FontWeight.w400,
          fontFeatures: const [FontFeature.tabularFigures()],
        ),
      ),
      textDirection: TextDirection.ltr,
    )..layout();
    tp.paint(canvas, at - Offset(tp.width / 2, tp.height / 2));
  }

  @override
  bool shouldRepaint(_RadarPainter old) =>
      old.sectors != sectors ||
      old.maxRange != maxRange ||
      old.showValues != showValues;
}
