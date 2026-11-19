import 'dart:ui' as ui;

import 'package:flutter/material.dart';

import '../../theme.dart';

// 毛玻璃层。
//
// FlClash 的玻璃是 prism_ui 的 shader 实现（折射 + 边缘高光，1777 行），照抄
// 不现实；这里用 BackdropFilter 复刻它的观感配方，参数取自
// `LiquidGlassStyle.regular` / `.pill` 的暗色版：
//
//     填充      surfaceContainer 提亮，alpha 0.62（regular）/ 0.48（pill）
//     模糊      hazeBlur 15.6 → sigma 16
//     上缘      0.28 白，0.67 px（kLine）
//     下缘      0.67 px 黑，给一个「有厚度」的收边
//
// 一处刻意的差别：FlClash 的玻璃背后是桌面壁纸，我们背后是纯色页面，模糊出来
// 几乎看不出变化。所以 ApplicationBackdrop 在底色上铺了两团很淡的光晕 ——
// 玻璃要有东西可折射，否则只是「一块稍微浅一点的色块」。
class Glass extends StatelessWidget {
  const Glass({
    super.key,
    required this.child,
    this.radius = R.card,
    this.blur = 16,
    this.alpha = 0.62,
    this.tint,
    this.rim = C.glassRim,
    this.enabled = true,
    this.shadow = true,
    this.padding = EdgeInsets.zero,
  });

  final Widget child;
  final double radius;
  final double blur;
  final double alpha;
  final Color? tint;
  final double rim;

  //: 关掉时退化成纯填充（设置里的开关，也给低端设备留一条路）。
  final bool enabled;
  final bool shadow;
  final EdgeInsets padding;

  @override
  Widget build(BuildContext context) {
    final shape = BorderRadius.circular(radius);
    final fill = tint ?? C.glassFill;

    final body = DecoratedBox(
      decoration: BoxDecoration(
        borderRadius: shape,
        gradient: LinearGradient(
          begin: Alignment.topCenter,
          end: Alignment.bottomCenter,
          colors: [
            fill.withValues(alpha: (alpha + 0.06).clamp(0.0, 1.0)),
            fill.withValues(alpha: (alpha - 0.06).clamp(0.0, 1.0)),
          ],
        ),
      ),
      child: Padding(padding: padding, child: child),
    );

    // 边缘高光用 foregroundDecoration 画，不再套一层 Stack。
    // Stack 的尺寸由非定位子节点决定，而 fit: expand 会把子节点的 tight 约束
    // 反过来依赖 Stack 尺寸 —— 在「只有 top 没有 bottom」的 Positioned（顶栏
    // 就是）里高度会算成 0，整条顶栏直接消失。试过，别再用 Stack。
    final rimmed = DecoratedBox(
      position: DecorationPosition.foreground,
      decoration: BoxDecoration(
        borderRadius: shape,
        border: Border.all(
          color: Colors.white.withValues(alpha: rim),
          width: 0.67,
        ),
      ),
      child: body,
    );

    // Web 上也开模糊：CanvasKit 的 BackdropFilter 在纯色底上不会污染画面
    // （开关两版逐像素比对过，卡片/侧栏同色）。只有背后确实有东西时才有
    // 意义 —— 遥控页「相机/雷达」切换条就压在画面上。
    final blurred = enabled
        ? BackdropFilter(
            filter: ui.ImageFilter.blur(sigmaX: blur, sigmaY: blur),
            child: rimmed,
          )
        : rimmed;

    return DecoratedBox(
      decoration: BoxDecoration(
        borderRadius: shape,
        boxShadow: shadow
            ? const [
                BoxShadow(
                  color: Color(0x59000000),
                  blurRadius: 24,
                  offset: Offset(0, 10),
                ),
              ]
            : null,
      ),
      child: ClipRRect(borderRadius: shape, child: blurred),
    );
  }
}
