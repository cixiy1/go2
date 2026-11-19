import 'package:flutter/material.dart';

// 配色直接取 FlClash（chen08209/FlClash）的派生结果，不是照着截图调近似值。
//
// 它的主题是 `seededColorScheme(seed: Color(0xFF007AFF), dark)`（constant.dart:175
// 的 defaultPrimaryColor），卡片色来自 M3 的 surfaceContainer 色阶。这几个色值
// 是用同一个种子在本机跑 ColorScheme.fromSeed 得到的，和它一致到每一位：
//
//     surfaceContainerLowest  #0C0E13   页面底
//     surfaceContainerLow     #1A1B20   卡片
//     surfaceContainerHigh    #282A2F   卡片上的块（输入框、按钮）
//     surfaceContainerHighest #33353A   再上一层（进度槽、浮层）
//
// 色阶整体偏蓝紫而不是中性灰 —— 这是种子色带过来的，照抄中性灰会「看着差不多
// 但就是不像」。
abstract final class C {
  static const bg = Color(0xFF0C0E13);
  static const surface = Color(0xFF1A1B20);
  static const surfaceHi = Color(0xFF282A2F);
  static const surfaceMax = Color(0xFF33353A);

  // 描边只在确实要分隔的地方用（进度槽、表格线），不再包住每张卡片。
  // 取 M3 的 outlineVariant 再压一档：全亮会让无框卡片重新长出灰边。
  static const line = Color(0xFF3A3D45);

  static const text = Color(0xFFE2E2E9);
  static const dim = Color(0xFFA8ABB5);
  static const faint = Color(0xFF75787F);

  // FlClash 的种子色本身，不是 M3 派生的 primary（那是 #ADC6FF，浅蓝，
  // 用在按钮上会没力气）。
  static const accent = Color(0xFF007AFF);

  // 主色作**文字**时要亮一档。M3 在暗色主题里把 primary 提到 tone 80 就是
  // 这个道理：#007AFF 的蓝光分量低，压在 16% 蓝底上对比只有 2.5:1，
  // 「手动遥控」「连接」这类字看着像没写上去。底用 accent，字用 accentText
  // —— 一个给色块，一个给字，别混。
  static const accentText = Color(0xFF6FB4FF);
  static const accentDim = Color(0xFF0B3B66);
  static const warn = Color(0xFFFF9F0A);
  static const danger = Color(0xFFFF453A);
  static const ok = Color(0xFF30D158);
  static const info = Color(0xFF64D2FF);
  static const vio = Color(0xFFBF5AF2);

  // 玻璃。FlClash 的 LiquidGlassStyle.regular 暗色版是「surfaceContainer 提亮
  // + alpha 0.62 + 上缘 0.28 白」。
  //
  // 填充取 surfaceHi 而不是 surface：用 surface 时，0.55 的透明度叠在页面底上
  // 只比背景亮 6/255，侧栏等于看不见（实测 #12141A vs #0C0E13）。纯色背景上
  // 没有可模糊的内容，玻璃的「存在感」只能靠这一档亮度差。
  static const glassFill = Color(0xFF282A2F);
  static const glassRim = 0.18;
}

// 圆角只有三档，别在页面里随手写。超过三档，界面上就会出现「这块比那块圆一点」
// 的细差别，说不清哪里不对但看着脏。
abstract final class R {
  //: 容器：卡片、画面、舞台。
  static const card = 16.0;

  //: 可点的控件：按钮、输入框、模式块、控制端 chip。
  static const ctl = 12.0;

  //: 更小的块：进度槽、代码块、内嵌小标签。
  static const inner = 10.0;

  //: 胶囊。
  static const pill = 999.0;
}

// 数字一律等宽。这一屏里所有数字都在跳，比例字宽会让整行左右抖。
const numStyle = TextStyle(
  fontFeatures: [FontFeature.tabularFigures()],
  letterSpacing: -0.2,
);

ThemeData buildTheme() {
  final scheme = ColorScheme.fromSeed(
    seedColor: const Color(0xFF007AFF),
    brightness: Brightness.dark,
  ).copyWith(
    primary: C.accent,
    onPrimary: Colors.white,
    primaryContainer: C.accentDim,
    onPrimaryContainer: C.text,
    surface: C.bg,
    onSurface: C.text,
    surfaceContainer: C.surface,
    surfaceContainerHigh: C.surfaceHi,
    surfaceContainerHighest: C.surfaceMax,
    onSurfaceVariant: C.dim,
    outline: C.line,
    outlineVariant: C.line,
    error: C.danger,
    onError: Colors.white,
  );

  return ThemeData(
    useMaterial3: true,
    brightness: Brightness.dark,
    colorScheme: scheme,
    // 内嵌子集字体（见 pubspec）。不给 fontFamilyFallback：缺字时交给平台的
    // 中文兜底，比列一串可能不存在的字体名靠谱。
    fontFamily: 'NotoSansSC',
    scaffoldBackgroundColor: C.bg,
    canvasColor: C.bg,
    dividerColor: C.line,
    splashFactory: InkSparkle.splashFactory,
    textTheme: const TextTheme(
      // 字重参照 FlClash：页面标题 20/w700，卡片标题 13/w600。
      titleLarge: TextStyle(
          fontSize: 20, fontWeight: FontWeight.w700, letterSpacing: -0.2),
      titleMedium:
          TextStyle(fontSize: 14, fontWeight: FontWeight.w600, letterSpacing: -0.1),
      bodyMedium: TextStyle(fontSize: 13.5, height: 1.45),
      bodySmall: TextStyle(fontSize: 12, color: C.dim, height: 1.4),
      labelSmall: TextStyle(fontSize: 11, color: C.dim, letterSpacing: 0.2),
    ),
    cardTheme: CardThemeData(
      color: C.surface,
      elevation: 0,
      margin: EdgeInsets.zero,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(R.card)),
    ),
    dialogTheme: DialogThemeData(
      backgroundColor: C.surface,
      surfaceTintColor: Colors.transparent,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(R.card)),
    ),
    snackBarTheme: SnackBarThemeData(
      backgroundColor: C.surfaceMax,
      contentTextStyle: const TextStyle(fontSize: 12.5, color: C.text),
      behavior: SnackBarBehavior.floating,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(R.ctl)),
    ),
    tooltipTheme:
        const TooltipThemeData(waitDuration: Duration(milliseconds: 400)),
    sliderTheme: SliderThemeData(
      trackHeight: 3,
      activeTrackColor: C.accent,
      inactiveTrackColor: C.surfaceMax,
      thumbColor: Colors.white,
      overlayShape: const RoundSliderOverlayShape(overlayRadius: 14),
    ),
    switchTheme: SwitchThemeData(
      thumbColor: WidgetStateProperty.resolveWith(
          (s) => s.contains(WidgetState.selected) ? Colors.white : C.dim),
      trackColor: WidgetStateProperty.resolveWith((s) =>
          s.contains(WidgetState.selected) ? C.accent : C.surfaceMax),
      trackOutlineColor: const WidgetStatePropertyAll(Colors.transparent),
    ),
    inputDecorationTheme: InputDecorationTheme(
      filled: true,
      fillColor: C.surfaceHi,
      isDense: true,
      hintStyle: const TextStyle(color: C.faint, fontSize: 13),
      contentPadding: const EdgeInsets.symmetric(horizontal: 12, vertical: 12),
      border: OutlineInputBorder(
        borderRadius: BorderRadius.circular(R.ctl),
        borderSide: BorderSide.none,
      ),
      enabledBorder: OutlineInputBorder(
        borderRadius: BorderRadius.circular(R.ctl),
        borderSide: BorderSide.none,
      ),
      focusedBorder: OutlineInputBorder(
        borderRadius: BorderRadius.circular(R.ctl),
        borderSide: const BorderSide(color: C.accent, width: 1.2),
      ),
    ),
  );
}

// 卡片标题：FlClash 的小标题是一行灰字，前面常带一个小图标。
class Kicker extends StatelessWidget {
  const Kicker(this.text, {super.key, this.color = C.dim, this.icon,
      this.shadows});
  final String text;
  final Color color;
  final IconData? icon;

  //: 压在画面上时传一组阴影当描边（纯色卡片上不需要）。
  final List<Shadow>? shadows;

  @override
  Widget build(BuildContext context) => Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          if (icon != null) ...[
            // 图标放进固定盒子：不同图标字形宽度不一样，不定宽就一排对不齐。
            SizedBox(
              width: 14,
              height: 14,
              child: Icon(icon, size: 14, color: color),
            ),
            const SizedBox(width: 6),
          ],
          Text(text,
              style: TextStyle(
                  fontSize: 12.5,
                  color: color,
                  fontWeight: FontWeight.w600,
                  shadows: shadows)),
        ],
      );
}
