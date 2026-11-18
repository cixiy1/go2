
import 'package:flutter/material.dart';

// 配色是照着「地面站」来的：底色接近黑，信息靠亮度分层，强调色只有两种。
// 机器狗的画面本身就是彩色的，界面再花就看不清它了。
abstract final class C {
  static const bg = Color(0xFF0A0D10);
  static const surface = Color(0xFF12171C);
  static const surfaceHi = Color(0xFF1A222A);
  static const line = Color(0xFF232D36);
  static const text = Color(0xFFE9EFF4);
  static const dim = Color(0xFF8695A3);
  static const faint = Color(0xFF4C5A66);

  // 青绿是主强调色（和 VLA 的萤火虫意象一致），只用在「活的、能动的」东西上。
  static const accent = Color(0xFF3DE0A8);
  static const accentDim = Color(0xFF1C6E56);
  static const warn = Color(0xFFFFB020);
  static const danger = Color(0xFFFF5C5C);
  static const info = Color(0xFF62A8FF);
  static const vio = Color(0xFFB08CFF);
}

// 数字一律等宽。这一屏里所有数字都在跳，比例字宽会让整行左右抖。
const numStyle = TextStyle(
  fontFeatures: [FontFeature.tabularFigures()],
  letterSpacing: -0.2,
);

ThemeData buildTheme() {
  const scheme = ColorScheme.dark(
    primary: C.accent,
    onPrimary: Color(0xFF04150F),
    secondary: C.info,
    surface: C.surface,
    onSurface: C.text,
    error: C.danger,
    onError: Colors.white,
  );
  return ThemeData(
    useMaterial3: true,
    brightness: Brightness.dark,
    colorScheme: scheme,
    scaffoldBackgroundColor: C.bg,
    canvasColor: C.bg,
    dividerColor: C.line,
    fontFamily: null,
    splashFactory: InkSparkle.splashFactory,
    textTheme: const TextTheme(
      titleLarge: TextStyle(fontWeight: FontWeight.w600, letterSpacing: 0.2),
      titleMedium: TextStyle(fontWeight: FontWeight.w600, fontSize: 15),
      bodyMedium: TextStyle(fontSize: 13.5, height: 1.45),
      bodySmall: TextStyle(fontSize: 12, color: C.dim, height: 1.4),
      labelSmall: TextStyle(fontSize: 11, color: C.dim, letterSpacing: 0.6),
    ),
    tooltipTheme: const TooltipThemeData(
      waitDuration: Duration(milliseconds: 400),
    ),
    sliderTheme: SliderThemeData(
      trackHeight: 3,
      activeTrackColor: C.accent,
      inactiveTrackColor: C.line,
      thumbColor: C.text,
      overlayShape: const RoundSliderOverlayShape(overlayRadius: 14),
    ),
    inputDecorationTheme: InputDecorationTheme(
      filled: true,
      fillColor: C.surfaceHi,
      isDense: true,
      hintStyle: const TextStyle(color: C.faint, fontSize: 13),
      contentPadding: const EdgeInsets.symmetric(horizontal: 12, vertical: 12),
      border: OutlineInputBorder(
        borderRadius: BorderRadius.circular(10),
        borderSide: const BorderSide(color: C.line),
      ),
      enabledBorder: OutlineInputBorder(
        borderRadius: BorderRadius.circular(10),
        borderSide: const BorderSide(color: C.line),
      ),
      focusedBorder: OutlineInputBorder(
        borderRadius: BorderRadius.circular(10),
        borderSide: const BorderSide(color: C.accent),
      ),
    ),
  );
}

// 标签小字：全大写英文 + 中文，密度高但不吵。
class Kicker extends StatelessWidget {
  const Kicker(this.text, {super.key, this.color = C.dim});
  final String text;
  final Color color;

  @override
  Widget build(BuildContext context) => Text(
    text,
    style: TextStyle(
      fontSize: 10.5,
      letterSpacing: 1.2,
      color: color,
      fontWeight: FontWeight.w600,
    ),
  );
}
