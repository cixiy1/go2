import 'package:flutter/material.dart';

import '../../theme.dart';

// 面板。FlClash 的卡片是「填充 + 大圆角 + 无边框」，靠比背景亮一档的颜色
// 分层；描边只在需要额外强调时传进来（比如危险区）。
class Panel extends StatelessWidget {
  const Panel({
    super.key,
    required this.child,
    this.title,
    this.icon,
    this.trailing,
    this.padding = const EdgeInsets.all(16),
    this.color,
    this.border,
    this.clip = false,
    this.fill = false,
  });

  final Widget child;
  final String? title;
  final IconData? icon;
  final Widget? trailing;
  final EdgeInsets padding;
  final Color? color;
  final Color? border;
  final bool clip;

  //: true = 内容包一层 Expanded（占满卡片剩下的高度）。
  //:
  //: 只在外层确实给了卡片一个高度时才能用（`Expanded(child: Panel(...))`、
  //: `SizedBox(height:)`）。放在「按内容撑高」的位置上会抛
  //: 「non-zero flex but unbounded height」—— 所以默认关，由调用方声明。
  final bool fill;

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        color: color ?? C.surface,
        borderRadius: BorderRadius.circular(R.card),
        border: border == null ? null : Border.all(color: border!),
      ),
      clipBehavior: clip ? Clip.antiAlias : Clip.none,
      // 一层透明 Material。Panel 自己是个带底色的 DecoratedBox，而卡片里的
      // `SwitchListTile` / `Slider` 要把水波纹画在最近的 Material 上 —— 中间
      // 隔着这层底色，水波纹就画在它下面，看不见。debug 下直接抛
      // 「ListTile background color or ink splashes may be invisible」。
      // 放在 Panel 里，所有卡片一次解决，不用逐处包。
      child: Material(
        type: MaterialType.transparency,
        child: Padding(
          padding: padding,
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            mainAxisSize: MainAxisSize.min,
            children: [
              if (title != null) ...[
                Row(
                  children: [
                    Expanded(child: Kicker(title!, icon: icon)),
                    ?trailing,
                  ],
                ),
                const SizedBox(height: 12),
              ],
              fill ? Expanded(child: child) : child,
            ],
          ),
        ),
      ),
    );
  }
}

// 指标格子：标签在上、数字在下。数字用等宽字体，跳的时候不抖。
class MetricTile extends StatelessWidget {
  const MetricTile({
    super.key,
    required this.label,
    required this.value,
    this.unit,
    this.color = C.text,
    this.width,
    this.emphasis = false,
  });

  final String label;
  final String value;
  final String? unit;
  final Color color;
  final double? width;
  final bool emphasis;

  @override
  Widget build(BuildContext context) {
    final body = Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      mainAxisSize: MainAxisSize.min,
      children: [
        Text(label,
            style: const TextStyle(
                fontSize: 11, color: C.dim, fontWeight: FontWeight.w500)),
        const SizedBox(height: 3),
        Row(
          crossAxisAlignment: CrossAxisAlignment.baseline,
          textBaseline: TextBaseline.alphabetic,
          children: [
            Flexible(
              child: Text(
                value,
                maxLines: 1,
                overflow: TextOverflow.ellipsis,
                style: numStyle.copyWith(
                  fontSize: emphasis ? 22 : 17,
                  fontWeight: FontWeight.w600,
                  color: color,
                  height: 1.15,
                ),
              ),
            ),
            if (unit != null) ...[
              const SizedBox(width: 3),
              Text(unit!,
                  style: const TextStyle(fontSize: 10.5, color: C.faint)),
            ],
          ],
        ),
      ],
    );
    // 宽度要自己兜住。下面那个 Row 里有个 Flexible —— 放在横向滚动的
    // `SingleChildScrollView` 里时，父级给的是无界宽度，Flexible 必然抛
    // 「RenderFlex children have non-zero flex but incoming width constraints
    // are unbounded」。遥控页的状态条和 VLA 页的指标条都是横滚的，整页直接
    // 白掉。有界时照旧铺满；无界时给一个上限，短值仍然按内容宽度收缩。
    return LayoutBuilder(
      builder: (context, box) {
        final cap = width ?? (box.hasBoundedWidth ? null : 160.0);
        return cap == null ? body : SizedBox(width: cap, child: body);
      },
    );
  }
}

// 状态胶囊。FlClash 的 chip 是纯填充 + 全圆角、没有描边：
// 普通态用主色的低透明度底，强调态（filled）直接用实心主色。
class StatusChip extends StatelessWidget {
  const StatusChip({
    super.key,
    required this.text,
    this.color = C.dim,
    this.icon,
    this.dense = false,
    this.onTap,
    this.filled = false,
  });

  final String text;
  final Color color;
  final IconData? icon;
  final bool dense;
  final VoidCallback? onTap;

  //: true = 实心强调（主色底 + 白字），用于「我在控制」这类当前态。
  final bool filled;

  @override
  Widget build(BuildContext context) {
    // 非实心 chip 的文字：accent 在暗底上偏暗，提一档再用。其余颜色
    // （紫/绿/橙/红）本身就是给暗色主题挑的，不动。
    final fg = filled
        ? Colors.white
        : (color == C.accent ? C.accentText : color);
    final bg = filled ? color : color.withValues(alpha: 0.13);
    final content = Container(
      padding: EdgeInsets.symmetric(
          horizontal: dense ? 9 : 11, vertical: dense ? 4 : 6),
      decoration: BoxDecoration(
        color: bg,
        borderRadius: BorderRadius.circular(999),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          // 固定 12 宽的图标位：有点用点、有图标用图标，文字的起始 x 都不变。
          SizedBox(
            width: 12,
            height: 12,
            child: icon != null
                ? Icon(icon, size: 12, color: fg)
                : Center(
                    child: Container(
                      width: 6,
                      height: 6,
                      decoration:
                          BoxDecoration(color: fg, shape: BoxShape.circle),
                    ),
                  ),
          ),
          const SizedBox(width: 6),
          Flexible(
            child: Text(text,
                maxLines: 1,
                overflow: TextOverflow.ellipsis,
                style: TextStyle(
                    fontSize: dense ? 11 : 12,
                    color: fg,
                    fontWeight: FontWeight.w500)),
          ),
        ],
      ),
    );
    if (onTap == null) return content;
    return InkWell(
      onTap: onTap,
      borderRadius: BorderRadius.circular(999),
      child: content,
    );
  }
}

// 方块按钮。默认 tonal（灰底、无边框），主操作用 filled。
class ActionButton extends StatelessWidget {
  const ActionButton({
    super.key,
    required this.label,
    required this.icon,
    this.onPressed,
    this.color = C.text,
    this.busy = false,
    this.expand = false,
    this.filled = false,
  });

  final String label;
  final IconData icon;
  final VoidCallback? onPressed;
  final Color color;
  final bool busy;
  final bool expand;
  final bool filled;

  @override
  Widget build(BuildContext context) {
    final enabled = onPressed != null;
    final bg = !enabled ? C.surfaceHi : (filled ? color : C.surfaceHi);
    final fg = !enabled ? C.faint : (filled ? Colors.white : color);
    final child = Container(
      // 横向 9 而不是 12：遥控页手机上是五个按钮等宽并排，414 pt 宽时每个只有
      // 69 pt，原来的内边距把「起立」那一格撑出 3.3 px（右边缘黄黑条纹）。
      padding: const EdgeInsets.symmetric(vertical: 11, horizontal: 9),
      decoration: BoxDecoration(
        color: bg,
        borderRadius: BorderRadius.circular(R.ctl),
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        mainAxisSize: MainAxisSize.min,
        children: [
          // 图标位固定 16：按钮并排时文字基线才一致。
          SizedBox(
            width: 16,
            height: 16,
            child: busy
                ? CircularProgressIndicator(strokeWidth: 2, color: fg)
                : Icon(icon, size: 16, color: fg),
          ),
          const SizedBox(width: 7),
          // 标签可压缩：窄屏上宁可截断，也不要整行溢出。
          Flexible(
            child: Text(label,
                maxLines: 1,
                overflow: TextOverflow.ellipsis,
                style: TextStyle(
                    fontSize: 12.5, fontWeight: FontWeight.w500, color: fg)),
          ),
        ],
      ),
    );
    return InkWell(
      onTap: onPressed,
      borderRadius: BorderRadius.circular(R.ctl),
      child: expand ? SizedBox(width: double.infinity, child: child) : child,
    );
  }
}

// 一行「标签 —— 值」。
class KeyValue extends StatelessWidget {
  const KeyValue(this.label, this.value, {super.key, this.color = C.text,
      this.mono = true});
  final String label;
  final String value;
  final Color color;
  final bool mono;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          SizedBox(
            width: 84,
            child: Text(label,
                style: const TextStyle(fontSize: 11.5, color: C.dim)),
          ),
          Expanded(
            child: Text(
              value,
              style: TextStyle(
                fontSize: 12.5,
                color: color,
                height: 1.35,
                fontFeatures: mono ? numStyle.fontFeatures : null,
              ),
            ),
          ),
        ],
      ),
    );
  }
}
