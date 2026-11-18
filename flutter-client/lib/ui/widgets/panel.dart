import 'package:flutter/material.dart';

import '../../theme.dart';

// 面板：所有内容都装在这种卡片里。统一圆角、统一边框，界面才不像拼出来的。
class Panel extends StatelessWidget {
  const Panel({
    super.key,
    required this.child,
    this.title,
    this.trailing,
    this.padding = const EdgeInsets.all(14),
    this.color,
    this.border,
    this.clip = false,
  });

  final Widget child;
  final String? title;
  final Widget? trailing;
  final EdgeInsets padding;
  final Color? color;
  final Color? border;
  final bool clip;

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        color: color ?? C.surface,
        borderRadius: BorderRadius.circular(14),
        border: Border.all(color: border ?? C.line),
      ),
      clipBehavior: clip ? Clip.antiAlias : Clip.none,
      child: Padding(
        padding: padding,
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          mainAxisSize: MainAxisSize.min,
          children: [
            if (title != null) ...[
              Row(
                children: [
                  Expanded(child: Kicker(title!)),
                  ?trailing,
                ],
              ),
              const SizedBox(height: 10),
            ],
            child,
          ],
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
        Kicker(label),
        const SizedBox(height: 4),
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
              Text(
                unit!,
                style: const TextStyle(fontSize: 10.5, color: C.faint),
              ),
            ],
          ],
        ),
      ],
    );
    return width == null ? body : SizedBox(width: width, child: body);
  }
}

// 状态小胶囊：一个点 + 一句话。
class StatusChip extends StatelessWidget {
  const StatusChip({
    super.key,
    required this.text,
    this.color = C.dim,
    this.icon,
    this.dense = false,
    this.onTap,
  });

  final String text;
  final Color color;
  final IconData? icon;
  final bool dense;
  final VoidCallback? onTap;

  @override
  Widget build(BuildContext context) {
    final content = Container(
      padding: EdgeInsets.symmetric(
        horizontal: dense ? 7 : 10,
        vertical: dense ? 3 : 5,
      ),
      decoration: BoxDecoration(
        color: color.withValues(alpha: 0.12),
        borderRadius: BorderRadius.circular(999),
        border: Border.all(color: color.withValues(alpha: 0.35)),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          if (icon != null)
            Icon(icon, size: 12, color: color)
          else
            Container(
              width: 6,
              height: 6,
              decoration: BoxDecoration(color: color, shape: BoxShape.circle),
            ),
          const SizedBox(width: 6),
          Text(
            text,
            style: TextStyle(
              fontSize: dense ? 10.5 : 11.5,
              color: color,
              fontWeight: FontWeight.w500,
            ),
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

// 方形图标按钮，带文字。遥控页那一排动作按钮用它。
class ActionButton extends StatelessWidget {
  const ActionButton({
    super.key,
    required this.label,
    required this.icon,
    this.onPressed,
    this.color = C.text,
    this.busy = false,
    this.expand = false,
  });

  final String label;
  final IconData icon;
  final VoidCallback? onPressed;
  final Color color;
  final bool busy;
  final bool expand;

  @override
  Widget build(BuildContext context) {
    final enabled = onPressed != null;
    final child = Container(
      padding: const EdgeInsets.symmetric(vertical: 10, horizontal: 12),
      decoration: BoxDecoration(
        color: enabled ? color.withValues(alpha: 0.10) : C.surfaceHi,
        borderRadius: BorderRadius.circular(11),
        border: Border.all(
          color: enabled ? color.withValues(alpha: 0.30) : C.line,
        ),
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        mainAxisSize: MainAxisSize.min,
        children: [
          if (busy)
            SizedBox(
              width: 14,
              height: 14,
              child: CircularProgressIndicator(strokeWidth: 2, color: color),
            )
          else
            Icon(icon, size: 16, color: enabled ? color : C.faint),
          const SizedBox(width: 7),
          Text(
            label,
            style: TextStyle(
              fontSize: 12.5,
              fontWeight: FontWeight.w500,
              color: enabled ? color : C.faint,
            ),
          ),
        ],
      ),
    );
    return InkWell(
      onTap: onPressed,
      borderRadius: BorderRadius.circular(11),
      child: expand ? SizedBox(width: double.infinity, child: child) : child,
    );
  }
}

// 一行「标签 —— 值」，用于详情列表。
class KeyValue extends StatelessWidget {
  const KeyValue(
    this.label,
    this.value, {
    super.key,
    this.color = C.text,
    this.mono = true,
  });
  final String label;
  final String value;
  final Color color;
  final bool mono;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 3),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          SizedBox(
            width: 86,
            child: Text(
              label,
              style: const TextStyle(fontSize: 11.5, color: C.faint),
            ),
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
