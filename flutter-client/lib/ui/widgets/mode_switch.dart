import 'package:flutter/material.dart';

import '../../app_state.dart';
import '../../core/models.dart';
import '../../theme.dart';

// 模式切换：手动遥控 ↔ VLA 自主。
//
// 两个按钮之间不是「选项卡」，是交接控制权。所以切换过程中显示进度和理由
// （「正在交还机器人控制权（park）…」），而不是一个转圈了事 —— 中间那几秒
// 狗是蹲着的，用户得知道它在干嘛。
class ModeSwitch extends StatelessWidget {
  const ModeSwitch({super.key, this.compact = false});

  final bool compact;

  Future<void> _switch(BuildContext context, String mode) async {
    final app = AppScope.read(context);
    final out = await app.setMode(mode);
    if (out['ok'] != true && context.mounted) {
      app.client.error.value = (out['msg'] ?? '切换失败').toString();
    }
  }

  @override
  Widget build(BuildContext context) {
    final app = AppScope.read(context);
    return ValueListenableBuilder<GatewayState?>(
      valueListenable: app.client.state,
      builder: (context, st, _) {
        final mode = st?.mode ?? 'off';
        final busy = mode == 'switching';
        return Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Row(
              children: [
                Expanded(
                  child: _Tile(
                    label: '手动遥控',
                    sub: '摇杆 / 手柄 / 键盘',
                    icon: Icons.sports_esports_outlined,
                    color: C.accentText,
                    selected: mode == 'manual',
                    enabled: !busy,
                    onTap: () => _switch(context, 'manual'),
                  ),
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: _Tile(
                    label: 'VLA 自主',
                    sub: '看画面、自己想、自己走',
                    icon: Icons.auto_awesome_outlined,
                    color: C.vio,
                    selected: mode == 'vla',
                    enabled: !busy,
                    onTap: () => _switch(context, 'vla'),
                  ),
                ),
              ],
            ),
            if (busy)
              Padding(
                padding: const EdgeInsets.only(top: 8),
                child: Row(
                  children: [
                    const SizedBox(
                      width: 12,
                      height: 12,
                      child: CircularProgressIndicator(
                        strokeWidth: 2,
                        color: C.warn,
                      ),
                    ),
                    const SizedBox(width: 8),
                    Expanded(
                      child: Text(
                        st?.note ?? '正在切换…',
                        style: const TextStyle(fontSize: 11.5, color: C.warn),
                      ),
                    ),
                  ],
                ),
              ),
          ],
        );
      },
    );
  }
}

class _Tile extends StatelessWidget {
  const _Tile({
    required this.label,
    required this.sub,
    required this.icon,
    required this.color,
    required this.selected,
    required this.enabled,
    required this.onTap,
  });

  final String label;
  final String sub;
  final IconData icon;
  final Color color;
  final bool selected;
  final bool enabled;
  final VoidCallback onTap;

  @override
  Widget build(BuildContext context) {
    final active = selected;
    return InkWell(
      borderRadius: BorderRadius.circular(R.ctl),
      onTap: enabled ? onTap : null,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 11),
        decoration: BoxDecoration(
          color: active ? color.withValues(alpha: 0.16) : C.surfaceHi,
          borderRadius: BorderRadius.circular(R.ctl),
        ),
        child: Row(
          children: [
            Icon(icon, size: 17, color: active ? color : C.dim),
            const SizedBox(width: 9),
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                mainAxisSize: MainAxisSize.min,
                children: [
                  Text(
                    label,
                    style: TextStyle(
                      fontSize: 12.5,
                      fontWeight: FontWeight.w600,
                      color: active ? color : (enabled ? C.text : C.faint),
                    ),
                  ),
                  const SizedBox(height: 2),
                  Text(
                    sub,
                    maxLines: 1,
                    overflow: TextOverflow.ellipsis,
                    style: const TextStyle(fontSize: 10.5, color: C.faint),
                  ),
                ],
              ),
            ),
            if (active)
              Icon(Icons.check_circle, size: 15, color: color)
            else if (!enabled)
              const Icon(Icons.hourglass_empty, size: 13, color: C.faint),
          ],
        ),
      ),
    );
  }
}
