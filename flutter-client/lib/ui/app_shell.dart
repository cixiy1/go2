import 'package:flutter/material.dart';

import '../app_state.dart';
import '../core/models.dart';
import '../theme.dart';
import 'connect_page.dart';
import 'drive_page.dart';
import 'gamepad_page.dart';
import 'settings_page.dart';
import 'vla_page.dart';
import 'widgets/panel.dart';

// 外壳：一条常驻顶栏 + 侧栏/底栏二选一。
//
// 顶栏不随页面变：模式、链路、操作权、急停必须在任何页面都点得到。遥控过程中
// 最怕的就是「想急停但要先退两页」，这个界面里没有那种路径。
class AppShell extends StatefulWidget {
  const AppShell({super.key});

  @override
  State<AppShell> createState() => _AppShellState();
}

class _AppShellState extends State<AppShell> {
  static const _tabs = [
    (icon: Icons.cable_rounded, label: '连接'),
    (icon: Icons.sports_esports_outlined, label: '遥控'),
    (icon: Icons.auto_awesome_outlined, label: 'VLA'),
    (icon: Icons.gamepad_outlined, label: '手柄'),
    (icon: Icons.tune, label: '设置'),
  ];

  @override
  void initState() {
    super.initState();
    // 在 initState 里抓一次 AppState：dispose 里还能用，不依赖 context 仍然有效。
    _app = AppScope.read(context);
    _app.client.notice.addListener(_onNotice);
    _app.client.error.addListener(_onError);
  }

  late final AppState _app;

  void _onNotice() {
    final text = _app.client.notice.value;
    if (text == null || !mounted) return;
    _app.client.notice.value = null;
    ScaffoldMessenger.of(context)
      ..removeCurrentSnackBar()
      ..showSnackBar(_bar(text, C.info));
  }

  void _onError() {
    final text = _app.client.error.value;
    if (text == null || !mounted) return;
    _app.client.error.value = null;
    ScaffoldMessenger.of(context)
      ..removeCurrentSnackBar()
      ..showSnackBar(_bar(text, C.danger));
  }

  SnackBar _bar(String text, Color color) => SnackBar(
    content: Row(
      children: [
        Icon(
          color == C.danger ? Icons.error_outline : Icons.info_outline,
          size: 16,
          color: color,
        ),
        const SizedBox(width: 8),
        Expanded(
          child: Text(
            text,
            style: const TextStyle(fontSize: 12.5, color: C.text),
          ),
        ),
      ],
    ),
    backgroundColor: C.surfaceHi,
    behavior: SnackBarBehavior.floating,
    margin: const EdgeInsets.all(14),
    duration: const Duration(seconds: 3),
    shape: RoundedRectangleBorder(
      borderRadius: BorderRadius.circular(11),
      side: BorderSide(color: color.withValues(alpha: 0.4)),
    ),
  );

  @override
  void dispose() {
    _app.client.notice.removeListener(_onNotice);
    _app.client.error.removeListener(_onError);
    super.dispose();
  }

  Widget _page(int index) => switch (index) {
    0 => const ConnectPage(),
    1 => const DrivePage(),
    2 => const VlaPage(),
    3 => const GamepadPage(),
    _ => const SettingsPage(),
  };

  @override
  Widget build(BuildContext context) {
    final app = AppScope.of(context);
    return ValueListenableBuilder<int>(
      valueListenable: app.tab,
      builder: (context, index, _) {
        return LayoutBuilder(
          builder: (context, box) {
            final wide = box.maxWidth >= 900;
            return Scaffold(
              body: SafeArea(
                child: Column(
                  children: [
                    const _TopBar(),
                    Expanded(
                      child: Row(
                        children: [
                          if (wide) _Rail(index: index, onSelect: app.go),
                          Expanded(child: ClipRect(child: _page(index))),
                        ],
                      ),
                    ),
                  ],
                ),
              ),
              bottomNavigationBar: wide
                  ? null
                  : _BottomBar(index: index, onSelect: app.go),
            );
          },
        );
      },
    );
  }
}

class _Rail extends StatelessWidget {
  const _Rail({required this.index, required this.onSelect});
  final int index;
  final ValueChanged<int> onSelect;

  @override
  Widget build(BuildContext context) {
    return Container(
      width: 96,
      decoration: const BoxDecoration(
        border: Border(right: BorderSide(color: C.line)),
      ),
      child: Column(
        children: [
          const SizedBox(height: 10),
          for (var i = 0; i < _AppShellState._tabs.length; i++)
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 3),
              child: InkWell(
                borderRadius: BorderRadius.circular(11),
                onTap: () => onSelect(i),
                child: Container(
                  height: 56,
                  decoration: BoxDecoration(
                    color: i == index ? C.accent.withValues(alpha: 0.10) : null,
                    borderRadius: BorderRadius.circular(11),
                    border: Border.all(
                      color: i == index
                          ? C.accent.withValues(alpha: 0.28)
                          : Colors.transparent,
                    ),
                  ),
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Icon(
                        _AppShellState._tabs[i].icon,
                        size: 20,
                        color: i == index ? C.accent : C.dim,
                      ),
                      const SizedBox(height: 3),
                      Text(
                        _AppShellState._tabs[i].label,
                        style: TextStyle(
                          fontSize: 10.5,
                          color: i == index ? C.accent : C.faint,
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ),
        ],
      ),
    );
  }
}

class _BottomBar extends StatelessWidget {
  const _BottomBar({required this.index, required this.onSelect});
  final int index;
  final ValueChanged<int> onSelect;

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: const BoxDecoration(
        color: C.surface,
        border: Border(top: BorderSide(color: C.line)),
      ),
      child: SafeArea(
        top: false,
        child: Row(
          children: [
            for (var i = 0; i < _AppShellState._tabs.length; i++)
              Expanded(
                child: InkWell(
                  onTap: () => onSelect(i),
                  child: Padding(
                    padding: const EdgeInsets.symmetric(vertical: 8),
                    child: Column(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        Icon(
                          _AppShellState._tabs[i].icon,
                          size: 20,
                          color: i == index ? C.accent : C.faint,
                        ),
                        const SizedBox(height: 3),
                        Text(
                          _AppShellState._tabs[i].label,
                          style: TextStyle(
                            fontSize: 10,
                            color: i == index ? C.accent : C.faint,
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
              ),
          ],
        ),
      ),
    );
  }
}

class _TopBar extends StatelessWidget {
  const _TopBar();

  String _modeLabel(String mode) => switch (mode) {
    'manual' => '手动',
    'vla' => 'VLA 自主',
    'switching' => '切换中',
    _ => '待机',
  };

  Color _modeColor(String mode) => switch (mode) {
    'manual' => C.accent,
    'vla' => C.vio,
    'switching' => C.warn,
    _ => C.faint,
  };

  @override
  Widget build(BuildContext context) {
    final app = AppScope.read(context);
    return Container(
      height: 56,
      padding: const EdgeInsets.symmetric(horizontal: 14),
      decoration: const BoxDecoration(
        border: Border(bottom: BorderSide(color: C.line)),
      ),
      child: Row(
        children: [
          const Text(
            'GO2',
            style: TextStyle(
              fontSize: 16,
              fontWeight: FontWeight.w800,
              letterSpacing: 2.5,
            ),
          ),
          const SizedBox(width: 10),
          const Text(
            '遥控台',
            style: TextStyle(fontSize: 11, color: C.faint, letterSpacing: 1),
          ),
          const SizedBox(width: 16),
          Expanded(
            child: ValueListenableBuilder<GatewayState?>(
              valueListenable: app.client.state,
              builder: (context, st, _) {
                // 手机上顶栏只有 414 px：三个胶囊加一个急停放不下，最后一个
                // 会被裁掉一半。窄屏只留「模式 + 链路」，操作权在遥控页能看。
                final narrow = MediaQuery.sizeOf(context).width < 520;
                return SingleChildScrollView(
                  scrollDirection: Axis.horizontal,
                  child: Row(
                    children: [
                      StatusChip(
                        text: _modeLabel(st?.mode ?? 'off'),
                        color: _modeColor(st?.mode ?? 'off'),
                        dense: true,
                      ),
                      const SizedBox(width: 6),
                      _linkChip(app, st),
                      const SizedBox(width: 6),
                      // VLA 模式下没有「操作权」这回事 —— 控制权在子进程手里，
                      // 继续显示「无人控制」会让人以为可以抢过来。
                      if (!narrow && st != null && st.mode == 'vla')
                        const StatusChip(
                            text: 'VLA 接管', color: C.vio, dense: true)
                      else if (!narrow && st != null && st.mode == 'manual')
                        StatusChip(
                          text: st.lease.mine
                              ? '我在控制'
                              : (st.lease.holder == null ? '无人控制' : '其他设备控制中'),
                          color: st.lease.mine
                              ? C.accent
                              : (st.lease.holder == null ? C.faint : C.warn),
                          dense: true,
                        ),
                      if (!narrow && st != null && st.link.rttMs != null) ...[
                        const SizedBox(width: 6),
                        StatusChip(
                          text: '${st.link.rttMs!.toStringAsFixed(1)} ms',
                          color: st.link.rttMs! < 30 ? C.faint : C.warn,
                          dense: true,
                        ),
                      ],
                    ],
                  ),
                );
              },
            ),
          ),
          const SizedBox(width: 10),
          _EstopButton(app: app),
        ],
      ),
    );
  }

  Widget _linkChip(AppState app, GatewayState? st) {
    return ValueListenableBuilder<LinkPhase>(
      valueListenable: app.client.phase,
      builder: (context, phase, _) {
        final (text, color) = switch (phase) {
          LinkPhase.open => (
            st?.link.error != null ? '链路异常' : '已连接',
            st?.link.error != null ? C.warn : C.accent,
          ),
          LinkPhase.connecting => ('连接中', C.warn),
          LinkPhase.retrying => ('重连中', C.warn),
          LinkPhase.failed => ('连不上', C.danger),
          LinkPhase.idle => ('未连接', C.faint),
        };
        return StatusChip(text: text, color: color, dense: true);
      },
    );
  }
}

class _EstopButton extends StatelessWidget {
  const _EstopButton({required this.app});
  final AppState app;

  Future<void> _hit(BuildContext context) async {
    final messenger = ScaffoldMessenger.of(context);
    await app.emergencyStop();
    messenger
      ..removeCurrentSnackBar()
      ..showSnackBar(
        SnackBar(
          content: const Text('已急停：零速 + 停步命令'),
          backgroundColor: C.surfaceHi,
          behavior: SnackBarBehavior.floating,
          margin: const EdgeInsets.all(14),
          duration: const Duration(seconds: 2),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(11),
            side: BorderSide(color: C.danger.withValues(alpha: 0.5)),
          ),
        ),
      );
  }

  @override
  Widget build(BuildContext context) {
    return InkWell(
      borderRadius: BorderRadius.circular(10),
      onTap: () => _hit(context),
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
        decoration: BoxDecoration(
          color: C.danger.withValues(alpha: 0.14),
          borderRadius: BorderRadius.circular(10),
          border: Border.all(color: C.danger.withValues(alpha: 0.45)),
        ),
        child: const Row(
          children: [
            Icon(Icons.pan_tool_alt_outlined, size: 15, color: C.danger),
            SizedBox(width: 6),
            Text(
              '急停',
              style: TextStyle(
                fontSize: 12,
                color: C.danger,
                fontWeight: FontWeight.w600,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
