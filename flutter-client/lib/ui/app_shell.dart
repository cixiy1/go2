import 'package:flutter/material.dart';

import '../app_state.dart';
import '../core/models.dart';
import '../theme.dart';
import 'connect_page.dart';
import 'drive_page.dart';
import 'gamepad_page.dart';
import 'settings_page.dart';
import 'vla_page.dart';
import 'widgets/glass.dart';
import 'widgets/panel.dart';

// 外壳参照 FlClash：桌面是一条窄图标栏 + 顶部标题栏，手机是悬浮的胶囊底栏。
//
// 顶栏不随页面变：模式、链路、操作权、急停必须在任何页面都点得到。遥控过程中
// 最怕「想急停但先要退两页」，这个界面里没有那种路径。
class AppShell extends StatefulWidget {
  const AppShell({super.key});

  @override
  State<AppShell> createState() => _AppShellState();
}

class _AppShellState extends State<AppShell> {
  static const _tabs = [
    (icon: Icons.cable_rounded, label: '连接', title: '连接网关'),
    (icon: Icons.sports_esports_outlined, label: '遥控', title: '遥控'),
    (icon: Icons.auto_awesome_outlined, label: 'VLA', title: 'VLA 自主'),
    (icon: Icons.gamepad_outlined, label: '手柄', title: '手柄'),
    (icon: Icons.tune, label: '设置', title: '设置'),
  ];

  late final AppState _app;

  @override
  void initState() {
    super.initState();
    // 在 initState 里抓一次 AppState：dispose 里还能用，不依赖 context 仍然有效。
    _app = AppScope.read(context);
    _app.client.notice.addListener(_onNotice);
    _app.client.error.addListener(_onError);
  }

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
        content: Row(children: [
          Icon(color == C.danger ? Icons.error_outline : Icons.info_outline,
              size: 16, color: color),
          const SizedBox(width: 8),
          Expanded(
              child: Text(text,
                  style: const TextStyle(fontSize: 12.5, color: C.text))),
        ]),
        backgroundColor: C.surfaceMax,
        behavior: SnackBarBehavior.floating,
        margin: const EdgeInsets.all(16),
        duration: const Duration(seconds: 3),
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(12),
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
        return LayoutBuilder(builder: (context, box) {
          final wide = box.maxWidth >= 900;
          // 横屏手机：896x414 这类。宽没到桌面档，但竖排布局在 414 高里塞不下
          // （指标条 64 + 舞台 + 动作 + 档位），必须换一套横排。
          final landscape = !wide && box.maxWidth > box.maxHeight * 1.25;
          return ListenableBuilder(
            listenable: app.settings,
            builder: (context, _) {
              final glass = app.settings.glass;
              final safeBottom = MediaQuery.paddingOf(context).bottom;
              // 内容铺到栏底下：玻璃要有东西可折射，否则只是「一块浅一点的
              // 色块」。代价是每页自己要留出栏的宽度 —— 所以内容用 Padding
              // 让开，而不是让栏占位把内容挤到一边。
              final contentPad = wide
                  ? const EdgeInsets.fromLTRB(86, 76, 12, 12)
                  : landscape
                      ? const EdgeInsets.fromLTRB(10, 60, 10, 52)
                      : EdgeInsets.fromLTRB(10, 72, 10, 88 + safeBottom * 0.4);
              return Scaffold(
                // 不能给 transparent：Web 上 CanvasKit 的底是白画布，
                // BackdropFilter 会采到那层白，玻璃整条发白（试过，侧栏能亮到
                // #4a4d52）。给不透明底色，ApplicationBackdrop 再盖一层光。
                backgroundColor: C.bg,
                body: Stack(
                  children: [
                    Positioned.fill(
                      child: SafeArea(
                        bottom: false,
                        child: Stack(
                          children: [
                            Positioned.fill(
                              child: Padding(
                                padding: contentPad,
                                child: ClipRect(child: _page(index)),
                              ),
                            ),
                            if (wide)
                              Positioned(
                                left: 8,
                                top: 8,
                                bottom: 8,
                                width: 68,
                                child: _Rail(
                                  index: index,
                                  onSelect: app.go,
                                  glass: glass,
                                ),
                              ),
                            Positioned(
                              left: wide ? 86 : 10,
                              right: wide ? 12 : 10,
                              top: 8,
                              child: _TopBar(
                                title: _tabs[index].title,
                                glass: glass,
                                wide: wide,
                                height: landscape ? 46 : 58,
                              ),
                            ),
                            if (!wide)
                              Positioned(
                                left: 10,
                                right: 10,
                                bottom: landscape
                                    ? 6
                                    : 8 + safeBottom * 0.4,
                                child: _Dock(
                                  index: index,
                                  onSelect: app.go,
                                  glass: glass,
                                  height: landscape ? 42 : 58,
                                ),
                              ),
                          ],
                        ),
                      ),
                    ),
                  ],
                ),
              );
            },
          );
        });
      },
    );
  }
}

// 桌面侧栏：FlClash 是窄条 + 纯图标，选中态用一块圆角高亮，不画边框。
// 这一条是毛玻璃 —— FlClash 的侧栏同样浮在内容上，不是把内容推到一边的背景条。
class _Rail extends StatelessWidget {
  const _Rail({
    required this.index,
    required this.onSelect,
    required this.glass,
  });
  final int index;
  final ValueChanged<int> onSelect;
  final bool glass;

  @override
  Widget build(BuildContext context) {
    return Glass(
      enabled: glass,
      radius: R.card,
      // pill 的配方：alpha 0.48，比卡片玻璃更透一点，让底下滚过的画面透出来。
      alpha: 0.48,
      child: Column(
        children: [
          const SizedBox(height: 12),
          // 站标：一个圆角方块，和侧栏其他按钮同尺寸。
          Container(
            width: 44,
            height: 44,
            alignment: Alignment.center,
            decoration: BoxDecoration(
              color: C.accent,
              borderRadius: BorderRadius.circular(R.ctl),
            ),
            child: const Text('G2',
                style: TextStyle(
                    fontSize: 13,
                    fontWeight: FontWeight.w800,
                    color: Colors.white,
                    letterSpacing: 0.5)),
          ),
          const SizedBox(height: 16),
          for (var i = 0; i < _AppShellState._tabs.length; i++)
            Padding(
              padding: const EdgeInsets.only(bottom: 6),
              child: Tooltip(
                message: _AppShellState._tabs[i].title,
                child: InkWell(
                  borderRadius: BorderRadius.circular(R.ctl),
                  onTap: () => onSelect(i),
                  child: Container(
                    width: 46,
                    height: 46,
                    decoration: BoxDecoration(
                      color: i == index ? C.accent.withValues(alpha: 0.18)
                          : Colors.transparent,
                      borderRadius: BorderRadius.circular(R.ctl),
                    ),
                    child: Icon(
                      _AppShellState._tabs[i].icon,
                      size: 21,
                      color: i == index ? C.accentText : C.dim,
                    ),
                  ),
                ),
              ),
            ),
        ],
      ),
    );
  }
}

// 手机底栏：FlClash 用悬浮胶囊，不贴屏幕边，并且浮在内容上。
class _Dock extends StatelessWidget {
  const _Dock({
    required this.index,
    required this.onSelect,
    required this.glass,
    this.height = 58,
  });
  final int index;
  final ValueChanged<int> onSelect;
  final bool glass;
  final double height;

  @override
  Widget build(BuildContext context) {
    return Glass(
      enabled: glass,
      radius: R.pill,
      alpha: 0.66,
      child: SizedBox(
        height: height,
        child: Row(
          children: [
            for (var i = 0; i < _AppShellState._tabs.length; i++)
              Expanded(
                child: InkWell(
                  borderRadius: BorderRadius.circular(R.pill),
                  onTap: () => onSelect(i),
                  // 竖屏：图标在上、文字在下。横屏只有 44 高，塞不下两行，
                  // 改成图标和文字同一行。
                  child: height < 50
                      ? Row(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: [
                            Icon(_AppShellState._tabs[i].icon,
                                size: 17,
                                color: i == index ? C.accentText : C.faint),
                            const SizedBox(width: 5),
                            Text(_AppShellState._tabs[i].label,
                                style: TextStyle(
                                    fontSize: 10.5,
                                    fontWeight: i == index
                                        ? FontWeight.w600
                                        : FontWeight.w400,
                                    color:
                                        i == index ? C.accentText : C.faint)),
                          ],
                        )
                      : Column(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: [
                            Icon(_AppShellState._tabs[i].icon,
                                size: 20,
                                color: i == index ? C.accentText : C.faint),
                            const SizedBox(height: 2),
                            Text(_AppShellState._tabs[i].label,
                                style: TextStyle(
                                    fontSize: 10,
                                    fontWeight: i == index
                                        ? FontWeight.w600
                                        : FontWeight.w400,
                                    color:
                                        i == index ? C.accentText : C.faint)),
                          ],
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
  const _TopBar({
    required this.title,
    required this.glass,
    required this.wide,
    this.height = 58,
  });
  final String title;
  final bool glass;
  final bool wide;
  //: 横屏手机只有 414 高，58 的顶栏 + 58 的底栏会把内容压到 200 出头。
  final double height;

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
    return Glass(
      enabled: glass,
      radius: R.card,
      child: SizedBox(
        height: height,
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 14),
          child: Row(
            children: [
              Text(title,
                  style: const TextStyle(
                      fontSize: 19,
                      fontWeight: FontWeight.w700,
                      letterSpacing: -0.2)),
              const SizedBox(width: 14),
              Expanded(
                child: ValueListenableBuilder<GatewayState?>(
                  valueListenable: app.client.state,
                  builder: (context, st, _) {
                    final narrow = !wide || MediaQuery.sizeOf(context).width < 560;
                    return SingleChildScrollView(
                      scrollDirection: Axis.horizontal,
                      child: Row(children: [
                        StatusChip(
                          text: _modeLabel(st?.mode ?? 'off'),
                          color: _modeColor(st?.mode ?? 'off'),
                          dense: true,
                        ),
                        const SizedBox(width: 6),
                        _linkChip(app, st),
                        if (st != null && st.mode == 'vla') ...[
                          const SizedBox(width: 6),
                          const StatusChip(text: 'VLA 接管', color: C.vio,
                              dense: true),
                        ] else if (!narrow && st != null && st.mode == 'manual') ...[
                          const SizedBox(width: 6),
                          StatusChip(
                            text: st.lease.mine
                                ? '我在控制'
                                : (st.lease.holder == null
                                    ? '无人控制'
                                    : '其他设备控制中'),
                            color: st.lease.mine
                                ? C.accent
                                : (st.lease.holder == null ? C.faint : C.warn),
                            dense: true,
                            filled: st.lease.mine,
                          ),
                        ],
                        if (!narrow && st != null && st.link.rttMs != null) ...[
                          const SizedBox(width: 6),
                          StatusChip(
                            text: '${st.link.rttMs!.toStringAsFixed(0)} ms',
                            color: st.link.rttMs! < 30 ? C.dim : C.warn,
                            dense: true,
                          ),
                        ],
                      ]),
                    );
                  },
                ),
              ),
              const SizedBox(width: 10),
              _EstopButton(app: app),
            ],
          ),
        ),
      ),
    );
  }

  Widget _linkChip(AppState app, GatewayState? st) {
    return ValueListenableBuilder<LinkPhase>(
      valueListenable: app.client.phase,
      builder: (context, phase, _) {
        final (text, color) = switch (phase) {
          LinkPhase.open => (st?.link.error != null ? '链路异常' : '已连接',
              st?.link.error != null ? C.warn : C.ok),
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

// 急停：实心红，尺寸比旁边的 chip 大一圈，永远在最右。
class _EstopButton extends StatelessWidget {
  const _EstopButton({required this.app});
  final AppState app;

  Future<void> _hit(BuildContext context) async {
    final messenger = ScaffoldMessenger.of(context);
    await app.emergencyStop();
    messenger
      ..removeCurrentSnackBar()
      ..showSnackBar(SnackBar(
        content: const Text('已急停：零速 + 停步命令'),
        backgroundColor: C.surfaceMax,
        behavior: SnackBarBehavior.floating,
        margin: const EdgeInsets.all(16),
        duration: const Duration(seconds: 2),
        shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(12)),
      ));
  }

  @override
  Widget build(BuildContext context) {
    return InkWell(
      borderRadius: BorderRadius.circular(12),
      onTap: () => _hit(context),
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 13, vertical: 9),
        decoration: BoxDecoration(
          color: C.danger,
          borderRadius: BorderRadius.circular(12),
        ),
        child: const Row(children: [
          Icon(Icons.pan_tool_alt_outlined, size: 15, color: Colors.white),
          SizedBox(width: 6),
          Text('急停',
              style: TextStyle(fontSize: 12.5, color: Colors.white,
                  fontWeight: FontWeight.w600)),
        ]),
      ),
    );
  }
}
