import 'dart:math' as math;

import 'package:flutter/material.dart';

import '../app_state.dart';
import '../core/models.dart';
import '../input/intent.dart';
import '../theme.dart';
import 'widgets/mode_switch.dart';
import 'widgets/glass.dart';
import 'widgets/joystick.dart';
import 'widgets/vla_visuals.dart';
import 'widgets/panel.dart';

// 遥控页。布局参照 Unitree 官方 App：左手平移、右手转向、中间是动作。
//
// 和官方 App 的两处不同，都是有意的：
//
//   * 使能（armed）默认关。官方是「进页面就能推」，这里要先打开一个开关才
//     下发速度 —— 手机在口袋里被碰到、平板放在桌上被猫踩到，都不该让狗动。
//   * 顶部一直有急停。不是在页面里藏一个「停止」，是在任何页面都点得到。
class DrivePage extends StatefulWidget {
  const DrivePage({super.key});

  @override
  State<DrivePage> createState() => _DrivePageState();
}

class _DrivePageState extends State<DrivePage> {
  Offset _move = Offset.zero;
  Offset _turn = Offset.zero;
  late final AppState _app;

  @override
  void initState() {
    super.initState();
    _app = AppScope.read(context);
    _app.keyboard.attach();
  }

  @override
  void dispose() {
    _app.keyboard.detach();
    _app.router.touch.value = ControlIntent.zero;
    super.dispose();
  }

  void _emitTouch() {
    final s = _app.settings;
    _app.router.touch.value = ControlIntent(
      vx: s.shape(-_move.dy),
      vy: s.shape(-_move.dx),
      vyaw: -s.shape(_turn.dx),
      source: '摇杆',
    );
  }

  @override
  Widget build(BuildContext context) {
    final app = AppScope.of(context);
    return ValueListenableBuilder<GatewayState?>(
      valueListenable: app.client.state,
      builder: (context, st, _) {
        return LayoutBuilder(
          builder: (context, box) {
            final wide = box.maxWidth >= 1000;
            return Padding(
              padding: const EdgeInsets.all(12),
              child: wide ? _wide(st, app) : _narrow(st, app),
            );
          },
        );
      },
    );
  }

  // -- 宽屏 --------------------------------------------------------------

  Widget _wide(GatewayState? st, AppState app) {
    return Row(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        SizedBox(width: 292, child: _leftColumn(st, app)),
        const SizedBox(width: 12),
        Expanded(
          child: Column(children: [
            Expanded(child: _stage(st, app)),
            const SizedBox(height: 10),
            _actions(app, st),
            const SizedBox(height: 10),
            _throttle(app, st),
          ]),
        ),
        const SizedBox(width: 12),
        SizedBox(width: 268, child: _rightColumn(st, app)),
      ],
    );
  }

  Widget _leftColumn(GatewayState? st, AppState app) {
    return SingleChildScrollView(
      child: Column(
        children: [
          Panel(title: '接管方式', child: ModeSwitch(compact: true)),
          const SizedBox(height: 12),
          Panel(
            title: '机器人',
            child: Column(
              children: [
                MetricTile(
                  label: '机身高度',
                  value: st?.robot.height == null
                      ? '—'
                      : st!.robot.height!.toStringAsFixed(3),
                  unit: 'm',
                  emphasis: true,
                ),
                const SizedBox(height: 10),
                Row(
                  children: [
                    Expanded(
                      child: MetricTile(
                        label: 'Roll',
                        value: _deg(st?.robot.roll),
                        unit: '°',
                      ),
                    ),
                    Expanded(
                      child: MetricTile(
                        label: 'Pitch',
                        value: _deg(st?.robot.pitch),
                        unit: '°',
                      ),
                    ),
                    Expanded(
                      child: MetricTile(
                        label: 'Yaw',
                        value: _deg(st?.robot.yaw),
                        unit: '°',
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 10),
                MetricTile(
                  label: '位置 x / y',
                  value: st?.robot.pos == null
                      ? '—'
                      : '${st!.robot.pos![0].toStringAsFixed(2)}, '
                            '${st.robot.pos![1].toStringAsFixed(2)}',
                  unit: 'm',
                ),
                const SizedBox(height: 12),
                _controllerRow(st, app),
                const SizedBox(height: 12),
                _gaitRow(st, app),
                const SizedBox(height: 12),
                _powerRow(st, app),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _rightColumn(GatewayState? st, AppState app) {
    return SingleChildScrollView(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Panel(
            title: '链路',
            child: Column(
              children: [
                KeyValue('模式', _modeLabel(st?.mode ?? 'off')),
                KeyValue(
                  '操作权',
                  st?.lease.mine == true
                      ? '本机'
                      : (st?.lease.holder == null ? '无人' : '其他设备'),
                ),
                KeyValue('在线设备', '${st?.lease.clients ?? 0} 台'),
                KeyValue(
                  '往返',
                  st?.link.rttMs == null
                      ? '—'
                      : '${st!.link.rttMs!.toStringAsFixed(1)} ms',
                ),
                KeyValue('机器人', st?.link.target ?? '—'),
                KeyValue('端点', st?.link.preset == 'real' ? '真机' : '仿真'),
                if (st?.link.error != null)
                  KeyValue('错误', st!.link.error!, color: C.danger),
                if (st?.robot.loop != null) KeyValue('回路', st!.robot.loop!),
              ],
            ),
          ),
          const SizedBox(height: 12),
          Panel(
            title: '正在下发',
            child: Column(
              children: [
                _bar('前后 vx', st?.input.vx ?? 0, st?.limits.vx ?? 1),
                const SizedBox(height: 6),
                _bar('横移 vy', st?.input.vy ?? 0, st?.limits.vy ?? 0.3),
                const SizedBox(height: 6),
                _bar('转向 vyaw', st?.input.vyaw ?? 0, st?.limits.vyaw ?? 4),
                const SizedBox(height: 10),
                Row(
                  children: [
                    StatusChip(
                      text: st?.input.driving == true ? '行进中' : '静止',
                      color: st?.input.driving == true ? C.accent : C.faint,
                      dense: true,
                    ),
                    const SizedBox(width: 6),
                    StatusChip(
                      text: '来源 ${st?.input.source ?? '-'}',
                      color: C.faint,
                      dense: true,
                    ),
                  ],
                ),
                if (st?.input.ageMs != null) ...[
                  const SizedBox(height: 6),
                  Text(
                    '上一帧速度 ${st!.input.ageMs} ms 前',
                    style: const TextStyle(fontSize: 10.5, color: C.faint),
                  ),
                ],
              ],
            ),
          ),
          const SizedBox(height: 12),
          Panel(
            title: '输入',
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text(
                  '左手摇杆 平移 · 右手摇杆 转向\nWASD / 方向键 · Q E 转向',
                  style: TextStyle(fontSize: 11.5, color: C.dim, height: 1.6),
                ),
                const SizedBox(height: 8),
                ValueListenableBuilder(
                  valueListenable: app.gamepad.pads,
                  builder: (context, pads, _) => Text(
                    pads.isEmpty ? '未连接手柄' : '手柄：${pads.join('、')}',
                    style: TextStyle(
                      fontSize: 11.5,
                      color: pads.isEmpty ? C.faint : C.accentText,
                    ),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  // 高层控制端：官方 / 自研 / 推理。
  //
  // 三者是「谁来解释 vx/vy/vyaw」，不是三档速度。换它等于重启服务端，
  // 所以点一下要确认 —— 这个动作会让狗先蹲下再站起来，期间操作权要重新取。
  static const _controllers = [
    ('official', '官方', 'SportClient，真机原生运动服务，39 个动作'),
    ('ours', '自研', '本仓库的 HighLevelController，200 Hz 回路 + MPC/WBC'),
    ('policy', '推理', '步态策略，直连 rt/lowcmd，没有高层动作'),
  ];

  Widget _controllerRow(GatewayState? st, AppState app, {bool compact = false}) {
    final current = st?.link.controller ?? '';
    final available = st?.link.available ?? const <String>[];
    final canSwitch = (st?.mode ?? 'off') == 'manual';
    final row = Row(
      children: [
        for (final (kind, short, _) in _controllers)
          Padding(
            padding: const EdgeInsets.only(right: 6),
                    child: _ctrlChip(
              kind: kind,
              short: short,
              on: current == kind,
              // available 空 = 老版本服务端没报，别拦着用户（网关那边还会校验）。
              usable: canSwitch && (available.isEmpty || available.contains(kind)),
              onTap: () => _askController(context, app, kind),
            ),
          ),
      ],
    );
    if (compact) {
      return Row(children: [
        const Kicker('控制端', color: C.faint),
        const SizedBox(width: 10),
        Expanded(child: row),
      ]);
    }
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        const Kicker('高层控制端', color: C.faint),
        const SizedBox(height: 6),
        row,
        const SizedBox(height: 6),
        Text(
          canSwitch
              ? '换控制端会重启服务端：狗先蹲下再站起来，操作权要重新取'
              : '切到手动遥控之后才能换控制端',
          style: const TextStyle(fontSize: 10, color: C.faint, height: 1.4),
        ),
      ],
    );
  }

  Widget _ctrlChip({
    required String kind,
    required String short,
    required bool on,
    required bool usable,
    required VoidCallback onTap,
  }) {
    final color = on ? C.accentText : (usable ? C.dim : C.faint);
    // 语义标签：屏幕阅读器要能念出「控制端-推理」，自动化测试也靠它定位。
    // Canvas 渲染的界面没有 DOM 文字，没有这层就只能靠坐标点。
    return Semantics(
      button: true,
      enabled: usable,
      selected: on,
      label: '控制端-$short',
      child: InkWell(
      borderRadius: BorderRadius.circular(R.inner),
      onTap: usable ? onTap : null,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
        decoration: BoxDecoration(
          color: on ? C.accent.withValues(alpha: 0.16) : C.surfaceHi,
          borderRadius: BorderRadius.circular(R.inner),
          border: Border.all(
              color: on ? C.accent.withValues(alpha: 0.5) : C.line),
        ),
        child: Text(short,
            style: TextStyle(fontSize: 11.5, fontWeight: FontWeight.w600,
                color: color)),
      ),
      ),
    );
  }

  Future<void> _askController(
      BuildContext context, AppState app, String kind) async {
    final st = app.client.state.value;
    if (st?.link.controller == kind) return;
    final label = _controllers
        .firstWhere((c) => c.$1 == kind,
            orElse: () => (kind, kind, ''))
        .$2;
    final ok = await showDialog<bool>(
      context: context,
      builder: (context) => AlertDialog(
        backgroundColor: C.surface,
        title: Text('切到$label控制端？', style: const TextStyle(fontSize: 16)),
        content: const Text(
          '换高层控制端要重启机器人服务端：狗会先蹲下再站起来（约 5~10 秒），'
          '这期间操作权会被收回，需要重新取。\n\n'
          '官方控制端只有真机有；仿真桥没有运动服务，只能用自研或推理。',
          style: TextStyle(fontSize: 12.5, color: C.dim, height: 1.6),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(false),
            child: const Text('取消', style: TextStyle(color: C.dim)),
          ),
          FilledButton(
            onPressed: () => Navigator.of(context).pop(true),
            child: Text('切换'),
          ),
        ],
      ),
    );
    if (ok == true) await app.switchController(kind);
  }

  Widget _gaitRow(GatewayState? st, AppState app) {
    final walkOnly = (st?.link.controller ?? '') == 'policy';
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Kicker('步态', color: C.faint),
        const SizedBox(height: 6),
        Row(
          children: [
            // 步态名可能是一整句中文（仿真报「机械（标准小跑，同官方灵动）」），
            // 用 Expanded + 截断，别让它把右边按钮挤出卡片。
            Expanded(
              child: Align(
                alignment: Alignment.centerLeft,
                child: StatusChip(
                  text: _shortGait(st?.robot.gait),
                  color: C.info,
                  icon: Icons.directions_walk,
                ),
              ),
            ),
            const SizedBox(width: 8),
            ActionButton(
              label: '换一档',
              icon: Icons.swap_horiz,
              onPressed: (st?.mode ?? 'off') == 'manual' && !walkOnly
                  ? () => app.send('gait')
                  : null,
            ),
          ],
        ),
      ],
    );
  }

  Widget _powerRow(GatewayState? st, AppState app) {
    return Row(
      children: [
        Expanded(
          child: ActionButton(
            label: '收工（蹲下）',
            icon: Icons.power_settings_new,
            color: C.warn,
            onPressed: (st?.mode ?? 'off') == 'off'
                ? null
                : () => app.setMode('off'),
          ),
        ),
      ],
    );
  }

  // -- 窄屏 --------------------------------------------------------------

  Widget _narrow(GatewayState? st, AppState app) {
    return Column(
      children: [
        _strip(st),
        const SizedBox(height: 10),
        Expanded(child: _stage(st, app)),
        const SizedBox(height: 10),
        _actions(app, st),
        const SizedBox(height: 10),
        _throttle(app, st, withController: true),
      ],
    );
  }

  Widget _strip(GatewayState? st) {
    // 手机上这一条只能放五个格子：六个会把「操作权」那一格挤到屏幕外，
    // 露出半截数字。往返在顶栏有 chip，这里先省掉。
    final narrow = MediaQuery.sizeOf(context).width < 430;
    final tiles = <Widget>[
      MetricTile(
        label: '模式',
        value: _modeLabel(st?.mode ?? 'off'),
        color: (st?.mode ?? 'off') == 'vla' ? C.vio : C.accentText,
      ),
      MetricTile(
        label: '高度',
        value: st?.robot.height == null
            ? '—'
            : st!.robot.height!.toStringAsFixed(2),
        unit: 'm',
      ),
      // 仿真报的步态名是一长句（「机械（标准小跑，同官方灵动）」），
      // 整句塞进状态条会把它撑到屏幕外，只留括号前那一段。
      MetricTile(
        label: '步态',
        value: _shortGait(st?.robot.gait),
        width: 96,
      ),
      MetricTile(
        label: '姿态',
        value: st?.robot.rpy == null
            ? '—'
            : '${_deg(st!.robot.roll)} / ${_deg(st.robot.pitch)}',
        unit: '°',
      ),
      if (!narrow)
        MetricTile(
          label: '往返',
          value: st?.link.rttMs == null
              ? '—'
              : st!.link.rttMs!.toStringAsFixed(0),
          unit: 'ms',
        ),
      if (!narrow)
        MetricTile(
          label: '操作权',
          value: st?.lease.mine == true
              ? '本机'
              : (st?.lease.holder == null ? '无人' : '其他'),
          color: st?.lease.mine == true ? C.accentText : C.dim,
        ),
    ];
    return Container(
      height: 64,
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
      decoration: BoxDecoration(
        color: C.surface,
        borderRadius: BorderRadius.circular(R.card),
      ),
      child: SingleChildScrollView(
        scrollDirection: Axis.horizontal,
        child: Row(
          children: [
            for (var i = 0; i < tiles.length; i++) ...[
              if (i > 0)
                Container(
                  width: 1,
                  height: 34,
                  color: C.line,
                  margin: const EdgeInsets.symmetric(horizontal: 14),
                ),
              tiles[i],
            ],
          ],
        ),
      ),
    );
  }

  // 舞台：摇杆区 + 背景层。
  //
  // 背景只铺在这一块，不铺整页：左右两栏是状态卡片，字压在图上读不清；而摇杆
  // 区本来就是留白的，铺一张画面正好。主视图铺满、小视图缩到右上角，两者共用
  // 同一个源，切换只改尺寸。
  Widget _stage(GatewayState? st, AppState app) {
    return Stack(
      children: [
        Positioned.fill(child: _backdrop(st, app)),
        Positioned.fill(child: _pads(st, app, withReadout: true)),
        Positioned(
          top: 0,
          left: 0,
          child: _bgControls(app),
        ),
      ],
    );
  }

  // 背景本体。三种源：相机 / 雷达 / 关；两种尺寸：铺满 / 小窗。
  Widget _backdrop(GatewayState? st, AppState app) {
    return ListenableBuilder(
      listenable: app.settings,
      builder: (context, _) {
        final src = app.settings.bgSource;
        if (src == 'none') return const SizedBox.shrink();
        final small = app.settings.bgSize == 'small';
        final visual = src == 'radar'
            ? _radarView(app, main: !small)
            : _cameraView(st, app, showLabel: !small);
        final shade = Colors.black.withValues(
            // 压暗只压到「数字还能读」，不压到「画面看不见」。0.42 时仿真
            // 场景本就偏暗，叠上去整块接近纯黑，用户以为相机没画面。
            // 数字可读性另外用描边保证（见 _speedReadout）。
            alpha: small ? 0.10 : (src == 'radar' ? 0.18 : 0.16));

        if (!small) {
          return Stack(
            fit: StackFit.expand,
            children: [
              ClipRRect(
                borderRadius: BorderRadius.circular(R.ctl),
                child: visual,
              ),
              // 压暗：摇杆和大号数字压在图上是浅色字，不压暗会看花。
              IgnorePointer(child: ColoredBox(color: shade)),
              IgnorePointer(
                child: DecoratedBox(
                  decoration: BoxDecoration(
                    borderRadius: BorderRadius.circular(R.ctl),
                  ),
                ),
              ),
            ],
          );
        }
        return Align(
          alignment: Alignment.topRight,
          child: Padding(
            padding: const EdgeInsets.only(top: 40),
            child: Container(
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(R.ctl),
              ),
              clipBehavior: Clip.antiAlias,
              child: SizedBox(
                width: 188,
                height: 141,
                child: Stack(
                  fit: StackFit.expand,
                  children: [
                    visual,
                    IgnorePointer(child: ColoredBox(color: shade)),
                    Positioned(
                      left: 6,
                      bottom: 5,
                      child: Container(
                        padding: const EdgeInsets.symmetric(
                            horizontal: 6, vertical: 2),
                        decoration: BoxDecoration(
                          color: Colors.black.withValues(alpha: 0.55),
                          borderRadius: BorderRadius.circular(R.card),
                        ),
                        child: Text(
                          src == 'radar' ? '雷达' : '画面',
                          style: const TextStyle(
                              fontSize: 9, color: Colors.white70),
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        );
      },
    );
  }

  Widget _cameraView(GatewayState? st, AppState app, {bool showLabel = true}) {
    return ValueListenableBuilder<CameraFrame?>(
      valueListenable: app.client.frame,
      builder: (context, frame, _) {
        if (frame == null) {
          final noSource =
              !(st?.vla.running ?? false) && !(st?.cam.running ?? false);
          // 网关报过原因就用它的原话：端口被占用、没有相机这类事，
          // App 这边猜不出来。
          final note = (st?.cam.note ?? '').trim();
          return _backdropHint(
            noSource ? '网关没有画面源' : '等待画面…',
            note.isNotEmpty
                ? note
                : (noSource ? '手动模式要看到画面，网关得开相机馈送（percept.feed）' : null),
          );
        }
        return Stack(
          fit: StackFit.expand,
          children: [
            ColoredBox(color: const Color(0xFF070A0C)),
            // contain 不裁：真机是 16:9 鱼眼，cover 会把两边切掉，而视野正好
            // 就在两边。多出来的用底色吃掉。
            Image.memory(
              frame.bytes,
              gaplessPlayback: true,
              fit: BoxFit.contain,
              filterQuality: FilterQuality.low,
              // 解码失败要说出来。画面黑着、又什么都不报，是最难查的一种：
              // 分不清是没帧、帧是黑的、还是解不出来。
              errorBuilder: (context, error, stack) => Center(
                child: Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 20),
                  child: Column(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      const Icon(Icons.broken_image_outlined,
                          size: 22, color: C.faint),
                      const SizedBox(height: 6),
                      const Text('这一帧解不出来',
                          style: TextStyle(fontSize: 11.5, color: C.dim)),
                      const SizedBox(height: 2),
                      Text(
                        error.toString().split('\n').first,
                        maxLines: 2,
                        overflow: TextOverflow.ellipsis,
                        textAlign: TextAlign.center,
                        style: const TextStyle(fontSize: 9.5, color: C.faint),
                      ),
                    ],
                  ),
                ),
              ),
            ),
            // 标出来源和这一帧的大小：数字在跳 = 帧一直在到，黑是内容问题；
            // 数字不动 = 帧停了；没有这块 = 压根没帧。
            if (showLabel)
              Positioned(
                right: 6,
                top: 5,
                child: Container(
                  padding:
                      const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                  decoration: BoxDecoration(
                    color: Colors.black.withValues(alpha: 0.5),
                    borderRadius: BorderRadius.circular(R.card),
                  ),
                  child: Text(
                    '${frame.label} · '
                    '${(frame.bytes.length / 1024).toStringAsFixed(1)} KB · '
                    '#${frame.seq}',
                    style: const TextStyle(fontSize: 9, color: Colors.white70),
                  ),
                ),
              ),
          ],
        );
      },
    );
  }

  Widget _radarView(AppState app, {bool main = true}) {
    return Container(
      color: const Color(0xFF070A0C),
      // 两个来源，优先级不同：
      //   手动模式 → 网关的 `radar`（感知层直接推，不依赖 VLA）
      //   仿真/VLA → VLA 帧里的 `sectors`（那是模型自己的视野，带渲染合成）
      // 所以这里嵌两层监听：只监听 VLA 的话，手动模式下雷达永远不刷新。
      child: ValueListenableBuilder<GatewayState?>(
        valueListenable: app.client.state,
        builder: (context, st, _) {
          return ValueListenableBuilder<VlaFrame?>(
            valueListenable: app.client.vla,
            builder: (context, f, _) {
              final radar = st?.radar;
              var sectors = const <SectorGap>[];
              var title = '没有测距数据';
              var note = '手动模式要等雷达出数据；仿真走 VLA';

              if (radar != null && radar.sectors.isNotEmpty) {
                sectors = radar.sectors;
              } else if (f != null && f.sectors.isNotEmpty) {
                sectors = f.sectors;
              } else if (radar != null && radar.why.isNotEmpty) {
                // 网关给了原因（雷达没起来、数据过期）就直接显示，别让人以为是
                // 「四周空旷」。
                title = radar.why;
              }

              if (sectors.isEmpty) {
                return _backdropHint(title, note);
              }
              // 主视图：雷达画大、贴底摆。居中的话正撞上大号速度读数，
              // 「0.00」会压在「0.82」上，两个数字都读不出来。
              return Padding(
                padding: EdgeInsets.fromLTRB(10, main ? 26 : 8, 10, main ? 8 : 8),
                // 主视图把它压淡：雷达是「余光里的障碍分布」，不是主角。
                // 全亮度时它和大号速度读数抢同一块地方，两组数字互相搅。
                child: Opacity(
                  opacity: main ? 0.55 : 1,
                  child: Align(
                    alignment: main ? Alignment.bottomCenter : Alignment.center,
                    child: SectorRadar(
                      sectors: sectors,
                      maxRange: 3,
                      maxSide: main ? 420 : 200,
                      showValues: !main,
                    ),
                  ),
                ),
              );
            },
          );
        },
      ),
    );
  }

  Widget _backdropHint(String title, String? note) {
    return Container(
      color: const Color(0xFF070A0C),
      alignment: Alignment.center,
      padding: const EdgeInsets.all(18),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          const Icon(Icons.image_not_supported_outlined, size: 20,
              color: C.faint),
          const SizedBox(height: 7),
          Text(title,
              style: const TextStyle(fontSize: 11.5, color: C.dim),
              textAlign: TextAlign.center),
          if (note != null) ...[
            const SizedBox(height: 4),
            Text(note,
                style: const TextStyle(fontSize: 10, color: C.faint,
                    height: 1.4),
                textAlign: TextAlign.center),
          ],
        ],
      ),
    );
  }

  // 背景的开关：源（相机 / 雷达 / 关）+ 尺寸（主 / 小）。
  Widget _bgControls(AppState app) {
    return ListenableBuilder(
      listenable: app.settings,
      builder: (context, _) {
        final s = app.settings;
        // 这一条压在相机画面上：背景有内容，玻璃的模糊在这里才真的看得出来。
        // 用黑色底而不是 C.glassFill —— 灰底压在画面上会泛白，字反而糊。
        return Glass(
          radius: R.ctl,
          blur: 12,
          alpha: 0.42,
          tint: Colors.black,
          rim: 0.22,
          shadow: false,
          padding: const EdgeInsets.symmetric(horizontal: 4, vertical: 3),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              _seg('相机', s.bgSource == 'camera', () {
                s.bgSource = 'camera';
                s.save();
              }),
              _seg('雷达', s.bgSource == 'radar', () {
                s.bgSource = 'radar';
                s.save();
              }),
              _seg('关', s.bgSource == 'none', () {
                s.bgSource = 'none';
                s.save();
              }),
              if (s.bgSource != 'none') ...[
                Container(width: 1, height: 14,
                    color: C.line, margin: const EdgeInsets.symmetric(horizontal: 3)),
                InkWell(
                  borderRadius: BorderRadius.circular(R.card),
                  onTap: () {
                    s.bgSize = s.bgSize == 'main' ? 'small' : 'main';
                    s.save();
                  },
                  child: Tooltip(
                    message: s.bgSize == 'main' ? '切成小视图' : '切成主视图',
                    child: Padding(
                      padding: const EdgeInsets.symmetric(
                          horizontal: 6, vertical: 4),
                      child: Icon(
                        s.bgSize == 'main'
                            ? Icons.fullscreen_exit
                            : Icons.fullscreen,
                        size: 15,
                        color: C.dim,
                      ),
                    ),
                  ),
                ),
              ],
            ],
          ),
        );
      },
    );
  }

  Widget _seg(String label, bool on, VoidCallback tap) {
    return InkWell(
      borderRadius: BorderRadius.circular(R.card),
      onTap: tap,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
        decoration: BoxDecoration(
          color: on ? C.accent.withValues(alpha: 0.18) : Colors.transparent,
          borderRadius: BorderRadius.circular(R.card),
        ),
        child: Text(label,
            style: TextStyle(fontSize: 10.5,
                fontWeight: FontWeight.w600,
                color: on ? C.accentText : C.faint)),
      ),
    );
  }

  // 摇杆区。两个摇杆各自占一半宽度，尺寸跟着可用空间走 —— 手机上要足够大
  // 才推得准，桌面上又不该被拉成两个巨盘。
  //
  // 摇杆靠下摆（拇指区），上面空出来的地方在宽屏上放速度读数：遥控时眼睛看的是
  // 「多快、往哪」，不是三个坐标轴的原始值。
  Widget _pads(GatewayState? st, AppState app, {bool withReadout = false}) {
    // 能推（模式对）和能发（有操作权）是两件事：没操作权时摇杆照样能拖，
    // 但一个字节都发不出去。所以视觉上要区分，否则「拖了没反应」看起来像坏了。
    final modeOk = (st?.mode ?? 'off') == 'manual';
    final canSend = modeOk && (st?.lease.mine ?? false);
    final enabled = modeOk;
    return LayoutBuilder(builder: (context, box) {
      final side =
          math.min(math.min(box.maxWidth / 2 - 24, box.maxHeight - 20), 248.0);
      final size = side.clamp(96.0, 248.0);
      // 舞台太矮时（360×640，扣掉悬浮底栏只剩 200 pt）大号读数连一行都放不下。
      // 这时候摇杆是唯一要紧的东西，读数整个让位 —— 硬塞进去的结果是整列溢出，
      // 把摇杆顶出画面。
      // 判据是「摇杆占完之后还剩多少」，不是舞台多高：摇杆尺寸跟着舞台走，
      // 同一个舞台高度在不同屏宽下留给读数的空间不一样。实测 360×640 只剩
      // 45 pt（放不下），414×896 有 274 pt（放得下）。
      final showReadout = withReadout && box.maxHeight - (size + 6) >= 70;
      return Column(
        mainAxisAlignment: MainAxisAlignment.end,
        children: [
          if (showReadout)
            Expanded(child: Center(child: _speedReadout(st, app, enabled))),
          if (!enabled)
            Padding(
              padding: const EdgeInsets.only(bottom: 8),
              child: StatusChip(
                text: st == null
                    ? '未连接网关'
                    : '当前 ${_modeLabel(st.mode)} · 摇杆不下发',
                color: C.warn,
                icon: Icons.info_outline,
                dense: true,
              ),
            ),
          SizedBox(
            height: size + 6,
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Padding(
                  padding: const EdgeInsets.only(left: 4),
                  child: Opacity(
                    opacity: canSend ? 1 : 0.45,
                    child: Joystick(
                      size: size,
                      label: '平移',
                      onChanged: (v) {
                        _move = v;
                        _emitTouch();
                      },
                    ),
                  ),
                ),
                Padding(
                  padding: const EdgeInsets.only(right: 4),
                  child: Opacity(
                    opacity: canSend ? 1 : 0.45,
                    child: Joystick(
                      size: size,
                      label: '转向',
                      color: C.info,
                      lockY: true,
                      onChanged: (v) {
                        _turn = v;
                        _emitTouch();
                      },
                    ),
                  ),
                ),
              ],
            ),
          ),
        ],
      );
    });
  }

  // 大号速度读数。数字大字、单位小字，加减号保留 —— 「在后退」和「在前进」
  // 只差一个符号，光看绝对值会看错。
  // 画面上的字要自带「底」。压暗从 0.42 降到 0.16 之后，浅色画面上白字会糊，
  // 用两层阴影（宽而淡 + 窄而深）当描边，比加半透明底板干净。
  static const _onImageShadow = [
    Shadow(color: Color(0xCC000000), blurRadius: 14),
    Shadow(color: Color(0xE6000000), blurRadius: 3, offset: Offset(0, 1)),
  ];

  Widget _speedReadout(GatewayState? st, AppState app, bool enabled) {
    final vx = st?.input.vx ?? 0;
    final max = st?.limits.vx ?? 1.0;
    return ValueListenableBuilder<bool>(
      valueListenable: app.drive.armed,
      builder: (context, armed, _) {
        final color = !enabled
            ? C.faint
            : (armed
                ? (vx.abs() > 0.01 ? C.accentText : C.text)
                : C.dim);
        final big = MediaQuery.sizeOf(context).height > 560;

        // 「能推」和「能发」是两件事：没操作权时摇杆照样能拖，但一个字节都
        // 发不出去。三种原因分开说，否则「拖了没反应」看起来像遥控坏了。
        final mine = st?.lease.mine ?? false;
        final blocked = enabled && !mine;
        final stateText = !enabled
            ? '遥控不可用'
            : (blocked
                ? '没有操作权 · 摇杆不下发'
                : (armed ? '正在下发' : '未使能 · 摇杆无效'));

        // 舞台被挤矮时（360×640 的手机，扣掉悬浮底栏的安全区只剩 200 pt），
        // 这一列写死的内容会溢出 101 px，把摇杆顶出画面。先牺牲次要信息：
        // 高度不够就去掉横移/转向/高度那一行，再不够连进度条一起去掉 ——
        // 只剩「多大速度」和状态，那是遥控时真正要看的。
        return LayoutBuilder(builder: (context, box) {
          final room = box.hasBoundedHeight ? box.maxHeight : 400.0;
          final showBar = room >= 96;
          final showMini = room >= 150;
          return Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            // 整行可点：点文字也算「取得控制」，不用非得点中那个小胶囊。
            GestureDetector(
              onTap: blocked ? () => app.drive.setArmed(true) : null,
              behavior: HitTestBehavior.opaque,
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Kicker(
                    stateText,
                    color: enabled && armed
                        ? C.accentText
                        : (blocked ? C.warn : C.dim),
                    shadows: _onImageShadow,
                  ),
                  if (blocked) ...[
                    const SizedBox(width: 8),
                    Glass(
                      radius: R.pill,
                      alpha: 0.4,
                      tint: Colors.black,
                      rim: 0.2,
                      shadow: false,
                      padding: const EdgeInsets.symmetric(
                          horizontal: 12, vertical: 8),
                      child: const Text('取得控制',
                          style: TextStyle(
                              fontSize: 11,
                              fontWeight: FontWeight.w600,
                              color: C.accentText)),
                    ),
                  ],
                ],
              ),
            ),
            // 使能失败/被解除的原因常驻，不靠 3 秒的提示条。
            ValueListenableBuilder<String?>(
              valueListenable: app.drive.blockReason,
              builder: (context, reason, _) => reason == null
                  ? const SizedBox(height: 2)
                  : Padding(
                      padding: const EdgeInsets.only(top: 3, bottom: 2),
                      child: Text(
                        reason,
                        textAlign: TextAlign.center,
                        style: const TextStyle(
                          fontSize: 10.5,
                          color: C.danger,
                          shadows: _onImageShadow,
                        ),
                      ),
                    ),
            ),
            Row(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.baseline,
              textBaseline: TextBaseline.alphabetic,
              children: [
                Text(vx.toStringAsFixed(2),
                    style: numStyle.copyWith(
                        fontSize: big ? 54 : 38,
                        height: 1.05,
                        fontWeight: FontWeight.w700,
                        color: color,
                        shadows: _onImageShadow)),
                const SizedBox(width: 5),
                const Text('m/s',
                    style: TextStyle(
                        fontSize: 12, color: C.dim,
                        shadows: _onImageShadow)),
              ],
            ),
            if (showBar) ...[
              const SizedBox(height: 10),
              SizedBox(
                width: 220,
                child: ClipRRect(
                  borderRadius: BorderRadius.circular(R.card),
                  child: SizedBox(
                    height: 5,
                    child: Stack(children: [
                      Container(color: C.surfaceHi),
                      FractionallySizedBox(
                        widthFactor: (vx.abs() / (max == 0 ? 1 : max))
                            .clamp(0.0, 1.0),
                        alignment: Alignment.centerLeft,
                        child: Container(
                            color: vx.isNegative ? C.warn : C.accent),
                      ),
                    ]),
                  ),
                ),
              ),
            ],
            if (showMini) ...[
              const SizedBox(height: 12),
              Row(mainAxisSize: MainAxisSize.min, children: [
                _mini('横移', st?.input.vy ?? 0, 'm/s'),
                const SizedBox(width: 18),
                _mini('转向', st?.input.vyaw ?? 0, 'r/s'),
                const SizedBox(width: 18),
                _mini('高度', st?.robot.height ?? 0, 'm', signed: false),
              ]),
            ],
          ],
          );
        });
      },
    );
  }

  Widget _mini(String label, double value, String unit, {bool signed = true}) {
    return Column(mainAxisSize: MainAxisSize.min, children: [
      Kicker(label, color: C.dim, shadows: _onImageShadow),
      const SizedBox(height: 2),
      Row(mainAxisSize: MainAxisSize.min, children: [
        Text(signed ? value.toStringAsFixed(2) : value.toStringAsFixed(3),
            style: numStyle.copyWith(fontSize: 15, color: C.text,
                shadows: _onImageShadow)),
        const SizedBox(width: 3),
        Text(unit, style: const TextStyle(fontSize: 9.5, color: C.dim,
            shadows: _onImageShadow)),
      ]),
    ]);
  }

  Widget _actions(AppState app, GatewayState? st) {
    final mode = st?.mode ?? 'off';
    // 推理控制端（policy）只会走。起立 / 蹲下 / 跳跃 / 换步态在它那边直接抛
    // NotImplementedError，点下去只会弹一句「不支持」。按钮留着但禁用，比让人
    // 反复点一个注定失败的东西清楚 —— 要这些动作就切官方（真机）或自研。
    final walkOnly = (st?.link.controller ?? '') == 'policy';
    final canAct = mode == 'manual' && !walkOnly;
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Row(
          children: [
            Expanded(
              child: ActionButton(
                label: '起立',
                icon: Icons.arrow_upward,
                color: C.accentText,
                onPressed: canAct ? () => app.send('stand') : null,
              ),
            ),
            const SizedBox(width: 6),
            Expanded(
              child: ActionButton(
                label: '蹲下',
                icon: Icons.arrow_downward,
                onPressed: canAct
                    ? () => app.send('crouch', {'depth': 0.08})
                    : null,
              ),
            ),
            const SizedBox(width: 6),
            Expanded(
              child: ActionButton(
                label: '跳跃',
                icon: Icons.bolt,
                color: C.vio,
                onPressed: canAct ? () => app.send('jump') : null,
              ),
            ),
            const SizedBox(width: 6),
            Expanded(
              child: ActionButton(
                label: '步态',
                icon: Icons.swap_horiz,
                color: C.info,
                onPressed: canAct ? () => app.send('gait') : null,
              ),
            ),
            const SizedBox(width: 6),
            Expanded(
              child: ActionButton(
                // 五个按钮等宽并排：标签统一两个字，图标才落在同一条竖线上。
                label: '停止',
                icon: Icons.pan_tool_alt_outlined,
                color: C.danger,
                onPressed: () => app.emergencyStop(),
              ),
            ),
          ],
        ),
        if (walkOnly && mode == 'manual') ...[
          const SizedBox(height: 6),
          const Text(
            '推理控制端只负责走：起立 / 蹲下 / 跳跃 / 换步态要用官方（真机）或自研',
            style: TextStyle(fontSize: 10, color: C.faint, height: 1.4),
          ),
        ],
      ],
    );
  }

  Widget _throttle(AppState app, GatewayState? st,
      {bool withController = false}) {
    final mode = st?.mode ?? 'off';
    return Container(
      padding: const EdgeInsets.fromLTRB(12, 8, 12, 8),
      decoration: BoxDecoration(
        color: C.surface,
        borderRadius: BorderRadius.circular(R.ctl),
        border: Border.all(
          color: mode == 'manual' ? C.accent.withValues(alpha: 0.25) : C.line,
        ),
      ),
      child: Column(
        children: [
          if (withController) ...[
            _controllerRow(st, app, compact: true),
            const SizedBox(height: 8),
          ],
          Row(
            children: [
              ValueListenableBuilder<bool>(
                valueListenable: app.drive.armed,
                builder: (context, armed, _) => InkWell(
                  borderRadius: BorderRadius.circular(R.inner),
                  onTap: mode == 'manual'
                      ? () => app.drive.setArmed(!armed)
                      : null,
                  child: Container(
                    padding: const EdgeInsets.symmetric(
                      horizontal: 14,
                      vertical: 9,
                    ),
                    decoration: BoxDecoration(
                      color: armed
                          ? C.accent.withValues(alpha: 0.16)
                          : C.surfaceHi,
                      borderRadius: BorderRadius.circular(R.inner),
                      border: Border.all(
                        color: armed
                            ? C.accent.withValues(alpha: 0.55)
                            : C.line,
                      ),
                    ),
                    child: Row(
                      children: [
                        Icon(
                          armed ? Icons.lock_open : Icons.lock_outline,
                          size: 15,
                          color: armed ? C.accentText : C.faint,
                        ),
                        const SizedBox(width: 6),
                        Text(
                          armed ? '已使能' : '使能',
                          style: TextStyle(
                            fontSize: 12.5,
                            fontWeight: FontWeight.w600,
                            color: armed
                                ? C.accentText
                                : (mode == 'manual' ? C.text : C.faint),
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
              ),
              const SizedBox(width: 10),
              Expanded(
                child: ValueListenableBuilder<bool>(
                  valueListenable: app.drive.sending,
                  builder: (context, sending, _) => Text(
                    sending ? '正在下发速度' : '静止',
                    style: TextStyle(
                      fontSize: 11,
                      color: sending ? C.accentText : C.faint,
                    ),
                  ),
                ),
              ),
            ],
          ),
          const SizedBox(height: 8),
          StepBar(
            values: const [0.25, 0.5, 0.75, 1.0],
            value: app.settings.speedScale,
            formatter: (v) => '${(v * 100).round()}%',
            onChanged: (v) {
              app.settings.speedScale = v;
              app.settings.save();
            },
          ),
        ],
      ),
    );
  }

  // -- 小工具 ------------------------------------------------------------

  static String _shortGait(String? gait) {
    if (gait == null || gait.isEmpty) return '—';
    final cut = gait.indexOf('（');
    return cut > 0 ? gait.substring(0, cut) : gait;
  }

  static String _deg(double? rad) =>
      rad == null ? '—' : (rad * 180 / math.pi).toStringAsFixed(0);

  static String _modeLabel(String mode) => switch (mode) {
    'manual' => '手动',
    'vla' => 'VLA',
    'switching' => '切换中',
    _ => '待机',
  };

  Widget _bar(String label, double value, double max) {
    final ratio = (value.abs() / (max == 0 ? 1 : max)).clamp(0.0, 1.0);
    return Row(
      children: [
        SizedBox(
          width: 62,
          child: Text(
            label,
            style: const TextStyle(fontSize: 10.5, color: C.faint),
          ),
        ),
        Expanded(
          child: ClipRRect(
            borderRadius: BorderRadius.circular(R.card),
            child: SizedBox(
              height: 7,
              child: Stack(
                children: [
                  Container(color: C.surfaceHi),
                  FractionallySizedBox(
                    widthFactor: ratio,
                    alignment: Alignment.centerLeft,
                    child: Container(
                      color: value.isNegative ? C.warn : C.accent,
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
        SizedBox(
          width: 52,
          child: Text(
            value.toStringAsFixed(2),
            textAlign: TextAlign.right,
            style: numStyle.copyWith(fontSize: 11, color: C.dim),
          ),
        ),
      ],
    );
  }
}
