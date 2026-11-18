import 'package:flutter/material.dart';

import '../app_state.dart';
import '../core/models.dart';
import '../theme.dart';
import 'widgets/chat_feed.dart';
import 'widgets/mode_switch.dart';
import 'widgets/panel.dart';
import 'widgets/vla_visuals.dart';

// VLA 页：把「它看到什么、怎么想、发了什么命令、身体在哪」放在同一屏。
//
// 布局按信息的关系分三列，不是按「卡片好看」：
//
//   左  画面 + 八方位   —— 输入（它看到的世界）
//   中  判断 + 对话     —— 决策（它怎么想、我说了什么）
//   右  指标 + 运行     —— 输出与健康度（命令、频率、安全闸）
//
// 窄屏退化成单列，但顺序不变：先看到世界，再看决策，最后看数字。
class VlaPage extends StatefulWidget {
  const VlaPage({super.key});

  @override
  State<VlaPage> createState() => _VlaPageState();
}

class _VlaPageState extends State<VlaPage> {
  final _input = TextEditingController();
  final _focus = FocusNode();
  bool _sending = false;

  @override
  void dispose() {
    _input.dispose();
    _focus.dispose();
    super.dispose();
  }

  Future<void> _say() async {
    final text = _input.text.trim();
    if (text.isEmpty || _sending) return;
    final app = AppScope.read(context);
    setState(() => _sending = true);
    app.client.sayLocal(text);
    _input.clear();
    final out = await app.client.say(text);
    if (!mounted) return;
    setState(() => _sending = false);
    if (out['ok'] != true) {
      app.client.error.value = (out['msg'] ?? '没发出去').toString();
    }
  }

  @override
  Widget build(BuildContext context) {
    final app = AppScope.of(context);
    return ValueListenableBuilder<GatewayState?>(
      valueListenable: app.client.state,
      builder: (context, st, _) {
        return LayoutBuilder(
          builder: (context, box) {
            final wide = box.maxWidth >= 1120;
            final mid = box.maxWidth >= 760;
            return Padding(
              padding: const EdgeInsets.all(12),
              child: wide
                  ? _wide(app, st)
                  : (mid ? _medium(app, st) : _narrow(app, st)),
            );
          },
        );
      },
    );
  }

  // -- 宽屏：三列 --------------------------------------------------------

  Widget _wide(AppState app, GatewayState? st) {
    return Row(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Expanded(flex: 11, child: _visionColumn(app, st, compact: false)),
        const SizedBox(width: 12),
        Expanded(flex: 9, child: _decisionColumn(app, st)),
        const SizedBox(width: 12),
        SizedBox(width: 300, child: _dataColumn(app, st)),
      ],
    );
  }

  // -- 中屏：两列 --------------------------------------------------------

  Widget _medium(AppState app, GatewayState? st) {
    return Column(
      children: [
        _topBar(app, st),
        const SizedBox(height: 10),
        _metricsStrip(app),
        const SizedBox(height: 10),
        Expanded(
          child: Row(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              Expanded(flex: 3, child: _visionColumn(app, st, compact: true)),
              const SizedBox(width: 12),
              Expanded(flex: 2, child: _decisionColumn(app, st)),
            ],
          ),
        ),
      ],
    );
  }

  // -- 窄屏：单列 --------------------------------------------------------

  // 窄屏：手机竖屏。摄像头、判断、雷达三块都得在，但聊天要留够高度 ——
  // 前两版把雷达整行放在底部，剩余高度全被它吃掉，对话只剩一个标题栏。
  // 这一版让「判断」和雷达并排，省下的 148 px 全部给对话。
  Widget _narrow(AppState app, GatewayState? st) {
    return Column(
      children: [
        _topBar(app, st),
        const SizedBox(height: 10),
        _metricsStrip(app),
        const SizedBox(height: 10),
        ValueListenableBuilder<CameraFrame?>(
          valueListenable: app.client.frame,
          builder: (context, frame, _) => ConstrainedBox(
            constraints: const BoxConstraints(maxHeight: 200),
            child: ValueListenableBuilder<VlaFrame?>(
              valueListenable: app.client.vla,
              builder: (context, f, _) => CameraPanel(
                frame: frame?.bytes,
                frontGap: f?.frontGap,
                alert: f?.alert ?? false,
                running: st?.vla.running ?? false,
              ),
            ),
          ),
        ),
        const SizedBox(height: 10),
        SizedBox(
          height: 168,
          child: Row(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              Expanded(flex: 3, child: _thoughtPanel(app, maxLines: 4)),
              const SizedBox(width: 10),
              Expanded(flex: 2, child: _radarPanel(app)),
            ],
          ),
        ),
        const SizedBox(height: 10),
        Expanded(child: _chatPanel(app, st)),
      ],
    );
  }

  Widget _radarPanel(AppState app) {
    return Panel(
      title: '八方位视野',
      padding: const EdgeInsets.fromLTRB(10, 10, 10, 6),
      child: Expanded(
        child: ValueListenableBuilder<VlaFrame?>(
          valueListenable: app.client.vla,
          builder: (context, f, _) =>
              SectorRadar(sectors: f?.sectors ?? const []),
        ),
      ),
    );
  }

  Widget _thoughtPanel(AppState app, {int maxLines = 6}) {
    return Panel(
      title: '判断',
      trailing: ValueListenableBuilder<VlaFrame?>(
        valueListenable: app.client.vla,
        builder: (context, f, _) => Text(
          f == null ? '' : '#${f.thoughts}',
          style: const TextStyle(fontSize: 10.5, color: C.faint),
        ),
      ),
      child: ValueListenableBuilder<VlaFrame?>(
        valueListenable: app.client.vla,
        builder: (context, f, _) =>
            ThoughtCard(frame: f, maxLines: maxLines),
      ),
    );
  }

  Widget _chatPanel(AppState app, GatewayState? st) {
    return Panel(
      title: '对话',
      trailing: ValueListenableBuilder<VlaFrame?>(
        valueListenable: app.client.vla,
        builder: (context, f, _) => f != null && f.reply.isNotEmpty
            ? const StatusChip(text: '它刚说了一句', color: C.accent, dense: true)
            : const SizedBox.shrink(),
      ),
      padding: const EdgeInsets.fromLTRB(10, 10, 10, 8),
      child: Expanded(
        child: Column(
          children: [
            Expanded(
              child: ValueListenableBuilder<int>(
                valueListenable: app.client.chatRev,
                builder: (context, rev, _) =>
                    ChatFeed(items: app.client.chat, rev: rev),
              ),
            ),
            const SizedBox(height: 8),
            _inputRow(app, st),
          ],
        ),
      ),
    );
  }

  // -- 列 ----------------------------------------------------------------

  Widget _visionColumn(
    AppState app,
    GatewayState? st, {
    required bool compact,
  }) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        ValueListenableBuilder<CameraFrame?>(
          valueListenable: app.client.frame,
          builder: (context, frame, _) {
            return ValueListenableBuilder<VlaFrame?>(
              valueListenable: app.client.vla,
              builder: (context, f, _) => CameraPanel(
                frame: frame?.bytes,
                frontGap: f?.frontGap,
                alert: f?.alert ?? false,
                running: st?.vla.running ?? false,
              ),
            );
          },
        ),
        const SizedBox(height: 10),
        Expanded(
          child: Row(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              Expanded(
                child: Panel(
                  title: '八方位视野',
                  padding: const EdgeInsets.fromLTRB(14, 12, 14, 8),
                  // Expanded：Panel 的 Column 拿到的是有界高度，把「还剩多少」
                  // 给雷达，它才不会按宽度铺满再压到下面的卡片上。
                  child: Expanded(
                    child: ValueListenableBuilder<VlaFrame?>(
                      valueListenable: app.client.vla,
                      builder: (context, f, _) =>
                          SectorRadar(sectors: f?.sectors ?? const []),
                    ),
                  ),
                ),
              ),
              if (!compact) ...[
                const SizedBox(width: 10),
                Expanded(child: _perceptionPanel(app)),
              ],
            ],
          ),
        ),
      ],
    );
  }

  Widget _perceptionPanel(AppState app) {
    return Panel(
      title: '感知',
      child: ValueListenableBuilder<VlaFrame?>(
          valueListenable: app.client.vla,
          builder: (context, f, _) => Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              SceneCard(frame: f),
              const SizedBox(height: 12),
              const Divider(height: 1, color: C.line),
              const SizedBox(height: 10),
              KeyValue('测距', (f?.sectors.isEmpty ?? true) ? '无' : '八方位'),
              KeyValue(
                '帧',
                f == null
                    ? '—'
                    : '${f.vision.frames} 帧'
                          '（丢 ${f.vision.dropped}）',
              ),
              KeyValue(
                '描述',
                f == null
                    ? '—'
                    : '${f.vision.views} 次 / '
                          '${f.vision.genS.toStringAsFixed(1)}s',
              ),
              KeyValue(
                '倾角',
                f?.tiltDeg == null ? '—' : '${f!.tiltDeg!.toStringAsFixed(1)}°',
              ),
            ],
          ),
      ),
    );
  }

  Widget _decisionColumn(AppState app, GatewayState? st) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        _thoughtPanel(app),
        const SizedBox(height: 10),
        Expanded(child: _chatPanel(app, st)),
      ],
    );
  }

  Widget _inputRow(AppState app, GatewayState? st) {
    final running = st?.vla.running ?? false;
    return Row(
      children: [
        Expanded(
          child: TextField(
            controller: _input,
            focusNode: _focus,
            enabled: running,
            textInputAction: TextInputAction.send,
            onSubmitted: (_) => _say(),
            style: const TextStyle(fontSize: 13),
            decoration: InputDecoration(
              hintText: running ? '对它说：往前走 / 你看到什么 / 在我面前看着我' : 'VLA 没在跑',
              contentPadding: const EdgeInsets.symmetric(
                horizontal: 12,
                vertical: 11,
              ),
            ),
          ),
        ),
        const SizedBox(width: 8),
        InkWell(
          borderRadius: BorderRadius.circular(10),
          onTap: running && !_sending ? _say : null,
          child: Container(
            width: 42,
            height: 40,
            decoration: BoxDecoration(
              color: running ? C.accent.withValues(alpha: 0.16) : C.surfaceHi,
              borderRadius: BorderRadius.circular(10),
              border: Border.all(
                color: running ? C.accent.withValues(alpha: 0.45) : C.line,
              ),
            ),
            child: _sending
                ? const Center(
                    child: SizedBox(
                      width: 14,
                      height: 14,
                      child: CircularProgressIndicator(
                        strokeWidth: 2,
                        color: C.accent,
                      ),
                    ),
                  )
                : Icon(
                    Icons.send_rounded,
                    size: 17,
                    color: running ? C.accent : C.faint,
                  ),
          ),
        ),
      ],
    );
  }

  Widget _dataColumn(AppState app, GatewayState? st) {
    return SingleChildScrollView(
      child: Column(
        children: [
          Panel(
            title: '这一拍的命令',
            child: ValueListenableBuilder<VlaFrame?>(
              valueListenable: app.client.vla,
              builder: (context, f, _) => Column(
                children: [
                  Row(
                    children: [
                      Expanded(
                        child: MetricTile(
                          label: 'vx',
                          value: (f?.vx ?? 0).toStringAsFixed(2),
                          unit: 'm/s',
                          emphasis: true,
                          color: (f?.vx ?? 0).abs() > 0.01 ? C.accent : C.dim,
                        ),
                      ),
                      Expanded(
                        child: MetricTile(
                          label: 'vy',
                          value: (f?.vy ?? 0).toStringAsFixed(2),
                          unit: 'm/s',
                        ),
                      ),
                      Expanded(
                        child: MetricTile(
                          label: 'vyaw',
                          value: (f?.vyaw ?? 0).toStringAsFixed(2),
                          unit: 'r/s',
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 8),
                  Row(
                    children: [
                      StatusChip(
                        text: f?.gait ?? '—',
                        color: C.info,
                        dense: true,
                      ),
                      const SizedBox(width: 6),
                      if (f?.turnLeftDeg != null)
                        StatusChip(
                          text: '转 ${f!.turnLeftDeg!.toStringAsFixed(0)}°',
                          color: C.warn,
                          dense: true,
                        ),
                      const SizedBox(width: 6),
                      StatusChip(
                        text: '第 ${f?.turns ?? 0} 次转',
                        color: C.faint,
                        dense: true,
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ),
          const SizedBox(height: 12),
          Panel(
            title: '频率与健康',
            child: ValueListenableBuilder<VlaFrame?>(
              valueListenable: app.client.vla,
              builder: (context, f, _) => Column(
                children: [
                  _healthRow(
                    '策略',
                    '${(f?.policyHz ?? 0).toStringAsFixed(0)} Hz',
                    ok: (f?.policyHz ?? 0) >= 30,
                  ),
                  _healthRow(
                    '反射',
                    '${(f?.reflexHz ?? 0).toStringAsFixed(0)} Hz',
                    ok: (f?.reflexHz ?? 0) >= 15,
                  ),
                  _healthRow(
                    '推理',
                    '${(f?.genS ?? 0).toStringAsFixed(2)} s/次',
                    ok: (f?.genS ?? 0) < 4,
                  ),
                  _healthRow(
                    '画面',
                    '${(f?.viewFps ?? 0).toStringAsFixed(0)} fps',
                    ok: (f?.viewFps ?? 0) >= 5,
                  ),
                  const SizedBox(height: 6),
                  const Divider(height: 1, color: C.line),
                  const SizedBox(height: 6),
                  _healthRow(
                    '安全闸',
                    '${f?.guardTrips ?? 0} 次',
                    ok: (f?.guardTrips ?? 0) == 0,
                    warnColor: C.warn,
                  ),
                  if ((f?.guardLast ?? '').isNotEmpty)
                    Padding(
                      padding: const EdgeInsets.only(top: 2, left: 2),
                      child: Text(
                        f!.guardLast!,
                        maxLines: 2,
                        overflow: TextOverflow.ellipsis,
                        style: const TextStyle(
                          fontSize: 10.5,
                          color: C.faint,
                          height: 1.4,
                        ),
                      ),
                    ),
                  const SizedBox(height: 6),
                  _healthRow(
                    '位置',
                    f?.pos == null
                        ? '无里程计'
                        : '${f!.pos![0].toStringAsFixed(1)}, '
                              '${f.pos![1].toStringAsFixed(1)} m',
                    ok: f?.pos != null,
                  ),
                  _healthRow(
                    '朝向',
                    f?.yawDeg == null
                        ? '—'
                        : '${f!.yawDeg!.toStringAsFixed(0)}°',
                    ok: true,
                  ),
                  _healthRow(
                    '记忆',
                    '${f?.memoryCells ?? 0} 格 / '
                        '${(f?.memoryMetres ?? 0).toStringAsFixed(1)} m',
                    ok: true,
                  ),
                ],
              ),
            ),
          ),
          const SizedBox(height: 12),
          Panel(
            title: '运行',
            child: ValueListenableBuilder<VlaFrame?>(
              valueListenable: app.client.vla,
              builder: (context, f, _) => Column(
                children: [
                  KeyValue('场景', (f?.scene ?? '').isEmpty ? '—' : f!.scene),
                  KeyValue('感知源', (f?.source ?? '').isEmpty ? '—' : f!.source),
                  KeyValue('决策', (f?.auto ?? false) ? '自驱' : '按句 / 固定目标'),
                  KeyValue(
                    '已跑',
                    f == null ? '—' : '${f.t.toStringAsFixed(0)} s',
                  ),
                  if (f?.driver != null) KeyValue('目标层', f!.driver!),
                  if ((f?.lastError ?? '').isNotEmpty)
                    KeyValue('错误', f!.lastError!, color: C.danger),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _healthRow(
    String label,
    String value, {
    required bool ok,
    Color warnColor = C.warn,
  }) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2.5),
      child: Row(
        children: [
          Container(
            width: 5,
            height: 5,
            decoration: BoxDecoration(
              color: ok ? C.accent : warnColor,
              shape: BoxShape.circle,
            ),
          ),
          const SizedBox(width: 7),
          Expanded(
            child: Text(
              label,
              style: const TextStyle(fontSize: 11.5, color: C.dim),
            ),
          ),
          Text(
            value,
            style: numStyle.copyWith(
              fontSize: 11.5,
              color: ok ? C.text : warnColor,
            ),
          ),
        ],
      ),
    );
  }

  // -- 顶栏与指标条 ------------------------------------------------------

  Widget _topBar(AppState app, GatewayState? st) {
    final running = st?.vla.running ?? false;
    return LayoutBuilder(builder: (context, box) {
      // 窄屏上「接管方式」两块已经占满一行，再塞一个带文字的按钮会把文字挤成
      // 两行。这里退成图标按钮，动作不变：停 VLA。
      if (box.maxWidth < 560) {
        return Row(children: [
          Expanded(child: ModeSwitch(compact: true)),
          const SizedBox(width: 8),
          InkWell(
            borderRadius: BorderRadius.circular(10),
            onTap: running ? () => app.client.vlaAction('stop') : null,
            child: Container(
              width: 44,
              height: 44,
              decoration: BoxDecoration(
                color: running
                    ? C.warn.withValues(alpha: 0.12)
                    : C.surfaceHi,
                borderRadius: BorderRadius.circular(10),
                border: Border.all(
                    color: running ? C.warn.withValues(alpha: 0.4) : C.line),
              ),
              child: Icon(
                running ? Icons.stop_circle_outlined : Icons.hourglass_empty,
                size: 19,
                color: running ? C.warn : C.faint,
              ),
            ),
          ),
        ]);
      }
      return Row(children: [
        Expanded(child: ModeSwitch(compact: true)),
        const SizedBox(width: 10),
        ActionButton(
          label: running ? '停止 VLA' : 'VLA 未运行',
          icon: running ? Icons.stop_circle_outlined : Icons.hourglass_empty,
          color: running ? C.warn : C.faint,
          onPressed: running ? () => app.client.vlaAction('stop') : null,
        ),
      ]);
    });
  }

  Widget _metricsStrip(AppState app) {
    return Container(
      height: 62,
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
      decoration: BoxDecoration(
        color: C.surface,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: C.line),
      ),
      child: ValueListenableBuilder<VlaFrame?>(
        valueListenable: app.client.vla,
        builder: (context, f, _) => SingleChildScrollView(
          scrollDirection: Axis.horizontal,
          child: Row(
            children: [
              _chipMetric(
                'vx',
                (f?.vx ?? 0).toStringAsFixed(2),
                (f?.vx ?? 0).abs() > 0.01 ? C.accent : null,
              ),
              _chipMetric(
                '前方',
                f?.frontGap == null ? '—' : f!.frontGap!.toStringAsFixed(2),
                (f?.frontGap ?? 9) < 0.9 ? C.warn : null,
              ),
              _chipMetric('策略', '${(f?.policyHz ?? 0).toStringAsFixed(0)}Hz'),
              _chipMetric('推理', '${(f?.genS ?? 0).toStringAsFixed(2)}s'),
              _chipMetric(
                '朝向',
                f?.yawDeg == null ? '—' : '${f!.yawDeg!.toStringAsFixed(0)}°',
              ),
              _chipMetric(
                '安全闸',
                '${f?.guardTrips ?? 0}',
                (f?.guardTrips ?? 0) > 0 ? C.warn : null,
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _chipMetric(String label, String value, [Color? color]) {
    return Padding(
      padding: const EdgeInsets.only(right: 18),
      child: MetricTile(label: label, value: value, color: color ?? C.text),
    );
  }
}
