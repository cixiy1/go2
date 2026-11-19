import 'dart:math' as math;

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
            // 横屏手机（896x414）可用高度只有 300 出头：三列并排时每一列都
            // 塞不下自己的卡片，逐处加约束只会把溢出从 250 压到 126、再到 27。
            // 换成「整页滚动 + 每块固定高度」，结构上不留溢出空间。
            final landscape = !wide && box.maxWidth > box.maxHeight * 1.25;
            return Padding(
              padding: EdgeInsets.all(landscape ? 8 : 12),
              child: wide
                  ? _wide(app, st)
                  : landscape
                      ? _landscape(app, st, box)
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

  // 横屏：整页滚动，画面给 210、判断给 300，剩下的滚出去看。
  // 固定高度而不是 Expanded —— Expanded 要求父级高度确定，在滚动视图里会崩。
  Widget _landscape(AppState app, GatewayState? st, BoxConstraints box) {
    final h = box.maxHeight;
    return ListView(
      padding: EdgeInsets.zero,
      children: [
        _topBar(app, st),
        const SizedBox(height: 10),
        _metricsStrip(app),
        const SizedBox(height: 10),
        SizedBox(
          height: math.max(224, h * 0.5),
          child: Row(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              Expanded(flex: 3, child: _visionColumn(app, st, compact: true)),
              const SizedBox(width: 10),
              Expanded(flex: 2, child: _judgeMini(app, st)),
            ],
          ),
        ),
        const SizedBox(height: 10),
        SizedBox(
          height: math.max(280, h * 0.62),
          child: _decisionColumn(app, st),
        ),
        const SizedBox(height: 10),
        _inputRow(app, st),
        const SizedBox(height: 8),
      ],
    );
  }

  // 横屏右侧那一列只留判断，不要对话（对话在下面单独一块占满宽）。
  Widget _judgeMini(AppState app, GatewayState? st) {
    return _thoughtPanel(app);
  }

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
  // 这一版让「判断」和雷达并排，省下的高度给对话。
  //
  // 除顶栏外**每一项都是弹性的**，没有一个写死的像素高度。写死的那一版在
  // 414×896 上溢出 46 px、360×640 上溢出 175 px（悬浮底栏的安全区吃掉的
  // 高度随机型变），表现是对话区被切掉一截。屏幕特别矮时把指标条省掉 ——
  // 那几个数在判断卡片和顶栏里都能看到。
  Widget _narrow(AppState app, GatewayState? st) {
    final tall = MediaQuery.sizeOf(context).height >= 720;
    return Column(
      children: [
        _topBar(app, st),
        if (tall) ...[
          const SizedBox(height: 10),
          _metricsStrip(app),
        ],
        const SizedBox(height: 10),
        Flexible(
          flex: 4,
          child: ValueListenableBuilder<CameraFrame?>(
            valueListenable: app.client.frame,
            builder: (context, frame, _) => ConstrainedBox(
              constraints: const BoxConstraints(maxHeight: 220),
              child: ValueListenableBuilder<VlaFrame?>(
                valueListenable: app.client.vla,
                builder: (context, f, _) => CameraPanel(
                  frame: frame?.bytes,
                  aspect: frame?.aspect,
                  frontGap: f?.frontGap,
                  alert: f?.alert ?? false,
                  running: st?.vla.running ?? false,
                ),
              ),
            ),
          ),
        ),
        const SizedBox(height: 10),
        Flexible(
          flex: 3,
          child: Row(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              Expanded(
                flex: 3,
                child: _thoughtPanel(app, maxLines: 4, fill: true),
              ),
              if (tall) ...[
                const SizedBox(width: 10),
                Expanded(flex: 2, child: _radarPanel(app)),
              ],
            ],
          ),
        ),
        const SizedBox(height: 10),
        Flexible(flex: 5, child: _chatPanel(app, st)),
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

  Widget _thoughtPanel(AppState app, {int maxLines = 6, bool fill = false}) {
    return Panel(
      title: '判断',
      trailing: ValueListenableBuilder<VlaFrame?>(
        valueListenable: app.client.vla,
        builder: (context, f, _) => Text(
          f == null ? '' : '#${f.thoughts}',
          style: const TextStyle(fontSize: 10.5, color: C.faint),
        ),
      ),
      // 内容可滚动。模型那句话一长（它经常一次写三四行）就会把卡片撑破，
      // 报 RenderFlex overflow；窄屏那张卡片的高度是跟屏幕按比例分的，
      // 只有 `fill` 才能把滚动视图压进这个高度里。
      fill: fill,
      child: SingleChildScrollView(
        child: ValueListenableBuilder<VlaFrame?>(
          valueListenable: app.client.vla,
          builder: (context, f, _) =>
              ThoughtCard(frame: f, maxLines: maxLines),
        ),
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
        // Flexible 不是装饰：CameraPanel 内部是 AspectRatio，按宽度算高度。
        // 在 Column 里当非弹性子节点时它拿到的是无界高度，会按 4:3 撑到 387，
        // 横屏（总高 302）直接溢出 250。给一个有界高度它才会按比例缩宽度。
        Flexible(
          flex: 3,
          child: ValueListenableBuilder<CameraFrame?>(
            valueListenable: app.client.frame,
            builder: (context, frame, _) {
              return ValueListenableBuilder<VlaFrame?>(
                valueListenable: app.client.vla,
                builder: (context, f, _) => CameraPanel(
                  frame: frame?.bytes,
                  aspect: frame?.aspect,
                  frontGap: f?.frontGap,
                  alert: f?.alert ?? false,
                  running: st?.vla.running ?? false,
                ),
              );
            },
          ),
        ),
        const SizedBox(height: 10),
        Expanded(
          flex: 2,
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
        // 想法面板原来是非弹性子节点：竖屏高度富余时没差别，横屏 302 高
        // 就顶出去 49。给 flex 2:3，两边按剩余高度分。
        Flexible(flex: 2, child: _thoughtPanel(app)),
        const SizedBox(height: 10),
        Expanded(flex: 3, child: _chatPanel(app, st)),
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
          borderRadius: BorderRadius.circular(R.inner),
          onTap: running && !_sending ? _say : null,
          child: Container(
            width: 42,
            height: 40,
            decoration: BoxDecoration(
              color: running ? C.accent.withValues(alpha: 0.16) : C.surfaceHi,
              borderRadius: BorderRadius.circular(R.inner),
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
                        color: C.accentText,
                      ),
                    ),
                  )
                : Icon(
                    Icons.send_rounded,
                    size: 17,
                    color: running ? C.accentText : C.faint,
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
                          color: (f?.vx ?? 0).abs() > 0.01
                              ? C.accentText
                              : C.dim,
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
                  // 用 Wrap 不用 Row：三枚胶囊的宽度都随内容变（步态名可能是
                  // 一整句，转角是三位数），右列只有 300 px 时 Row 会溢出 32 px
                  // 压出黄黑条纹。换行比截断好读。
                  Wrap(
                    spacing: 6,
                    runSpacing: 6,
                    crossAxisAlignment: WrapCrossAlignment.center,
                    children: [
                      StatusChip(
                        text: f?.gait ?? '—',
                        color: C.info,
                        dense: true,
                      ),
                      if (f?.turnLeftDeg != null)
                        StatusChip(
                          text: '转 ${f!.turnLeftDeg!.toStringAsFixed(0)}°',
                          color: C.warn,
                          dense: true,
                        ),
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
            borderRadius: BorderRadius.circular(R.inner),
            onTap: running ? () => app.client.vlaAction('stop') : null,
            child: Container(
              width: 44,
              height: 44,
              decoration: BoxDecoration(
                color: running
                    ? C.warn.withValues(alpha: 0.12)
                    : C.surfaceHi,
                borderRadius: BorderRadius.circular(R.inner),
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
        borderRadius: BorderRadius.circular(R.card),
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
