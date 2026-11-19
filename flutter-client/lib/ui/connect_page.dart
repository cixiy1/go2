import 'package:flutter/foundation.dart' show kIsWeb;
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import '../app_state.dart';
import '../core/models.dart';
import '../theme.dart';
import 'widgets/panel.dart';

// 连接页。第一屏只解决一件事：让 App 连上网关。
//
// 参数就三个（地址、端口、口令），但新手最容易卡住的不是填哪里，而是
// 「网关根本没起」。所以这一页把启动命令直接印出来，带复制按钮 —— 命令行参数
// 是这一层唯一需要背的东西，放在这里比写在 README 里有用。
class ConnectPage extends StatefulWidget {
  const ConnectPage({super.key});

  @override
  State<ConnectPage> createState() => _ConnectPageState();
}

class _ConnectPageState extends State<ConnectPage> {
  late final TextEditingController _host;
  late final TextEditingController _port;
  late final TextEditingController _token;
  late final TextEditingController _name;
  bool _busy = false;

  @override
  void initState() {
    super.initState();
    final s = AppScope.read(context).settings;
    _host = TextEditingController(text: s.host);
    _port = TextEditingController(text: s.port.toString());
    _token = TextEditingController(text: s.token);
    _name = TextEditingController(text: s.name);
  }

  @override
  void dispose() {
    _host.dispose();
    _port.dispose();
    _token.dispose();
    _name.dispose();
    super.dispose();
  }

  Future<void> _connect() async {
    final app = AppScope.read(context);
    final s = app.settings;
    s.host = _host.text.trim().isEmpty ? '127.0.0.1' : _host.text.trim();
    s.port = int.tryParse(_port.text.trim()) ?? 8780;
    s.token = _token.text.trim();
    s.name = _name.text.trim().isEmpty ? 'App' : _name.text.trim();
    await s.save();
    setState(() => _busy = true);
    await app.connect();
    if (!mounted) return;
    setState(() => _busy = false);
    if (app.client.connected) app.go(1);
  }

  void _useCurrentHost() {
    final host = Uri.base.host;
    if (host.isNotEmpty) _host.text = host;
  }

  @override
  Widget build(BuildContext context) {
    final app = AppScope.read(context);
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Center(
        child: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 760),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              _connectionPanel(app),
              const SizedBox(height: 12),
              _commandsPanel(context),
              const SizedBox(height: 12),
              LayoutBuilder(
                builder: (context, box) {
                  final two = box.maxWidth > 620;
                  final a = _hintPanel();
                  final b = _usagePanel();
                  return two
                      ? Row(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Expanded(child: a),
                            const SizedBox(width: 12),
                            Expanded(child: b),
                          ],
                        )
                      : Column(children: [a, const SizedBox(height: 12), b]);
                },
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _connectionPanel(AppState app) {
    return Panel(
      title: '连接网关',
      trailing: ValueListenableBuilder<LinkPhase>(
        valueListenable: app.client.phase,
        builder: (context, phase, _) {
          final (text, color) = switch (phase) {
            LinkPhase.open => ('在线', C.accent),
            LinkPhase.connecting => ('连接中', C.warn),
            LinkPhase.retrying => ('重连中', C.warn),
            LinkPhase.failed => ('连不上', C.danger),
            LinkPhase.idle => ('离线', C.faint),
          };
          return StatusChip(text: text, color: color, dense: true);
        },
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          LayoutBuilder(
            builder: (context, box) {
              final wide = box.maxWidth > 520;
              final hostField = _field(
                '网关地址',
                _host,
                hint: '192.168.123.1 或 127.0.0.1',
              );
              final portField = _field('端口', _port, hint: '8780',
                width: wide ? 110 : null);
              return wide
                  ? Row(
                      children: [
                        Expanded(flex: 3, child: hostField),
                        const SizedBox(width: 10),
                        portField,
                      ],
                    )
                  : Column(
                      children: [
                        hostField,
                        const SizedBox(height: 10),
                        portField,
                      ],
                    );
            },
          ),
          const SizedBox(height: 10),
          LayoutBuilder(
            builder: (context, box) {
              final wide = box.maxWidth > 520;
              final tokenField = _field('口令（网关没设就留空）', _token, hint: '可选');
              final nameField = _field('本机名称', _name, hint: '手机 / 平板');
              return wide
                  ? Row(
                      children: [
                        Expanded(flex: 3, child: tokenField),
                        const SizedBox(width: 10),
                        Expanded(flex: 2, child: nameField),
                      ],
                    )
                  : Column(
                      children: [
                        tokenField,
                        const SizedBox(height: 10),
                        nameField,
                      ],
                    );
            },
          ),
          const SizedBox(height: 14),
          Row(
            children: [
              Expanded(
                child: ValueListenableBuilder<LinkPhase>(
                  valueListenable: app.client.phase,
                  builder: (context, phase, _) {
                    final open = phase == LinkPhase.open;
                    return InkWell(
                      borderRadius: BorderRadius.circular(R.ctl),
                      onTap: _busy
                          ? null
                          : (open ? () => app.disconnect() : _connect),
                      child: Container(
                        height: 44,
                        alignment: Alignment.center,
                        decoration: BoxDecoration(
                          // 主操作用实心主色 + 白字。原来是 accent 16% 底
                          // 配 accent 字，等于蓝底蓝字 —— 截图里看不出有字。
                          color: open
                              ? C.surfaceHi
                              : (_busy
                                  ? C.accent.withValues(alpha: 0.5)
                                  : C.accent),
                          borderRadius: BorderRadius.circular(R.ctl),
                          border: open ? Border.all(color: C.line) : null,
                        ),
                        child: _busy
                            ? const SizedBox(
                                width: 16,
                                height: 16,
                                child: CircularProgressIndicator(
                                  strokeWidth: 2,
                                  color: C.accent,
                                ),
                              )
                            : Text(
                                open ? '断开' : '连接',
                                style: TextStyle(
                                  fontSize: 13,
                                  fontWeight: FontWeight.w600,
                                  color: open ? C.dim : Colors.white,
                                ),
                              ),
                      ),
                    );
                  },
                ),
              ),
              if (kIsWeb) ...[
                const SizedBox(width: 10),
                ActionButton(
                  label: '用当前地址',
                  icon: Icons.link,
                  onPressed: _useCurrentHost,
                ),
              ],
            ],
          ),
          ValueListenableBuilder<GatewayState?>(
            valueListenable: app.client.state,
            builder: (context, st, _) {
              if (st == null) return const SizedBox.shrink();
              return Padding(
                padding: const EdgeInsets.only(top: 14),
                child: Column(
                  children: [
                    const Divider(height: 1, color: C.line),
                    const SizedBox(height: 10),
                    KeyValue('机器人', st.link.target ?? '未上报'),
                    KeyValue('端点', st.link.preset == 'real' ? '真机' : '仿真'),
                    KeyValue('模式', _modeLabel(st.mode)),
                    KeyValue(
                      '延迟',
                      st.link.rttMs == null
                          ? '—'
                          : '${st.link.rttMs!.toStringAsFixed(1)} ms',
                    ),
                    KeyValue('网关', '已运行 ${st.uptime.toStringAsFixed(0)} s'),
                  ],
                ),
              );
            },
          ),
        ],
      ),
    );
  }

  Widget _field(
    String label,
    TextEditingController controller, {
    String? hint,
    double? width,
  }) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Kicker(label),
        const SizedBox(height: 5),
        SizedBox(
          width: width,
          child: TextField(
            controller: controller,
            style: const TextStyle(fontSize: 13),
            decoration: InputDecoration(hintText: hint),
            keyboardType: label == '端口' ? TextInputType.number : null,
            autocorrect: false,
            enableSuggestions: false,
          ),
        ),
      ],
    );
  }

  Widget _commandsPanel(BuildContext context) {
    const local = 'python -m quadruped.comm.gateway --preset sim';
    const real =
        'python -m quadruped.comm.gateway --preset real '
        '--host 0.0.0.0 --token 123456';
    return Panel(
      title: '在机器人那台机器上起网关',
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _cmd(context, '仿真（本机调试）', local),
          const SizedBox(height: 8),
          _cmd(context, '真机（手机要连进来，必须给 token）', real),
          const SizedBox(height: 10),
          const Text(
            '网关默认只监听 127.0.0.1。手机要连，必须 --host 0.0.0.0 并设口令；'
            '这个接口能让机器狗动起来，别把它暴露到公网。',
            style: TextStyle(fontSize: 11.5, color: C.faint, height: 1.5),
          ),
        ],
      ),
    );
  }

  Widget _cmd(BuildContext context, String label, String command) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(label, style: const TextStyle(fontSize: 11, color: C.dim)),
        const SizedBox(height: 4),
        Container(
          padding: const EdgeInsets.only(left: 10, right: 4, top: 2, bottom: 2),
          decoration: BoxDecoration(
            color: const Color(0xFF070A0C),
            borderRadius: BorderRadius.circular(R.inner),
          ),
          child: Row(
            children: [
              Expanded(
                child: SingleChildScrollView(
                  scrollDirection: Axis.horizontal,
                  child: Text(
                    command,
                    style: const TextStyle(
                      fontFamily: 'monospace',
                      fontSize: 11.5,
                      color: C.accent,
                    ),
                  ),
                ),
              ),
              IconButton(
                tooltip: '复制',
                iconSize: 15,
                color: C.faint,
                onPressed: () {
                  Clipboard.setData(ClipboardData(text: command));
                  ScaffoldMessenger.of(context)
                    ..removeCurrentSnackBar()
                    ..showSnackBar(
                      const SnackBar(
                        content: Text('已复制'),
                        duration: Duration(milliseconds: 900),
                        behavior: SnackBarBehavior.floating,
                        backgroundColor: C.surfaceHi,
                      ),
                    );
                },
                icon: const Icon(Icons.copy_rounded),
              ),
            ],
          ),
        ),
      ],
    );
  }

  static String _modeLabel(String mode) => switch (mode) {
    'manual' => '手动遥控',
    'vla' => 'VLA 自主',
    'switching' => '切换中',
    _ => '待机',
  };

  Widget _hintPanel() {
    return const Panel(
      title: '连不上先看这三条',
      child: Column(
        children: [
          KeyValue('1', '网关卡起了吗：终端里应该打印「网关监听 ws://…」'),
          KeyValue('2', '同一个网吗：手机 WiFi 和机器人要在同一网段'),
          KeyValue('3', '口令对吗：网关启动时会打印 token，填进上面那个框'),
        ],
      ),
    );
  }

  Widget _usagePanel() {
    return const Panel(
      title: '两种控制方式互斥',
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            '手动遥控时，机器狗的控制权在「机器人服务端」手里；VLA 自主时，'
            '控制权在 VLA 子进程手里。同一时刻只能有一个 —— 这是 rt/lowcmd 的硬约束，'
            '不是界面上的约定。切模式时网关会先 park、再交接，中间那几秒狗是蹲着的。',
            style: TextStyle(fontSize: 11.5, color: C.dim, height: 1.6),
          ),
        ],
      ),
    );
  }
}
