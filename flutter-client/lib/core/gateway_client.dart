import 'dart:async';
import 'dart:convert';

import 'package:flutter/foundation.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

import 'models.dart';

// 一条到网关的 WebSocket。上层只读 ValueNotifier，不碰 socket 细节。
//
// 三件必须做对的事：
//
//   1. 状态帧 10 Hz、画面 5 Hz，都不该触发整棵树重建 —— 所以分发成一堆细粒度
//      的 notifier，页面只在用到的那一个上挂监听。
//   2. 速度设定值不等应答。等 20 Hz 的往返，延迟比指令本身还长；丢了下一帧就补上。
//   3. 断线自动重连，但重连成功要重新取操作权 —— 网关那头认为「你走了」，
//      会把操作权释放掉（这是安全阀，不是 bug）。
class GatewayClient {
  final ValueNotifier<LinkPhase> phase = ValueNotifier(LinkPhase.idle);
  final ValueNotifier<GatewayState?> state = ValueNotifier(null);
  final ValueNotifier<VlaFrame?> vla = ValueNotifier(null);
  final ValueNotifier<CameraFrame?> frame = ValueNotifier(null);
  final ValueNotifier<String?> error = ValueNotifier(null);
  final ValueNotifier<String?> notice = ValueNotifier(null);
  final ValueNotifier<int> chatRev = ValueNotifier(0);
  final ValueNotifier<int> eventRev = ValueNotifier(0);
  final ValueNotifier<double?> rttMs = ValueNotifier(null);

  final List<ChatItem> chat = [];
  final List<GatewayEvent> events = [];
  Limits limits = const Limits();

  WebSocketChannel? _ch;
  StreamSubscription<dynamic>? _sub;
  Timer? _ping;
  Timer? _retry;
  int _rid = 1;
  int _attempt = 0;
  int _lastReplySeq = -1;
  bool _want = false;
  Uri? _uri;
  String _name = 'App';
  final Map<int, Completer<Map<String, dynamic>>> _pending = {};

  bool get connected => phase.value == LinkPhase.open;
  String get deviceName => _name;

  // -- 连接 ------------------------------------------------------------

  Future<void> connect(Uri uri, {String name = 'App'}) async {
    _want = true;
    _uri = uri;
    _name = name;
    _attempt = 0;
    await _teardown();
    await _open(uri);
  }

  Future<void> _open(Uri uri) async {
    phase.value = _attempt == 0 ? LinkPhase.connecting : LinkPhase.retrying;
    error.value = null;
    try {
      final ch = WebSocketChannel.connect(uri);
      await ch.ready.timeout(const Duration(seconds: 6));
      if (!_want) {
        await ch.sink.close();
        return;
      }
      _ch = ch;
      _sub = ch.stream.listen(
        _onData,
        onError: _onError,
        onDone: _onDone,
        cancelOnError: false,
      );
      phase.value = LinkPhase.open;
      _attempt = 0;
      _send({
        'op': 'hello',
        'proto': 1,
        'token': uri.queryParameters['t'] ?? '',
        'name': _name,
      });
      _ping?.cancel();
      _ping = Timer.periodic(const Duration(seconds: 2), (_) {
        _send({'op': 'ping', 'ts': DateTime.now().millisecondsSinceEpoch});
      });
    } catch (e) {
      error.value = _explain(e);
      phase.value = LinkPhase.failed;
      _scheduleRetry();
    }
  }

  void _scheduleRetry() {
    if (!_want || _uri == null) return;
    _attempt++;
    // 1s → 2s → 4s → 8s 封顶。网关重启通常几秒，退避太久会让用户以为连不上。
    final wait = Duration(seconds: 1 << (_attempt.clamp(0, 3)));
    phase.value = LinkPhase.retrying;
    _retry?.cancel();
    _retry = Timer(wait, () {
      if (_want && _uri != null) _open(_uri!);
    });
  }

  String _explain(Object e) {
    final text = e.toString();
    if (text.contains('Connection refused')) return '网关没起（连接被拒绝）';
    if (text.contains('timed out') || text.contains('Timeout')) {
      return '连接超时：地址对吗？手机和机器人在同一个网吗？';
    }
    if (text.contains('401')) return '口令不对';
    return text;
  }

  Future<void> disconnect() async {
    _want = false;
    _retry?.cancel();
    await _teardown();
    phase.value = LinkPhase.idle;
    state.value = null;
    vla.value = null;
    frame.value = null;
  }

  Future<void> _teardown() async {
    _ping?.cancel();
    _ping = null;
    await _sub?.cancel();
    _sub = null;
    try {
      await _ch?.sink.close();
    } catch (_) {
      // 已经断了的 socket 再关一次会抛，忽略。
    }
    _ch = null;
    for (final c in _pending.values) {
      if (!c.isCompleted) c.completeError(StateError('连接已断开'));
    }
    _pending.clear();
  }

  void _onError(Object e) {
    error.value = _explain(e);
  }

  void _onDone() {
    if (phase.value != LinkPhase.idle) {
      phase.value = LinkPhase.retrying;
    }
    _ping?.cancel();
    _sub = null;
    _ch = null;
    for (final c in _pending.values) {
      if (!c.isCompleted) c.completeError(StateError('连接已断开'));
    }
    _pending.clear();
    _scheduleRetry();
  }

  // -- 收发 ------------------------------------------------------------

  void _send(Map<String, dynamic> obj) {
    final ch = _ch;
    if (ch == null) return;
    try {
      ch.sink.add(jsonEncode(obj));
    } catch (_) {
      // 半关状态的 socket：丢掉这条，重连逻辑会补上。
    }
  }

  void _onData(dynamic data) {
    if (data is String) {
      _onText(data);
      return;
    }
    if (data is List<int>) {
      final raw = data is Uint8List ? data : Uint8List.fromList(data);
      if (raw.isEmpty) return;
      // 首字节是来源标记，剩下的是 JPEG。`sublistView` 不复制像素数据。
      frame.value = CameraFrame(raw[0], Uint8List.sublistView(raw, 1));
    }
  }

  void _onText(String raw) {
    Map<String, dynamic> msg;
    try {
      final decoded = jsonDecode(raw);
      if (decoded is! Map) return;
      msg = decoded.map((k, v) => MapEntry(k.toString(), v));
    } catch (_) {
      return;
    }
    switch (msg['op']) {
      case 'welcome':
        limits = Limits.fromJson(
          (msg['limits'] as Map?)?.map((k, v) => MapEntry(k.toString(), v)) ??
              const {},
        );
        final dialog = (msg['dialog'] as List?) ?? const [];
        for (final item in dialog) {
          if (item is Map) {
            final text = item['text']?.toString() ?? '';
            if (text.isNotEmpty) {
              _appendChat(
                ChatItem(
                  text: text,
                  mine: item['mine'] == true,
                  seq: item['seq'] is num ? (item['seq'] as num).toInt() : null,
                ),
              );
            }
          }
        }
        final last = msg['vla_last'];
        if (last is Map) {
          vla.value = VlaFrame.fromJson(
            last.map((k, v) => MapEntry(k.toString(), v)),
          );
        }
        phase.value = LinkPhase.open;
      case 'state':
        final s = GatewayState.fromJson(msg);
        limits = s.limits;
        state.value = s;
        final r = rttMs.value;
        if (s.link.rttMs != null) {
          rttMs.value = s.link.rttMs;
        } else if (r != null) {
          rttMs.value = r;
        }
      case 'vla':
        final data = msg['data'];
        if (data is Map) {
          final f = VlaFrame.fromJson(
            data.map((k, v) => MapEntry(k.toString(), v)),
          );
          vla.value = f;
          // 回话是「狗说的」，只在真的换了一句时入列。逐帧比对 seq 而不是
          // 文本：同一句话重复说两遍（模型复读）也要看得见。
          if (f.reply.isNotEmpty && f.replySeq != _lastReplySeq) {
            _lastReplySeq = f.replySeq;
            _appendChat(ChatItem(text: f.reply, mine: false, seq: f.replySeq));
          }
        }
      case 'ack':
        final id = msg['id'];
        if (id is num) {
          final c = _pending.remove(id.toInt());
          if (c != null && !c.isCompleted) c.complete(msg);
        }
      case 'event':
        final ev = GatewayEvent(
          msg['kind']?.toString() ?? 'note',
          msg['text']?.toString() ?? '',
        );
        _appendEvent(ev);
        if (ev.isError) error.value = ev.text;
      case 'error':
        error.value = msg['msg']?.toString() ?? '网关报错';
      case 'bye':
        _appendEvent(GatewayEvent('bye', '网关已关闭'));
        phase.value = LinkPhase.retrying;
    }
  }

  void _appendChat(ChatItem item) {
    if (!item.mine &&
        chat.isNotEmpty &&
        !chat.last.mine &&
        chat.last.text == item.text) {
      chat[chat.length - 1] = ChatItem(
        text: item.text,
        mine: false,
        at: chat.last.at,
        seq: item.seq,
        repeat: chat.last.repeat + 1,
      );
      chatRev.value++;
      return;
    }
    chat.add(item);
    if (chat.length > 200) chat.removeAt(0);
    chatRev.value++;
  }

  // 我说的话先本地入列。等网关回显要一个往返，而输入框下面立刻出现自己那句
  // 才算「发出去了」；狗的回话由 `vla` 帧里的 reply 补进来。
  void sayLocal(String text) => _appendChat(ChatItem(text: text, mine: true));

  void _appendEvent(GatewayEvent ev) {
    events.add(ev);
    if (events.length > 200) events.removeAt(0);
    eventRev.value++;
    if (ev.kind == 'mode' || ev.kind == 'lease') notice.value = ev.text;
  }

  // 发一条要应答的请求。切模式这类「几秒后才完成」的 op 也是立刻应答
  // （网关回 switching），进度靠 event 和状态帧推进。
  Future<Map<String, dynamic>> request(
    String op, [
    Map<String, dynamic> extra = const {},
  ]) async {
    if (!connected) {
      return {'ok': false, 'msg': '没有连接'};
    }
    final id = _rid++;
    final completer = Completer<Map<String, dynamic>>();
    _pending[id] = completer;
    _send({'op': op, 'id': id, ...extra});
    try {
      return await completer.future.timeout(const Duration(seconds: 8));
    } on TimeoutException {
      _pending.remove(id);
      return {'ok': false, 'msg': '网关 8 秒没回应'};
    } catch (e) {
      return {'ok': false, 'msg': e.toString()};
    }
  }

  // 速度设定值：不排队、不等应答、不重传。20 Hz 的流里丢一帧无所谓，
  // 排队反而会把旧速度在新速度之后发出去。
  void setpoint(double vx, double vy, double vyaw, {String source = 'touch'}) {
    _send({
      'op': 'setpoint',
      'vx': vx,
      'vy': vy,
      'vyaw': vyaw,
      'source': source,
    });
  }

  Future<Map<String, dynamic>> lease(bool take) =>
      request('lease', {'action': take ? 'acquire' : 'release'});

  Future<Map<String, dynamic>> setMode(String mode) =>
      request('mode', {'mode': mode});

  Future<Map<String, dynamic>> command(
    String cmd, [
    Map<String, dynamic> args = const {},
  ]) => request('cmd', {'cmd': cmd, 'args': args});

  Future<Map<String, dynamic>> say(String text) =>
      request('say', {'text': text});

  Future<Map<String, dynamic>> vlaAction(String action) =>
      request('vla', {'action': action});

  void camera(bool on) => _send({'op': 'camera', 'on': on});

  Future<Map<String, dynamic>> logLines([int n = 120]) =>
      request('log', {'n': n});
}
