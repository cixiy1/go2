// 网关报文的 Dart 一侧。字段名与 `quadruped/comm/gateway.py`、
// `quadruped/control/embodied/runner.py` 一一对应，改一侧就要改另一侧。
//
// 全部可选字段都给了默认值：网关在切模式的中间态会发出「半份」状态帧
// （机器人已经放下、VLA 还没起来），这时候字段就是缺的。缺字段当 0 显示还行，
// 抛异常会白屏 —— 遥控界面白屏比少显示一个数字危险得多。

import 'dart:typed_data';

// 二进制帧 = 1 字节来源 + JPEG。来源分两种：VLA 渲染窗口（叠了想法字幕、
// 视野条）和纯相机图。界面上要标出来 —— 把合成图当成「实景」会误判。
class CameraFrame {
  const CameraFrame(this.kind, this.bytes);
  final int kind;
  final Uint8List bytes;

  static const vla = 1;
  static const camera = 2;

  bool get isVla => kind == vla;
  bool get isCamera => kind == camera;
  String get label => isVla ? 'VLA 渲染' : (isCamera ? '机载相机' : '画面');
}

double? _d(dynamic v) {
  if (v is num) return v.toDouble();
  if (v is String) return double.tryParse(v);
  return null;
}

int? _i(dynamic v) {
  if (v is num) return v.toInt();
  if (v is String) return int.tryParse(v);
  return null;
}

bool _b(dynamic v) => v == true;

String _s(dynamic v, [String fallback = '']) =>
    v == null ? fallback : v.toString();

Map<String, dynamic> _m(dynamic v) =>
    v is Map ? v.map((k, val) => MapEntry(k.toString(), val)) : const {};

List<double>? _vec(dynamic v) {
  if (v is List && v.isNotEmpty) {
    return v.map((e) => _d(e) ?? 0).toList(growable: false);
  }
  return null;
}

class Limits {
  const Limits({
    this.vx = 1.0,
    this.vy = 0.3,
    this.vyaw = 4.0,
    this.crouchMin = 0.02,
    this.crouchMax = 0.18,
  });
  final double vx, vy, vyaw, crouchMin, crouchMax;

  factory Limits.fromJson(Map<String, dynamic> j) => Limits(
    vx: _d(j['vx']) ?? 1.0,
    vy: _d(j['vy']) ?? 0.3,
    vyaw: _d(j['vyaw']) ?? 4.0,
    crouchMin: _d((j['crouch'] as List?)?.first) ?? 0.02,
    crouchMax: _d((j['crouch'] as List?)?.last) ?? 0.18,
  );
}

class LinkInfo {
  const LinkInfo({
    this.ok = false,
    this.error,
    this.preset,
    this.target,
    this.rttMs,
  });
  final bool ok;
  final String? error;
  final String? preset;
  final String? target;
  final double? rttMs;

  factory LinkInfo.fromJson(Map<String, dynamic> j) => LinkInfo(
    ok: _b(j['ok']),
    error: j['error'] == null ? null : _s(j['error']),
    preset: j['preset'] == null ? null : _s(j['preset']),
    target: j['target'] == null ? null : _s(j['target']),
    rttMs: _d(j['rtt_ms']),
  );
}

class RobotInfo {
  const RobotInfo({
    this.pos,
    this.rpy,
    this.height,
    this.gait,
    this.busy,
    this.moving,
    this.loop,
    this.error,
  });
  final List<double>? pos;
  final List<double>? rpy;
  final double? height;
  final String? gait;
  final bool? busy;
  final bool? moving;
  final String? loop;
  final String? error;

  double? get roll => (rpy?.isNotEmpty ?? false) ? rpy![0] : null;
  double? get pitch => rpy != null && rpy!.length > 1 ? rpy![1] : null;
  double? get yaw => rpy != null && rpy!.length > 2 ? rpy![2] : null;

  factory RobotInfo.fromJson(Map<String, dynamic> j) => RobotInfo(
    pos: _vec(j['pos']),
    rpy: _vec(j['rpy']),
    height: _d(j['height']),
    gait: j['gait'] == null ? null : _s(j['gait']),
    busy: j['busy'] is bool ? j['busy'] as bool : null,
    moving: j['moving'] is bool ? j['moving'] as bool : null,
    loop: j['loop'] == null ? null : _s(j['loop']),
    error: j['error'] == null ? null : _s(j['error']),
  );
}

class InputInfo {
  const InputInfo({
    this.vx = 0,
    this.vy = 0,
    this.vyaw = 0,
    this.ageMs,
    this.source = '-',
    this.driving = false,
  });
  final double vx, vy, vyaw;
  final int? ageMs;
  final String source;
  final bool driving;

  factory InputInfo.fromJson(Map<String, dynamic> j) => InputInfo(
    vx: _d(j['vx']) ?? 0,
    vy: _d(j['vy']) ?? 0,
    vyaw: _d(j['vyaw']) ?? 0,
    ageMs: _i(j['age_ms']),
    source: _s(j['source'], '-'),
    driving: _b(j['driving']),
  );
}

class LeaseInfo {
  const LeaseInfo({this.holder, this.mine = false, this.clients = 0});
  final int? holder;
  final bool mine;
  final int clients;

  factory LeaseInfo.fromJson(Map<String, dynamic> j) => LeaseInfo(
    holder: _i(j['holder']),
    mine: _b(j['mine']),
    clients: _i(j['clients']) ?? 0,
  );
}

class CamInfo {
  const CamInfo({this.running = false, this.pid, this.note = '', this.exit});
  final bool running;
  final int? pid;

  // 没在跑时，`note` 是它最后一行的死因（端口被占用、没有相机…）。
  // 界面直接把这句话显示出来，比「没有画面」四个字有用。
  final String note;
  final int? exit;

  factory CamInfo.fromJson(Map<String, dynamic> j) => CamInfo(
    running: _b(j['running']),
    pid: _i(j['pid']),
    note: _s(j['note']),
    exit: _i(j['exit']),
  );
}

class VlaRuntime {
  const VlaRuntime({this.running = false, this.pid, this.uptime, this.log});
  final bool running;
  final int? pid;
  final double? uptime;
  final String? log;

  factory VlaRuntime.fromJson(Map<String, dynamic> j) => VlaRuntime(
    running: _b(j['running']),
    pid: _i(j['pid']),
    uptime: _d(j['uptime']),
    log: j['log'] == null ? null : _s(j['log']),
  );
}

class GatewayState {
  const GatewayState({
    required this.at,
    this.mode = 'off',
    this.note = '',
    this.link = const LinkInfo(),
    this.robot = const RobotInfo(),
    this.input = const InputInfo(),
    this.lease = const LeaseInfo(),
    this.vla = const VlaRuntime(),
    this.cam = const CamInfo(),
    this.limits = const Limits(),
    this.uptime = 0,
  });

  final DateTime at;
  final String mode;
  final String note;
  final LinkInfo link;
  final RobotInfo robot;
  final InputInfo input;
  final LeaseInfo lease;
  final VlaRuntime vla;
  final CamInfo cam;
  final Limits limits;
  final double uptime;

  factory GatewayState.fromJson(Map<String, dynamic> j) => GatewayState(
    at: DateTime.now(),
    mode: _s(j['mode'], 'off'),
    note: _s(j['note']),
    link: LinkInfo.fromJson(_m(j['link'])),
    robot: RobotInfo.fromJson(_m(j['robot'])),
    input: InputInfo.fromJson(_m(j['input'])),
    lease: LeaseInfo.fromJson(_m(j['lease'])),
    vla: VlaRuntime.fromJson(_m(j['vla'])),
    limits: Limits.fromJson(_m(j['limits'])),
    uptime: _d(_m(j['server'])['uptime']) ?? 0,
  );
}

class SectorGap {
  const SectorGap(this.name, this.gap, this.top);
  final String name;
  final double gap;
  final double top;

  factory SectorGap.fromJson(Map<String, dynamic> j) =>
      SectorGap(_s(j['name']), _d(j['gap']) ?? 0, _d(j['top']) ?? 0);
}

class VisionInfo {
  const VisionInfo({
    this.frames = 0,
    this.dropped = 0,
    this.views = 0,
    this.genS = 0,
  });
  final int frames, dropped, views;
  final double genS;
}

// VLA 的一帧。5 Hz 从 `--events` 的 UDP 通道过来，网关转成 WS 文本帧。
class VlaFrame {
  const VlaFrame({
    required this.at,
    this.seq = 0,
    this.t = 0,
    this.scene = '',
    this.source = '',
    this.auto = false,
    this.vx = 0,
    this.vy = 0,
    this.vyaw = 0,
    this.gait,
    this.frontGap,
    this.sectors = const [],
    this.sceneText = '',
    this.sceneAge,
    this.thought = '',
    this.reply = '',
    this.replySeq = 0,
    this.emitThought = false,
    this.goal,
    this.turns = 0,
    this.turnLeftDeg,
    this.thoughts = 0,
    this.genS = 0,
    this.thinkRate = 0,
    this.lastError,
    this.guardTrips = 0,
    this.guardLast,
    this.memoryCells = 0,
    this.memoryMetres = 0,
    this.pos,
    this.yawDeg,
    this.tiltDeg,
    this.policyHz = 0,
    this.reflexHz = 0,
    this.viewFps,
    this.vision = const VisionInfo(),
    this.driver,
    this.alert = false,
  });

  final DateTime at;
  final int seq;
  final double t;
  final String scene;
  final String source;
  final bool auto;
  final double vx, vy, vyaw;
  final String? gait;
  final double? frontGap;
  final List<SectorGap> sectors;
  final String sceneText;
  final double? sceneAge;
  final String thought;
  final String reply;
  final int replySeq;
  final bool emitThought;
  final String? goal;
  final int turns;
  final double? turnLeftDeg;
  final int thoughts;
  final double genS;
  final double thinkRate;
  final String? lastError;
  final int guardTrips;
  final String? guardLast;
  final int memoryCells;
  final double memoryMetres;
  final List<double>? pos;
  final double? yawDeg;
  final double? tiltDeg;
  final double policyHz;
  final double reflexHz;
  final double? viewFps;
  final VisionInfo vision;
  final String? driver;
  final bool alert;

  bool get hasData => seq > 0;

  factory VlaFrame.fromJson(Map<String, dynamic> j) {
    final cmd = _m(j['cmd']);
    final guard = _m(j['guard']);
    final mem = _m(j['memory']);
    final vision = _m(j['vision']);
    final driver = _m(j['driver']);
    final secs = (j['sectors'] as List?) ?? const [];
    return VlaFrame(
      at: DateTime.now(),
      seq: _i(j['seq']) ?? 0,
      t: _d(j['t']) ?? 0,
      scene: _s(j['scene']),
      source: _s(j['source']),
      auto: _b(j['auto']),
      vx: _d(cmd['vx']) ?? 0,
      vy: _d(cmd['vy']) ?? 0,
      vyaw: _d(cmd['vyaw']) ?? 0,
      gait: j['gait'] == null ? null : _s(j['gait']),
      frontGap: _d(j['front_gap']),
      sectors: secs
          .whereType<Map>()
          .map(
            (e) =>
                SectorGap.fromJson(e.map((k, v) => MapEntry(k.toString(), v))),
          )
          .toList(growable: false),
      sceneText: _s(j['scene_text']),
      sceneAge: _d(j['scene_age']),
      thought: _s(j['thought']),
      reply: _s(j['reply']),
      replySeq: _i(j['reply_seq']) ?? 0,
      emitThought: _b(j['emit_thought']),
      goal: j['goal'] == null ? null : _s(j['goal']),
      turns: _i(j['turns']) ?? 0,
      turnLeftDeg: _d(j['turn_left_deg']),
      thoughts: _i(j['thoughts']) ?? 0,
      genS: _d(j['gen_s']) ?? 0,
      thinkRate: _d(j['think_rate']) ?? 0,
      lastError: j['last_error'] == null ? null : _s(j['last_error']),
      guardTrips: _i(guard['trips']) ?? 0,
      guardLast: guard['last'] == null ? null : _s(guard['last']),
      memoryCells: _i(mem['cells']) ?? 0,
      memoryMetres: _d(mem['travelled']) ?? 0,
      pos: _vec(j['pos']),
      yawDeg: _d(j['yaw_deg']),
      tiltDeg: _d(j['tilt_deg']),
      policyHz: _d(j['policy_hz']) ?? 0,
      reflexHz: _d(j['reflex_hz']) ?? 0,
      viewFps: _d(j['view_fps']),
      vision: VisionInfo(
        frames: _i(vision['frames']) ?? 0,
        dropped: _i(vision['dropped']) ?? 0,
        views: _i(vision['views']) ?? 0,
        genS: _d(vision['gen_s']) ?? 0,
      ),
      driver: driver['direction'] == null
          ? null
          : '${_s(driver['direction'])}'
                '（换 ${_i(driver['changes']) ?? 0} 次，受阻 ${_i(driver['blocked']) ?? 0} 次）',
      alert: _b(j['alert']),
    );
  }
}

// 一条对话。`mine` 是「我说的」，界面上靠右；狗说的靠左。
class ChatItem {
  ChatItem({
    required this.text,
    required this.mine,
    DateTime? at,
    this.seq,
    this.repeat = 1,
  }) : at = at ?? DateTime.now();
  final String text;
  final bool mine;
  final DateTime at;
  final int? seq;

  // 模型会复读：同一句话连说三遍，界面上就连出三条一样的泡泡，看着像卡住。
  // 合并成一条并标 ×N —— 复读本身是有信息量的（说明它没在推进），不该删掉。
  final int repeat;
}

enum LinkPhase { idle, connecting, open, retrying, failed }

class GatewayEvent {
  GatewayEvent(this.kind, this.text, {DateTime? at})
    : at = at ?? DateTime.now();
  final String kind;
  final String text;
  final DateTime at;
  bool get isError => kind == 'error';
  bool get isMode => kind == 'mode';
}
