import 'package:flutter/material.dart';

import '../app_state.dart';
import '../core/models.dart';
import '../theme.dart';
import 'widgets/panel.dart';
import 'widgets/update_panel.dart';

// 设置页：连接参数的「只看」、手感、画面开关、收工与断开。
//
// 连接参数不在这里再放一套输入框 —— 两个地方都能改，迟早会出现「改了这个
// 没生效，因为那个页面还留着一份旧值」。改地址去连接页。
class SettingsPage extends StatelessWidget {
  const SettingsPage({super.key});

  @override
  Widget build(BuildContext context) {
    final app = AppScope.of(context);
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Center(
        child: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 780),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              _connection(app),
              const SizedBox(height: 12),
              _feel(app),
              const SizedBox(height: 12),
              _camera(app),
              const SizedBox(height: 12),
              _danger(app),
              const SizedBox(height: 12),
              const UpdatePanel(),
              const SizedBox(height: 12),
              _about(),
            ],
          ),
        ),
      ),
    );
  }

  Widget _connection(AppState app) {
    final s = app.settings;
    return Panel(
      title: '连接',
      trailing: ActionButton(
        label: '去连接页修改',
        icon: Icons.cable_rounded,
        onPressed: () => app.go(0),
      ),
      child: Column(
        children: [
          KeyValue('地址', '${s.host}:${s.port}'),
          KeyValue('口令', s.token.isEmpty ? '（空）' : '已设置'),
          KeyValue('本机名称', s.name),
          ValueListenableBuilder<LinkPhase>(
            valueListenable: app.client.phase,
            builder: (context, phase, _) => KeyValue('状态', switch (phase) {
              LinkPhase.open => '已连接',
              LinkPhase.connecting => '连接中',
              LinkPhase.retrying => '重连中',
              LinkPhase.failed => '连不上',
              LinkPhase.idle => '未连接',
            }, color: phase == LinkPhase.open ? C.accent : C.dim),
          ),
        ],
      ),
    );
  }

  Widget _feel(AppState app) {
    return ListenableBuilder(
      listenable: app.settings,
      builder: (context, _) {
        final s = app.settings;
        return Panel(
          title: '手感',
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              const Kicker('速度上限（占协议上限的比例）'),
              const SizedBox(height: 6),
              Row(
                children: [
                  for (final v in const [0.25, 0.5, 0.75, 1.0])
                    Expanded(
                      child: Padding(
                        padding: const EdgeInsets.only(right: 6),
                        child: InkWell(
                          borderRadius: BorderRadius.circular(R.ctl),
                          onTap: () {
                            s.speedScale = v;
                            s.save();
                          },
                          child: Container(
                            height: 36,
                            alignment: Alignment.center,
                            decoration: BoxDecoration(
                              color: (s.speedScale - v).abs() < 1e-6
                                  ? C.accent.withValues(alpha: 0.16)
                                  : C.surfaceHi,
                              borderRadius: BorderRadius.circular(R.ctl),
                              border: Border.all(
                                color: (s.speedScale - v).abs() < 1e-6
                                    ? C.accent.withValues(alpha: 0.5)
                                    : C.line,
                              ),
                            ),
                            child: Text(
                              '${(v * 100).round()}%',
                              style: TextStyle(
                                fontSize: 12,
                                fontWeight: FontWeight.w600,
                                color: (s.speedScale - v).abs() < 1e-6
                                    ? C.accent
                                    : C.dim,
                              ),
                            ),
                          ),
                        ),
                      ),
                    ),
                ],
              ),
              const SizedBox(height: 10),
              Text(
                '实际速度 = 摇杆 × 协议上限 × 这个比例。协议上限由网关下发'
                '（vx 1.0 / vy 0.30 / vyaw 4.0），App 不自己写死数字。',
                style: const TextStyle(
                  fontSize: 11,
                  color: C.faint,
                  height: 1.5,
                ),
              ),
              const SizedBox(height: 4),
              const Text(
                '死区、曲线、横移反转在手柄页调。',
                style: TextStyle(fontSize: 11, color: C.faint),
              ),
            ],
          ),
        );
      },
    );
  }

  Widget _camera(AppState app) {
    return Panel(
      title: '画面',
      child: Column(
        children: [
          SwitchListTile(
            contentPadding: EdgeInsets.zero,
            dense: true,
            activeThumbColor: C.accent,
            title: const Text('接收 VLA 机载画面', style: TextStyle(fontSize: 12.5)),
            subtitle: const Text(
              '5 Hz JPEG。弱网关掉能省一半带宽',
              style: TextStyle(fontSize: 11, color: C.faint),
            ),
            value: app.settings.cameraOn,
            onChanged: (v) {
              app.settings.cameraOn = v;
              app.settings.save();
              app.client.camera(v);
            },
          ),
          const Divider(height: 18, color: C.line),
          SwitchListTile(
            contentPadding: EdgeInsets.zero,
            dense: true,
            activeThumbColor: C.accent,
            title: const Text('毛玻璃', style: TextStyle(fontSize: 12.5)),
            subtitle: const Text(
              '侧栏、顶栏、手机底栏用背景模糊（对齐 FlClash）。卡的话关掉',
              style: TextStyle(fontSize: 11, color: C.faint),
            ),
            value: app.settings.glass,
            onChanged: (v) {
              app.settings.glass = v;
              app.settings.save();
            },
          ),
        ],
      ),
    );
  }

  Widget _danger(AppState app) {
    return Panel(
      title: '收工',
      border: C.danger.withValues(alpha: 0.3),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          const Text(
            '收工会停步 → 蹲下 → 阻尼，并把机器人控制权交还。VLA 在跑时先停 VLA。'
            '真机上这一步走完，官方遥控器才能重新接管。',
            style: TextStyle(fontSize: 11.5, color: C.dim, height: 1.55),
          ),
          const SizedBox(height: 10),
          Row(
            children: [
              Expanded(
                child: ActionButton(
                  label: '收工',
                  icon: Icons.power_settings_new,
                  color: C.warn,
                  expand: true,
                  onPressed: () => app.setMode('off'),
                ),
              ),
              const SizedBox(width: 8),
              Expanded(
                child: ActionButton(
                  label: '断开连接',
                  icon: Icons.link_off,
                  color: C.danger,
                  expand: true,
                  onPressed: () => app.disconnect(),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _about() {
    return const Panel(
      title: '关于',
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          KeyValue('客户端', 'Flutter · 手机 / 桌面 / 网页'),
          KeyValue('网关', 'quadruped.comm.gateway（WebSocket）'),
          KeyValue('协议', 'JSON 文本帧 + JPEG 二进制帧'),
          SizedBox(height: 8),
          Text(
            '这个 App 不直接碰 DDS。机器狗的控制权由网关统一分配：手动遥控走'
            '「机器人服务端」，VLA 自主走 VLA 子进程，两者互斥。',
            style: TextStyle(fontSize: 11.5, color: C.faint, height: 1.55),
          ),
        ],
      ),
    );
  }
}
