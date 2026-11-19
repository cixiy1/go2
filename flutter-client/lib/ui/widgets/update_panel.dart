import 'dart:io';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:package_info_plus/package_info_plus.dart';
import 'package:url_launcher/url_launcher.dart';

import '../../core/apk_downloader.dart';
import '../../core/apk_installer.dart';
import '../../core/update_service.dart';
import '../../theme.dart';
import 'panel.dart';

// 应用更新卡片。
//
// 交互照 myclass 那套来（检查 → 说明弹窗 → 下载进度 → 唤起安装器），
// 因为那条链路已经在真机上验证过：下载写缓存目录、open_filex 走 FileProvider
// 授权、失败清残包。这里只把界面换成这个项目的 Panel / ActionButton。
class UpdatePanel extends StatefulWidget {
  const UpdatePanel({super.key});

  @override
  State<UpdatePanel> createState() => _UpdatePanelState();
}

class _UpdatePanelState extends State<UpdatePanel> {
  final UpdateService _service = UpdateService();

  bool _checking = false;
  String _version = '—';
  String? _status;

  @override
  void initState() {
    super.initState();
    _loadVersion();
  }

  Future<void> _loadVersion() async {
    try {
      final info = await PackageInfo.fromPlatform();
      if (mounted) setState(() => _version = 'v${info.version}+${info.buildNumber}');
    } catch (_) {
      // Web 上拿不到就保持占位符，不影响检查更新。
    }
  }

  @override
  void dispose() {
    _service.dispose();
    super.dispose();
  }

  Future<void> _check() async {
    setState(() {
      _checking = true;
      _status = null;
    });
    final result = await _service.check();
    if (!mounted) return;
    setState(() {
      _checking = false;
      _status = switch (result.kind) {
        UpdateCheckKind.update => '发现新版本 ${result.release!.tagName}',
        UpdateCheckKind.upToDate => '已是最新版本',
        UpdateCheckKind.noReleases => result.detail ?? '仓库还没发布过 Release',
        UpdateCheckKind.failed => result.detail ?? '检查失败',
      };
    });
    if (result.kind == UpdateCheckKind.update) {
      await _showRelease(result.release!);
    }
  }

  Future<void> _showRelease(AppRelease r) async {
    final go = await showDialog<bool>(
      context: context,
      builder: (context) => AlertDialog(
        backgroundColor: C.surface,
        title: Text('发现新版本 ${r.tagName}',
            style: const TextStyle(fontSize: 17, fontWeight: FontWeight.w700)),
        content: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 460, maxHeight: 360),
          child: SingleChildScrollView(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text('当前 $_version',
                    style: const TextStyle(fontSize: 11.5, color: C.faint)),
                const SizedBox(height: 10),
                Text(
                  r.displayNotes.isEmpty ? '这个版本没有写更新说明。' : r.displayNotes,
                  style: const TextStyle(
                      fontSize: 12.5, color: C.text, height: 1.65),
                ),
              ],
            ),
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(false),
            child: const Text('稍后', style: TextStyle(color: C.dim)),
          ),
          FilledButton(
            onPressed: () => Navigator.of(context).pop(true),
            child: const Text('下载并安装'),
          ),
        ],
      ),
    );
    if (go == true && mounted) await _download(r);
  }

  Future<void> _download(AppRelease r) async {
    // Web / iOS 没有 APK 安装器，直接在浏览器里下 —— 应用内下载在 stub 里
    // 抛异常，与其让用户看到报错，不如把 Release 页打开。
    if (kIsWeb || !Platform.isAndroid) {
      final ok = await _openReleasePage(r);
      if (mounted) {
        setState(() => _status = ok ? '已打开 Release 页面' : '打不开浏览器，地址：${r.releaseUrl}');
      }
      return;
    }

    final progress = ValueNotifier<DownloadProgress?>(
        const DownloadProgress(received: 0, total: 0));
    final token = DownloadCancelToken();
    final name = r.apkUrl.split('/').last.isEmpty
        ? 'go2-remote-${r.tagName}.apk'
        : r.apkUrl.split('/').last;

    if (!mounted) return;
    final dialog = showDialog<void>(
      context: context,
      barrierDismissible: false,
      builder: (context) => _ProgressDialog(
        tag: r.tagName,
        progress: progress,
        onCancel: () {
          token.cancel();
          Navigator.of(context).pop();
        },
      ),
    );

    String? path;
    String? error;
    try {
      path = await downloadToFile(
        r.apkUrl,
        fileName: name,
        onProgress: (p) => progress.value = p,
        cancelToken: token,
      );
    } on DownloadCancelledException {
      error = null; // 用户主动取消，不算失败
    } on DownloadException catch (e) {
      error = e.message;
    } catch (e) {
      error = '下载失败：$e';
    } finally {
      progress.dispose();
      if (mounted) Navigator.of(context, rootNavigator: true).pop();
      await dialog;
    }

    if (!mounted) return;
    if (error != null) {
      setState(() => _status = error);
      return;
    }
    if (path == null) {
      setState(() => _status = '已取消下载');
      return;
    }

    final outcome = await launchApkInstaller(path);
    if (!mounted) return;
    setState(() {
      _status = switch (outcome) {
        InstallOutcome.launched => '已打开安装界面',
        InstallOutcome.permissionDenied => '系统没允许安装应用：去设置里给本应用开「安装未知应用」',
        InstallOutcome.unsupported => '当前平台不支持安装 APK',
        InstallOutcome.failed => '安装器打不开，安装包在：$path',
      };
    });
  }

  Future<bool> _openReleasePage(AppRelease r) async {
    final url = r.releaseUrl.isEmpty ? r.apkUrl : r.releaseUrl;
    if (url.isEmpty) return false;
    try {
      return await launchUrl(Uri.parse(url),
          mode: LaunchMode.externalApplication);
    } catch (_) {
      return false;
    }
  }

  @override
  Widget build(BuildContext context) {
    return Panel(
      title: '应用更新',
      trailing: StatusChip(
        text: _version,
        color: C.dim,
        dense: true,
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          Text(
            kIsWeb
                ? '网页版没有安装包：检查更新会跳到 Release 页面自己下。'
                : '更新包从 GitHub Releases 下载，装完覆盖原应用，设置不用重填。',
            style: const TextStyle(fontSize: 11.5, color: C.faint, height: 1.55),
          ),
          if (_status != null) ...[
            const SizedBox(height: 8),
            Text(
              _status!,
              style: TextStyle(
                fontSize: 12,
                color: _status!.contains('失败') || _status!.contains('没允许')
                    ? C.danger
                    : C.dim,
                height: 1.5,
              ),
            ),
          ],
          const SizedBox(height: 10),
          Row(
            children: [
              ActionButton(
                label: _checking ? '检查中…' : '检查更新',
                icon: Icons.system_update_alt,
                color: C.accentText,
                onPressed: _checking ? null : _check,
              ),
              const SizedBox(width: 8),
              Expanded(
                child: Text(
                  _checking ? '正在问 GitHub 要最新 Release…' : '',
                  style: const TextStyle(fontSize: 10.5, color: C.faint),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }
}

// 下载进度：一条进度条 + 已下载/总量 + 取消。
//
// 总长度未知时（服务端没给 Content-Length）退化成不确定进度条，不要假装
// 知道百分比 —— 那种「卡在 0%」的进度条比没有更让人焦虑。
class _ProgressDialog extends StatelessWidget {
  const _ProgressDialog({
    required this.tag,
    required this.progress,
    required this.onCancel,
  });

  final String tag;
  final ValueNotifier<DownloadProgress?> progress;
  final VoidCallback onCancel;

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      backgroundColor: C.surface,
      title: Text('下载 $tag',
          style: const TextStyle(fontSize: 16, fontWeight: FontWeight.w700)),
      content: SizedBox(
        width: 320,
        child: ValueListenableBuilder<DownloadProgress?>(
          valueListenable: progress,
          builder: (context, p, _) {
            final frac = p?.fraction;
            final done = p?.received ?? 0;
            final total = p?.total ?? 0;
            return Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                ClipRRect(
                  borderRadius: BorderRadius.circular(4),
                  child: LinearProgressIndicator(
                    value: frac,
                    minHeight: 6,
                    backgroundColor: C.surfaceHi,
                    valueColor: const AlwaysStoppedAnimation(C.accent),
                  ),
                ),
                const SizedBox(height: 10),
                Text(
                  frac == null
                      ? _kb(done)
                      : '${(frac * 100).toStringAsFixed(0)}% · ${_kb(done)} / ${_kb(total)}',
                  style: const TextStyle(fontSize: 11.5, color: C.dim),
                ),
              ],
            );
          },
        ),
      ),
      actions: [
        TextButton(
          onPressed: onCancel,
          child: const Text('取消', style: TextStyle(color: C.dim)),
        ),
      ],
    );
  }

  static String _kb(int bytes) =>
      bytes >= 1024 * 1024
          ? '${(bytes / 1024 / 1024).toStringAsFixed(1)} MB'
          : '${(bytes / 1024).toStringAsFixed(0)} KB';
}
