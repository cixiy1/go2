import 'dart:async';
import 'dart:convert';

import 'package:http/http.dart' as http;
import 'package:package_info_plus/package_info_plus.dart';

/// GitHub 上的最新 Release 信息。
class AppRelease {
  const AppRelease({
    required this.tagName,
    required this.notes,
    required this.apkUrl,
    required this.releaseUrl,
  });

  /// 形如 `v1.1.0`。
  final String tagName;
  final String notes;

  /// Release 资产里的 APK 下载直链（GitHub Actions 构建产物）。
  final String apkUrl;

  /// Release 页面（APK 直链不可用时的兜底跳转）。
  final String releaseUrl;

  /// 直接展示用的更新说明：GitHub 自动生成的是 Markdown，去掉符号再显示。
  String get displayNotes => stripMarkdown(notes);
}

/// 把 Release 说明里的 Markdown 记号清掉，只留下可读正文。
///
/// GitHub 自动生成的说明形如 `**Full Changelog**: https://github.com/…`，
/// 直接塞进弹窗会带着一堆星号与井号；这里做最小化处理：去强调符、链接只留
/// 文字、标题去掉井号、无序列表换成圆点，并折叠多余空行。
String stripMarkdown(String source) {
  var text = source.replaceAll('\r\n', '\n');
  text = text.replaceAllMapped(
    RegExp(r'\[([^\]]*)\]\([^)]*\)'),
    (m) => m.group(1) ?? '',
  );
  text = text.replaceAll(RegExp(r'\*\*|__|`|~~'), '');
  // 注意用 [^\S\n]（非换行空白）而非 \s：后者会连同前面的空行一起吃进匹配，
  // 把段落间距压掉。
  // 标题：`#` 后面必须有空白。不加这个限制的话，行首的 `[#12](url)` 去掉链接
  // 语法后变成 `#12`，会被当成标题连 `#` 一起删掉，issue 号就丢了。
  text = text.replaceAllMapped(
      RegExp(r'^#{1,6}[^\S\n]+', multiLine: true), (_) => '');
  text = text.replaceAllMapped(
      RegExp(r'^[^\S\n]*[-*+][^\S\n]+', multiLine: true), (_) => '• ');
  text = text.replaceAll(RegExp(r'\n{3,}'), '\n\n');
  return text.trim();
}

/// 检查结果的四种去向。不能只有「有更新 / 没更新」两态：
///
/// - 404 是「仓库没有 Release」，不是「已是最新」。混在一起的话，发布通道
///   还没搭好时界面会一本正经地说「已是最新版本」，让人以为更新没用。
/// - 超时、断网、限流（GitHub 匿名 60 次/小时）要能和「没更新」分开，
///   否则用户看到「已是最新」就再也不会点第二次。
enum UpdateCheckKind { update, upToDate, noReleases, failed }

class UpdateCheck {
  const UpdateCheck(this.kind, {this.release, this.detail});

  final UpdateCheckKind kind;
  final AppRelease? release;

  /// 失败/无发布通道时的原因，直接显示给用户。
  final String? detail;
}

/// 应用内更新检查：比对 GitHub 最新 Release 与当前安装版本。
class UpdateService {
  UpdateService({http.Client? client}) : _client = client ?? http.Client();

  static const _repo = 'cixiy1/go2';
  final http.Client _client;

  /// 拉取最新 Release 并按四种结果返回。
  Future<UpdateCheck> check() async {
    try {
      final resp = await _client.get(
        Uri.parse('https://api.github.com/repos/$_repo/releases/latest'),
        headers: {'Accept': 'application/vnd.github+json'},
      ).timeout(const Duration(seconds: 8));

      if (resp.statusCode == 404) {
        return const UpdateCheck(UpdateCheckKind.noReleases,
            detail: '仓库还没发布过 Release —— 不是「已是最新」，是没有可下载的包');
      }
      if (resp.statusCode == 403) {
        return const UpdateCheck(UpdateCheckKind.failed,
            detail: 'GitHub 限流了（匿名请求每小时 60 次），过一会再试');
      }
      if (resp.statusCode != 200) {
        return UpdateCheck(UpdateCheckKind.failed,
            detail: '检查失败：HTTP ${resp.statusCode}');
      }

      final json = jsonDecode(utf8.decode(resp.bodyBytes)) as Map<String, dynamic>;
      final tag = json['tag_name'] as String? ?? '';
      if (tag.isEmpty) {
        return const UpdateCheck(UpdateCheckKind.failed,
            detail: 'Release 没有 tag，读不出新版本号');
      }

      final info = await PackageInfo.fromPlatform();
      if (!isNewer(tag, info.version)) {
        return const UpdateCheck(UpdateCheckKind.upToDate);
      }

      final assets = (json['assets'] as List<dynamic>? ?? [])
          .cast<Map<String, dynamic>>();
      final apk = assets.where((a) {
        final name = (a['name'] as String? ?? '').toLowerCase();
        return name.endsWith('.apk');
      }).map((a) => a['browser_download_url'] as String).toList();

      return UpdateCheck(
        UpdateCheckKind.update,
        release: AppRelease(
          tagName: tag,
          notes: (json['body'] as String? ?? '').trim(),
          apkUrl: apk.isNotEmpty ? apk.first : '',
          releaseUrl: json['html_url'] as String? ?? '',
        ),
      );
    } on TimeoutException {
      return const UpdateCheck(UpdateCheckKind.failed,
          detail: '连接 GitHub 超时，检查一下网络');
    } catch (e) {
      return UpdateCheck(UpdateCheckKind.failed, detail: '检查失败：$e');
    }
  }

  /// 语义化版本比较：remote 大于 current 返回 true。
  /// 兼容 `v` 前缀、`+build` 与 `-版本名` 后缀（如 `v1.4.12-test`），
  /// 解析失败的段按 0 处理。
  static bool isNewer(String remote, String current) {
    final r = _parse(remote);
    final c = _parse(current);
    for (var i = 0; i < 3; i++) {
      if (r[i] != c[i]) return r[i] > c[i];
    }
    return false;
  }

  static List<int> _parse(String version) {
    final core = version
        .trim()
        .replaceFirst(RegExp(r'^v', caseSensitive: false), '')
        // 版本名后缀（`1.4.12-test`）不参与数值比较，先剥掉：
        // 否则第三段 `12-test` parseInt 失败按 0，会被误判成 1.4.0。
        .split('-')
        .first
        .split('+')
        .first;
    final parts = core.split('.');
    // 段数不够要补 0，不能直接 parts[i]：`v2` 只有一段，越界会抛 RangeError，
    // 一次异常就让整个检查更新变成「失败」。
    return List.generate(
        3, (i) => i < parts.length ? int.tryParse(parts[i]) ?? 0 : 0);
  }

  void dispose() => _client.close();
}
