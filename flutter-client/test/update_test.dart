import 'dart:convert';

import 'package:flutter_test/flutter_test.dart';
import 'package:go2_remote/core/update_service.dart';
import 'package:http/http.dart' as http;
import 'package:http/testing.dart';
import 'package:package_info_plus/package_info_plus.dart';

// 应用内更新最容易出错的两处：版本号比较（`v1.4.12-test` 这类后缀）和
// 「404 到底算什么」。前者错了会漏更，后者错了会把「没发布通道」说成
// 「已是最新」，让人以为功能没用。
void main() {
  setUp(() {
    PackageInfo.setMockInitialValues(
      appName: 'go2',
      packageName: 'top.yiyu14.go2_remote',
      version: '1.0.0',
      buildNumber: '1',
      buildSignature: '',
    );
  });

  group('版本比较', () {
    test('主版本号更大的算新', () {
      expect(UpdateService.isNewer('v1.1.0', '1.0.0'), isTrue);
      expect(UpdateService.isNewer('v2.0.0', '1.9.9'), isTrue);
    });

    test('相同版本不算新', () {
      expect(UpdateService.isNewer('v1.0.0', '1.0.0'), isFalse);
      expect(UpdateService.isNewer('1.0.0', '1.0.0'), isFalse);
    });

    test('更旧的版本不算新', () {
      expect(UpdateService.isNewer('v0.9.9', '1.0.0'), isFalse);
    });

    test('后缀不参与数值比较', () {
      // `v1.4.12-test` 的第三段必须读成 12，不是 0。读错的话 1.4.12 会被
      // 判成 1.4.0，低于已装的 1.4.5，更新被漏掉。
      expect(UpdateService.isNewer('v1.4.12-test', '1.4.5'), isTrue);
      expect(UpdateService.isNewer('v1.4.12+build7', '1.4.5'), isTrue);
    });

    test('段数不够按 0 补', () {
      expect(UpdateService.isNewer('v2', '1.9.9'), isTrue);
      expect(UpdateService.isNewer('v1.0', '1.0.0'), isFalse);
    });
  });

  group('Release 说明清理', () {
    test('去掉强调与链接语法', () {
      expect(stripMarkdown('**Full Changelog**: https://x'), 'Full Changelog: https://x');
      expect(stripMarkdown('[#12](https://github.com/a/b/pull/12) 修了遥控'),
          '#12 修了遥控');
    });

    test('标题去井号、列表换圆点', () {
      expect(stripMarkdown('## 修复\n- 第一条\n- 第二条'), '修复\n• 第一条\n• 第二条');
    });

    test('折叠多余空行', () {
      expect(stripMarkdown('a\n\n\n\nb'), 'a\n\nb');
    });
  });

  group('检查更新的四种结果', () {
    UpdateService withResponse(int code, String body) => UpdateService(
          client: MockClient((_) async => http.Response(body, code,
              headers: {'content-type': 'application/json; charset=utf-8'})),
        );

    test('404 是「没发布过」，不是「已是最新」', () async {
      final r = await withResponse(404, '{"message":"Not Found"}').check();
      expect(r.kind, UpdateCheckKind.noReleases);
      expect(r.detail, contains('还没发布过'));
    });

    test('403 是限流，单独说', () async {
      final r = await withResponse(403, '{}').check();
      expect(r.kind, UpdateCheckKind.failed);
      expect(r.detail, contains('限流'));
    });

    test('新版本 → update，并带出 APK 直链', () async {
      final body = jsonEncode({
        'tag_name': 'v1.2.0',
        'body': '修复了若干问题',
        'html_url': 'https://github.com/cixiy1/go2/releases/tag/v1.2.0',
        'assets': [
          {'name': 'go2-remote-v1.2.0.apk', 'browser_download_url': 'https://x/a.apk'},
        ],
      });
      final r = await withResponse(200, body).check();
      expect(r.kind, UpdateCheckKind.update);
      expect(r.release!.tagName, 'v1.2.0');
      expect(r.release!.apkUrl, 'https://x/a.apk');
    });

    test('同版本 → upToDate', () async {
      final body = jsonEncode({'tag_name': 'v1.0.0', 'assets': []});
      final r = await withResponse(200, body).check();
      expect(r.kind, UpdateCheckKind.upToDate);
    });

    test('tag 为空 → failed，不当成没更新', () async {
      final r = await withResponse(200, '{"tag_name":""}').check();
      expect(r.kind, UpdateCheckKind.failed);
    });
  });
}
