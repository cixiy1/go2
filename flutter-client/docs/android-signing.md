# Android 签名：应用内更新必须固定签名

## 为什么

Android 只允许「同一个包名 + 同一个签名」的 APK 覆盖安装。签名变了，系统会报
「应用未安装」，用户只能先卸载 —— 设置、连接地址全丢。

`flutter build apk` 默认用 `~/.android/debug.keystore`，而**每台机器、每个
CI runner 的 debug 密钥都不一样**。所以：

- 你在本机打的包装上了，CI 发的更新包装不上去；
- 换一台机器打包，同样装不上去。

这也是 `flutter-client/android/app/build.gradle.kts` 里那段 `key.properties`
判断存在的原因：有固定密钥就用，没有才退回 debug。

## 一次性生成（只做一次，密钥丢了就再也签不出同一个包）

```sh
cd flutter-client/android
keytool -genkey -v \
  -keystore release.jks \
  -keyalg RSA -keysize 2048 -validity 10000 \
  -alias go2-remote
```

口令记牢。`release.jks` 和下面的 `key.properties` 都已经在 `.gitignore` 里，
**不要提交**。

`flutter-client/android/key.properties`：

```properties
storePassword=你的口令
keyPassword=你的口令
keyAlias=go2-remote
storeFile=release.jks
```

`storeFile` 是相对 `android/` 的路径。

## 塞进 GitHub Secrets（给 CI 用）

```sh
# 两条，都是单行 base64
base64 -i android/release.jks | tr -d '\n' > /tmp/ks.b64
base64 -i android/key.properties | tr -d '\n' > /tmp/kp.b64
```

在仓库 Settings → Secrets and variables → Actions 里建：

| Secret | 内容 |
|---|---|
| `ANDROID_KEYSTORE_BASE64` | `/tmp/ks.b64` 的内容 |
| `ANDROID_KEY_PROPERTIES_BASE64` | `/tmp/kp.b64` 的内容 |

`pbcopy < /tmp/ks.b64` 直接进剪贴板。

没配这两个 Secret 时，`.github/workflows/release.yml` 会打一条 warning 并回退
debug 签名 —— 包能出，但用户装不上去。

## 验证两个包是同一个签名

```sh
APKSIGNER=$(ls "$ANDROID_HOME"/build-tools/*/apksigner | tail -1)
"$APKSIGNER" verify --print-certs build/app/outputs/flutter-apk/app-release.apk \
  | grep -i "SHA-256"
```

两次输出的 SHA-256 必须一样。CI 里也打了这一行，方便对照。

## 发版流程

```sh
# 1) 改版本号（pubspec.yaml 的 version: 1.0.1+2 —— 数字段必须涨，客户端按它比）
# 2) 提交、打 tag、推
git tag v1.0.1 && git push origin v1.0.1
```

tag 推上去 → Actions 构建 → 出 Release（资产名 `go2-remote-v1.0.1.apk`）→
App 里「设置 → 应用更新 → 检查更新」就能看到。

**注意**：`version:` 里的版本号和 tag 必须同步涨。客户端拿 `PackageInfo.version`
和 tag 比，只改 tag 不改 pubspec，用户会反复被提示「有新版本」但装完还是旧的。
