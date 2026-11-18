// Maven 镜像（Kotlin 版 init script）。
//
// 这台机器连不上 repo.maven.apache.org / dl.google.com，Gradle 默认仓库全部超时。
//
// 为什么是 .kts 不是 .gradle：这台机器的 JDK 是 27（class file major 71），
// Gradle 9.3.1 内置的 Groovy 读不了这个版本，报
// `Unsupported class file major version 71`。Kotlin 编译这关不受影响。
//
// 覆盖三处，缺一个都会漏：
//   1. settings 的 pluginManagement —— settings 里 `plugins { id(...) }` 的解析
//   2. 各项目 buildscript 的 classpath —— 插件自己声明的 AGP/Kotlin 版本
//   3. 各项目的普通依赖
//
// 第 2 条最容易漏：`gamepads_android` 在自己的 build.gradle 里写死了
// `buildscript { repositories { google(); mavenCentral() } }`。只改我们自己工程的
// repositories 管不到它，结果卡在 `com.android.tools.build:gradle:7.3.0` 超时，
// 报错还伪装成「kotlin-android plugin requires one of the Android Gradle plugins」。
//
// 用法：
//     cd android && ./gradlew assembleRelease -I mirror.init.gradle.kts

val mirrors = listOf(
    "https://maven.aliyun.com/repository/public",
    "https://maven.aliyun.com/repository/google",
    "https://maven.aliyun.com/repository/gradle-plugin",
    "https://maven.aliyun.com/repository/central",
)

fun RepositoryHandler.addMirrors() {
    mirrors.forEach { u -> maven { url = uri(u) } }
}

gradle.beforeSettings {
    pluginManagement.repositories.addMirrors()
}

// 只碰 buildscript：`flutter_tools` 那个 included build 用的是 PREFER_SETTINGS，
// 从 beforeProject 往里塞 project 仓库会直接报
// "Build was configured to prefer settings repositories"。
// 工程自身的依赖仓库改在 build.gradle.kts 里，那边不受这个模式限制。
gradle.beforeProject {
    buildscript.repositories.addMirrors()
}
