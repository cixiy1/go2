"""外置包管理命令。"""

# noinspection PyUnresolvedReferences
from yuki_kernel.core.app import App

# noinspection PyUnresolvedReferences
from yuki_kernel.skills.sources import LocalDirSource, ZipSource


def _print_scan(scan) -> None:
    for package_id in scan.packages:
        print(f"发现外置工具包：{package_id}")
    for name, reason in scan.skipped:
        print(f"跳过外置工具包 {name}：{reason}")
    if scan.available:
        print(f"可用外置工具包：{'、'.join(scan.available)}")
    else:
        print("可用外置工具包：无")


async def handle_pkg(app: App, arg: str) -> None:
    parts = arg.split(maxsplit=1)
    sub = parts[0] if parts else ""
    ref = parts[1].strip() if len(parts) > 1 else ""

    if sub == "install":
        if not ref:
            print("用法：/pkg install <目录|zip>")
            return
        source = ZipSource() if ref.lower().endswith(".zip") else LocalDirSource()
        try:
            info = await app.package_manager.install(source, ref)
            print(f"已安装：{info.id} {info.version}")
            _print_scan(app.registry.scan_packages(
                app.settings.packages_dir,
                available=app.settings.packages or None,
            ))
        except Exception as err:  # noqa: BLE001 - CLI 安装命令兜底，需捕获一切异常
            print(f"安装失败：{err}")
    elif sub == "remove":
        if not ref:
            print("用法：/pkg remove <id>")
            return
        try:
            app.package_manager.remove(ref)
            print(f"已卸载：{ref}")
            _print_scan(app.registry.scan_packages(
                app.settings.packages_dir,
                available=app.settings.packages or None,
            ))
        except Exception as err:  # noqa: BLE001 - CLI 卸载命令兜底，需捕获一切异常
            print(f"卸载失败：{err}")
    elif sub == "list":
        infos = app.package_manager.list_installed()
        if not infos:
            print("暂无已安装包")
            return
        for info in infos:
            print(f"{info.id} {info.version} {info.source}")
    else:
        print("用法：/pkg install <目录|zip> | /pkg remove <id> | /pkg list")
