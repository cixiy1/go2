"""外置包来源抽象：本地目录、zip、marketplace 占位。"""

import tempfile
import zipfile
from abc import ABC, abstractmethod
from collections.abc import Callable
from pathlib import Path

from .external import MANIFEST_NAME, PackageError


class PackageSource(ABC):
    @abstractmethod
    async def fetch(self, ref: str) -> tuple[Path, Callable[[], None]]:
        """返回包根目录和清理函数。"""


class LocalDirSource(PackageSource):
    async def fetch(self, ref: str) -> tuple[Path, Callable[[], None]]:
        path = Path(ref).expanduser().resolve()
        if not path.is_dir():
            raise PackageError(f"不是目录：{ref}")
        return path, lambda: None


class ZipSource(PackageSource):
    async def fetch(self, ref: str) -> tuple[Path, Callable[[], None]]:
        tmp = tempfile.TemporaryDirectory(prefix="yuki-pkg-")
        target = Path(tmp.name).resolve()
        with zipfile.ZipFile(ref) as archive:
            for member in archive.infolist():
                dest = (target / member.filename).resolve()
                if not dest.is_relative_to(target):
                    tmp.cleanup()
                    raise PackageError(f"zip 包含非法路径：{member.filename}")
                if member.is_dir():
                    dest.mkdir(parents=True, exist_ok=True)
                else:
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    dest.write_bytes(archive.read(member))
        return target, tmp.cleanup


class MarketplaceSource(PackageSource):
    async def fetch(self, ref: str) -> tuple[Path, Callable[[], None]]:
        raise NotImplementedError("marketplace 尚未接入")


def find_package_root(directory: Path) -> Path:
    if (directory / MANIFEST_NAME).is_file():
        return directory
    for child in directory.iterdir():
        if child.is_dir() and (child / MANIFEST_NAME).is_file():
            return child
    raise PackageError("包内找不到 manifest.json")
