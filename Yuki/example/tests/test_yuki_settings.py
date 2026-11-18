"""Yuki 示例配置加载契约。"""

from yuki.settings import load_settings


def test_load_settings_has_default_dirs():
    settings = load_settings()
    assert settings.data_dir is not None
    assert settings.packages_dir is not None
