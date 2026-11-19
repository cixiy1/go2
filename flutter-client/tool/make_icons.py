"""把所有平台的启动图标一次性从一张方图生成。

    python3 tool/make_icons.py ~/图片/新图标.jpg

Android 两套：`ic_launcher.png` 给旧系统（圆角方形、四角透明），
`ic_launcher_foreground.png` + `mipmap-anydpi-v26/ic_launcher.xml` 给 8.0 以上
（满幅铺开，系统按自己的遮罩裁圆，所以内容要留在中间）。iOS 14 个尺寸必须是
不透明方块，带 alpha 会被 App Store 拒收。Web 的 maskable 版本把内容缩到 80%，
四角填底色，否则被裁成圆形时边缘会露白。

源图要求正方形。不是正方形就先居中裁，别让 resize 把脸拉扁。
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
LEGACY = {"mdpi": 48, "hdpi": 72, "xhdpi": 96, "xxhdpi": 144, "xxxhdpi": 192}
ADAPTIVE = {"mdpi": 108, "hdpi": 162, "xhdpi": 216, "xxhdpi": 324, "xxxhdpi": 432}


def square(path: str) -> Image.Image:
    im = Image.open(path).convert("RGB")
    w, h = im.size
    if w != h:
        side = min(w, h)
        im = im.crop(((w - side) // 2, (h - side) // 2,
                      (w + side) // 2, (h + side) // 2))
    return im.resize((1024, 1024), Image.LANCZOS)


def rounded(im: Image.Image, ratio: float = 0.22) -> Image.Image:
    w, h = im.size
    mask = Image.new("L", (w, h), 0)
    ImageDraw.Draw(mask).rounded_rectangle([0, 0, w - 1, h - 1],
                                           radius=int(w * ratio), fill=255)
    out = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    out.paste(im, (0, 0), mask)
    return out


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__)
        return 2
    base = square(argv[1])
    corners = [base.getpixel(p) for p in [(2, 2), (1021, 2), (2, 1021), (1021, 1021)]]
    bg = tuple(sum(c[i] for c in corners) // 4 for i in range(3))

    res = ROOT / "android/app/src/main/res"
    for d, px in LEGACY.items():
        icon = rounded(base.resize((px, px), Image.LANCZOS))
        icon.save(res / f"mipmap-{d}/ic_launcher.png", optimize=True)
        icon.save(res / f"mipmap-{d}/ic_launcher_round.png", optimize=True)
    for d, px in ADAPTIVE.items():
        base.resize((px, px), Image.LANCZOS).save(
            res / f"mipmap-{d}/ic_launcher_foreground.png", optimize=True)

    vals = res / "values"
    (vals / "colors.xml").write_text(
        '<?xml version="1.0" encoding="utf-8"?>\n<resources>\n'
        '    <color name="ic_launcher_background">#%02X%02X%02X</color>\n'
        '</resources>\n' % bg)

    ios = ROOT / "ios/Runner/Assets.xcassets/AppIcon.appiconset"
    for p in ios.glob("Icon-App-*.png"):
        m = re.match(r"Icon-App-(\d+)x(\d+)@(\d)x\.png", p.name)
        if m:
            px = int(m.group(1)) * int(m.group(3))
            base.resize((px, px), Image.LANCZOS).convert("RGB").save(p, optimize=True)

    web = ROOT / "web"
    base.resize((64, 64), Image.LANCZOS).save(web / "favicon.png", optimize=True)
    for px in (192, 512):
        rounded(base.resize((px, px), Image.LANCZOS)).save(
            web / f"icons/Icon-{px}.png", optimize=True)
        canvas = Image.new("RGB", (px, px), bg)
        inner = int(px * 0.8)
        art = base.resize((inner, inner), Image.LANCZOS)
        mask = Image.new("L", (inner, inner), 0)
        ImageDraw.Draw(mask).ellipse([0, 0, inner - 1, inner - 1], fill=255)
        canvas.paste(art, ((px - inner) // 2, (px - inner) // 2), mask)
        canvas.save(web / f"icons/Icon-maskable-{px}.png", optimize=True)

    print(f"图标已生成，底色 #%02X%02X%02X" % bg)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
