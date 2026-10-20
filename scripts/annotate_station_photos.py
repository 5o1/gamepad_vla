"""Draw measurement segments on the station worktable photos (coords in 2000x1500 display space)."""
import math, sys
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

RAW, OUT = Path(sys.argv[1]), Path(sys.argv[2])
OUT.mkdir(parents=True, exist_ok=True)
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
if not Path(FONT).exists():
    FONT = "/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc"
COLORS = {"A": "#ff2d2d", "B": "#ffd400", "C": "#00e5ff", "D": "#ff00d4", "E": "#ff8c00", "F": "#7cff00", "G": "#ffffff"}

def arrow(d, p, q, color, w):
    d.line([p, q], fill=color, width=w)
    for a, b in ((p, q), (q, p)):
        ang = math.atan2(b[1] - a[1], b[0] - a[0])
        for s in (-1, 1):
            t = ang + math.pi + s * 0.45
            d.line([b, (b[0] + 32 * math.cos(t) * w / 6, b[1] + 32 * math.sin(t) * w / 6)], fill=color, width=w)

def label(d, xy, text, color, font):
    x, y = xy
    box = d.textbbox((x, y), text, font=font)
    d.rectangle([box[0] - 8, box[1] - 6, box[2] + 8, box[3] + 6], fill="#000000c0")
    d.text((x, y), text, fill=color, font=font)

SPECS = {
    "0": [("arrow", "B1", (352, 990), (1852, 1022), (1300, 1030), "B1 台面宽（左右）"),
          ("arrow", "B2", (340, 985), (18, 1440), (60, 1180), "B2 台面深（前后）"),
          ("arrow", "C1", (352, 940), (928, 940), (420, 880), "C1 台面左侧边→立柱左面"),
          ("arrow", "C2", (930, 905), (1058, 905), (1070, 880), "C2 立柱宽"),
          ("arrow", "D1", (1205, 470), (1205, 995), (1225, 700), "D1 左臂基座轴线→台面（竖直）"),
          ("arrow", "D2", (995, 445), (1100, 445), (1050, 360), "D2 立柱中线→左臂基座安装面"),
          ("dot", "D", (1205, 470), None, (1380, 560), "左臂基座（黑环中心=关节1轴）"),
          ("dot", "G", (1000, 255), None, (1060, 200), "G 相机（可选）")],
    "1": [("arrow", "A", (1650, 1292), (1650, 1500), (1300, 1380), "A 台面上表面→地面"),
          ],
    "2": [("circle", "C3", (885, 950), 70, (960, 860), "C3 立柱前面↔台面后沿 间距"),
          ("circle", "C4", (835, 1085), 90, (930, 1060), "C4 立柱怎样固定？（桌架/地面）")],
    "3": [("arrow", "E1", (1250, 800), (1250, 650), (1280, 560), "E1 臂法兰端面→手掌根 轴向总厚"),
          ("dot", "E", (960, 800), None, (300, 830), "臂法兰端面（黑环顶面）"),
          ("dot", "E", (960, 655), None, (300, 620), "手的安装面"),
          ("text", "E2", None, None, (1000, 950), "E2 手绕法兰轴的安装角（见说明）")],
    "4": [("arrow", "F1", (1030, 945), (1030, 680), (1080, 760), "F1 杯底中心→台面后沿"),
          ("arrow", "F2", (1000, 945), (0, 945), (300, 975), "F2 杯底中心→台面左侧边"),
          ("circle", "B3", (1000, 1350), 60, (1080, 1260), "B3 台面板厚（含其下亚克力层？）")],
}

for name, items in SPECS.items():
    image = Image.open(RAW / f"{name}.png").convert("RGB")
    s = image.width / 2000
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(overlay)
    font = ImageFont.truetype(FONT, int(40 * s))
    w = max(3, int(6 * s))
    S = lambda p: (p[0] * s, p[1] * s)
    for kind, key, p, q, at, text in items:
        color = COLORS[key[0]]
        if kind == "arrow":
            arrow(d, S(p), S(q), color, w)
        elif kind == "dot":
            d.line([S(p), (at[0] * s, at[1] * s + 25 * s)], fill=color, width=max(2, w // 2))
            r = 14 * s
            d.ellipse([p[0] * s - r, p[1] * s - r, p[0] * s + r, p[1] * s + r], outline=color, width=w)
        elif kind == "circle":
            r = q * s
            d.ellipse([p[0] * s - r, p[1] * s - r, p[0] * s + r, p[1] * s + r], outline=color, width=w)
        label(d, S(at), text, color, font)
    out = Image.alpha_composite(image.convert("RGBA"), overlay).convert("RGB")
    out.thumbnail((2000, 2000))
    out.save(OUT / f"{name}_annotated.jpg", quality=88)
    raw = image.copy(); raw.thumbnail((2000, 2000)); raw.save(OUT / f"{name}.jpg", quality=88)
print("ok")
