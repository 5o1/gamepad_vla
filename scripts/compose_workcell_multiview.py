#!/usr/bin/env python3
"""Combine three original simulation renders into a labelled review sheet."""
import argparse
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("render_dir", type=Path)
    args = parser.parse_args()
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    heading = ImageFont.truetype(font_path, 32)
    label = ImageFont.truetype(font_path, 25)
    note = ImageFont.truetype(font_path, 23)
    views = [("camera_3", "01  Calibrated camera"),
             ("side", "02  Side view"),
             ("opposite", "03  Opposite camera / front of table")]
    sheet = Image.new("RGB", (3840, 874), "#101923")
    draw = ImageDraw.Draw(sheet)
    draw.text((24, 15), "AR5-5 0.8R x 2  |  Workcell static preview", font=heading, fill="#ffffff")
    for i, (name, title) in enumerate(views):
        picture = Image.open(args.render_dir / f"{name}.png").convert("RGB")
        assert picture.size == (1280, 720), picture.size
        draw.text((i * 1280 + 24, 65), title, font=label, fill="#b9dcf4")
        sheet.paste(picture, (i * 1280, 108))
    draw.text((24, 841), "Archived camera calibration retained. Mounts and joint poses inherited; physical pose and collision clearance not verified.",
              font=note, fill="#d6dce1")
    sheet.save(args.render_dir / "multiview.png")


if __name__ == "__main__":
    main()
