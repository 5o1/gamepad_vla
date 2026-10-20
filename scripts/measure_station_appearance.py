#!/usr/bin/env python3
"""Photo-based PBR initial values, with explicit neutral-arm white balance."""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("photos", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    images = {i: np.asarray(Image.open(args.photos / f"{i}.jpg").convert("RGB")) / 255 for i in (0, 4)}
    # Coordinates are in the actual 2000x1500 JPEG, not the image viewer's
    # automatically resized 1823x1367 presentation.
    regions = {"white_arm": (0, [653, 400, 697, 433]),
               "table": (4, [263, 988, 494, 1130]),
               "column": (0, [943, 609, 962, 686]),
               "silver_hand": (0, [378, 729, 422, 752]),
               "dark_hand": (4, [428, 724, 450, 779]),
               "floor": (0, [40, 1130, 120, 1210]),
               "white_frame": (4, [1200, 1400, 1300, 1430]),
               "camera": (0, [973, 237, 1036, 251])}
    samples = {}
    for name, (index, roi) in regions.items():
        x0,y0,x1,y1 = roi
        samples[name] = np.median(images[index][y0:y1,x0:x1], axis=(0,1))
    neutral = samples["white_arm"]
    gain = neutral.mean() / neutral
    linear = lambda rgb: np.where(rgb <= .04045, rgb / 12.92, ((rgb + .055)/1.055)**2.4)
    colors = {k: linear(np.clip(v * gain, 0, 1)).tolist() + [1.] for k,v in samples.items()}
    evidence = dict(white_balance_reference="white_arm", gains_srgb=gain.tolist(),
                    samples={name: dict(photo=i, roi_xyxy=roi, median_srgb=samples[name].tolist(),
                                        corrected_linear_rgba=colors[name]) for name,(i,roi) in regions.items()},
                    limits="Phone exposure/WB unknown; one global diagonal WB in sRGB is approximate. Colors are appearance initials, not reflectance measurements.")
    rules = []
    def add(name, patterns, color, metal, rough, source):
        rules.append(dict(name=name, match=dict(link_glob=patterns, mesh_glob=[]),
                          base_color=color, metallic=metal, roughness=rough,
                          status="estimated", source=source))
    def src(region):
        i,roi=regions[region]
        return f"photo {i}, pixel ROI {roi}; neutral WB photo 0 white_arm; metallic/roughness assumed initial values"
    add("table_green", ["table"], colors["table"], 0., .75, src("table"))
    add("table_frame_white", ["station_table_frame*"], colors["white_frame"], 0., .7, src("white_frame"))
    add("column_aluminium", ["station_column*"], colors["column"], .8, .4, src("column"))
    add("ar5_base_gray", ["AR5_*_base"], linear(np.array([.33]*3)).tolist()+[1.], 0., .5,
        "RobotHMI_5.153.11.0314/model/AR5*/param.json RGB(.33,.33,.33), shininess30; assumed sRGB to linear; roughness not inferred from shininess")
    warm_white=list(colors["white_arm"])
    warm_white[1]*=.98;warm_white[2]*=.95
    add("ar5_warm_white", ["AR5_*"], warm_white, 0., .48,
        src("white_arm")+"; HMI arm white; residual warm tint linear RGB x[1,.98,.95] assumed to initialize warm-white shell, not measured reflectance. Each mixed shell/black-ring STL uses white shell; rings cannot be separately colored.")
    add("g20_dark", ["*distal", "*hand_base_link"], colors["dark_hand"], 0., .8,
        src("dark_hand")+"; palm/distal mixed STL uses dark face, sacrifices silver trim")
    add("g20_silver", ["*proximal", "*metacarpals*", "*middle"], colors["silver_hand"], .9, .35,
        src("silver_hand")+"; photo0/3/4 silver shell, no PBR vendor asset; URDF red/yellow ignored")
    add("realsense_shell", ["camera_*_body"], colors["camera"], .65, .4,
        src("camera")+"; D435 mesh Apache2.0; integrated black sensor strip not independently textured")
    add("floor", ["station_floor"], colors["floor"], 0., .9, src("floor"))
    (args.output / "station_v1.json").write_text(json.dumps(dict(name="station-v1", color_space="linear_rgb", materials=rules), indent=2))
    (args.output / "sampling.json").write_text(json.dumps(evidence, indent=2))
    for index in (0,4):
        picture = Image.open(args.photos / f"{index}.jpg").convert("RGB")
        draw=ImageDraw.Draw(picture)
        for name,(i,roi) in regions.items():
            if i==index:
                draw.rectangle(roi, outline="red", width=3)
                draw.text((roi[0],roi[1]-15),name,fill="red")
        picture.save(args.output / f"sampling_photo{index}.jpg")


if __name__ == "__main__":
    main()
