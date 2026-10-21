#!/usr/bin/env python3
"""Compile original snapshot functions and compare the Python port numerically.

The private source is read, never imported or executed as a ROS/robot node.
Only pure mapping/clamping functions are extracted into an offline executable.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import numpy as np
from gamepad_vla.workstation.arm_mapping import ArmMapper


def extract(text, name):
    start = re.search(r"(?:double|std::vector<double>)\s+" + name + r"\(", text).start()
    brace = text.index("{", start)
    depth, end = 1, brace + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end].replace(") const", ")")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    text = a.source.read_text()
    declarations = "\n".join(re.findall(r"const std::vector<double> kJoint(?:Lower|Upper)LimitDeg\{[^;]+;", text))
    functions = "\n".join(extract(text, name) for name in ("Clamp", "ClampToJointLimits", "ClampAroundInitial", "map_left", "map_right"))
    code = '#include <algorithm>\n#include <vector>\n#include <iostream>\n#include <iomanip>\n' + declarations + functions + '''
int main(){int side,limit; double margin,delta; while(std::cin>>side>>limit>>margin>>delta){
std::vector<double> initial(7),raw(7); for(auto &x:initial)std::cin>>x;for(auto &x:raw)std::cin>>x;
auto q=ClampToJointLimits(side?map_right(raw):map_left(raw),margin);
if(limit)q=ClampAroundInitial(q,initial,delta);
for(auto x:q)std::cout<<std::setprecision(17)<<x<<" ";std::cout<<"\\n";}}
'''
    cpp, exe = a.output / "reference.cpp", a.output / "reference"
    cpp.write_text(code)
    subprocess.run(["g++", "-std=c++17", "-O2", str(cpp), "-o", str(exe)], check=True)
    rng = np.random.default_rng(42)
    lines, expected = [], []
    for limit in (True, False):
        for side_index, side in enumerate(("left", "right")):
            for raw in [np.zeros(7), np.full(7, 360.), np.full(7, -360.), *rng.uniform(-360,360,(64,7))]:
                initial = rng.uniform(-30,30,7)
                mapper = ArmMapper({s: np.deg2rad(initial) for s in ("left","right")}, limit_around_initial=limit)
                lines.append(" ".join(map(str, [side_index,int(limit),3.,60.,*initial,*raw])))
                expected.append(np.rad2deg(mapper.command(side,raw,0.).positions))
    output = subprocess.check_output([str(exe)], input="\n".join(lines) + "\n", text=True)
    actual = np.array([np.fromstring(line, sep=" ") for line in output.splitlines()])
    error = np.abs(actual - expected)
    result = dict(cases=len(expected), scalar_comparisons=int(error.size), max_error_deg=float(error.max()),
                  source_sha256=hashlib.sha256(text.encode()).hexdigest(), passed=bool(error.max() < 1e-10))
    (a.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))
    raise SystemExit(0 if result["passed"] else 1)
