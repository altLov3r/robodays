"""Run real firmware against an approximate, photo-traced crossing course (no robot IO)."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from zipfile import ZipFile


ROOT = Path(__file__).resolve().parent.parent


def solve(matrix, values):
    rows = [list(row) + [value] for row, value in zip(matrix, values)]
    for col in range(len(rows)):
        pivot = max(range(col, len(rows)), key=lambda i: abs(rows[i][col]))
        rows[col], rows[pivot] = rows[pivot], rows[col]
        divisor = rows[col][col]
        assert abs(divisor) > 1e-12
        rows[col] = [v / divisor for v in rows[col]]
        for i in range(len(rows)):
            if i != col:
                factor = rows[i][col]
                rows[i] = [a - factor * b for a, b in zip(rows[i], rows[col])]
    return [row[-1] for row in rows]


def geometry():
    source = json.loads((ROOT / "tests/photo_track.json").read_text())
    matrix, values = [], []
    for (x, y), (u, v) in zip(source["mat_corners_px"], [(0, 0), (1, 0), (1, 1), (0, 1)]):
        matrix += [[x, y, 1, 0, 0, 0, -u*x, -u*y], [0, 0, 0, x, y, 1, -v*x, -v*y]]
        values += [u, v]
    h = solve(matrix, values)

    def transform(point):
        x, y = point
        divisor = h[6]*x + h[7]*y + 1
        return ((h[0]*x + h[1]*y + h[2]) / divisor,
                (h[3]*x + h[4]*y + h[5]) / divisor)

    top = [transform(p) for p in source["top_reference_px"]]
    right = [transform(p) for p in source["right_reference_px"]]
    width = source["top_reference_mm"] / abs(top[1][0] - top[0][0])
    height = math.sqrt(source["right_reference_mm"]**2 - ((right[1][0] - right[0][0])*width)**2)
    height /= abs(right[1][1] - right[0][1])
    dense = []
    previous = source["start_px"]
    for segment in source["segments"]:
        controls = [previous] + segment
        for step in range(400):
            t = step / 400
            points = controls
            while len(points) > 1:
                points = [tuple((1-t)*a + t*b for a, b in zip(p, q)) for p, q in zip(points, points[1:])]
            u, v = transform(points[0])
            dense.append((u*width, v*height))
        previous = segment[-1]
    dense.append(dense[0])
    # Constant arc spacing makes route scoring and optical distance independent of trace density.
    sampled, distance, target = [], 0.0, 0.0
    for p, q in zip(dense, dense[1:]):
        length = math.dist(p, q)
        while target <= distance + length:
            t = (target - distance) / length
            sampled.append(tuple((1-t)*a + t*b for a, b in zip(p, q)))
            target += 2
        distance += length
    # Start halfway along the measured long straight, away from either crossing.
    start = round(source["right_reference_mm"] / 2 / 2)
    sampled = sampled[start:] + sampled[:start]
    assert len(sampled) > 1000
    assert max(math.dist(p, q) for p, q in zip(sampled, sampled[1:] + sampled[:1])) < 2.1
    coarse = sampled[::10]
    edges = list(zip(coarse, coarse[1:] + coarse[:1]))
    crossings = []
    for i, (a, b) in enumerate(edges):
        for j in range(i+2, len(edges)):
            if i == 0 and j == len(edges)-1:
                continue
            c, d = edges[j]
            ab, cd = (b[0]-a[0], b[1]-a[1]), (d[0]-c[0], d[1]-c[1])
            determinant = ab[0]*cd[1] - ab[1]*cd[0]
            if abs(determinant) < 1e-9:
                continue
            ac = (c[0]-a[0], c[1]-a[1])
            t = (ac[0]*cd[1] - ac[1]*cd[0]) / determinant
            u = (ac[0]*ab[1] - ac[1]*ab[0]) / determinant
            if 0 < t < 1 and 0 < u < 1:
                crossings.append([round(a[0]+t*ab[0], 1), round(a[1]+t*ab[1], 1)])
    assert len(crossings) == 2, "Trace must contain exactly the two photographed crossings"
    return sampled, {"length_mm": round(distance, 1), "mat_width_mm": round(width, 1),
                     "mat_height_mm": round(height, 1), "point_spacing_mm": 2,
                     "crossings_mm": crossings,
                     "description": source["description"]}


def write_plot(points, destination):
    xmin, ymin = (min(p[i] for p in points) for i in (0, 1))
    xmax, ymax = (max(p[i] for p in points) for i in (0, 1))
    path = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    x, y = points[0]
    destination.write_text(f'''<svg xmlns="http://www.w3.org/2000/svg" width="700" height="1050" viewBox="{xmin-120} {ymin-150} {xmax-xmin+240} {ymax-ymin+300}">
<rect x="{xmin-120}" y="{ymin-150}" width="{xmax-xmin+240}" height="{ymax-ymin+300}" fill="white"/>
<polyline points="{path} {points[0][0]},{points[0][1]}" stroke="#18232d" stroke-width="20" fill="none" stroke-linejoin="round"/>
<circle cx="{x}" cy="{y}" r="25" fill="#db412e"/><text x="{x-180}" y="{y-50}" font-size="55" fill="#b02819">START</text>
<text x="{xmin}" y="{ymin-65}" font-size="45">Approximate photo trace • two crossings</text>
<text x="{xmin}" y="{ymax+100}" font-size="38">Geometry and motor/sensor response require physical validation</text></svg>''', encoding="utf-8")


def write_crossing_plot(traces, length, destination):
    candidate = traces["candidate"]
    center = next((float(row["progress"])*length for row in candidate
                   if row["estimated_lap"] == "1" and int(row["dark"]) >= 4), None)
    if center is None:
        destination.write_text('<svg xmlns="http://www.w3.org/2000/svg" width="600" height="100">'
                               '<text x="20" y="40">Candidate did not reach the first replay crossing.</text></svg>')
        return
    shapes = []
    for label, rows in traces.items():
        points = []
        for row in rows:
            offset = float(row["progress"])*length-center
            if -100 <= offset <= 140:
                pwm = (int(row["left"])+int(row["right"]))/2
                points.append(f"{70+(offset+100)*3:.1f},{360-pwm*.75:.1f}")
        color = "#078265" if label == "candidate" else "#c03e37"
        shapes.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" stroke-width="3"/>')
    destination.write_text('''<svg xmlns="http://www.w3.org/2000/svg" width="860" height="450" viewBox="0 0 860 450">
<rect width="860" height="450" fill="white"/>
<text x="70" y="25" font-size="19">First replay crossing: delivered wheel-average PWM (model)</text>
<path d="M70 60 V360 H790 M370 60 V360" fill="none" stroke="#bbb"/>
<text x="25" y="65">400</text><text x="40" y="365">0</text>
<text x="65" y="385">-100</text><text x="365" y="385">0</text><text x="765" y="385">140</text>
<text x="210" y="420">Distance relative to first detected crossing sample (mm)</text>
<text x="610" y="45" fill="#c03e37">Frozen baseline</text>
<text x="610" y="70" fill="#078265">Candidate</text>
''' + "\n".join(shapes) + "</svg>", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", default="R44")
    parser.add_argument("--candidate", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--quick", action="store_true", help="Only nominal forward/reverse Beast cases.")
    parser.add_argument("--traces", action="store_true", help="Retain per-case CSV traces beside the report.")
    parser.add_argument("--check", action="store_true", help="Require the candidate's nominal cases to complete the true route.")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    points, metadata = geometry()
    write_plot(points, args.output / "track.svg")
    compiler = shutil.which("g++") or "C:/msys64/mingw64/bin/g++.exe"
    environment = os.environ.copy()
    environment["PATH"] = str(Path(compiler).parent) + os.pathsep + environment["PATH"]
    cases = [dict(mode=2, direction=d, gain=3.25, response=.25, pitch=10, width=20,
                  noise=0, mismatch=0, period=5, scale=1, replay_encoder_bias=0,
                  gyro_scale=1, sensor_delay_ms=0) for d in (1, -1)]
    if not args.quick:
        for change in (dict(mode=0), dict(mode=1), dict(gain=3.75), dict(response=.15),
                       dict(noise=30, mismatch=.03), dict(pitch=9, width=16),
                       dict(period=10), dict(scale=.9), dict(scale=1.1),
                       dict(sensor_delay_ms=2), dict(gyro_scale=1.001),
                       dict(gyro_scale=1.005), dict(gyro_scale=.995),
                       dict(replay_encoder_bias=.005), dict(replay_encoder_bias=-.005),
                       dict(replay_encoder_bias=.01), dict(replay_encoder_bias=-.01)):
            cases += [dict(case, **change) for case in cases[:2]]
    results, sources, nominal_traces = {}, {}, {}
    with tempfile.TemporaryDirectory(prefix="robodays-photo-") as folder:
        work = Path(folder)
        (work / "tests").mkdir()
        (work / "PololuLineFollower").mkdir()
        for name in ("test_line_follower.cpp", "photo_track.cpp"):
            shutil.copy(ROOT / "tests" / name, work / "tests" / name)
        shutil.copytree(ROOT / "tests/fakes", work / "tests/fakes")
        course = work / "course.txt"
        course.write_text("\n".join(f"{x:.6f} {y:.6f}" for x, y in points))
        for label in (args.baseline, "candidate"):
            if label == "candidate":
                for name in ("PololuLineFollower.ino", "RaceSession.h"):
                    shutil.copy(args.candidate / "PololuLineFollower" / name, work / "PololuLineFollower" / name)
            else:
                with ZipFile(ROOT / "releases" / f"PololuLineFollower-{label}.zip") as release:
                    for name in ("PololuLineFollower.ino", "RaceSession.h"):
                        (work / "PololuLineFollower" / name).write_bytes(release.read("PololuLineFollower/" + name))
            binary = work / (label + (".exe" if os.name == "nt" else ""))
            sources[label] = {name: hashlib.sha256((work / "PololuLineFollower" / name).read_bytes()).hexdigest()
                              for name in ("PololuLineFollower.ino", "RaceSession.h")}
            subprocess.run([compiler, "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-I",
                            str(work / "tests/fakes"), str(work / "tests/photo_track.cpp"), "-o", str(binary)],
                           check=True, env=environment)
            results[label] = []
            for number, case in enumerate(cases):
                trajectory = (args.output if args.traces else work) / f"{label}-{number}.csv"
                run = subprocess.run([str(binary), str(course), str(trajectory)] + [str(v) for v in case.values()],
                                     check=True, capture_output=True, text=True, env=environment, timeout=60)
                result = json.loads(run.stdout)
                results[label].append(dict(parameters=case, **result))
                if number == 0:
                    nominal_traces[label] = list(csv.DictReader(trajectory.read_text().splitlines()))
                print(label, number, result["stop"], "estimated/true", result["estimated_laps"],
                      result["true_laps"], "at", result["route_fraction"], flush=True)
    write_crossing_plot(nominal_traces, metadata["length_mm"], args.output / "crossing.svg")
    report = dict(geometry=metadata, model="ideal differential drive; five finite-width sensor spots; union of all line strokes; no tyre-slip/grip model",
                  sensor_forward_offset_mm=30, sensor_edge_transition_mm=4, seed="0x51a7",
                  complete_definition="Firmware finishes and independently scored route covers at least three circuits minus 6 mm discretization tolerance. Overrun is reported separately by route_fraction.",
                  sources_sha256=sources, results=results)
    (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    if args.check:
        for case in results["candidate"][:2]:
            assert case["complete"], "Candidate failed a nominal photo-course case"
            assert sum(case["crossing_samples"]) > 0, "Nominal model did not exercise crossings"
            assert max(case["max_crossing_common_boost_pwm"]) == 0, "Candidate accelerated inside a crossing"
            assert max(case["max_crossing_common_drop_pwm"]) <= 3, "Nominal crossing bypassed routine braking"


if __name__ == "__main__":
    main()
