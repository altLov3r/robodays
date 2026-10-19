"""Compile and run the actual sketch against fake hardware; no robot access."""
import os
import argparse
import re
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--focused", action="store_true", help="Skip full-course motion models.")
options = parser.parse_args()
sketch = (root.parent / "PololuLineFollower/PololuLineFollower.ino").read_text(encoding="utf-8")
font = sketch.split("raceOledFont[][5] PROGMEM = {", 1)[1].split("};", 1)[0]
assert len(re.findall(r"0b[01]{8}", font)) == 49 * 5
display_text = re.findall(r'F\("([^"\\]*)"\)', sketch)
display_text += re.findall(r"display\.print\('([^'\\])'\)", sketch)
for text in display_text:
    for char in text.upper():
        assert char in " !" or "+" <= char <= "Z", f"Unsupported OLED glyph: {char!r}"
compiler = shutil.which("g++") or shutil.which("clang++")
if not compiler and Path("C:/msys64/mingw64/bin/g++.exe").is_file():
    compiler = "C:/msys64/mingw64/bin/g++.exe"
if not compiler:
    raise SystemExit("Install a native g++ or clang++ compiler to run these tests.")

environment = os.environ.copy()
environment["PATH"] = str(Path(compiler).parent) + os.pathsep + environment["PATH"]
with tempfile.TemporaryDirectory(prefix="robodays-tests-") as output:
    for name in ("test_line_follower", "test_race_session", "test_corner_exit", "test_large_map"):
        executable = Path(output) / (name + (".exe" if os.name == "nt" else ""))
        subprocess.run(
            [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", "-I", str(root / "fakes"),
             str(root / (name + ".cpp")), "-o", str(executable)],
            check=True, env=environment,
        )
        arguments = [str(executable)] + (["--focused"] if options.focused else [])
        subprocess.run(arguments, check=True, env=environment)
