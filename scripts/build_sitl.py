"""Build SITL with the Webots clock fix, then restore the exact upstream source."""

from contextlib import contextmanager
import os
import shutil
from pathlib import Path
import subprocess
import sys
import tempfile

PROJECT = Path(__file__).resolve().parents[1]
SOURCE = Path("libraries/SITL/SIM_Webots_Python.cpp")
PATCH = PROJECT / "sim/ardupilot/patches/webots-clock.patch"


@contextmanager
def patched_source(ardupilot, patch=PATCH):
    """Restore upstream files after a build while preserving concurrent edits."""
    paths = [Path(line[6:]) for line in patch.read_text().splitlines() if line.startswith("+++ b/")]
    if not paths or any(path.is_absolute() or ".." in path.parts for path in paths):
        raise ValueError("Invalid patch paths")
    originals = {path: ((ardupilot / path).read_bytes(), (ardupilot / path).stat()) for path in paths}
    # Validate the exact revision in a scratch tree before changing any upstream file.
    with tempfile.TemporaryDirectory(prefix="swarm-sitl-") as directory:
        scratch = Path(directory)
        for path, (data, _) in originals.items():
            target = scratch / path
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
        subprocess.run(["patch", "--batch", "--fuzz=0", "-p1", "-i", str(patch.resolve())],
                       cwd=scratch, check=True)
        patched = {path: (scratch / path).read_bytes() for path in paths}
    lock = ardupilot / ".swarm-sitl-build.lock"
    backup = ardupilot / ".swarm-sitl-source-backup"
    with lock.open("x"):
        backup.mkdir()
        for path, (data, _) in originals.items():
            target = backup / path
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
        try:
            for path in paths:
                source = ardupilot / path
                if source.read_bytes() != originals[path][0]:
                    raise RuntimeError(f"Source changed before build: {source}")
                source.write_bytes(patched[path])
            yield
        finally:
            changed = []
            for path, (data, stat) in originals.items():
                source = ardupilot / path
                if not source.is_file() or source.read_bytes() not in (patched[path], data):
                    changed.append(str(path))
                    continue
                source.write_bytes(data)
                os.utime(source, ns=(stat.st_atime_ns, stat.st_mtime_ns))
            if changed:
                raise RuntimeError(f"Source changed during build: {changed}; originals saved to {backup}")
            shutil.rmtree(backup)
            lock.unlink()


def main():
    ardupilot = Path(os.environ.get("ARDUPILOT_DIR", PROJECT / "projectData/ardupilot")).resolve()
    python = Path(os.environ.get("SITL_PYTHON", ardupilot / ".venv/bin/python3"))
    interpreter = str(python) if python.is_file() else sys.executable
    build_env = dict(os.environ)
    if sys.platform == "darwin":
        # Current Apple ld rejects the unaligned pointers in ArduPilot's packed FWVersion.
        build_env["LDFLAGS"] = build_env.get("LDFLAGS", "") + " -Wl,-no_fixup_chains"
    with patched_source(ardupilot):
        subprocess.run([interpreter, "waf", "configure", "--board", "sitl"],
                       cwd=ardupilot, env=build_env, check=True)
        subprocess.run([interpreter, "waf", "copter", "-j", os.environ.get("SITL_JOBS", "2")],
                       cwd=ardupilot, env=build_env, check=True)
    run_dir = Path(os.environ.get("RUN_DIR", PROJECT / "projectData/run"))
    output = run_dir / "bin/arducopter-webots"
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=output.parent, prefix="sitl-build-") as directory:
        binary = Path(directory) / output.name
        shutil.copy2(ardupilot / "build/sitl/bin/arducopter", binary)
        binary.replace(output)
    print(f"SITL с исправлением часов собран: {output}")
    print("Исходный файл ArduPilot восстановлен.")


if __name__ == "__main__":
    main()
