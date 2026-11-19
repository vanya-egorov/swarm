"""Run sensor preflight and three autonomous agents in the main village.wbt."""

import argparse
import json
import math
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import time

PROJECT = Path(__file__).resolve().parents[2]
RUN = Path(os.environ.get("RUN_DIR", PROJECT / "projectData/run")).resolve()


def read_records(path):
    records = []
    for line in path.read_text().splitlines():
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return records


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGCONT)
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=75)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def probe(sockets, output, processes):
    frames = {i: [] for i in range(1, 4)}
    deadline = time.monotonic()+60
    while min(map(len, frames.values())) < 20:
        if time.monotonic() > deadline or any(p.poll() is not None for p in processes):
            raise RuntimeError("Sensor preflight did not receive 20 frames per drone")
        for channel in select.select(sockets, [], [], 0.2)[0]:
            data, source = channel.recvfrom(8192)
            frame = json.loads(data)
            i = frame["SystemID"]
            if source != ("127.0.0.1", 14800+i):
                raise RuntimeError("Unexpected sensor source")
            if frame["Lidar"]["Resolution"] != 192 or len(frame["Lidar"]["RangesM"]) != 576 or len(frame["Sonars"]) != 10:
                raise RuntimeError("Incomplete lidar/sonar frame")
            if any(not math.isfinite(v) or (v != -1 and not 0.12 <= v <= 12) for v in frame["Lidar"]["RangesM"]):
                raise RuntimeError("Invalid lidar measurement")
            if any(not math.isfinite(s["RangeM"]) or not 0 <= s["RangeM"] <= 6 for s in frame["Sonars"]):
                raise RuntimeError("Invalid sonar measurement")
            if frames[i] and (frame["Sequence"] <= frames[i][-1]["Sequence"] or frame["TimeS"] <= frames[i][-1]["TimeS"]):
                raise RuntimeError("Sensor clock or sequence did not advance")
            frames[i].append(frame)
    (output / "sensor-preflight.json").write_text(json.dumps(frames, indent=2)+"\n")
    for i, measurements in frames.items():
        frame = measurements[-1]
        print(f"Sensors ID {i}: {len(measurements)} frames; pose={frame['Pose']}; "
              f"sonars={[round(s['RangeM'], 3) for s in frame['Sonars']]}", flush=True)


def validate_agents(output, failed_leader):
    replacement = None
    fault = json.loads((output / "fault.json").read_text()) if failed_leader else None
    for i in range(1, 4):
        records = read_records(output / f"agent-{i}.log")
        if not any(r.get("msg") == "landed" for r in records):
            raise RuntimeError(f"ID {i} did not confirm landing")
        started = [r for r in records if r.get("msg") == "coverage agent started"]
        if len(started) != 1 or started[0].get("waypoints", 0) < 2:
            raise RuntimeError(f"ID {i} did not report its planned route")
        expected = list(range(1, started[0]["waypoints"]+1))
        reached = [r.get("index") for r in records if r.get("msg") == "waypoint reached"]
        if i != failed_leader and reached != expected:
            raise RuntimeError(f"ID {i} did not complete its planned waypoints: {reached}")
        if fault and i != failed_leader:
            elected = [r for r in records if r.get("msg") == "coordination leader changed" and
                       r.get("term", 0) > fault["term"] and r.get("leaderId", 0) not in (0, failed_leader)]
            if not elected:
                raise RuntimeError(f"ID {i} did not observe a replacement leader")
            if not any("coverage incomplete" in r.get("error", "") for r in records):
                raise RuntimeError(f"ID {i} did not report the unavailable drone's incomplete coverage")
            replacement = elected[-1]["leaderId"]
    return replacement


def validate_physics(output, finished_at):
    path = output / "physics-summary.json"
    if finished_at-path.stat().st_mtime > 2:
        raise RuntimeError("Physical observer stopped before the agents finished")
    physics = json.loads(path.read_text())
    ids = {"1", "2", "3"}
    for field in ("airborneContacts", "outsideRegion", "maxHeightM"):
        if set(physics[field]) != ids:
            raise RuntimeError("Physical observer did not track all three drones")
    if any(physics["airborneContacts"].values()) or any(physics["outsideRegion"].values()) or physics["minSeparationM"] < 4:
        raise RuntimeError("Physical acceptance criteria violated")
    if physics["samples"] < 100 or min(physics["maxHeightM"].values()) < 0.8:
        raise RuntimeError("Physical flight evidence is incomplete")
    return physics


def run(args):
    mission_path = RUN / "webots/coverage-mission.json"
    mission = json.loads(mission_path.read_text())
    mission["RunID"] = f"village-{time.time_ns()}"
    output = RUN / "coverage-checks" / mission["RunID"]
    output.mkdir(parents=True)
    mission_path = output / "mission.json"
    mission_path.write_text(json.dumps(mission, indent=2)+"\n")
    print(f"Evidence: {output}", flush=True)
    world = RUN / "webots/worlds/village.wbt"
    original = world.read_text()
    if 'name "Coverage observer"' in original:
        raise RuntimeError("Another acceptance run is using village.wbt")
    observer = RUN / "webots/controllers/coverage_observer"
    source = PROJECT / "sim/webots/controllers/coverage_observer"
    if not observer.exists():
        observer.symlink_to(source, target_is_directory=True)
    elif observer.resolve() != source.resolve():
        raise RuntimeError("Foreign coverage observer controller")
    world.write_text(original+f'\nRobot {{ name "Coverage observer" supervisor TRUE '
                     f'controller "coverage_observer" customData "{mission_path}" }}\n')
    env = dict(os.environ, COVERAGE_EVIDENCE_DIR=str(output))
    processes, agents, sockets, files = [], [], [], []

    def launch(command, log, cwd=PROJECT):
        stream = (output / log).open("w")
        files.append(stream)
        process = subprocess.Popen(command, cwd=cwd, stdout=stream, stderr=subprocess.STDOUT, env=env)
        processes.append(process)
        return process

    try:
        for i in range(1, 4):
            channel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            channel.bind(("127.0.0.1", 14700+i))
            sockets.append(channel)
        for instance in range(3):
            directory = output / f"sitl-I{instance}"
            directory.mkdir()
            launch([str(RUN / "bin/arducopter-webots"), "--model", "webots-python", "--speedup", "1",
                    "--defaults", str(RUN / "webots/village-flight.parm"), "--sim-address=127.0.0.1",
                    f"--serial0=udpclient:127.0.0.1:{14560+instance*10}", "--sysid", str(instance+1),
                    f"-I{instance}",
                    f"--home={mission['Origin']['LatitudeDeg']},{mission['Origin']['LongitudeDeg']},584,0"],
                   f"sitl-{instance+1}.log", directory)
        webots = launch([args.webots, "--batch", "--mode=fast", "--no-rendering", "--stdout", "--stderr", str(world)],
                        "webots.log")
        probe(sockets, output, processes)
        for channel in sockets:
            channel.close()
        sockets.clear()
        if args.probe_only:
            return
        for i in range(1, 4):
            agents.append(launch([str(RUN / "bin/swarm"), "--coverage-mission", str(mission_path),
                                  "--system-id", str(i), "--listen", f"127.0.0.1:{14550+i*10}",
                                  "--coordination-state-dir", str(output / "coordination")], f"agent-{i}.log"))
        deadline = time.monotonic()+600
        failed_leader = None
        paused_at = None
        while any(p.poll() is None for p in agents):
            if webots.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("Webots exited or mission exceeded ten minutes")
            if args.failover and failed_leader is None:
                for i, process in enumerate(agents, 1):
                    records = read_records(output / f"agent-{i}.log")
                    leaders = [r for r in records if r.get("msg") == "coordination leader changed"]
                    moving = any(r.get("msg") == "waypoint started" for r in records)
                    if moving and leaders and leaders[-1].get("leaderId") == i:
                        process.send_signal(signal.SIGSTOP)
                        failed_leader = i
                        paused_at = time.monotonic()
                        print(f"Injected agent failure: leader ID {i}", flush=True)
                        (output / "fault.json").write_text(json.dumps({"leader": i, "signal": "SIGSTOP",
                                                                    "term": leaders[-1]["term"]})+"\n")
                        break
            if paused_at is not None and time.monotonic()-paused_at >= 8:
                agents[failed_leader-1].send_signal(signal.SIGCONT)
                print(f"Resumed failed agent ID {failed_leader} after eight seconds", flush=True)
                paused_at = None
            time.sleep(0.2)
        codes = [p.returncode for p in agents]
        physics = validate_physics(output, time.time())
        result = {"exitCodes": codes, "failedLeader": failed_leader, "physics": physics}
        (output / "result.json").write_text(json.dumps(result, indent=2)+"\n")
        print(json.dumps(result, indent=2), flush=True)
        if not args.failover and codes != [0, 0, 0]:
            raise RuntimeError("Coverage agents did not all succeed")
        if args.failover and failed_leader is None:
            raise RuntimeError("Leader failure was not injected")
        result["replacementLeader"] = validate_agents(output, failed_leader)
        result["accepted"] = True
        (output / "result.json").write_text(json.dumps(result, indent=2)+"\n")
        print(f"Accepted; replacement leader: {result['replacementLeader']}", flush=True)
    finally:
        for process in reversed(agents):
            stop(process)
        for process in reversed(processes):
            if process not in agents:
                stop(process)
        for channel in sockets:
            channel.close()
        for stream in files:
            stream.close()
        world.write_text(original)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--webots", default=os.environ.get("WEBOTS_BIN", "/Applications/Webots.app/Contents/MacOS/webots"))
    parser.add_argument("--probe-only", action="store_true")
    parser.add_argument("--failover", action="store_true")
    args = parser.parse_args()
    lock = RUN / "coverage-check.lock"
    try:
        fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    except FileExistsError:
        parser.exit(1, f"Coverage check already owns {lock}\n")
    try:
        with os.fdopen(fd, "w") as stream:
            stream.write(str(os.getpid())+"\n")
        run(args)
    finally:
        lock.unlink()
