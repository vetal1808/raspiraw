#!/usr/bin/env python3
"""
Differential test: runs two builds of raspiraw on the mock (see mock_mmal.c) with
many command lines and reports where they behave differently: exit status,
messages, I2C register writes, MMAL calls and output files.

    make test-diff BASELINE=<git rev>
    tools/test/difftest.py BASELINE_BINARY NEW_BINARY [only cases containing TEXT]

A difference is not necessarily a regression - it is for you to judge. In the
report "orig" is the baseline and "new" the current build.
"""
import os
import re
import shutil
import subprocess
import sys

BINS = {}
WORK = "/dev/shm/raspiraw_diff"

# (name, args, env). {d} is replaced by the per-run output directory.
CASES = []


def case(name, args, **env):
    CASES.append((name, args.split(), env))


for sensor, modes in (("ov5647", 8), ("imx219", 8), ("imx477", 4)):
    for m in range(modes):
        case("%s mode %d" % (sensor, m), "-md %d -t 700 -n -sr 1 -o {d}/o.%%04d.raw -hd0 {d}/hd0.bin" % m,
             MOCK_SENSOR=sensor)
    case(sensor + " flips", "-md 1 -t 200 -n -hf -vf -hd0 {d}/hd0.bin", MOCK_SENSOR=sensor)
    case(sensor + " hflip", "-md 1 -t 200 -n -hf -hd0 {d}/hd0.bin", MOCK_SENSOR=sensor)
    case(sensor + " vflip", "-md 1 -t 200 -n -vf -hd0 {d}/hd0.bin", MOCK_SENSOR=sensor)
    case(sensor + " exposure/gain", "-md 1 -t 200 -n -e 1000 -g 64", MOCK_SENSOR=sensor)
    case(sensor + " expus", "-md 1 -t 200 -n -eus 5000", MOCK_SENSOR=sensor)
    case(sensor + " fps", "-md 1 -t 200 -n -f 20", MOCK_SENSOR=sensor)
    case(sensor + " fps+expus", "-md 1 -t 200 -n -f 25 -eus 20000", MOCK_SENSOR=sensor)
    case(sensor + " fps high", "-md 1 -t 200 -n -f 1000", MOCK_SENSOR=sensor)
    for b in (8, 10, 12, 16):
        case("%s bitdepth %d" % (sensor, b), "-md 1 -t 200 -n -b %d -sr 1 -o {d}/o.%%04d.raw -hd -hd0 {d}/hd0.bin" % b,
             MOCK_SENSOR=sensor)
    case(sensor + " crop", "-md 1 -t 200 -n -w 320 -h 240 -lt 16 -tp 8 -sr 1 -o {d}/o.%%04d.raw", MOCK_SENSOR=sensor)
    case(sensor + " hinc/vinc", "-md 1 -t 200 -n -hi 31 -vi 31", MOCK_SENSOR=sensor)
    case(sensor + " regs", "-md 1 -t 200 -n -r 380A,01;380B,40", MOCK_SENSOR=sensor)

case("long options", "--mode 2 --timeout 200 --nopreview --saverate 3 --output {d}/o.%%04d.raw --gain 32")
case("camera 0", "-md 7 -t 200 -n -c 0")
case("camera 1", "-md 7 -t 200 -n -c 1")
case("awb gains", "-md 7 -t 200 -n -awbg 1.5,1.2")
case("awb gains bad", "-md 7 -t 200 -n -awbg 1.5")
case("pgm header", "-md 7 -t 200 -n -hdg {d}/hd.pgm -sr 1 -o {d}/o.%%04d.raw")
case("timestamps", "-md 7 -t 300 -n -ts {d}/ts.csv -sr 2 -o {d}/o.%%04d.raw")
case("timestamps no output", "-md 7 -t 300 -n -ts {d}/ts.csv")
case("empty files", "-md 7 -t 200 -n -emp -sr 1 -o {d}/o.%%04d.raw")
case("header+empty", "-md 7 -t 200 -n -emp -hd -sr 1 -o {d}/o.%%04d.raw")
case("metadata", "-md 7 -t 200 -n -m -sr 1 -o {d}/o.%%04d.raw", MOCK_METADATA="1", MOCK_FPS="10")
case("awb thread", "-md 7 -t 400 -n -awb")
case("processing thread", "-md 7 -t 400 -n -P")
case("yuv processing", "-md 7 -t 400 -n -PY")
case("all threads + files", "-md 7 -t 400 -awb -P -PY -sr 1 -o {d}/o.%%04d.raw -oY {d}/y.%%04d.yuv -ts {d}/ts.csv")
case("yuv output", "-md 7 -t 300 -n -sr 1 -oY {d}/y.%%04d.yuv")
case("preview default", "-md 7 -t 300")
case("preview window", "-md 7 -t 300 -p 10,20,320,240")
case("preview window + opacity", "-md 7 -t 300 -p 10,20,320,240 -op 128")
case("preview fullscreen then output", "-md 7 -t 300 -fs -sr 1 -o {d}/o.%%04d.raw")
case("preview bad window", "-md 7 -t 300 -p 10,20")
case("opacity 300", "-md 7 -t 200 -op 300")
case("opacity -1", "-md 7 -t 200 -op -1")
case("no sensor", "-md 7 -t 200 -n", MOCK_SENSOR="none")
case("no i2c bus at all", "-md 7 -t 200 -n", MOCK_I2C_BUSES="")
case("only bus 0 (fallback)", "-md 7 -t 200 -n", MOCK_I2C_BUSES="0")
case("-y 0 on bus 0", "-md 7 -t 200 -n -y 0", MOCK_I2C_BUSES="0")
case("-y 3 missing", "-md 7 -t 200 -n -y 3")
case("mode out of range", "-md 99 -t 200 -n")
case("mode negative", "-md -1 -t 200 -n")
case("bitdepth 9", "-md 7 -t 200 -n -b 9")
case("bitdepth 14", "-md 7 -t 200 -n -b 14")
case("no args", "")
case("help", "-?")
case("missing value -md", "-md")
case("missing value -t at end", "-n -t")
case("missing value -o", "-md 7 -t 200 -n -o")
case("unknown option", "-zz")
case("stray argument", "stray")
case("-sr 0", "-md 7 -t 200 -n -sr 0 -o {d}/o.%%04d.raw")
case("-c 5", "-c 5")
case("bad output pattern", "-o bad_%%s.raw")
case("output without %%d", "-md 7 -t 200 -n -sr 1 -o {d}/single.raw")
case("crop too big", "-md 7 -t 200 -n -w 9999")
case("crop height too big", "-md 7 -t 200 -n -h 9999")
case("odd width", "-md 7 -t 200 -n -w 333 -h 111 -sr 1 -o {d}/o.%%04d.raw")
for fail in ("create:vc.ril.rawcam", "create:vc.ril.isp", "create:vc.ril.video_render", "commit:rawcam:out:0",
             "commit:isp:in:0", "commit:isp:out:0", "commit:render:in:0", "cenable:vc.ril.rawcam", "cenable:vc.ril.isp",
             "cenable:vc.ril.video_render", "pool:rawcam:out:0", "pool:isp:in:0", "pool:isp:out:0", "pool:render:in:0",
             "enable:isp:in:0", "enable:isp:out:0", "enable:render:in:0", "enable:rawcam:out:0"):
    case("FAIL " + fail, "-md 7 -t 200 -awb -P -PY -sr 1 -o {d}/o.%%04d.raw", MOCK_FAIL=fail)
    case("FAIL nothreads " + fail, "-md 7 -t 200", MOCK_FAIL=fail)


def normalise(text, directory):
    out = []
    for line in text.replace(directory, "<DIR>").splitlines():
        line = re.sub(r"0x[0-9a-f]{6,}", "<PTR>", line)
        line = re.sub(r"==\d+==", "==PID==", line)
        if line.startswith("MOCK AWB"):
            continue
        # b770dd2 and older printed these two values under each other's name
        line = re.sub(r"^Set pack to (\d+), unpack to (\d+)$", r"Set unpack to \1, pack to \2", line)
        out.append(line)
    return out


def run(which, args, env, timeout=30):
    d = os.path.join(WORK, which)
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d)
    e = dict(os.environ, ASAN_OPTIONS="detect_leaks=0", MOCK_NO_FLUSH="1", MOCK_LOG=os.path.join(d, "mock.log"))
    e.update(env)
    cmd = [BINS[which]] + [a.replace("{d}", d).replace("%%", "%") for a in args]
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout, env=e)
        rc, so, se = p.returncode, p.stdout.decode(errors="replace"), p.stderr.decode(errors="replace")
    except subprocess.TimeoutExpired:
        rc, so, se = "TIMEOUT", "", ""
    log, summary = [], ""
    if os.path.exists(e["MOCK_LOG"]):
        for line in open(e["MOCK_LOG"]).read().splitlines():
            if line.startswith("SUMMARY"):
                summary = line
            else:
                log.append(line.replace(d, "<DIR>"))
    files = {}
    for name in sorted(os.listdir(d)):
        if name == "mock.log":
            continue
        files[name] = open(os.path.join(d, name), "rb").read()
    return dict(rc=rc, out=normalise(so, d), err=normalise(se, d), log=log, summary=summary, files=files, dir=d)


def first_diff(a, b):
    for i, (x, y) in enumerate(zip(a, b)):
        if x != y:
            return "line %d:\n      orig: %s\n      new:  %s" % (i + 1, x, y)
    if len(a) != len(b):
        longer, who = (a, "orig") if len(a) > len(b) else (b, "new")
        return "%s has %d extra lines, first: %s" % (who, abs(len(a) - len(b)), longer[min(len(a), len(b))])
    return None


def compare(name, o, n):
    notes = []
    if o["rc"] != n["rc"]:
        notes.append("exit status: orig %s, new %s" % (o["rc"], n["rc"]))
    d = first_diff(o["log"], n["log"])
    if d:
        notes.append("mock event log (I2C/MMAL) differs, %s" % d)
    # Messages: order is thread dependent, so compare as sets, ignoring the log lines the refactor added
    for key, label in (("out", "stdout"), ("err", "stderr")):
        ign = re.compile(r"^(Create pool of \d+ buffers of size \d+)$")
        a = set(l for l in o[key] if not ign.match(l))
        b = set(l for l in n[key] if not ign.match(l))
        for l in sorted(a - b)[:4]:
            notes.append("%s only in orig: %s" % (label, l[:150]))
        for l in sorted(b - a)[:4]:
            notes.append("%s only in new:  %s" % (label, l[:150]))
    # Files: the fixed ones must be identical; frame files are timing dependent, so compare
    # the first frame and all sizes.
    fo, fn = o["files"], n["files"]
    fixed = [f for f in set(fo) | set(fn) if not re.match(r"[oy]\.\d+\.", f) and f != "ts.csv"]
    for f in sorted(fixed):
        if fo.get(f) != fn.get(f):
            notes.append("file %s differs (orig %s bytes, new %s bytes)" % (
                f, len(fo[f]) if f in fo else "missing", len(fn[f]) if f in fn else "missing"))
    for prefix in ("o.", "y."):
        a = sorted(f for f in fo if f.startswith(prefix))
        b = sorted(f for f in fn if f.startswith(prefix))
        if bool(a) != bool(b):
            notes.append("%s* files: orig %d, new %d" % (prefix, len(a), len(b)))
        elif a:
            if a[0] != b[0] or fo[a[0]] != fn[b[0]]:
                notes.append("first %s* file differs (%s %d bytes vs %s %d bytes)" % (
                    prefix, a[0], len(fo[a[0]]), b[0], len(fn[b[0]])))
            if a[:3] != b[:3]:
                notes.append("%s* numbering differs: %s vs %s" % (prefix, a[:3], b[:3]))
            if set(len(fo[f]) for f in a) != set(len(fn[f]) for f in b):
                notes.append("%s* sizes differ: %s vs %s" % (
                    prefix, sorted(set(len(fo[f]) for f in a)), sorted(set(len(fn[f]) for f in b))))
            if abs(len(a) - len(b)) > 2 + len(a) // 5:
                notes.append("%s* file count: orig %d, new %d" % (prefix, len(a), len(b)))
    if ("ts.csv" in fo) != ("ts.csv" in fn):
        notes.append("ts.csv: orig %s, new %s" % ("ts.csv" in fo, "ts.csv" in fn))
    elif "ts.csv" in fo and fo["ts.csv"].splitlines()[:5] != fn["ts.csv"].splitlines()[:5]:
        notes.append("ts.csv first lines differ: %r vs %r" % (fo["ts.csv"][:60], fn["ts.csv"][:60]))
    return notes


def sanitizer_hits(r):
    text = "\n".join(r["err"] + r["out"])
    hits = re.findall(r"(ERROR: AddressSanitizer[^\n]*|runtime error:[^\n]*|MOCK ERROR[^\n]*|LeakSanitizer[^\n]*)", text)
    return sorted(set(hits))


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    BINS["orig"], BINS["new"] = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
    only = sys.argv[3] if len(sys.argv) > 3 else None
    same = differ = 0
    for name, args, env in CASES:
        if only and only not in name:
            continue
        o = run("orig", args, env)
        n = run("new", args, env)
        notes = compare(name, o, n)
        for which, r in (("orig", o), ("new", n)):
            for h in sanitizer_hits(r):
                notes.append("[%s] %s" % (which, h))
            if isinstance(r["rc"], int) and r["rc"] < 0:
                notes.append("[%s] killed by signal %d" % (which, -r["rc"]))
        envs = " ".join("%s=%s" % kv for kv in env.items())
        if notes:
            differ += 1
            print("DIFF  %-34s rc %s/%s  [%s %s]" % (name, o["rc"], n["rc"], envs, " ".join(args)))
            for note in notes:
                print("    - " + note)
        else:
            same += 1
            print("same  %-34s rc %s  %s" % (name, n["rc"], n["summary"].replace("SUMMARY ", "")[:60]))
        if only:
            for which, r in (("orig", o), ("new", n)):
                print("---- %s stderr tail" % which)
                print("\n".join(r["err"][-12:]))
    print("\n%d cases identical, %d with differences" % (same, differ))


if __name__ == "__main__":
    sys.exit(main())
