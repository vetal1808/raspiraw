#!/usr/bin/env python3
"""
Tests of raspiraw on a mock of the firmware libraries and of the sensor
(mock_mmal.c). No camera and no Pi are needed; run them with "make test".

They cover what can be checked without the real firmware: the command line,
the I2C bus selection, the files that are written, the AWB calculation, the
behaviour when part of the pipeline fails to set up, and - through the
sanitizers the test binary is built with - memory errors on all those paths.
They cannot tell whether the real firmware accepts what raspiraw asks of it:
for that, run tools/hwtest.py on a Pi with the legacy camera stack.
"""

import argparse
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
HWTEST = os.path.join(os.path.dirname(HERE), "hwtest.py")
BRCM_HEADER_LEN = 32768
BAD_OUTPUT = re.compile(r"AddressSanitizer|runtime error:|MOCK ERROR")

# Levels of the mock's test image: R 30%, G 55%, B 20% of full scale
AWB_R_GAIN = 256 * 0.55 / 0.30
AWB_B_GAIN = 256 * 0.55 / 0.20


class Tests:
    def __init__(self, raspiraw, pixtest, work):
        self.raspiraw = raspiraw
        self.pixtest = pixtest
        self.work = work
        self.fails = 0
        self.count = 0

    def check(self, cond, msg, detail=""):
        self.count += 1
        if cond:
            print("  ok    " + msg)
        else:
            self.fails += 1
            print("  FAIL  " + msg)
            for line in detail.strip().splitlines()[-12:]:
                print("        | " + line)
        return cond

    def section(self, title):
        print("\n== " + title)

    def fresh_dir(self):
        d = os.path.join(self.work, "run")
        shutil.rmtree(d, ignore_errors=True)
        os.makedirs(d)
        return d

    def run(self, args, timeout=60, **env):
        """Runs raspiraw. Returns (exit status or None on timeout, output, directory, mock log lines)."""
        d = self.fresh_dir()
        e = dict(os.environ, ASAN_OPTIONS="detect_leaks=0", MOCK_LOG=os.path.join(d, "mock.log"))
        e.update(env)
        cmd = [self.raspiraw] + [a.replace("{d}", d) for a in args.split()]
        try:
            p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout, env=e)
            rc, out = p.returncode, p.stdout.decode(errors="replace")
        except subprocess.TimeoutExpired as ex:
            rc, out = None, (ex.stdout or b"").decode(errors="replace") + "\n[timed out]"
        log = []
        if os.path.exists(e["MOCK_LOG"]):
            log = open(e["MOCK_LOG"]).read().splitlines()
        return rc, out, d, log

    def clean(self, out, what):
        """No sanitizer report and no complaint from the mock."""
        return self.check(not BAD_OUTPUT.search(out), what + ": no memory error or mock error", out)

    # ------------------------------------------------------------------

    def test_get_pixel(self):
        self.section("get_pixel() against a reference decoder")
        p = subprocess.run([self.pixtest], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           env=dict(os.environ, ASAN_OPTIONS="detect_leaks=0"))
        out = p.stdout.decode(errors="replace")
        self.check(p.returncode == 0, "all pixel formats decode correctly", out)

    def test_hwtest(self):
        self.section("tools/hwtest.py on the mock")
        for args, fps in (("-md 7 -t 1500", None), ("-md 1 -t 1500 --fps 60", "60"),
                          ("-md 4 -t 1500 --preview -- -awb -P -PY -g 64", None)):
            d = os.path.join(self.work, "hwtest")
            shutil.rmtree(d, ignore_errors=True)
            e = dict(os.environ, ASAN_OPTIONS="detect_leaks=0")
            if fps:
                e["MOCK_FPS"] = fps
            p = subprocess.run([sys.executable, HWTEST, "--raspiraw", self.raspiraw, "--dir", d] + args.split(),
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=e)
            out = p.stdout.decode(errors="replace")
            bad = "\n".join(l for l in out.splitlines() if "FAIL" in l or "WARN" in l)
            self.check(p.returncode == 0, "hwtest.py %s passes" % args, bad or out)
            shutil.rmtree(d, ignore_errors=True)

    def test_cmdline(self):
        self.section("Command line")
        for args, what in (("-md", "option without its value"), ("-n -t", "option without its value at the end"),
                           ("-md 7 -n -o", "-o without a file name"), ("-zz", "unknown option"),
                           ("stray", "argument that is not an option"), ("-md 7 -n -sr 0", "save rate 0"),
                           ("-c 5", "camera number 5"), ("-o bad_%s.raw", "output pattern with %s"),
                           ("-md 7 -n -awbg 1.5", "incomplete AWB gains"), ("-md 7 -n -b 9", "bit depth 9"),
                           ("-md 99 -n", "mode 99"), ("-md -1 -n", "mode -1")):
            rc, out, _, _ = self.run(args + " -t 100" if "-t" not in args.split() else args)
            self.check(rc not in (0, None) and rc > 0, "%s is rejected (exit status %s)" % (what, rc), out)
            self.clean(out, what)
        rc, out, _, log = self.run("-md 7 -t 200 -fs -sr 1 -o {d}/o.%04d.raw")
        self.check(rc == 0 and any(l.startswith("ENABLE render") for l in log),
                   "-fs takes no value: the options after it are still used", out)
        rc, out, _, _ = self.run("-?")
        self.check("-mode" in out, "help text is printed", out)

    def test_i2c(self):
        self.section("I2C bus and sensor detection")
        rc, out, _, _ = self.run("-md 7 -t 200 -n")
        self.check(rc == 0 and "Using I2C device /dev/i2c-10" in out, "default bus is /dev/i2c-10", out)
        rc, out, _, _ = self.run("-md 7 -t 200 -n", MOCK_I2C_BUSES="0")
        self.check(rc == 0 and "Using I2C device /dev/i2c-0" in out, "falls back to /dev/i2c-0", out)
        rc, out, _, _ = self.run("-md 7 -t 200 -n -y 3", MOCK_I2C_BUSES="3")
        self.check(rc == 0 and "Using I2C device /dev/i2c-3" in out, "-y selects the bus", out)
        rc, out, _, _ = self.run("-md 7 -t 200 -n", MOCK_I2C_BUSES="")
        self.check(rc not in (0, None) and "Failed to open I2C" in out, "fails when there is no bus", out)
        rc, out, _, _ = self.run("-md 7 -t 200 -n", MOCK_SENSOR="none")
        self.check(rc not in (0, None) and "No sensor found" in out, "fails when no sensor answers", out)
        for sensor in ("ov5647", "imx219", "imx477"):
            rc, out, _, log = self.run("-md 1 -t 200 -n -e 500 -g 16 -f 20", MOCK_SENSOR=sensor)
            writes = [l for l in log if l.startswith("I2C W ")]
            self.check(rc == 0 and "Found sensor " + sensor in out and len(writes) > 20,
                       "%s is detected and configured (%d register writes)" % (sensor, len(writes)), out)
            self.check(writes and writes[-1] == "I2C W 0100 00", sensor + ": streaming is stopped at the end", out)
            self.clean(out, sensor)

    def frame_number(self, path, offset=0):
        with open(path, "rb") as f:
            f.seek(offset)
            return struct.unpack("<I", f.read(4))[0]

    def test_files(self):
        self.section("Output files")
        rc, out, d, _ = self.run("-md 7 -t 800 -n -sr 3 -o {d}/o.%04d.raw -oY {d}/y.%04d.yuv -ts {d}/ts.csv "
                                 "-hd0 {d}/hd0.bin")
        raws = sorted(f for f in os.listdir(d) if f.startswith("o."))
        yuvs = sorted(f for f in os.listdir(d) if f.startswith("y."))
        self.check(rc == 0 and len(raws) >= 4 and len(yuvs) >= 4, "raw and YUV frames are captured (%d, %d)" %
                   (len(raws), len(yuvs)), out)
        self.clean(out, "capture")
        sizes = set(os.path.getsize(os.path.join(d, f)) for f in raws)
        self.check(sizes == {384000}, "every raw file holds one 640x480 10 bit frame (sizes %s)" % sorted(sizes))
        sizes = set(os.path.getsize(os.path.join(d, f)) for f in yuvs)
        self.check(sizes == {460800}, "every YUV file holds one 640x480 I420 frame (sizes %s)" % sorted(sizes))
        numbers = [int(f.split(".")[1]) for f in raws]
        self.check(numbers == list(range(1, 3 * len(raws), 3)), "-sr 3 saves frames 1, 4, 7, ... (%s...)" % numbers[:4])
        # the mock writes its frame counter into the first 4 bytes of every frame
        inside = [self.frame_number(os.path.join(d, f)) for f in raws]
        self.check(inside == numbers, "each file holds the frame its name says (%s...)" % inside[:4])
        rows = [l.split(",") for l in open(os.path.join(d, "ts.csv")).read().split()]
        self.check([int(r[1]) for r in rows] == numbers, "one timestamp per saved frame, with its number")
        pts = [int(r[2]) for r in rows]
        self.check(all(b - a == 3 * 33333 for a, b in zip(pts, pts[1:])) and pts[0] > 0,
                   "timestamps are 3 frame times apart (%s...)" % pts[:3])
        self.check(os.path.getsize(os.path.join(d, "hd0.bin")) == BRCM_HEADER_LEN, "-hd0 writes the BRCM header")

        rc, out, d, _ = self.run("-md 7 -t 400 -n -sr 1 -hd -o {d}/o.%04d.raw")
        raws = sorted(f for f in os.listdir(d) if f.startswith("o."))
        sizes = set(os.path.getsize(os.path.join(d, f)) for f in raws)
        self.check(rc == 0 and sizes == {384000 + BRCM_HEADER_LEN}, "-hd puts the header in front of every frame", out)
        if raws:
            first = open(os.path.join(d, raws[0]), "rb").read(4)
            self.check(first == b"BRCM" and self.frame_number(os.path.join(d, raws[0]), BRCM_HEADER_LEN) == 1,
                       "the frame follows the header")

        rc, out, d, _ = self.run("-md 7 -t 400 -n -sr 1 -emp -o {d}/o.%04d.raw")
        raws = [f for f in os.listdir(d) if f.startswith("o.")]
        self.check(rc == 0 and len(raws) >= 4 and all(os.path.getsize(os.path.join(d, f)) == 0 for f in raws),
                   "-emp creates empty files", out)

        rc, out, d, _ = self.run("-md 7 -t 300 -n -hdg {d}/hd.pgm -o {d}/o.%04d.raw")
        pgm = os.path.join(d, "hd.pgm")
        self.check(rc == 0 and os.path.exists(pgm) and open(pgm, "rb").read() == b"P5\n640 480\n255\n",
                   "-hdg writes a PGM header", out)

        for bits, size in ((8, 640 * 480), (10, 800 * 480), (12, 960 * 480), (16, 1280 * 480)):
            rc, out, d, _ = self.run("-md 7 -t 300 -n -sr 1 -b %d -o {d}/o.%%04d.raw" % bits)
            raws = [f for f in os.listdir(d) if f.startswith("o.")]
            sizes = set(os.path.getsize(os.path.join(d, f)) for f in raws)
            self.check(rc == 0 and sizes == {size}, "-b %d gives frames of %d bytes (got %s)" % (bits, size, sorted(sizes)),
                       out)
            self.clean(out, "-b %d" % bits)

    def test_shutdown(self):
        self.section("Shutdown (the mock hands the unused buffers back, empty)")
        for opts in ("-n", "", "-awb -P -PY", "-n -awb -P -PY"):
            rc, out, d, log = self.run("-md 7 -t 500 -sr 1 -o {d}/o.%04d.raw -oY {d}/y.%04d.yuv -ts {d}/ts.csv " + opts)
            summary = [l for l in log if l.startswith("SUMMARY")]
            flushed = int(re.search(r"flushed=(\d+)", summary[0]).group(1)) if summary else 0
            files = [f for f in os.listdir(d) if f[:2] in ("o.", "y.")]
            empty = [f for f in files if os.path.getsize(os.path.join(d, f)) == 0]
            self.check(rc == 0 and flushed > 0 and files and not empty,
                       "[%s] no file is written for the %d returned buffers (%d files, %d empty)" %
                       (opts, flushed, len(files), len(empty)), out)
            pts = [int(l.split(",")[2]) for l in open(os.path.join(d, "ts.csv")).read().split()]
            self.check(pts and all(p > 0 for p in pts), "[%s] every timestamp is a real one" % opts)
            self.clean(out, "[%s]" % opts)

    def test_metadata(self):
        self.section("Embedded metadata")
        rc, out, d, _ = self.run("-md 7 -t 500 -n -m -sr 1 -o {d}/o.%04d.raw", MOCK_METADATA="1", MOCK_FPS="10")
        self.check(rc == 0 and "Register 0x3000 = 0x11" in out and "Register 0x3001 = 0x22" in out,
                   "-m decodes the register values", out)
        sizes = set(os.path.getsize(os.path.join(d, f)) for f in os.listdir(d) if f.startswith("o."))
        self.check(sizes == {384000}, "metadata buffers are not saved as frames (sizes %s)" % sorted(sizes))
        self.clean(out, "metadata")

    def test_awb(self):
        self.section("Grey world AWB (expected gains: r %d/256, b %d/256)" % (AWB_R_GAIN, AWB_B_GAIN))
        for sensor, opts, order in (("ov5647", "", "GBRG"), ("ov5647", "-hf", "BGGR"), ("ov5647", "-vf", "RGGB"),
                                    ("ov5647", "-hf -vf", "GRBG"), ("imx219", "", "BGGR"), ("imx477", "", "RGGB")):
            rc, out, _, _ = self.run("-md 1 -t 500 -n -awb " + opts, MOCK_SENSOR=sensor)
            m = re.search(r"MOCK AWB gains r=(\d+)/256 b=(\d+)/256", out)
            r, b = (int(m.group(1)), int(m.group(2))) if m else (0, 0)
            good = abs(r - AWB_R_GAIN) < 0.05 * AWB_R_GAIN and abs(b - AWB_B_GAIN) < 0.05 * AWB_B_GAIN
            self.check(rc == 0 and good, "%s %s (%s): r %d/256, b %d/256" % (sensor, opts, order, r, b), out)

    def test_failures(self):
        self.section("Pipeline setup failures")
        points = ("create:vc.ril.rawcam", "create:vc.ril.isp", "create:vc.ril.video_render", "commit:rawcam:out:0",
                  "commit:isp:in:0", "commit:isp:out:0", "commit:render:in:0", "cenable:vc.ril.rawcam",
                  "cenable:vc.ril.isp", "cenable:vc.ril.video_render", "pool:rawcam:out:0", "pool:isp:in:0",
                  "pool:isp:out:0", "pool:render:in:0", "enable:isp:in:0", "enable:isp:out:0", "enable:render:in:0",
                  "enable:rawcam:out:0")
        for point in points:
            for opts in ("", "-awb -P -PY -sr 1 -o {d}/o.%04d.raw"):
                rc, out, _, _ = self.run("-md 7 -t 200 " + opts, timeout=20, MOCK_FAIL=point)
                ok = rc not in (0, None) and rc > 0 and not BAD_OUTPUT.search(out)
                self.check(ok, "%s%s: clean exit with an error status (%s)" %
                           (point, " with worker threads" if opts else "", rc), out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raspiraw", required=True, help="raspiraw linked with mock_mmal.c")
    ap.add_argument("--pixtest", required=True, help="the pixtest binary")
    ap.add_argument("only", nargs="*", help="run only these tests (get_pixel, hwtest, cmdline, i2c, files, "
                                             "shutdown, metadata, awb, failures)")
    args = ap.parse_args()

    base = "/dev/shm" if os.path.isdir("/dev/shm") else None
    work = tempfile.mkdtemp(prefix="raspiraw_test_", dir=base)
    t = Tests(os.path.abspath(args.raspiraw), os.path.abspath(args.pixtest), work)
    try:
        for name in ("get_pixel", "hwtest", "cmdline", "i2c", "files", "shutdown", "metadata", "awb", "failures"):
            if not args.only or name in args.only:
                getattr(t, "test_" + name)()
    finally:
        shutil.rmtree(work, ignore_errors=True)

    print("\n%s: %d checks, %d failed" % ("FAILED" if t.fails else "PASSED", t.count, t.fails))
    return 1 if t.fails else 0


if __name__ == "__main__":
    sys.exit(main())
