#!/usr/bin/env python3
"""
Hardware smoke test for raspiraw. Run it on the Pi, with the camera connected
and after camera_i2c has been run:

    tools/hwtest.py                 # test sensor mode 0
    tools/hwtest.py -md 7 --fps 90  # test another mode / check the frame rate
    tools/hwtest.py --preview       # also exercise the HDMI preview path

What it does:
  1. command line checks: bad options must be rejected (no camera needed)
  2. captures frames for a few seconds, with timestamps and the BRCM header
  3. checks frame count, timestamps (skipped frames, frame rate) and the header
  4. decodes a few raw frames and checks size and pixel statistics
  5. writes a small colour preview (preview.ppm, and preview.jpg if PIL is
     installed) for you to look at - this is the real "is the picture right" test
  6. captures again with -hd and -sr 5 to check per-frame headers and save rate

Other uses:
    tools/hwtest.py --check DIR     # only analyse an earlier capture in DIR
    tools/hwtest.py --selftest      # test this script on synthetic frames (no Pi needed)

Only the Python 3 standard library is needed. Exit status is 0 if nothing failed.
"""

import argparse
import glob
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile

BRCM_SIG = 0x4D435242
BRCM_HEADER_LEN = 32768
BRCM_MODE_OFFSET = 176 + 32  # brcm_camera_mode, after its name[32]

# VC_IMAGE_BAYER_ORDER_T / VC_IMAGE_BAYER_FORMAT_T values written into the header
BAYER_ORDERS = {0: "RGGB", 1: "GBRG", 2: "BGGR", 3: "GRBG"}
BAYER_BITS = {2: 8, 3: 10, 4: 12, 5: 14, 6: 16}

FRAME_GLOB = "out.*.raw"
OUR_FILES = (FRAME_GLOB, "tstamps.csv", "header.bin", "preview.ppm", "preview.jpg", "raspiraw*.log")


class Report:
    def __init__(self):
        self.fails = 0
        self.warns = 0

    def ok(self, msg):
        print("  ok    " + msg)

    def warn(self, msg):
        self.warns += 1
        print("  WARN  " + msg)

    def fail(self, msg):
        self.fails += 1
        print("  FAIL  " + msg)

    def check(self, cond, msg):
        if cond:
            self.ok(msg)
        else:
            self.fail(msg)
        return cond

    def section(self, title):
        print("\n== " + title)


def align_up(value, to):
    return (value + to - 1) // to * to


# ---------------------------------------------------------------------------
# Raw frame decoding
# ---------------------------------------------------------------------------

def packed_row_bytes(width, bits):
    """Bytes needed for one row of packed Bayer data."""
    return {8: width, 10: (width * 5 + 3) // 4, 12: (width * 3 + 1) // 2, 16: width * 2}.get(bits)


def get_pixel(data, stride, x, y, bits):
    row = y * stride
    if bits == 8:
        return data[row + x]
    if bits == 10:  # 4 pixels in 5 bytes: 4 x MSBs, then 4 x 2 LSBs
        group = row + (x // 4) * 5
        return (data[group + (x & 3)] << 2) | ((data[group + 4] >> (2 * (x & 3))) & 3)
    if bits == 12:  # 2 pixels in 3 bytes: 2 x MSBs, then 2 x 4 LSBs
        group = row + (x // 2) * 3
        return (data[group + (x & 1)] << 4) | ((data[group + 2] >> (4 * (x & 1))) & 0xF)
    if bits == 16:
        return data[row + 2 * x] | (data[row + 2 * x + 1] << 8)
    raise ValueError("unsupported bit depth %d" % bits)


def frame_geometry(size, width, height, bits):
    """Returns (stride, rows) of a frame file, or None if the size makes no sense."""
    need = packed_row_bytes(width, bits)
    if not need:
        return None
    # raspiraw asks for the height aligned up to 16; the stride is the GPU's choice
    for rows in (align_up(height, 16), height):
        if size % rows == 0 and need <= size // rows <= need + 64:
            return size // rows, rows
    return None


def sample_cells(data, stride, width, height, bits, order, max_cols=640):
    """
    Samples 2x2 Bayer cells on a regular grid.
    Returns (cols, rows, [(r, g, b), ...]) in raw sensor units.
    """
    step = 2 * max(1, -(-width // (2 * max_cols)))
    idx = {c: [i for i, ch in enumerate(order) if ch == c] for c in "RGB"}
    cells = []
    xs = range(0, width - 1, step)
    ys = range(0, height - 1, step)
    for y in ys:
        for x in xs:
            v = (get_pixel(data, stride, x, y, bits), get_pixel(data, stride, x + 1, y, bits),
                 get_pixel(data, stride, x, y + 1, bits), get_pixel(data, stride, x + 1, y + 1, bits))
            cells.append((v[idx["R"][0]], (v[idx["G"][0]] + v[idx["G"][1]]) / 2.0, v[idx["B"][0]]))
    return len(xs), len(ys), cells


def write_preview(path_base, cols, rows, cells):
    """Grey-world white balance + stretch, written as PPM (and JPEG when PIL is there)."""
    means = [max(sum(c[i] for c in cells) / len(cells), 1e-6) for i in range(3)]
    gains = [means[1] / m for m in means]
    balanced = [[c[i] * gains[i] for i in range(3)] for c in cells]
    levels = sorted(max(p) for p in balanced)
    white = max(levels[int(len(levels) * 0.99)], 1e-6)
    black = min(min(p) for p in balanced)
    scale = 255.0 / max(white - black, 1e-6)
    rgb = bytearray()
    for p in balanced:
        for v in p:
            v = (v - black) * scale
            v = 255.0 * (max(v, 0.0) / 255.0) ** (1 / 2.2)  # display gamma
            rgb.append(min(255, int(v)))

    written = [path_base + ".ppm"]
    with open(written[0], "wb") as f:
        f.write(b"P6\n%d %d\n255\n" % (cols, rows))
        f.write(rgb)
    try:
        from PIL import Image
        Image.frombytes("RGB", (cols, rows), bytes(rgb)).save(path_base + ".jpg", quality=90)
        written.append(path_base + ".jpg")
    except ImportError:
        pass
    return written


# ---------------------------------------------------------------------------
# Checks on a capture directory
# ---------------------------------------------------------------------------

def read_brcm_header(path, rep):
    try:
        with open(path, "rb") as f:
            hdr = f.read()
    except OSError as e:
        rep.fail("cannot read BRCM header %s (%s)" % (path, e))
        return None
    return parse_brcm_header(hdr, rep, os.path.basename(path))


def parse_brcm_header(hdr, rep, name):
    if not rep.check(len(hdr) >= BRCM_HEADER_LEN, "%s is %d bytes long" % (name, BRCM_HEADER_LEN)):
        return None
    sig, version = struct.unpack_from("<II", hdr, 0)
    if not rep.check(sig == BRCM_SIG, "%s starts with the BRCM signature" % name):
        return None
    width, height = struct.unpack_from("<HH", hdr, BRCM_MODE_OFFSET)
    order, fmt = struct.unpack_from("<BB", hdr, BRCM_MODE_OFFSET + 8 + 24 + 4)
    info = {"width": width, "height": height, "order": BAYER_ORDERS.get(order), "bits": BAYER_BITS.get(fmt)}
    good = 16 <= width <= 8192 and 16 <= height <= 8192 and info["order"] and info["bits"]
    rep.check(good, "%s describes %dx%d, bayer order %s, %s bit (version %d)"
              % (name, width, height, info["order"], info["bits"], version))
    return info if good else None


def frame_index(path):
    m = re.search(r"out\.(\d+)\.raw$", path)
    return int(m.group(1)) if m else -1


def list_frames(directory):
    return sorted(glob.glob(os.path.join(directory, FRAME_GLOB)), key=frame_index)


def check_timestamps(path, frames, fps, rep):
    try:
        with open(path) as f:
            rows = [line.strip().split(",") for line in f if line.strip()]
    except OSError as e:
        rep.fail("cannot read timestamps %s (%s)" % (path, e))
        return
    try:
        idx = [int(r[1]) for r in rows]
        pts = [int(r[2]) for r in rows]
    except (IndexError, ValueError):
        rep.fail("timestamps file is not in '<delta>,<index>,<pts>' format")
        return

    rep.check(len(rows) == len(frames), "one timestamp per frame file (%d timestamps, %d files)"
              % (len(rows), len(frames)))
    rep.check(idx == [frame_index(f) for f in frames][:len(idx)], "timestamp indices match the file names")
    if len(pts) < 3:
        rep.warn("too few frames to analyse timing")
        return

    deltas = [b - a for a, b in zip(pts, pts[1:])]
    if not rep.check(all(d > 0 for d in deltas), "timestamps are strictly increasing"):
        return
    median = sorted(deltas)[len(deltas) // 2]
    skips = sum(1 for d in deltas if d > 1.5 * median)
    print("        frame interval %d us (%.2f fps), capture spans %.2f s"
          % (median, 1e6 / median, (pts[-1] - pts[0]) / 1e6))
    if skips:
        rep.warn("%d of %d frame intervals are longer than 1.5x the median (skipped frames)"
                 % (skips, len(deltas)))
    else:
        rep.ok("no skipped frames")
    if fps:
        rep.check(abs(1e6 / median - fps) <= 0.05 * fps, "frame rate is within 5%% of the requested %g fps" % fps)


def check_frame(path, info, rep, preview_base=None):
    """Checks one raw frame. Returns its raw bytes, or None."""
    name = os.path.basename(path)
    with open(path, "rb") as f:
        data = f.read()

    geom = frame_geometry(len(data), info["width"], info["height"], info["bits"])
    if not geom:
        need = packed_row_bytes(info["width"], info["bits"])
        if need is None:
            rep.warn("%s: %d bit frames are not decoded by this script (%d bytes)" % (name, info["bits"], len(data)))
        else:
            rep.fail("%s: size %d does not fit %dx%d at %d bit (at least %d bytes per row)"
                     % (name, len(data), info["width"], info["height"], info["bits"], need))
        return None
    stride, rows = geom
    rep.ok("%s: %d bytes = %d rows x stride %d" % (name, len(data), rows, stride))

    cols, nrows, cells = sample_cells(data, stride, info["width"], info["height"], info["bits"], info["order"])
    full = (1 << info["bits"]) - 1
    flat = [v for c in cells for v in c]
    lo, hi = min(flat), max(flat)
    mean = sum(flat) / len(flat)
    std = (sum((v - mean) ** 2 for v in flat) / len(flat)) ** 0.5
    r, g, b = (sum(c[i] for c in cells) / len(cells) for i in range(3))
    print("        min %d  max %d  mean %.1f  std %.1f  (full scale %d)   R %.1f  G %.1f  B %.1f"
          % (lo, hi, mean, std, full, r, g, b))

    rep.check(hi > lo, "%s: image is not a constant value" % name)
    if hi == 0:
        rep.fail("%s: image is all zero - no data arrived from the sensor" % name)
    elif std < 0.002 * full:
        rep.warn("%s: almost no contrast - lens cap on, or completely dark/defocused scene?" % name)
    if mean > 0.97 * full:
        rep.warn("%s: image is almost fully saturated - reduce exposure/gain" % name)
    saturated = sum(1 for v in flat if v >= full) / len(flat)
    if saturated > 0.2 and mean <= 0.97 * full:
        rep.warn("%s: %.0f%% of the samples are saturated" % (name, 100 * saturated))

    if preview_base and hi > lo:
        for p in write_preview(preview_base, cols, nrows, cells):
            print("        preview written to %s  <-- look at this picture" % p)
    return data


def check_capture(directory, rep, fps=None, expect_min_frames=1):
    """Analyses the result of the main capture run in directory. Returns the frame size or None."""
    frames = list_frames(directory)
    if not rep.check(len(frames) >= expect_min_frames,
                     "%d frame files captured (expected at least %d)" % (len(frames), expect_min_frames)):
        return None
    indices = [frame_index(f) for f in frames]
    rep.check(indices == list(range(1, len(frames) + 1)), "frame files are numbered 1..%d without gaps" % len(frames))
    sizes = set(os.path.getsize(f) for f in frames)
    rep.check(len(sizes) == 1, "all frame files have the same size")

    rep.section("Timestamps")
    check_timestamps(os.path.join(directory, "tstamps.csv"), frames, fps, rep)

    rep.section("BRCM header")
    info = read_brcm_header(os.path.join(directory, "header.bin"), rep)
    if not info:
        return None

    rep.section("Frame contents")
    picks = sorted(set([0, len(frames) // 2, len(frames) - 1]))
    middle = len(frames) // 2
    datas = []
    for i in picks:
        base = os.path.join(directory, "preview") if i == middle else None
        datas.append(check_frame(frames[i], info, rep, base))
    datas = [d for d in datas if d is not None]
    if len(datas) >= 2:
        # Sensor noise makes real frames differ; identical ones mean a buffer is being reused
        rep.check(len(set(datas)) == len(datas), "sampled frames are not byte-identical to each other")
    return min(sizes)


def check_header_run(directory, frame_size, saverate, rep):
    """Second run: every file has the BRCM header prepended, and only every Nth frame is saved."""
    frames = list_frames(directory)
    if not rep.check(len(frames) >= 1, "%d frame files captured" % len(frames)):
        return
    indices = [frame_index(f) for f in frames]
    expected = list(range(1, 1 + saverate * len(frames), saverate))
    rep.check(indices == expected, "with -sr %d the files are numbered %s..." % (saverate, expected[:4]))
    sizes = set(os.path.getsize(f) for f in frames)
    rep.check(sizes == {frame_size + BRCM_HEADER_LEN},
              "with -hd each file is frame + header = %d bytes (got %s)"
              % (frame_size + BRCM_HEADER_LEN, sorted(sizes)))
    with open(frames[0], "rb") as f:
        parse_brcm_header(f.read(BRCM_HEADER_LEN), rep, os.path.basename(frames[0]))


# ---------------------------------------------------------------------------
# Running raspiraw
# ---------------------------------------------------------------------------

def find_raspiraw(explicit):
    if explicit:
        return explicit
    local = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "raspiraw")
    if os.path.isfile(local):
        return local
    return shutil.which("raspiraw")


def run(cmd, log_path=None, timeout=60):
    """Returns (exit status, combined output). Exit status is None on timeout."""
    print("  $ " + " ".join(cmd))
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
        rc, out = proc.returncode, proc.stdout.decode(errors="replace")
    except subprocess.TimeoutExpired as e:
        rc, out = None, (e.stdout or b"").decode(errors="replace") + "\n[timed out]"
    if log_path:
        with open(log_path, "w") as f:
            f.write(out)
    return rc, out


def clean_dir(directory):
    """Removes only the files this script creates."""
    os.makedirs(directory, exist_ok=True)
    for pattern in OUR_FILES:
        for path in glob.glob(os.path.join(directory, pattern)):
            os.remove(path)


def explain_failure(out, rep):
    hints = (("Failed to open I2C", "I2C device could not be opened - is i2c enabled (dtparam=i2c_vc=on)?"),
             ("No sensor found", "no sensor answered on I2C - run camera_i2c first, check the cable"),
             ("Failed to create rawcam", "rawcam component missing - legacy camera stack / start_x.elf needed"))
    for needle, hint in hints:
        if needle in out:
            rep.fail(hint)
    tail = [line for line in out.strip().splitlines() if line.strip()][-8:]
    print("        last output lines:")
    for line in tail:
        print("        | " + line)


def test_cmdline(raspiraw, rep):
    rep.section("Command line checks")
    cases = (
        (["-md"], "option without its value"),
        (["-zz"], "unknown option"),
        (["-sr", "0"], "save rate 0"),
        (["-c", "5"], "camera number other than 0/1"),
        (["-o", "bad_%s.raw"], "output pattern with a format other than %d"),
        (["stray"], "argument that is not an option"),
    )
    for args, what in cases:
        rc, out = run([raspiraw] + args, timeout=20)
        rep.check(rc not in (0, None) and rc >= 0, "%s is rejected (exit status %s)" % (what, rc))
    rc, out = run([raspiraw, "-?"], timeout=20)
    rep.check("--mode" in out or "-mode" in out, "help text is printed")


def test_capture(raspiraw, args, rep):
    directory = args.dir
    clean_dir(directory)

    rep.section("Capture: mode %d for %d ms" % (args.mode, args.time))
    cmd = [raspiraw, "-md", str(args.mode), "-t", str(args.time), "-sr", "1",
           "-ts", os.path.join(directory, "tstamps.csv"), "-hd0", os.path.join(directory, "header.bin"),
           "-o", os.path.join(directory, "out.%04d.raw")]
    if args.fps:
        cmd += ["--fps", str(args.fps)]
    if not args.preview:
        cmd += ["-n"]
    cmd += args.extra
    rc, out = run(cmd, os.path.join(directory, "raspiraw.log"), timeout=args.time / 1000.0 + 30)
    if not rep.check(rc == 0, "raspiraw exits with status 0 (got %s)" % rc):
        explain_failure(out, rep)
        if rc is None or not list_frames(directory):
            return

    frame_size = check_capture(directory, rep, args.fps)
    if frame_size is None:
        return

    # Keep the evidence of the first run, then capture again with other options
    rep.section("Capture with -hd and -sr 5")
    keep = os.path.join(directory, "run2")
    clean_dir(keep)
    cmd = [raspiraw, "-md", str(args.mode), "-t", "2000", "-sr", "5", "-hd", "-n",
           "-o", os.path.join(keep, "out.%04d.raw")] + args.extra
    rc, out = run(cmd, os.path.join(keep, "raspiraw.log"), timeout=32)
    if not rep.check(rc == 0, "raspiraw exits with status 0 (got %s)" % rc):
        explain_failure(out, rep)
    check_header_run(keep, frame_size, 5, rep)


# ---------------------------------------------------------------------------
# Self test on synthetic data (runs anywhere)
# ---------------------------------------------------------------------------

def synth_header(width, height, order, bits):
    hdr = bytearray(BRCM_HEADER_LEN)
    struct.pack_into("<II", hdr, 0, BRCM_SIG, 111)
    struct.pack_into("<HH", hdr, BRCM_MODE_OFFSET, width, height)
    struct.pack_into("<BB", hdr, BRCM_MODE_OFFSET + 36,
                     [k for k, v in BAYER_ORDERS.items() if v == order][0],
                     [k for k, v in BAYER_BITS.items() if v == bits][0])
    return bytes(hdr)


def synth_frame(width, height, bits, seed, constant=None):
    """Packs a diagonal gradient the same way the GPU does."""
    need = packed_row_bytes(width, bits)
    stride = align_up(need, 32)
    out = bytearray(stride * align_up(height, 16))
    full = (1 << bits) - 1
    for y in range(height):
        vals = [constant if constant is not None else ((x * 7 + y * 5 + seed * 3) * 4) % (full + 1)
                for x in range(width)]
        row = y * stride
        if bits == 8:
            out[row:row + width] = bytes(vals)
        elif bits == 10:
            for g in range(0, width, 4):
                grp = (vals[g:g + 4] + [0, 0, 0])[:4]
                base = row + g // 4 * 5
                for i, v in enumerate(grp):
                    out[base + i] = v >> 2
                    out[base + 4] |= (v & 3) << (2 * i)
        elif bits == 12:
            for g in range(0, width, 2):
                grp = (vals[g:g + 2] + [0])[:2]
                base = row + g // 2 * 3
                for i, v in enumerate(grp):
                    out[base + i] = v >> 4
                    out[base + 2] |= (v & 0xF) << (4 * i)
        elif bits == 16:
            for x, v in enumerate(vals):
                struct.pack_into("<H", out, row + 2 * x, v)
    return bytes(out), stride


def synth_capture(directory, width, height, order, bits, nframes, constant=None, truncate=0):
    with open(os.path.join(directory, "header.bin"), "wb") as f:
        f.write(synth_header(width, height, order, bits))
    with open(os.path.join(directory, "tstamps.csv"), "w") as f:
        for i in range(nframes):
            data, _ = synth_frame(width, height, bits, i, constant)
            with open(os.path.join(directory, "out.%04d.raw" % (i + 1)), "wb") as raw:
                raw.write(data[:len(data) - truncate])
            pts = 1000000 + i * 33333
            f.write("%s,%d,%d\n" % ("" if i == 0 else 33333, i + 1, pts))


def selftest():
    problems = 0

    # 1. packing/unpacking round trip for every supported depth
    for bits in (8, 10, 12, 16):
        w, h = 70, 20
        data, stride = synth_frame(w, h, bits, 1)
        full = (1 << bits) - 1
        bad = sum(1 for y in range(h) for x in range(w)
                  if get_pixel(data, stride, x, y, bits) != ((x * 7 + y * 5 + 3) * 4) % (full + 1))
        geom = frame_geometry(len(data), w, h, bits)
        print("round trip %2d bit: %d wrong pixels, geometry %s" % (bits, bad, geom))
        problems += bad != 0 or geom != (stride, align_up(h, 16))

    # 2. a good capture passes, broken ones are caught
    scenarios = (("good capture", {}, 0), ("constant image", {"constant": 0}, 1), ("truncated frames", {"truncate": 100}, 1))
    for title, kwargs, want_fail in scenarios:
        with tempfile.TemporaryDirectory() as d:
            print("\n--- scenario: %s (should %s)" % (title, "FAIL" if want_fail else "pass"))
            synth_capture(d, 640, 480, "GBRG", 10, 5, **kwargs)
            rep = Report()
            check_capture(d, rep, fps=30)
            got_fail = rep.fails > 0
            if not want_fail:
                got_fail = got_fail or not os.path.exists(os.path.join(d, "preview.ppm"))
            problems += got_fail != bool(want_fail)

    print("\nselftest: %s" % ("PASSED" if not problems else "%d PROBLEMS" % problems))
    return 1 if problems else 0


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Hardware smoke test for raspiraw",
                                 epilog="Arguments after -- are passed to raspiraw unchanged, e.g. -- -g 64 -eus 10000")
    ap.add_argument("-md", "--mode", type=int, default=0, help="sensor mode to test (default 0)")
    ap.add_argument("-t", "--time", type=int, default=3000, help="capture time in ms (default 3000)")
    ap.add_argument("--fps", type=float, help="request this frame rate and check that it is achieved")
    ap.add_argument("--preview", action="store_true", help="also send the stream to the display")
    ap.add_argument("--dir", default="/dev/shm/raspiraw_test", help="working directory (default: ramdisk)")
    ap.add_argument("--raspiraw", help="path to the raspiraw binary (default: next to tools/, then PATH)")
    ap.add_argument("--check", metavar="DIR", help="only analyse an existing capture directory")
    ap.add_argument("--selftest", action="store_true", help="test this script on synthetic frames")
    ap.add_argument("extra", nargs="*", help=argparse.SUPPRESS)
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    rep = Report()
    if args.check:
        rep.section("Checking capture in " + args.check)
        check_capture(args.check, rep, args.fps)
    else:
        raspiraw = find_raspiraw(args.raspiraw)
        if not raspiraw or not os.path.isfile(raspiraw):
            print("raspiraw binary not found - build it with 'make' or pass --raspiraw PATH")
            return 2
        test_cmdline(raspiraw, rep)
        test_capture(raspiraw, args, rep)

    print("\n%s: %d failed, %d warnings" % ("FAILED" if rep.fails else "PASSED", rep.fails, rep.warns))
    return 1 if rep.fails else 0


if __name__ == "__main__":
    sys.exit(main())
