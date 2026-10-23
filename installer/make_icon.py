"""Draws the Puppet Pin icon (a yellow pin stuck in a bent mesh strip) and writes a
multi-size .ico (PNG entries). Usage: make_icon.py out.ico [preview.png]"""
import math
import struct
import sys
import zlib


def png(size, rgba):
    raw = b"".join(b"\x00" + bytes(rgba[y * size * 4:(y + 1) * size * 4]) for y in range(size))

    def chunk(kind, data):
        c = kind + data
        return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 6, 0, 0, 0)) +
            chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def render(size):
    s = size / 256.0
    out = bytearray(size * size * 4)
    ss = 3  # supersampling per axis
    for y in range(size):
        for x in range(size):
            acc = [0.0, 0.0, 0.0, 0.0]
            for j in range(ss):
                for i in range(ss):
                    px = (x + (i + 0.5) / ss) / s
                    py = (y + (j + 0.5) / ss) / s
                    col = None
                    # rounded square background
                    rx, ry = max(abs(px - 128) - 96, 0), max(abs(py - 128) - 96, 0)
                    if math.hypot(rx, ry) <= 24:
                        col = (0.13, 0.14, 0.17, 1.0)
                        # bent strip ("mesh") from lower left curving to the right
                        a = math.atan2(py - 300, px - 40)
                        r = math.hypot(px - 40, py - 300)
                        if 150 < r < 200 and -1.35 < a < -0.25:
                            band = (r - 150) / 50
                            u = (a + 1.35) / 1.1
                            c1 = (0.27 + 0.3 * u, 0.42, 0.85 - 0.25 * u)
                            line = (abs((u * 6) % 1 - 0.5) > 0.45) or abs(band - 0.5) < 0.05
                            col = (1.0, 0.82, 0.3, 1.0) if line else (*c1, 1.0)
                        # needle
                        if seg_dist(px, py, 150, 110, 112, 168) < 5:
                            col = (0.85, 0.85, 0.9, 1.0)
                        # pin head with outline
                        d = math.hypot(px - 160, py - 92)
                        if d < 38:
                            col = (0.05, 0.05, 0.05, 1.0)
                        if d < 32:
                            col = (1.0, 0.8, 0.1, 1.0)
                            if math.hypot(px - 150, py - 82) < 9:
                                col = (1.0, 0.95, 0.7, 1.0)
                    if col:
                        for k in range(3):
                            acc[k] += col[k] * col[3]
                        acc[3] += col[3]
            n = ss * ss
            o = (y * size + x) * 4
            a = acc[3] / n
            for k in range(3):
                out[o + k] = int(round(255 * min(1.0, (acc[k] / acc[3]) if acc[3] > 0 else 0)))
            out[o + 3] = int(round(255 * a))
    return out


def main():
    sizes = [16, 32, 48, 256]
    images = [png(sz, render(sz)) for sz in sizes]
    header = struct.pack("<HHH", 0, 1, len(sizes))
    offset = 6 + 16 * len(sizes)
    entries = b""
    for sz, data in zip(sizes, images):
        entries += struct.pack("<BBBBHHII", sz % 256, sz % 256, 0, 0, 1, 32, len(data), offset)
        offset += len(data)
    with open(sys.argv[1], "wb") as f:
        f.write(header + entries + b"".join(images))
    if len(sys.argv) > 2:
        with open(sys.argv[2], "wb") as f:
            f.write(images[-1])
    print("icon written")


main()
