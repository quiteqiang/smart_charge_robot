#!/usr/bin/env python3
"""Generate the smart_charge_robot demo site map (PGM + YAML).

Pure standard library so it can run on the host without ROS.

World layout (meters, origin at map corner, x right / y up, 20x20 m):
  - perimeter walls (0.2 m thick)
  - central obstacle zone (boxes + pillar) for global planning / avoidance demos
  - start area near (1, 1)
  - task zone 1 near (15, 4), task zone 2 near (13, 15)
  - charging dock against the west wall at (0.6, 17.0), facing +x
    pre-dock pose at (2.2, 17.0)
"""
import os
import struct

RES = 0.05          # m / pixel
W_M = 20.0
H_M = 20.0
W = int(W_M / RES)  # 400 px
H = int(H_M / RES)  # 400 px

FREE = 254
OCC = 0


def px(x_m: float, y_m: float) -> tuple[int, int]:
    """world -> pixel indices (y flipped: PGM rows go top->bottom), clamped."""
    c = min(max(int(x_m / RES), 0), W - 1)
    r = min(max(int((H_M - y_m) / RES), 0), H - 1)
    return c, r


def fill_rect(grid: bytearray, x0: float, y0: float, x1: float, y1: float, val: int) -> None:
    cx0, cy1 = px(x0, y0)   # top row
    cx1, cy0 = px(x1, y1)   # bottom row
    for r in range(cy0, cy1 + 1):
        row = r * W
        for c in range(cx0, cx1 + 1):
            grid[row + c] = val


def fill_circle(grid: bytearray, cx: float, cy: float, r: float, val: int) -> None:
    cxc, cyc = px(cx, cy)
    rp = int(r / RES)
    for dr in range(-rp, rp + 1):
        for dc in range(-rp, rp + 1):
            if dc * dc + dr * dr <= rp * rp:
                r_, c_ = cyc + dr, cxc + dc
                if 0 <= r_ < H and 0 <= c_ < W:
                    grid[r_ * W + c_] = val


def main() -> None:
    grid = bytearray([FREE]) * (W * H)

    # perimeter walls
    fill_rect(grid, 0.0, 0.0, W_M, 0.2, OCC)
    fill_rect(grid, 0.0, H_M - 0.2, W_M, H_M, OCC)
    fill_rect(grid, 0.0, 0.0, 0.2, H_M, OCC)
    fill_rect(grid, W_M - 0.2, 0.0, W_M, H_M, OCC)

    # central obstacle zone
    fill_rect(grid, 6.0, 2.0, 7.0, 6.0, OCC)     # box A
    fill_rect(grid, 9.0, 10.0, 10.0, 13.0, OCC)  # box B
    fill_rect(grid, 4.0, 12.0, 5.0, 14.0, OCC)   # box C
    fill_circle(grid, 12.0, 8.0, 0.5, OCC)       # pillar D

    out_dir = os.path.join(os.path.dirname(__file__), "..", "maps")
    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    pgm_path = os.path.join(out_dir, "site.pgm")
    with open(pgm_path, "wb") as f:
        f.write(b"P5\n")
        f.write(f"{W} {H}\n255\n".encode())
        f.write(bytes(grid))

    yaml_path = os.path.join(out_dir, "site.yaml")
    with open(yaml_path, "w") as f:
        f.write(
            "image: site.pgm\n"
            f"resolution: {RES}\n"
            f"origin: [0.0, 0.0, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )
    print(f"wrote {pgm_path} ({W}x{H}) and {yaml_path}")


if __name__ == "__main__":
    main()
