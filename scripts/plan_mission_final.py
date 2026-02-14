#!/usr/bin/env python3
"""Final path design: NE diagonal crossing ridge obliquely with higher AGL."""

import json
import math
import numpy as np
from PIL import Image

with open('src/fiber_nav_gazebo/terrain/terrain_data.json') as f:
    meta = json.load(f)
hm = np.array(Image.open('src/fiber_nav_gazebo/terrain/terrain_heightmap.png'), dtype=np.float64)
size_x, size_y = meta['size_meters']['x'], meta['size_meters']['y']
min_elev, elev_range = meta['min_elevation_msl'], meta['heightmap_range_m']
res = meta['resolution_px']
max_val = 65535.0 if hm.max() > 255 else 255.0

def h(x, y):
    px = (x + size_x/2) / size_x * (res-1)
    py = (size_y/2 - y) / size_y * (res-1)
    px = max(0.0, min(float(res-1), px))
    py = max(0.0, min(float(res-1), py))
    x0, y0 = int(px), int(py)
    x1, y1 = min(x0+1, res-1), min(y0+1, res-1)
    fx, fy = px-x0, py-y0
    v = hm[y0,x0]*(1-fx)*(1-fy) + hm[y0,x1]*fx*(1-fy) + hm[y1,x0]*(1-fx)*fy + hm[y1,x1]*fx*fy
    return min_elev + (v/max_val) * elev_range


def full_analysis(wps, name, target_agl=40.0, gs=18.0, rs=0.25):
    print(f"\n{'='*70}")
    print(f"Path: {name}")
    print(f"Target AGL: {target_agl}m, Speed: {gs}m/s, Rate slew: {rs}m/s")
    print(f"{'='*70}")

    total_dist = 0
    max_internal_grad = 0
    worst_agl = 999
    min_terrain = 999
    max_terrain = 0

    prev_heading = None

    for i in range(len(wps)-1):
        x0, y0 = wps[i]
        x1, y1 = wps[i+1]
        dx, dy = x1-x0, y1-y0
        seg_dist = math.sqrt(dx*dx + dy*dy)
        total_dist += seg_dist
        heading = math.degrees(math.atan2(dx, dy)) % 360

        d_heading = 0
        if prev_heading is not None:
            d_heading = heading - prev_heading
            if d_heading > 180: d_heading -= 360
            if d_heading < -180: d_heading += 360

        # Fine sample every 10m
        n = max(int(seg_dist/10), 2)
        prev_e = None
        seg_max_grad = 0
        seg_elevs = []

        for j in range(n+1):
            t = j/n
            x, y = x0+t*dx, y0+t*dy
            e = h(x, y)
            seg_elevs.append(e)
            min_terrain = min(min_terrain, e)
            max_terrain = max(max_terrain, e)

            if prev_e is not None:
                step_d = seg_dist/n
                g = abs(e - prev_e) / step_d
                if g > seg_max_grad:
                    seg_max_grad = g
                if g > max_internal_grad:
                    max_internal_grad = g
            prev_e = e

        # Simulate terrain tracking lag
        # Worst case: terrain rises while controller is still descending
        # At max_grad, the terrain change rate = grad * gs
        # Controller can adjust at rate_slew
        # Deficit accumulation = (grad * gs - rs) * time_at_grad
        # Minimum AGL = target_agl - accumulated_deficit
        # Estimate worst AGL conservatively:
        terrain_change_rate = seg_max_grad * gs  # m/s
        if terrain_change_rate > rs:
            # How long does terrain rise at this rate within this segment?
            # Assume worst case: entire segment at max gradient
            time_at_grad = min(seg_dist / gs, 20)  # cap at 20s
            deficit = (terrain_change_rate - rs) * time_at_grad
            # Feedforward helps: ff_correction = slope * lookahead * gain
            ff = min(seg_max_grad, 0.5) * min(3.0 * gs, 100) * 0.5
            deficit -= ff
            deficit = max(0, deficit)
            seg_worst_agl = target_agl - deficit
        else:
            seg_worst_agl = target_agl

        worst_agl = min(worst_agl, seg_worst_agl)

        e0, e1 = h(x0, y0), h(x1, y1)
        print(f"  Seg {i}→{i+1}: ({x0:.0f},{y0:.0f})→({x1:.0f},{y1:.0f})")
        print(f"    dist={seg_dist:.0f}m, elev={e0:.1f}→{e1:.1f}m MSL, "
              f"range={min(seg_elevs):.1f}-{max(seg_elevs):.1f}m")
        print(f"    hdg={heading:.1f}° (Δ={d_heading:+.1f}°), "
              f"max_grad={seg_max_grad:.4f}, "
              f"vz_need={seg_max_grad*gs:.2f}m/s, "
              f"est_min_agl={seg_worst_agl:.1f}m "
              f"{'OK' if seg_worst_agl > 15 else 'DANGER'}")

        prev_heading = heading

    print(f"\n  SUMMARY:")
    print(f"  Total distance: {total_dist:.0f}m ({total_dist/gs:.0f}s flight time)")
    print(f"  Terrain range: {min_terrain:.1f} - {max_terrain:.1f}m MSL ({max_terrain-min_terrain:.1f}m)")
    print(f"  Max gradient: {max_internal_grad:.4f} (Vz={max_internal_grad*gs:.2f}m/s)")
    print(f"  Estimated worst AGL: {worst_agl:.1f}m (min_agl=10m)")
    print(f"  Safety margin: {worst_agl - 10:.1f}m above min_agl")

    if worst_agl > 15:
        print(f"  VERDICT: SAFE ✓")
    elif worst_agl > 10:
        print(f"  VERDICT: MARGINAL — consider higher target_agl")
    else:
        print(f"  VERDICT: UNSAFE — increase target_agl or avoid steep segments")

    # Generate YAML
    xs = [wp[0] for wp in wps]
    ys = [wp[1] for wp in wps]
    print(f"\n  YAML waypoints:")
    print(f"    x: [{', '.join(f'{x:.1f}' for x in xs)}]")
    print(f"    y: [{', '.join(f'{y:.1f}' for y in ys)}]")


# ============================================================
# FINAL PATH: NE diagonal crossing ridge obliquely
# The ridge at y≈0 is steep near x=0 (gradient ~0.10) but softens
# at x>500 (gradient ~0.04). So we go east first, then cross.
# ============================================================

# Path: East then SE, crossing ridge at x≈600-800
# Higher AGL (40m) with rate_slew=0.25
path_final = [
    (0, 200),       # Start
    (200, 400),     # NE at 45°
    (400, 600),     # Continue NE
    (600, 400),     # Curve east-south (gentle 20° turn)
    (800, 100),     # SE through the ridge at x=800 (soft gradient area)
    (1000, -200),   # Continue SE into higher terrain
    (1100, -500),   # Continue south
    (1200, -900),   # Deep into southern plateau
    (1200, -1300),  # Straight south
]
full_analysis(path_final, "FINAL: NE then SE crossing ridge at x=800", target_agl=40, rs=0.25)

# Alternative: Pure NE that doesn't cross ridge but has terrain variation
path_ne_alt = [
    (0, 200),
    (100, 500),
    (250, 800),
    (400, 1100),
    (600, 1400),
    (800, 1700),
    (900, 2000),
    (1000, 2300),
]
full_analysis(path_ne_alt, "ALT: Pure NE (no ridge crossing)", target_agl=40, rs=0.25)

# Variation: Go south through ridge but at high angle to spread the climb
# heading ~150° (SSE) means crossing the N-S ridge at 60° angle
path_sse = [
    (0, 200),
    (150, 0),       # SE
    (350, -200),    # SSE
    (550, -450),    # SSE
    (750, -700),    # SSE
    (950, -950),    # SSE
    (1100, -1200),  # SSE
    (1200, -1500),  # South
]
full_analysis(path_sse, "ALT2: SSE crossing ridge at 150°", target_agl=40, rs=0.25)

# Variation: higher AGL=50m for maximum safety
full_analysis(path_final, "FINAL with AGL=50m", target_agl=50, rs=0.25)
full_analysis(path_sse, "SSE with AGL=50m", target_agl=50, rs=0.25)
