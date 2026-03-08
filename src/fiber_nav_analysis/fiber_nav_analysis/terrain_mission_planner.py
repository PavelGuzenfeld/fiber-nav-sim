#!/usr/bin/env python3
"""TERCOM-Aware Mission Planner + Foxglove GeoJSON Visualization.

Plans routes through high-discriminability corridors, respects quadtailsitter
flight constraints, and outputs mission YAML + Foxglove map layers.

Two modes:
  - CLI mode: generate mission YAML + GeoJSON files + text report
  - ROS publisher mode (--publish): publishes GeoJSON layers to Foxglove

Usage:
    # From precomputed road network
    python3 -m fiber_nav_analysis.terrain_mission_planner \
        --terrain-data src/fiber_nav_gazebo/terrain/terrain_data.json \
        --road-network road_network_output/road_network.yaml \
        --start 0,0 --goal 800,800 --output-dir ./mission_output

    # Recompute road network on-the-fly
    python3 -m fiber_nav_analysis.terrain_mission_planner \
        --terrain-data src/fiber_nav_gazebo/terrain/terrain_data.json \
        --start 0,0 --goal 800,800 --output-dir ./mission_output

    # Publish to Foxglove
    python3 -m fiber_nav_analysis.terrain_mission_planner \
        --terrain-data ... --road-network ... \
        --start 0,0 --goal 800,800 --publish
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import yaml

from .terrain_road_network import (
    TerrainMap,
    altitude_scaled_params,
    build_road_graph,
    compute_discriminability_grid,
    extract_corridors,
)

# ---------------------------------------------------------------------------
# Vehicle constraints (quadtailsitter 4251)
# ---------------------------------------------------------------------------

MAX_COURSE_CHANGE_DEG = 30.0       # max instantaneous course change
FW_NAV_TURN_RATE_DPS = 1.5         # deg/s during FW_NAVIGATE
FW_CRUISE_SPEED_MS = 18.0          # FW_AIRSPD_TRIM
MIN_TURN_RADIUS_M = 200.0          # v / omega ≈ 18 / 0.087
TECS_MAX_CLIMB_MS = 6.0            # FW_T_CLMB_MAX
TECS_MAX_SINK_MS = 3.0             # FW_T_SINK_MAX
MAX_TERRAIN_GRADIENT = TECS_MAX_SINK_MS / FW_CRUISE_SPEED_MS  # ~0.167
MC_FW_ALT_LOSS_M = 50.0            # transition altitude loss
SPOOL_CAPACITY_M = 7500.0          # cable spool capacity
SPOOL_WARN_FRACTION = 0.80         # warn at 80% = 6000m one-way
FW_ACCEPT_RADIUS_M = 80.0          # default acceptance radius

CANDIDATE_ALTITUDES = [40.0, 60.0, 80.0, 100.0, 150.0]

# Recovery zone thresholds
RECOVERY_GREEN_DIST_M = 0.0        # on corridor
RECOVERY_YELLOW_DIST_M = 200.0     # within 200m of corridor


# ---------------------------------------------------------------------------
# Route optimization: Dijkstra on road network
# ---------------------------------------------------------------------------

def plan_route(
    terrain: TerrainMap,
    best_disc: np.ndarray,
    stride: int,
    start_enu: tuple[float, float],
    goal_enu: tuple[float, float],
    road_network_path: str | None = None,
    disc_threshold: float = 0.4,
    altitude_agl: float | None = None,
) -> dict:
    """Plan a TERCOM-optimal route from start to goal.

    If road_network_path is provided, loads precomputed corridor data.
    Otherwise computes corridors from discriminability grid.

    Returns dict with keys: optimal_route, corridors_enu, best_disc, stride.
    """
    if road_network_path:
        return _plan_from_yaml(
            terrain, best_disc, stride, start_enu, goal_enu, road_network_path)

    # Compute corridors from disc grid
    n_headings = 8
    headings_rad = np.linspace(0, np.pi, n_headings, endpoint=False)
    corridors = extract_corridors(
        best_disc, np.argmax(
            compute_discriminability_grid(
                terrain, stride=stride, altitude_agl=altitude_agl)[2],
            axis=0) if best_disc is None else np.zeros_like(best_disc, dtype=int),
        headings_rad, threshold=disc_threshold)

    result = build_road_graph(
        corridors, best_disc, terrain, stride=stride,
        start_enu=start_enu, goal_enu=goal_enu)

    return result


def _plan_from_yaml(
    terrain: TerrainMap,
    best_disc: np.ndarray,
    stride: int,
    start_enu: tuple[float, float],
    goal_enu: tuple[float, float],
    yaml_path: str,
) -> dict:
    """Rebuild networkx graph from saved road_network.yaml and route through it."""
    import networkx as nx

    with open(yaml_path) as f:
        # road_network.yaml may contain numpy scalar tags from serialization
        data = yaml.unsafe_load(f)

    rn = data['road_network']
    corridors_raw = rn.get('corridors', [])

    # Rebuild corridors in the format build_road_graph expects
    # We need to convert ENU waypoints back to pixel coordinates
    half_x = terrain.size_x / 2.0
    half_y = terrain.size_y / 2.0
    res = terrain.res

    def enu_to_grid_px(x_enu, y_enu):
        """ENU -> grid-pixel (row, col) at given stride."""
        px_col = (x_enu + half_x) / terrain.size_x * (res - 1)
        px_row = (half_y - y_enu) / terrain.size_y * (res - 1)
        return px_row / stride, px_col / stride

    corridors_for_graph = []
    corridors_enu = []
    for corr in corridors_raw:
        xs = corr['waypoints']['x']
        ys = corr['waypoints']['y']
        points_px = []
        for x, y in zip(xs, ys):
            r, c = enu_to_grid_px(x, y)
            points_px.append([r, c])

        corridors_for_graph.append({
            'points_px': points_px,
            'mean_disc': corr['mean_disc'],
            'best_heading_deg': corr['best_heading_deg'],
            'length_px': corr['length_m'] / (stride * terrain.mpp),
        })
        corridors_enu.append(corr)

    result = build_road_graph(
        corridors_for_graph, best_disc, terrain, stride=stride,
        start_enu=start_enu, goal_enu=goal_enu)

    # Preserve the original corridor ENU data
    result['corridors_enu'] = corridors_enu
    return result


# ---------------------------------------------------------------------------
# RDP simplification for route waypoints
# ---------------------------------------------------------------------------

def _rdp_simplify_route(points: list[tuple[float, float]],
                        tolerance: float) -> list[tuple[float, float]]:
    """Ramer-Douglas-Peucker simplification on a list of (x, y) tuples."""
    if len(points) <= 2:
        return points

    pts = np.array(points)
    start, end = pts[0], pts[-1]
    line_vec = end - start
    line_len = np.linalg.norm(line_vec)

    if line_len < 1e-10:
        return [points[0], points[-1]]

    line_unit = line_vec / line_len
    diff = pts - start
    proj = np.outer(diff @ line_unit, line_unit)
    perp = diff - proj
    dists = np.sqrt(np.sum(perp**2, axis=1))

    idx = int(np.argmax(dists))
    if dists[idx] > tolerance:
        left = _rdp_simplify_route(
            [tuple(p) for p in pts[:idx + 1]], tolerance)
        right = _rdp_simplify_route(
            [tuple(p) for p in pts[idx:]], tolerance)
        return left[:-1] + right
    else:
        return [points[0], points[-1]]


# ---------------------------------------------------------------------------
# Turn smoothing: ensure no consecutive segment pair exceeds max course change
# ---------------------------------------------------------------------------

def smooth_turns(
    waypoints: list[tuple[float, float]],
    max_course_change_deg: float = MAX_COURSE_CHANGE_DEG,
    min_turn_radius_m: float = MIN_TURN_RADIUS_M,
) -> list[tuple[float, float]]:
    """Insert arc waypoints where course change exceeds vehicle limits.

    Walks the waypoint sequence; when a turn exceeds max_course_change_deg,
    inserts intermediate waypoints on an arc of min_turn_radius_m.
    """
    if len(waypoints) < 3:
        return list(waypoints)

    result = [waypoints[0]]

    for i in range(1, len(waypoints) - 1):
        prev = np.array(result[-1])
        curr = np.array(waypoints[i])
        nxt = np.array(waypoints[i + 1])

        # Headings of incoming and outgoing legs
        d_in = curr - prev
        d_out = nxt - curr

        hdg_in = math.atan2(d_in[0], d_in[1])   # ENU: atan2(x_east, y_north)
        hdg_out = math.atan2(d_out[0], d_out[1])

        delta = math.degrees(hdg_out - hdg_in)
        # Normalize to [-180, 180]
        while delta > 180:
            delta -= 360
        while delta < -180:
            delta += 360

        if abs(delta) <= max_course_change_deg:
            result.append(waypoints[i])
            continue

        # Insert arc waypoints: entry and exit tangent points + interpolated arc
        n_segments = math.ceil(abs(delta) / max_course_change_deg)

        # Tangent distance from vertex to arc entry/exit
        arc_dist = min(
            min_turn_radius_m * abs(math.tan(math.radians(delta) / 2)),
            np.linalg.norm(d_in) * 0.4,
            np.linalg.norm(d_out) * 0.4,
        )

        d_in_unit = d_in / np.linalg.norm(d_in)
        d_out_unit = d_out / np.linalg.norm(d_out)
        entry = curr - d_in_unit * arc_dist
        exit_pt = curr + d_out_unit * arc_dist

        for j in range(n_segments + 1):
            t = j / n_segments
            # Spherical-style interpolation through the turn vertex
            # Weight the path to curve through a point offset from the vertex
            if j == 0:
                pt = entry
            elif j == n_segments:
                pt = exit_pt
            else:
                # Lerp between entry and exit, with a bulge toward vertex
                lerp = entry * (1.0 - t) + exit_pt * t
                # Pull toward vertex proportional to how far from endpoints
                bulge = 4.0 * t * (1.0 - t) * 0.3  # max 0.3 at midpoint
                pt = lerp + (curr - (entry + exit_pt) / 2.0) * bulge
            result.append((float(pt[0]), float(pt[1])))

    result.append(waypoints[-1])
    return result


# ---------------------------------------------------------------------------
# Altitude selection
# ---------------------------------------------------------------------------

def select_altitude(
    terrain: TerrainMap,
    route_enu: list[tuple[float, float]],
    candidate_altitudes: list[float] | None = None,
    ground_speed: float = FW_CRUISE_SPEED_MS,
    rate_slew: float = 0.25,
) -> dict:
    """Pick the best cruise altitude maximizing disc * safety.

    Returns dict with recommended_altitude, mean_disc, safety_factor,
    max_terrain, per_altitude analysis, and transition_buffer_ok.
    """
    if candidate_altitudes is None:
        candidate_altitudes = CANDIDATE_ALTITUDES

    # Sample terrain along route at 10m intervals
    terrain_samples_x = []
    terrain_samples_y = []
    for i in range(len(route_enu) - 1):
        x0, y0 = route_enu[i]
        x1, y1 = route_enu[i + 1]
        seg_dist = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        n_samples = max(2, int(seg_dist / 10.0))
        for j in range(n_samples):
            t = j / n_samples
            terrain_samples_x.append(x0 + t * (x1 - x0))
            terrain_samples_y.append(y0 + t * (y1 - y0))
    terrain_samples_x.append(route_enu[-1][0])
    terrain_samples_y.append(route_enu[-1][1])

    xs = np.array(terrain_samples_x)
    ys = np.array(terrain_samples_y)
    terrain_heights = terrain.heights_at(xs, ys)
    max_terrain = float(np.max(terrain_heights))
    min_terrain = float(np.min(terrain_heights))

    # Compute gradients
    dists = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    dists[dists < 0.01] = 0.01
    gradients = np.abs(np.diff(terrain_heights)) / dists
    max_gradient = float(np.max(gradients))

    results = []
    for alt in candidate_altitudes:
        # Check transition buffer: cruise_alt must be above max_terrain + MC_FW_ALT_LOSS + 10m safety
        # Note: cruise_altitude is relative to home (origin), terrain_heights are Gazebo Z (above min_elev)
        # Actual AGL = cruise_alt - terrain_height (since home is at origin elevation)
        min_agl = alt - max_terrain
        transition_ok = min_agl >= (MC_FW_ALT_LOSS_M + 10.0)

        # Evaluate discriminability at this altitude
        params = altitude_scaled_params(alt)
        # Approximate mean disc using variance floor as proxy
        # Higher altitude → higher noise floor → lower effective disc
        # Score: inverse of variance floor (lower noise = better disc)
        noise_penalty = params['variance_floor']

        # Safety factor: check gradient feasibility
        # Max trackable gradient = sink_rate / ground_speed
        max_trackable = TECS_MAX_SINK_MS / ground_speed
        gradient_ok = max_gradient < max_trackable

        # AGL deficit simulation (from plan_mission_final.py logic)
        terrain_change_rate = max_gradient * ground_speed
        if terrain_change_rate > rate_slew:
            time_at_grad = min(10.0, 200.0 / ground_speed)  # cap
            deficit = (terrain_change_rate - rate_slew) * time_at_grad
            ff = min(max_gradient, 0.5) * min(3.0 * ground_speed, 100) * 0.5
            deficit = max(0, deficit - ff)
            worst_agl = min_agl - deficit
        else:
            worst_agl = min_agl

        safety_factor = 1.0
        if worst_agl < 10.0:
            safety_factor = 0.0
        elif worst_agl < 20.0:
            safety_factor = 0.5
        elif not transition_ok:
            safety_factor = 0.3

        # Disc score: inversely proportional to noise floor
        # At 50m baseline, variance_floor ≈ 2.65. Scale relative to that.
        disc_score = 1.0 / (1.0 + noise_penalty)

        # Combined score
        score = disc_score * safety_factor

        results.append({
            'altitude_m': alt,
            'min_agl': round(min_agl, 1),
            'worst_agl': round(worst_agl, 1),
            'noise_sigma': round(params['noise_sigma'], 3),
            'variance_floor': round(noise_penalty, 3),
            'disc_score': round(disc_score, 3),
            'safety_factor': round(safety_factor, 2),
            'combined_score': round(score, 3),
            'transition_ok': transition_ok,
            'gradient_ok': gradient_ok,
        })

    # Pick best
    best = max(results, key=lambda r: r['combined_score'])

    return {
        'recommended_altitude_m': best['altitude_m'],
        'max_terrain_m': round(max_terrain, 1),
        'min_terrain_m': round(min_terrain, 1),
        'max_gradient': round(max_gradient, 4),
        'per_altitude': results,
    }


# ---------------------------------------------------------------------------
# Cable range check
# ---------------------------------------------------------------------------

def check_cable_range(
    route_enu: list[tuple[float, float]],
    spool_capacity: float = SPOOL_CAPACITY_M,
    warn_fraction: float = SPOOL_WARN_FRACTION,
) -> dict:
    """Check if route distance is within spool limits."""
    total_dist = 0.0
    for i in range(len(route_enu) - 1):
        dx = route_enu[i + 1][0] - route_enu[i][0]
        dy = route_enu[i + 1][1] - route_enu[i][1]
        total_dist += math.sqrt(dx * dx + dy * dy)

    # Return trip is roughly the same distance
    round_trip = total_dist * 2.0
    max_one_way = spool_capacity * warn_fraction

    return {
        'outbound_distance_m': round(total_dist, 1),
        'round_trip_estimate_m': round(round_trip, 1),
        'spool_capacity_m': spool_capacity,
        'max_safe_one_way_m': round(max_one_way, 1),
        'within_limits': total_dist <= max_one_way,
        'utilization_pct': round(total_dist / max_one_way * 100, 1),
    }


# ---------------------------------------------------------------------------
# Recovery field: distance to nearest corridor
# ---------------------------------------------------------------------------

def compute_recovery_field(
    best_disc: np.ndarray,
    disc_threshold: float = 0.4,
    stride: int = 4,
    terrain: TerrainMap | None = None,
    yellow_dist_m: float = RECOVERY_YELLOW_DIST_M,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute recovery distance field from discriminability grid.

    Returns (recovery_dist, recovery_zone) arrays:
      - recovery_dist: distance in grid pixels to nearest corridor pixel
      - recovery_zone: 0=green (on corridor), 1=yellow (<200m), 2=red (>200m)
    """
    from scipy.ndimage import distance_transform_edt

    corridor_mask = best_disc >= disc_threshold
    if not np.any(corridor_mask):
        # No corridors — everything is red
        h, w = best_disc.shape
        return np.full((h, w), 9999.0), np.full((h, w), 2, dtype=np.int8)

    # Distance transform: distance from each non-corridor pixel to nearest corridor pixel
    # Input: True where we need distance (non-corridor pixels)
    dist_px = distance_transform_edt(~corridor_mask)

    # Convert pixel distance to meters
    mpp = stride * (terrain.mpp if terrain else 11.7)
    dist_m = dist_px * mpp

    # Classify zones
    zone = np.full_like(best_disc, 2, dtype=np.int8)  # red by default
    zone[dist_m <= yellow_dist_m] = 1  # yellow
    zone[corridor_mask] = 0             # green

    return dist_m, zone


# ---------------------------------------------------------------------------
# Safety analysis per segment
# ---------------------------------------------------------------------------

def analyze_segments(
    terrain: TerrainMap,
    waypoints_enu: list[tuple[float, float]],
    cruise_altitude: float,
    best_disc: np.ndarray,
    stride: int,
    recovery_dist: np.ndarray | None = None,
    ground_speed: float = FW_CRUISE_SPEED_MS,
    rate_slew: float = 0.25,
) -> list[dict]:
    """Per-segment safety analysis: gradient, turn angle, TECS, recovery, cable."""
    half_x = terrain.size_x / 2.0
    half_y = terrain.size_y / 2.0
    res = terrain.res

    segments = []
    cable_used = 0.0
    prev_heading = None

    for i in range(len(waypoints_enu) - 1):
        x0, y0 = waypoints_enu[i]
        x1, y1 = waypoints_enu[i + 1]
        dx, dy = x1 - x0, y1 - y0
        seg_dist = math.sqrt(dx * dx + dy * dy)
        cable_used += seg_dist

        # Heading (ENU: x=East, y=North → atan2(dx, dy) for heading from North)
        heading_deg = math.degrees(math.atan2(dx, dy)) % 360

        # Turn angle
        turn_angle = 0.0
        if prev_heading is not None:
            turn_angle = heading_deg - prev_heading
            if turn_angle > 180:
                turn_angle -= 360
            if turn_angle < -180:
                turn_angle += 360

        # Sample terrain every 10m
        n = max(2, int(seg_dist / 10.0))
        t_vals = np.linspace(0, 1, n + 1)
        sx = x0 + t_vals * dx
        sy = y0 + t_vals * dy
        elevs = terrain.heights_at(sx, sy)

        seg_max_terrain = float(np.max(elevs))
        seg_min_agl = cruise_altitude - seg_max_terrain

        # Gradient
        step_d = seg_dist / n
        grads = np.abs(np.diff(elevs)) / step_d
        seg_max_grad = float(np.max(grads))

        # TECS feasibility
        tecs_ok = seg_max_grad < MAX_TERRAIN_GRADIENT

        # AGL deficit (from plan_mission_final.py)
        terrain_change_rate = seg_max_grad * ground_speed
        if terrain_change_rate > rate_slew:
            time_at_grad = min(seg_dist / ground_speed, 20)
            deficit = (terrain_change_rate - rate_slew) * time_at_grad
            ff = min(seg_max_grad, 0.5) * min(3.0 * ground_speed, 100) * 0.5
            deficit = max(0, deficit - ff)
            worst_agl = seg_min_agl - deficit
        else:
            worst_agl = seg_min_agl

        # Disc score along segment (sample from grid)
        col_px = ((sx + half_x) / terrain.size_x * (res - 1) / stride).astype(int)
        row_px = ((half_y - sy) / terrain.size_y * (res - 1) / stride).astype(int)
        col_px = np.clip(col_px, 0, best_disc.shape[1] - 1)
        row_px = np.clip(row_px, 0, best_disc.shape[0] - 1)
        seg_disc = float(np.mean(best_disc[row_px, col_px]))

        # Recovery distance (average along segment)
        recovery_m = None
        if recovery_dist is not None:
            rec_vals = recovery_dist[row_px, col_px]
            recovery_m = float(np.mean(rec_vals))  # already in metres

        # Verdict
        if worst_agl > 15 and tecs_ok and abs(turn_angle) <= MAX_COURSE_CHANGE_DEG:
            verdict = 'SAFE'
        elif worst_agl > 10 and tecs_ok:
            verdict = 'MARGINAL'
        else:
            verdict = 'UNSAFE'

        segments.append({
            'index': i,
            'from_enu': (round(x0, 1), round(y0, 1)),
            'to_enu': (round(x1, 1), round(y1, 1)),
            'distance_m': round(seg_dist, 1),
            'heading_deg': round(heading_deg, 1),
            'turn_angle_deg': round(turn_angle, 1),
            'max_gradient': round(seg_max_grad, 4),
            'max_terrain_m': round(seg_max_terrain, 1),
            'min_agl_m': round(seg_min_agl, 1),
            'worst_agl_m': round(worst_agl, 1),
            'mean_disc': round(seg_disc, 3),
            'tecs_ok': tecs_ok,
            'recovery_dist_m': round(recovery_m, 1) if recovery_m is not None else None,
            'cable_deployed_m': round(cable_used, 1),
            'verdict': verdict,
        })

        prev_heading = heading_deg

    return segments


# ---------------------------------------------------------------------------
# Coordinate conversions
# ---------------------------------------------------------------------------

def _load_terrain_metadata(terrain_data_path: str) -> dict:
    """Load terrain_data.json metadata for coordinate conversion."""
    with open(terrain_data_path) as f:
        return json.load(f)


def _enu_to_ned(x_east: float, y_north: float) -> tuple[float, float]:
    """Convert ENU (x=East, y=North) to NED (x=North, y=East)."""
    return float(y_north), float(x_east)


def _ned_to_latlon(
    x_north: float,
    y_east: float,
    meta: dict,
) -> tuple[float, float]:
    """Convert NED offset from home to lat/lon (matching map_bridge_node.py)."""
    bbox = meta['bbox']
    center_lat = meta['center_lat']
    center_lon = meta['center_lon']
    size_x = meta['size_meters']['x']
    size_y = meta['size_meters']['y']

    lat = center_lat + (x_north / size_x) * (bbox['north'] - bbox['south'])
    lon = center_lon + (y_east / size_y) * (bbox['east'] - bbox['west'])
    return lat, lon


def _enu_to_latlon(
    x_east: float,
    y_north: float,
    meta: dict,
) -> tuple[float, float]:
    """Convert ENU to lat/lon via NED intermediate."""
    ned_n, ned_e = _enu_to_ned(x_east, y_north)
    return _ned_to_latlon(ned_n, ned_e, meta)


# ---------------------------------------------------------------------------
# GeoJSON generation
# ---------------------------------------------------------------------------

def _disc_color(disc: float) -> str:
    """Map discriminability 0..1 to red(0) -> yellow(0.5) -> green(1)."""
    disc = max(0.0, min(1.0, disc))
    if disc < 0.5:
        r = 200
        g = int(disc * 2 * 200)
    else:
        r = int((1.0 - disc) * 2 * 200)
        g = 200
    return f'#{r:02x}{g:02x}00'


def _recovery_color(zone: int) -> str:
    """Zone color: 0=green, 1=yellow, 2=red."""
    if zone == 0:
        return '#00cc44'
    elif zone == 1:
        return '#cccc00'
    return '#cc2200'


def generate_disc_heatmap_geojson(
    best_disc: np.ndarray,
    terrain: TerrainMap,
    meta: dict,
    grid_size: int = 40,
) -> dict:
    """Generate discriminability heatmap as GeoJSON FeatureCollection."""
    bbox = meta['bbox']
    lat_step = (bbox['north'] - bbox['south']) / grid_size
    lon_step = (bbox['east'] - bbox['west']) / grid_size

    gh, gw = best_disc.shape
    features = []

    for row in range(grid_size):
        for col in range(grid_size):
            s = bbox['north'] - (row + 1) * lat_step
            n = bbox['north'] - row * lat_step
            w = bbox['west'] + col * lon_step
            e = bbox['west'] + (col + 1) * lon_step

            # Sample disc at cell center
            gr = int(row / grid_size * gh)
            gc = int(col / grid_size * gw)
            gr = min(gr, gh - 1)
            gc = min(gc, gw - 1)
            disc = float(best_disc[gr, gc])
            color = _disc_color(disc)

            features.append({
                'type': 'Feature',
                'geometry': {
                    'type': 'Polygon',
                    'coordinates': [[[w, s], [e, s], [e, n], [w, n], [w, s]]],
                },
                'properties': {
                    'disc': round(disc, 3),
                    'fill': color,
                    'fill-opacity': 0.35,
                    'stroke': color,
                    'stroke-width': 0,
                    'stroke-opacity': 0,
                },
            })

    return {'type': 'FeatureCollection', 'features': features}


def generate_corridors_geojson(
    corridors_enu: list[dict],
    meta: dict,
) -> dict:
    """Generate corridor LineStrings as GeoJSON."""
    features = []
    for corr in corridors_enu:
        xs = corr['waypoints']['x']
        ys = corr['waypoints']['y']
        coords = []
        for x, y in zip(xs, ys):
            lat, lon = _enu_to_latlon(x, y, meta)
            coords.append([lon, lat])

        disc = corr.get('mean_disc', 0.5)
        features.append({
            'type': 'Feature',
            'geometry': {
                'type': 'LineString',
                'coordinates': coords,
            },
            'properties': {
                'name': corr.get('id', 'corridor'),
                'mean_disc': round(disc, 3),
                'stroke': '#9900cc',
                'stroke-width': 2,
                'stroke-opacity': 0.8,
            },
        })

    return {'type': 'FeatureCollection', 'features': features}


def generate_requested_plan_geojson(
    start_enu: tuple[float, float],
    goal_enu: tuple[float, float],
    waypoints_enu: list[tuple[float, float]] | None,
    meta: dict,
) -> dict:
    """Generate user's original request as GeoJSON (orange dashed line)."""
    features = []

    # Build coordinate list
    all_pts = [start_enu]
    if waypoints_enu:
        all_pts.extend(waypoints_enu)
    all_pts.append(goal_enu)

    coords = []
    for x, y in all_pts:
        lat, lon = _enu_to_latlon(x, y, meta)
        coords.append([lon, lat])

    # Dashed line
    features.append({
        'type': 'Feature',
        'geometry': {'type': 'LineString', 'coordinates': coords},
        'properties': {
            'name': 'Requested Route',
            'stroke': '#ff8800',
            'stroke-width': 3,
            'stroke-opacity': 0.8,
            'stroke-dasharray': '10 5',
        },
    })

    # Start marker
    slat, slon = _enu_to_latlon(start_enu[0], start_enu[1], meta)
    features.append({
        'type': 'Feature',
        'geometry': {'type': 'Point', 'coordinates': [slon, slat]},
        'properties': {
            'name': 'START',
            'marker-color': '#00ff00',
            'marker-size': 'large',
        },
    })

    # Goal marker
    glat, glon = _enu_to_latlon(goal_enu[0], goal_enu[1], meta)
    features.append({
        'type': 'Feature',
        'geometry': {'type': 'Point', 'coordinates': [glon, glat]},
        'properties': {
            'name': 'GOAL',
            'marker-color': '#ff0000',
            'marker-size': 'large',
        },
    })

    return {'type': 'FeatureCollection', 'features': features}


def generate_optimal_route_geojson(
    route_enu: list[tuple[float, float]],
    meta: dict,
) -> dict:
    """Generate TERCOM-optimal route as GeoJSON (red solid line)."""
    coords = []
    for x, y in route_enu:
        lat, lon = _enu_to_latlon(x, y, meta)
        coords.append([lon, lat])

    features = [{
        'type': 'Feature',
        'geometry': {'type': 'LineString', 'coordinates': coords},
        'properties': {
            'name': 'TERCOM Optimal Route',
            'stroke': '#cc0000',
            'stroke-width': 4,
            'stroke-opacity': 0.9,
        },
    }]

    return {'type': 'FeatureCollection', 'features': features}


def generate_mission_plan_geojson(
    waypoints_ned: list[tuple[float, float]],
    meta: dict,
    accept_radius: float = FW_ACCEPT_RADIUS_M,
) -> dict:
    """Generate final mission waypoints with acceptance circles as GeoJSON."""
    bbox = meta['bbox']
    size_x = meta['size_meters']['x']
    size_y = meta['size_meters']['y']
    center_lat = meta['center_lat']
    center_lon = meta['center_lon']

    features = []

    # Home marker
    features.append({
        'type': 'Feature',
        'geometry': {'type': 'Point', 'coordinates': [center_lon, center_lat]},
        'properties': {
            'name': 'HOME',
            'marker-color': '#00ff00',
            'marker-size': 'large',
        },
    })

    # Outbound path: home -> WP0 -> ... -> WPn
    outbound_coords = [[center_lon, center_lat]]
    wp_latlon = []
    for xn, ye in waypoints_ned:
        lat, lon = _ned_to_latlon(xn, ye, meta)
        wp_latlon.append((lat, lon))
        outbound_coords.append([lon, lat])

    features.append({
        'type': 'Feature',
        'geometry': {'type': 'LineString', 'coordinates': outbound_coords},
        'properties': {
            'name': 'Mission Path',
            'stroke': '#00cc44',
            'stroke-width': 3,
            'stroke-opacity': 0.9,
        },
    })

    # Return path
    if wp_latlon:
        last_lat, last_lon = wp_latlon[-1]
        features.append({
            'type': 'Feature',
            'geometry': {
                'type': 'LineString',
                'coordinates': [[last_lon, last_lat], [center_lon, center_lat]],
            },
            'properties': {
                'name': 'Return',
                'stroke': '#0088ff',
                'stroke-width': 2,
                'stroke-opacity': 0.7,
                'stroke-dasharray': '8 4',
            },
        })

    # Waypoint markers + acceptance circles
    for i, (lat, lon) in enumerate(wp_latlon):
        features.append({
            'type': 'Feature',
            'geometry': {'type': 'Point', 'coordinates': [lon, lat]},
            'properties': {
                'name': f'WP{i}',
                'marker-color': '#ff4400',
                'marker-size': 'medium',
            },
        })

        # Acceptance circle (24-point polygon)
        circle_coords = []
        for a in range(25):
            angle = 2 * math.pi * a / 24
            dlat = (accept_radius * math.cos(angle) / size_x) * (
                bbox['north'] - bbox['south'])
            dlon = (accept_radius * math.sin(angle) / size_y) * (
                bbox['east'] - bbox['west'])
            circle_coords.append([lon + dlon, lat + dlat])

        features.append({
            'type': 'Feature',
            'geometry': {'type': 'Polygon', 'coordinates': [circle_coords]},
            'properties': {
                'name': f'WP{i} radius',
                'stroke': '#ff4400',
                'stroke-width': 1,
                'stroke-opacity': 0.5,
                'fill': '#ff4400',
                'fill-opacity': 0.08,
            },
        })

    return {'type': 'FeatureCollection', 'features': features}


def generate_recovery_zones_geojson(
    recovery_zone: np.ndarray,
    meta: dict,
    grid_size: int = 40,
) -> dict:
    """Generate recovery zone grid as GeoJSON (green/yellow/red)."""
    bbox = meta['bbox']
    lat_step = (bbox['north'] - bbox['south']) / grid_size
    lon_step = (bbox['east'] - bbox['west']) / grid_size

    zh, zw = recovery_zone.shape
    features = []

    for row in range(grid_size):
        for col in range(grid_size):
            s = bbox['north'] - (row + 1) * lat_step
            n = bbox['north'] - row * lat_step
            w = bbox['west'] + col * lon_step
            e = bbox['west'] + (col + 1) * lon_step

            zr = int(row / grid_size * zh)
            zc = int(col / grid_size * zw)
            zr = min(zr, zh - 1)
            zc = min(zc, zw - 1)
            zone = int(recovery_zone[zr, zc])
            color = _recovery_color(zone)

            features.append({
                'type': 'Feature',
                'geometry': {
                    'type': 'Polygon',
                    'coordinates': [[[w, s], [e, s], [e, n], [w, n], [w, s]]],
                },
                'properties': {
                    'zone': ['green', 'yellow', 'red'][zone],
                    'fill': color,
                    'fill-opacity': 0.25,
                    'stroke': color,
                    'stroke-width': 0,
                    'stroke-opacity': 0,
                },
            })

    return {'type': 'FeatureCollection', 'features': features}


# ---------------------------------------------------------------------------
# Mission YAML output
# ---------------------------------------------------------------------------

def generate_mission_yaml(
    waypoints_ned: list[tuple[float, float]],
    cruise_altitude: float,
    analysis: dict,
    cable_check: dict,
    segments: list[dict],
    recovery_info: list[dict] | None = None,
) -> dict:
    """Generate vtol_navigation_node compatible mission YAML.

    Excludes the first waypoint (start/home) since vtol_navigation_node
    treats home (0,0) as the implicit origin.
    """
    # Skip the start point — vtol_navigation_node starts from home (0,0)
    nav_wps = waypoints_ned[1:] if len(waypoints_ned) > 1 else waypoints_ned
    xs = [round(wp[0], 1) for wp in nav_wps]
    ys = [round(wp[1], 1) for wp in nav_wps]

    # Compute per-leg times: distance / cruise speed + margin
    wp_times = []
    for i, wp in enumerate(nav_wps):
        prev = waypoints_ned[i]  # previous point (includes start at index 0)
        dist = math.hypot(wp[0] - prev[0], wp[1] - prev[1])
        wp_times.append(round(dist / FW_CRUISE_SPEED_MS + 3.0, 1))

    total_dist = sum(s['distance_m'] for s in segments)
    mean_disc = (sum(s['mean_disc'] for s in segments) / len(segments)
                 if segments else 0.0)

    tercom = {
        'mean_disc': round(mean_disc, 3),
        'recommended_altitude_m': analysis['recommended_altitude_m'],
        'total_distance_m': round(total_dist, 1),
        'max_terrain_m': analysis['max_terrain_m'],
        'max_gradient': analysis['max_gradient'],
        'cable_utilization_pct': cable_check['utilization_pct'],
    }

    if recovery_info:
        tercom['recovery'] = recovery_info

    data = {
        'vtol_navigation_node': {
            'ros__parameters': {
                'cruise_altitude': float(cruise_altitude),
                'fw_accept_radius': FW_ACCEPT_RADIUS_M,
                'waypoints': {
                    'x': xs,
                    'y': ys,
                    'wp_time_s': wp_times,
                },
                'tercom_analysis': tercom,
            },
        },
    }
    return data


# ---------------------------------------------------------------------------
# Text report
# ---------------------------------------------------------------------------

def generate_report(
    analysis: dict,
    cable_check: dict,
    segments: list[dict],
    route_enu: list[tuple[float, float]],
    waypoints_ned: list[tuple[float, float]],
    cruise_altitude: float,
) -> str:
    """Generate human-readable mission analysis report."""
    lines = []
    lines.append('=' * 70)
    lines.append('TERCOM Mission Analysis Report')
    lines.append('=' * 70)
    lines.append('')

    lines.append(f'Cruise altitude: {cruise_altitude:.0f}m AGL')
    lines.append(f'Terrain range: {analysis["min_terrain_m"]:.1f} - '
                 f'{analysis["max_terrain_m"]:.1f}m (Gazebo Z)')
    lines.append(f'Max gradient: {analysis["max_gradient"]:.4f}')
    lines.append(f'Route waypoints: {len(waypoints_ned)} (NED)')
    lines.append('')

    # Altitude comparison
    lines.append('--- Altitude Comparison ---')
    lines.append(f'{"Alt(m)":>7} {"MinAGL":>7} {"WorstAGL":>9} {"Noise":>7} '
                 f'{"Disc":>6} {"Safety":>7} {"Score":>7} {"Trans":>5} {"TECS":>5}')
    for a in analysis['per_altitude']:
        lines.append(
            f'{a["altitude_m"]:7.0f} {a["min_agl"]:7.1f} {a["worst_agl"]:9.1f} '
            f'{a["noise_sigma"]:7.3f} {a["disc_score"]:6.3f} {a["safety_factor"]:7.2f} '
            f'{a["combined_score"]:7.3f} {"OK" if a["transition_ok"] else "NO":>5} '
            f'{"OK" if a["gradient_ok"] else "NO":>5}')
    lines.append(f'\nRecommended: {analysis["recommended_altitude_m"]:.0f}m')
    lines.append('')

    # Cable range
    lines.append('--- Cable Range ---')
    lines.append(f'Outbound: {cable_check["outbound_distance_m"]:.0f}m')
    lines.append(f'Round trip (est): {cable_check["round_trip_estimate_m"]:.0f}m')
    lines.append(f'Spool capacity: {cable_check["spool_capacity_m"]:.0f}m')
    lines.append(f'Max safe one-way: {cable_check["max_safe_one_way_m"]:.0f}m')
    lines.append(f'Utilization: {cable_check["utilization_pct"]:.1f}%')
    lines.append(f'Status: {"WITHIN LIMITS" if cable_check["within_limits"] else "WARNING: EXCEEDS SPOOL"}')
    lines.append('')

    # Per-segment
    lines.append('--- Per-Segment Analysis ---')
    for s in segments:
        lines.append(
            f'Seg {s["index"]}: {s["from_enu"]} -> {s["to_enu"]}  '
            f'dist={s["distance_m"]:.0f}m  hdg={s["heading_deg"]:.1f}deg  '
            f'turn={s["turn_angle_deg"]:+.1f}deg')
        lines.append(
            f'    grad={s["max_gradient"]:.4f}  terrain_max={s["max_terrain_m"]:.1f}m  '
            f'min_agl={s["min_agl_m"]:.1f}m  worst_agl={s["worst_agl_m"]:.1f}m')
        lines.append(
            f'    disc={s["mean_disc"]:.3f}  tecs={"OK" if s["tecs_ok"] else "FAIL"}  '
            f'cable={s["cable_deployed_m"]:.0f}m  '
            f'recovery={s["recovery_dist_m"]:.0f}m' if s["recovery_dist_m"] is not None
            else f'    disc={s["mean_disc"]:.3f}  tecs={"OK" if s["tecs_ok"] else "FAIL"}  '
                 f'cable={s["cable_deployed_m"]:.0f}m')
        lines.append(f'    VERDICT: {s["verdict"]}')
        lines.append('')

    # NED waypoints
    lines.append('--- NED Waypoints (for vtol_navigation_node) ---')
    for i, (xn, ye) in enumerate(waypoints_ned):
        lines.append(f'  WP{i}: north={xn:.1f}m  east={ye:.1f}m')

    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# ROS2 publisher mode
# ---------------------------------------------------------------------------

def publish_geojson_layers(geojson_layers: dict[str, dict],
                           layers: str = 'route'):
    """Publish GeoJSON layers to Foxglove via ROS2 (one-shot, latched).

    layers='route': only optimal_route (lightweight, good for flight monitoring)
    layers='all': all analysis layers (disc_heatmap, corridors, etc.)
    """
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (
            QoSProfile,
            DurabilityPolicy,
            HistoryPolicy,
            ReliabilityPolicy,
        )
        from foxglove_msgs.msg import GeoJSON
    except ImportError as e:
        print(f'ROS2/foxglove_msgs not available: {e}')
        print('Run inside the fiber-nav container with ROS sourced.')
        sys.exit(1)

    rclpy.init()
    node = Node('tercom_mission_planner')

    qos_latched = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    all_topics = {
        'disc_heatmap': '/map/disc_heatmap',
        'corridors': '/map/tercom_corridors',
        'requested_plan': '/map/requested_plan',
        'optimal_route': '/map/optimal_route',
        'recovery_zones': '/map/recovery_zones',
    }

    if layers == 'all':
        topic_map = all_topics
    else:
        # Default: only the lightweight optimal route line
        topic_map = {'optimal_route': '/map/optimal_route'}

    publishers = {}
    for key, topic in topic_map.items():
        if key in geojson_layers:
            pub = node.create_publisher(GeoJSON, topic, qos_latched)
            publishers[key] = pub

    # Brief delay for discovery
    time.sleep(0.5)

    for key, pub in publishers.items():
        msg = GeoJSON()
        msg.geojson = json.dumps(geojson_layers[key])
        pub.publish(msg)
        n_features = len(geojson_layers[key].get('features', []))
        print(f'  Published {topic_map[key]}: {n_features} features')

    print(f'Published {len(publishers)} GeoJSON layers (TRANSIENT_LOCAL).')
    print('Keeping node alive for Foxglove (Ctrl+C to stop)...')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print('Done.')


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_point(s: str) -> tuple[float, float]:
    """Parse 'x,y' string to (float, float) in ENU."""
    parts = s.split(',')
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(f"Expected x,y format, got '{s}'")
    return float(parts[0]), float(parts[1])


def main():
    parser = argparse.ArgumentParser(
        description='TERCOM-Aware Mission Planner + Foxglove Visualization')
    parser.add_argument('--terrain-data', required=True,
                        help='Path to terrain_data.json')
    parser.add_argument('--road-network', default=None,
                        help='Path to precomputed road_network.yaml')
    parser.add_argument('--start', type=parse_point, required=True,
                        help='Start point as x,y in ENU metres')
    parser.add_argument('--goal', type=parse_point, required=True,
                        help='Goal point as x,y in ENU metres')
    parser.add_argument('--waypoints', type=parse_point, nargs='*', default=None,
                        help='Intermediate waypoints as x,y pairs')
    parser.add_argument('--altitude', type=float, default=None,
                        help='Force cruise altitude (auto-select if omitted)')
    parser.add_argument('--disc-threshold', type=float, default=0.4,
                        help='Discriminability threshold (default: 0.4)')
    parser.add_argument('--stride', type=int, default=4,
                        help='Grid sub-sampling stride (default: 4)')
    parser.add_argument('--output-dir', default='./mission_output',
                        help='Output directory')
    parser.add_argument('--publish', action='store_true',
                        help='Publish GeoJSON layers to ROS2/Foxglove')
    parser.add_argument('--layers', choices=['route', 'all'], default='route',
                        help='Which layers to publish: route (default) or all')
    parser.add_argument('--rdp-tolerance', type=float, default=50.0,
                        help='RDP simplification tolerance in metres (default: 50)')
    parser.add_argument('--max-waypoints', type=int, default=15,
                        help='Max waypoints after simplification (default: 15)')
    args = parser.parse_args()

    # Load terrain
    print(f'Loading terrain from {args.terrain_data}')
    terrain = TerrainMap(args.terrain_data)
    meta = _load_terrain_metadata(args.terrain_data)
    print(f'  {terrain.res}x{terrain.res} px, '
          f'{terrain.size_x:.0f}x{terrain.size_y:.0f}m, '
          f'elev {terrain.min_elev:.0f}-{terrain.min_elev + terrain.elev_range:.0f}m MSL')

    # Step 1: Compute or load discriminability grid
    print('\nStep 1: Computing discriminability grid...')
    altitude_for_disc = args.altitude or 80.0  # default for disc computation
    best_disc, best_heading, disc_all = compute_discriminability_grid(
        terrain, stride=args.stride, altitude_agl=altitude_for_disc)

    # Step 2: Route optimization
    print('\nStep 2: Route optimization...')
    result = plan_route(
        terrain, best_disc, args.stride,
        start_enu=args.start, goal_enu=args.goal,
        road_network_path=args.road_network,
        disc_threshold=args.disc_threshold,
        altitude_agl=altitude_for_disc)

    if not result.get('optimal_route'):
        print('ERROR: No route found between start and goal.')
        sys.exit(1)

    route_x = result['optimal_route']['x']
    route_y = result['optimal_route']['y']
    raw_route = list(zip(route_x, route_y))

    # Simplify route with RDP
    simplified = _rdp_simplify_route(raw_route, args.rdp_tolerance)

    # Enforce max waypoints by increasing tolerance
    tol = args.rdp_tolerance
    while len(simplified) > args.max_waypoints and tol < 500:
        tol *= 1.5
        simplified = _rdp_simplify_route(raw_route, tol)

    print(f'  Raw: {len(raw_route)} pts -> Simplified: {len(simplified)} pts '
          f'(tolerance={tol:.0f}m)')

    # Step 3: Turn smoothing
    print('\nStep 3: Turn smoothing...')
    smoothed = smooth_turns(simplified)
    print(f'  {len(simplified)} -> {len(smoothed)} waypoints after turn smoothing')

    route_enu = smoothed

    # Step 4: Altitude selection
    print('\nStep 4: Altitude selection...')
    if args.altitude:
        # User forced altitude — still do analysis
        alt_analysis = select_altitude(terrain, route_enu)
        cruise_alt = args.altitude
        print(f'  User-specified altitude: {cruise_alt:.0f}m '
              f'(recommended: {alt_analysis["recommended_altitude_m"]:.0f}m)')
    else:
        alt_analysis = select_altitude(terrain, route_enu)
        cruise_alt = alt_analysis['recommended_altitude_m']
        print(f'  Auto-selected altitude: {cruise_alt:.0f}m')

    # Step 5: Cable range check
    print('\nStep 5: Cable range check...')
    cable_check = check_cable_range(route_enu)
    if not cable_check['within_limits']:
        print(f'  WARNING: Route exceeds spool limits! '
              f'{cable_check["outbound_distance_m"]:.0f}m > '
              f'{cable_check["max_safe_one_way_m"]:.0f}m')
    else:
        print(f'  OK: {cable_check["outbound_distance_m"]:.0f}m / '
              f'{cable_check["max_safe_one_way_m"]:.0f}m '
              f'({cable_check["utilization_pct"]:.1f}%)')

    # Step 6: Recovery field
    print('\nStep 6: Recovery field...')
    recovery_dist, recovery_zone = compute_recovery_field(
        best_disc, disc_threshold=args.disc_threshold,
        stride=args.stride, terrain=terrain)
    n_green = int(np.sum(recovery_zone == 0))
    n_yellow = int(np.sum(recovery_zone == 1))
    n_red = int(np.sum(recovery_zone == 2))
    total = recovery_zone.size
    print(f'  Green: {n_green} ({100*n_green/total:.0f}%), '
          f'Yellow: {n_yellow} ({100*n_yellow/total:.0f}%), '
          f'Red: {n_red} ({100*n_red/total:.0f}%)')

    # Convert route to NED for mission YAML
    waypoints_ned = [_enu_to_ned(x, y) for x, y in route_enu]

    # Step 7: Safety analysis
    print('\nStep 7: Safety analysis...')
    segments = analyze_segments(
        terrain, route_enu, cruise_alt, best_disc, args.stride,
        recovery_dist=recovery_dist)

    verdicts = [s['verdict'] for s in segments]
    print(f'  {verdicts.count("SAFE")} SAFE, '
          f'{verdicts.count("MARGINAL")} MARGINAL, '
          f'{verdicts.count("UNSAFE")} UNSAFE')

    # Recovery annotations for mission YAML
    recovery_info = []
    half_x = terrain.size_x / 2.0
    half_y = terrain.size_y / 2.0
    res = terrain.res
    for i, (x, y) in enumerate(route_enu):
        col_px = int((x + half_x) / terrain.size_x * (res - 1) / args.stride)
        row_px = int((half_y - y) / terrain.size_y * (res - 1) / args.stride)
        col_px = min(max(col_px, 0), recovery_zone.shape[1] - 1)
        row_px = min(max(row_px, 0), recovery_zone.shape[0] - 1)
        dist_m = float(recovery_dist[row_px, col_px])  # already in metres
        zone = int(recovery_zone[row_px, col_px])
        zone_name = ['green', 'yellow', 'red'][zone]
        recovery_info.append({
            'wp_index': i,
            'distance_m': round(dist_m, 1),
            'zone': zone_name,
        })

    # Step 8: Generate outputs
    print('\nStep 8: Generating outputs...')

    # GeoJSON layers
    geojson_layers = {
        'disc_heatmap': generate_disc_heatmap_geojson(
            best_disc, terrain, meta),
        'corridors': generate_corridors_geojson(
            result.get('corridors_enu', []), meta),
        'requested_plan': generate_requested_plan_geojson(
            args.start, args.goal, args.waypoints, meta),
        'optimal_route': generate_optimal_route_geojson(route_enu, meta),
        'recovery_zones': generate_recovery_zones_geojson(recovery_zone, meta),
        'mission_plan': generate_mission_plan_geojson(
            waypoints_ned, meta, accept_radius=FW_ACCEPT_RADIUS_M),
    }

    # Mission YAML
    mission_data = generate_mission_yaml(
        waypoints_ned, cruise_alt, alt_analysis, cable_check,
        segments, recovery_info)

    # Text report
    report = generate_report(
        alt_analysis, cable_check, segments, route_enu,
        waypoints_ned, cruise_alt)

    if args.publish:
        print(f'\nPublishing to Foxglove (layers={args.layers})...')
        publish_geojson_layers(geojson_layers, layers=args.layers)

    # Save files
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Mission YAML
    yaml_path = out_dir / 'tercom_mission.yaml'
    with open(yaml_path, 'w') as f:
        yaml.dump(mission_data, f, default_flow_style=False, sort_keys=False)
    print(f'  Saved {yaml_path}')

    # GeoJSON files
    for name, data in geojson_layers.items():
        path = out_dir / f'foxglove_{name}.geojson'
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        n_features = len(data.get('features', []))
        print(f'  Saved {path} ({n_features} features)')

    # Report
    report_path = out_dir / 'mission_analysis.txt'
    with open(report_path, 'w') as f:
        f.write(report)
    print(f'  Saved {report_path}')

    print(f'\nAll outputs saved to {out_dir}/')
    print(report)


if __name__ == '__main__':
    main()
