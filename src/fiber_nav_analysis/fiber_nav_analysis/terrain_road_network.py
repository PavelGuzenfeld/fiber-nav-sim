#!/usr/bin/env python3
"""TERCOM-Optimal Road Network — Terrain Discriminability Analysis Tool.

Computes a discriminability heatmap across the terrain, extracts high-discriminability
corridors, connects them into a navigable graph, and outputs YAML waypoints + PNG plots.

Algorithm is a Python port of tercom.cpp:compute_mission_discriminability().

Usage:
    python3 -m fiber_nav_analysis.terrain_road_network \
        --terrain-data src/fiber_nav_gazebo/terrain/terrain_data.json \
        --output-dir ./road_network_output \
        --stride 4 --disc-threshold 0.4 \
        --start 0,0 --goal 800,800 --show
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

# Phase 2/3 imports — deferred to function scope to keep startup fast
# skimage.morphology, networkx


# ---------------------------------------------------------------------------
# Terrain loading (port of terrain_gis_node.py)
# ---------------------------------------------------------------------------

class TerrainMap:
    """Loads terrain_data.json + heightmap PNG, provides height queries."""

    def __init__(self, terrain_data_path: str):
        path = Path(terrain_data_path)
        with open(path) as f:
            meta = json.load(f)

        hm_path = path.parent / meta['heightmap_file']
        hm_img = Image.open(hm_path)
        raw = np.array(hm_img, dtype=np.float64)

        if hm_img.mode in ('I;16', 'I'):
            max_val = 65535.0
        else:
            max_val = 255.0

        self.size_x = float(meta['size_meters']['x'])
        self.size_y = float(meta['size_meters']['y'])
        self.min_elev = float(meta['min_elevation_msl'])
        self.elev_range = float(meta['heightmap_range_m'])
        self.res = int(meta['resolution_px'])
        self.mpp = float(meta['meters_per_pixel'])

        # Convert to elevation in metres (Gazebo Z frame: 0 = min_elev)
        self.elevation = (raw / max_val) * self.elev_range

        # Load texture for overlay plots
        tex_path = path.parent / meta.get('texture_file', 'terrain_texture.png')
        if tex_path.exists():
            self.texture = np.array(Image.open(tex_path))
        else:
            self.texture = None

    # Vectorised coordinate conversion: ENU (x,y) -> pixel (col, row)
    def xy_to_pixel(self, x: np.ndarray, y: np.ndarray):
        """Return (col, row) as float arrays, clamped to [0, res-1]."""
        col = (x + self.size_x / 2.0) / self.size_x * (self.res - 1)
        row = (self.size_y / 2.0 - y) / self.size_y * (self.res - 1)
        col = np.clip(col, 0.0, self.res - 1.0)
        row = np.clip(row, 0.0, self.res - 1.0)
        return col, row

    def height_at(self, x: float, y: float) -> float:
        """Scalar height query (ENU coords -> Gazebo Z)."""
        col, row = self.xy_to_pixel(np.asarray(x, dtype=np.float64),
                                    np.asarray(y, dtype=np.float64))
        return float(ndimage.map_coordinates(
            self.elevation, [[row], [col]], order=1, mode='nearest')[0])

    def heights_at(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Batch bilinear-interpolated height query."""
        col, row = self.xy_to_pixel(x, y)
        return ndimage.map_coordinates(
            self.elevation, [row, col], order=1, mode='nearest')


# ---------------------------------------------------------------------------
# Altitude model — scales profile geometry and noise floor with AGL
# ---------------------------------------------------------------------------

# Baseline parameters are tuned for 50m AGL
_BASELINE_AGL = 50.0
_BASELINE_PROFILE_SPACING = 12.0
_BASELINE_CROSS_SPACING = 12.0
_BASELINE_CORRIDOR_HALF_WIDTH = 100.0

# Rangefinder noise model: sigma_range = base_sigma * (altitude / baseline_alt)
# At 50m AGL: ~0.15m std (typical LiDAR), at 200m: ~0.6m
_RANGE_NOISE_BASE_SIGMA = 0.15
# Barometric altimeter noise: constant ~0.5m std
_BARO_NOISE_SIGMA = 0.5


def altitude_scaled_params(
    altitude_agl: float,
    profile_spacing: float = _BASELINE_PROFILE_SPACING,
    cross_spacing: float = _BASELINE_CROSS_SPACING,
    corridor_half_width: float = _BASELINE_CORRIDOR_HALF_WIDTH,
) -> dict:
    """Scale profile parameters and compute noise floor for given altitude.

    Higher altitude → wider sampling footprint (rangefinder beam divergence +
    barometric smoothing) and higher noise floor (range noise grows with distance).

    Returns dict with scaled profile_spacing, cross_spacing, corridor_half_width,
    and variance_floor (minimum terrain variance to be considered informative).
    """
    scale = altitude_agl / _BASELINE_AGL

    # Profile spacing scales with sqrt(altitude) — beam footprint grows but
    # terrain features don't shrink linearly, so sqrt is a reasonable compromise
    sqrt_scale = np.sqrt(scale)
    scaled_profile_spacing = profile_spacing * sqrt_scale
    scaled_cross_spacing = cross_spacing * sqrt_scale
    scaled_corridor_half_width = corridor_half_width * sqrt_scale

    # Noise floor: combined rangefinder + baro noise variance
    # At each profile sample, terrain height is estimated from baro_alt - range
    # sigma_total² = sigma_baro² + sigma_range²(alt)
    sigma_range = _RANGE_NOISE_BASE_SIGMA * scale
    sigma_total = np.sqrt(_BARO_NOISE_SIGMA**2 + sigma_range**2)
    # Variance floor: profile must have variance > (3*sigma)² to be informative
    # (terrain relief must exceed 3x measurement noise)
    variance_floor = (3.0 * sigma_total) ** 2

    return {
        'profile_spacing': scaled_profile_spacing,
        'cross_spacing': scaled_cross_spacing,
        'corridor_half_width': scaled_corridor_half_width,
        'variance_floor': variance_floor,
        'noise_sigma': sigma_total,
    }


# ---------------------------------------------------------------------------
# Phase 1: Discriminability Grid  (port of tercom.cpp lines 396-500)
# ---------------------------------------------------------------------------

def normalized_cross_correlation(a: np.ndarray, b: np.ndarray) -> float:
    """NCC between two 1-D profiles. Returns 0 for flat profiles."""
    a = a - a.mean()
    b = b - b.mean()
    den = np.sqrt(np.dot(a, a) * np.dot(b, b))
    if den < 1e-10:
        return 0.0
    return float(np.dot(a, b) / den)


def compute_discriminability_grid(
    terrain: TerrainMap,
    stride: int = 4,
    profile_length: int = 10,
    profile_spacing: float = 12.0,
    corridor_half_width: float = 100.0,
    cross_spacing: float = 12.0,
    n_headings: int = 8,
    altitude_agl: float | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute discriminability on a sub-sampled grid across all headings.

    If altitude_agl is provided, profile geometry and noise floor are scaled
    to reflect real TERCOM performance at that flight altitude.

    Returns
    -------
    best_disc : (H, W) max discriminability across headings
    best_heading : (H, W) heading index that gave best_disc
    disc_all : (n_headings, H, W) discriminability per heading
    """
    # Apply altitude scaling if requested
    variance_floor = 0.01  # default: flat-profile threshold from C++ tercom
    if altitude_agl is not None:
        params = altitude_scaled_params(
            altitude_agl, profile_spacing, cross_spacing, corridor_half_width)
        profile_spacing = params['profile_spacing']
        cross_spacing = params['cross_spacing']
        corridor_half_width = params['corridor_half_width']
        variance_floor = params['variance_floor']
        print(f"  Altitude model ({altitude_agl:.0f}m AGL):")
        print(f"    profile_spacing={profile_spacing:.1f}m, "
              f"cross_spacing={cross_spacing:.1f}m, "
              f"corridor_half_width={corridor_half_width:.0f}m")
        print(f"    noise_sigma={params['noise_sigma']:.2f}m, "
              f"variance_floor={variance_floor:.3f}m²")

    res = terrain.res
    rows = np.arange(0, res, stride)
    cols = np.arange(0, res, stride)
    gh, gw = len(rows), len(cols)

    # Grid coordinates in ENU
    half_x = terrain.size_x / 2.0
    half_y = terrain.size_y / 2.0
    col_idx, row_idx = np.meshgrid(cols, rows)
    gx = col_idx.astype(np.float64) / (res - 1) * terrain.size_x - half_x  # East
    gy = half_y - row_idx.astype(np.float64) / (res - 1) * terrain.size_y  # North

    headings = np.linspace(0, np.pi, n_headings, endpoint=False)
    n_offsets = int(corridor_half_width / cross_spacing)
    profile_offsets = (np.arange(profile_length) - profile_length // 2) * profile_spacing

    disc_all = np.zeros((n_headings, gh, gw), dtype=np.float32)

    total = n_headings * gh * gw
    print(f"Phase 1: computing discriminability grid ({gh}x{gw} x {n_headings} headings = {total:,} evals)")
    t0 = time.time()

    for hi, heading in enumerate(headings):
        along_x = np.cos(heading)
        along_y = np.sin(heading)
        cross_x = -along_y
        cross_y = along_x

        for ri in range(gh):
            for ci in range(gw):
                cx, cy = gx[ri, ci], gy[ri, ci]

                # Center profile along heading
                px = cx + along_x * profile_offsets
                py = cy + along_y * profile_offsets
                center = terrain.heights_at(px, py)

                # Check variance against noise floor
                var_c = np.var(center)
                if var_c < variance_floor:
                    disc_all[hi, ri, ci] = 0.0
                    continue

                # NCC with cross-track offset profiles
                max_ncc = -2.0
                for oi in range(1, n_offsets + 1):
                    for sign in (-1, 1):
                        ct = sign * oi * cross_spacing
                        ox = cx + along_x * profile_offsets + cross_x * ct
                        oy = cy + along_y * profile_offsets + cross_y * ct
                        offset_prof = terrain.heights_at(ox, oy)
                        ncc = normalized_cross_correlation(center, offset_prof)
                        if ncc > max_ncc:
                            max_ncc = ncc

                disc_all[hi, ri, ci] = np.clip(1.0 - max_ncc, 0.0, 1.0)

        elapsed = time.time() - t0
        done_frac = (hi + 1) / n_headings
        eta = elapsed / done_frac * (1 - done_frac) if done_frac > 0 else 0
        print(f"  heading {hi+1}/{n_headings} ({np.degrees(heading):.1f}°) done  "
              f"[{elapsed:.1f}s elapsed, ~{eta:.0f}s remaining]")

    best_heading = np.argmax(disc_all, axis=0)
    best_disc = np.max(disc_all, axis=0)
    print(f"Phase 1 complete in {time.time() - t0:.1f}s  "
          f"mean={best_disc.mean():.3f} max={best_disc.max():.3f}")

    return best_disc, best_heading, disc_all


# ---------------------------------------------------------------------------
# Phase 2: Corridor Extraction
# ---------------------------------------------------------------------------

def extract_corridors(
    best_disc: np.ndarray,
    best_heading: np.ndarray,
    headings: np.ndarray,
    threshold: float = 0.4,
    min_component_px: int = 5,
    simplify_tolerance_px: float = 2.0,
) -> list[dict]:
    """Extract corridor polylines from discriminability mask.

    Returns list of dicts with keys: points_px, mean_disc, best_heading_deg, length_px.
    """
    from skimage.morphology import skeletonize, closing, disk

    mask = best_disc >= threshold

    # Morphological cleanup
    mask = closing(mask, disk(2))

    # Remove small connected components
    labeled, n_labels = ndimage.label(mask)
    for i in range(1, n_labels + 1):
        if np.sum(labeled == i) < min_component_px:
            mask[labeled == i] = False

    if not np.any(mask):
        print("Phase 2: no pixels above threshold — try lowering --disc-threshold")
        return []

    # Skeletonize
    skel = skeletonize(mask)

    # Trace skeleton to polylines by following connected pixels
    corridors = _trace_skeleton(skel, best_disc, best_heading, headings, simplify_tolerance_px)

    print(f"Phase 2: {np.sum(mask)} mask pixels, {np.sum(skel)} skeleton pixels, "
          f"{len(corridors)} corridors extracted")
    return corridors


def _trace_skeleton(
    skel: np.ndarray,
    best_disc: np.ndarray,
    best_heading: np.ndarray,
    headings: np.ndarray,
    simplify_tolerance: float,
) -> list[dict]:
    """Walk skeleton pixels and split at branch points into polylines."""
    h, w = skel.shape
    visited = np.zeros_like(skel, dtype=bool)

    # Find branch points (pixels with >2 skeleton neighbours)
    kernel = np.ones((3, 3), dtype=int)
    kernel[1, 1] = 0
    neighbour_count = ndimage.convolve(skel.astype(int), kernel, mode='constant')
    branch_pts = skel & (neighbour_count > 2)

    # Endpoints: skeleton pixels with exactly 1 neighbour
    end_pts = skel & (neighbour_count == 1)

    # Start tracing from endpoints and branch points
    start_pixels = list(zip(*np.where(end_pts | branch_pts)))
    if not start_pixels:
        # No endpoints/branches — pick any skeleton pixel
        start_pixels = [tuple(np.argwhere(skel)[0])]

    corridors = []
    for sr, sc in start_pixels:
        if visited[sr, sc] and not branch_pts[sr, sc]:
            continue
        # Trace in each unvisited direction from this pixel
        for dr, dc in _neighbours(sr, sc, h, w):
            nr, nc = sr + dr, sc + dc
            if not skel[nr, nc] or visited[nr, nc]:
                continue
            chain = [(sr, sc)]
            cr, cc = nr, nc
            while True:
                chain.append((cr, cc))
                visited[cr, cc] = True
                # Find next unvisited skeleton neighbour (not backtrack)
                found = False
                for dr2, dc2 in _neighbours(cr, cc, h, w):
                    nr2, nc2 = cr + dr2, cc + dc2
                    if skel[nr2, nc2] and not visited[nr2, nc2]:
                        cr, cc = nr2, nc2
                        found = True
                        break
                if not found or branch_pts[cr, cc]:
                    if branch_pts[cr, cc]:
                        chain.append((cr, cc))
                    break

            if len(chain) < 2:
                continue

            pts = np.array(chain, dtype=np.float64)
            pts = _rdp_simplify(pts, simplify_tolerance)

            # Compute corridor stats
            rows = np.array([p[0] for p in chain], dtype=int)
            cols = np.array([p[1] for p in chain], dtype=int)
            mean_disc = float(np.mean(best_disc[rows, cols]))
            heading_indices = best_heading[rows, cols]
            mode_heading = int(np.bincount(heading_indices.astype(int)).argmax())
            diffs = np.diff(pts, axis=0)
            length = float(np.sum(np.sqrt(diffs[:, 0]**2 + diffs[:, 1]**2)))

            corridors.append({
                'points_px': pts.tolist(),
                'mean_disc': mean_disc,
                'best_heading_deg': float(np.degrees(headings[mode_heading])),
                'length_px': length,
            })

    return corridors


def _neighbours(r, c, h, w):
    """Yield 8-connected (dr, dc) offsets that stay in bounds."""
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w:
                yield dr, dc


def _rdp_simplify(points: np.ndarray, tolerance: float) -> np.ndarray:
    """Ramer-Douglas-Peucker polyline simplification."""
    if len(points) <= 2:
        return points

    # Distance from each point to line between first and last
    start, end = points[0], points[-1]
    line_vec = end - start
    line_len = np.linalg.norm(line_vec)
    if line_len < 1e-10:
        return points[[0, -1]]

    line_unit = line_vec / line_len
    diff = points - start
    proj = np.outer(diff @ line_unit, line_unit)
    perp = diff - proj
    dists = np.sqrt(np.sum(perp**2, axis=1))

    idx = int(np.argmax(dists))
    if dists[idx] > tolerance:
        left = _rdp_simplify(points[:idx + 1], tolerance)
        right = _rdp_simplify(points[idx:], tolerance)
        return np.vstack([left[:-1], right])
    else:
        return points[[0, -1]]


# ---------------------------------------------------------------------------
# Phase 3: Road Graph + Connectivity
# ---------------------------------------------------------------------------

def build_road_graph(
    corridors: list[dict],
    best_disc: np.ndarray,
    terrain: TerrainMap,
    stride: int,
    connector_max_dist_m: float = 500.0,
    start_enu: tuple[float, float] | None = None,
    goal_enu: tuple[float, float] | None = None,
) -> dict:
    """Build a networkx graph from corridors and find optimal route.

    Returns dict with keys: graph, corridors_enu, connectors, optimal_route.
    """
    import networkx as nx

    if not corridors:
        print("Phase 3: no corridors to build graph from")
        return {
            'graph': nx.Graph(),
            'corridors_enu': [],
            'connectors': [],
            'optimal_route': None,
        }

    half_x = terrain.size_x / 2.0
    half_y = terrain.size_y / 2.0
    res = terrain.res

    def px_to_enu(row, col):
        """Convert grid-pixel (row, col) at stride to ENU (x, y)."""
        px_col = col * stride
        px_row = row * stride
        x = px_col / (res - 1) * terrain.size_x - half_x
        y = half_y - px_row / (res - 1) * terrain.size_y
        return x, y

    graph = nx.Graph()
    node_id = 0
    corridor_nodes = []  # list of (corridor_idx, list_of_node_ids)

    # Add corridor edges
    for ci, corr in enumerate(corridors):
        pts = corr['points_px']
        ids = []
        for r, c in pts:
            x, y = px_to_enu(r, c)
            graph.add_node(node_id, x=x, y=y, corridor=ci)
            ids.append(node_id)
            node_id += 1

        # Edges within corridor
        for i in range(len(ids) - 1):
            n1, n2 = ids[i], ids[i + 1]
            x1, y1 = graph.nodes[n1]['x'], graph.nodes[n1]['y']
            x2, y2 = graph.nodes[n2]['x'], graph.nodes[n2]['y']
            dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            # Sample discriminability along segment
            r1, c1 = pts[i]
            r2, c2 = pts[i + 1]
            n_samp = max(2, int(dist / (stride * terrain.mpp)))
            rr = np.linspace(r1, r2, n_samp).astype(int).clip(0, best_disc.shape[0] - 1)
            cc = np.linspace(c1, c2, n_samp).astype(int).clip(0, best_disc.shape[1] - 1)
            seg_disc = best_disc[rr, cc]
            weight = float(np.sum(1.0 / (seg_disc + 0.01))) * dist / n_samp
            graph.add_edge(n1, n2, weight=weight, dist=dist, kind='corridor')

        corridor_nodes.append((ci, ids))

    # Collect all corridor endpoints
    endpoints = []
    for ci, ids in corridor_nodes:
        if ids:
            endpoints.append((ci, ids[0], 'start'))
            endpoints.append((ci, ids[-1], 'end'))

    # Connector edges between nearby corridor endpoints
    connectors = []
    for i, (ci1, nid1, _) in enumerate(endpoints):
        for j, (ci2, nid2, _) in enumerate(endpoints):
            if j <= i or ci1 == ci2:
                continue
            x1, y1 = graph.nodes[nid1]['x'], graph.nodes[nid1]['y']
            x2, y2 = graph.nodes[nid2]['x'], graph.nodes[nid2]['y']
            dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            if dist > connector_max_dist_m:
                continue
            # Sample disc along connector line
            n_samp = max(2, int(dist / 50))
            xs = np.linspace(x1, x2, n_samp)
            ys = np.linspace(y1, y2, n_samp)
            elev = terrain.heights_at(xs, ys)  # just to exercise terrain (unused)
            # Estimate disc along connector from grid
            col_px = ((xs + half_x) / terrain.size_x * (res - 1) / stride).astype(int)
            row_px = ((half_y - ys) / terrain.size_y * (res - 1) / stride).astype(int)
            col_px = np.clip(col_px, 0, best_disc.shape[1] - 1)
            row_px = np.clip(row_px, 0, best_disc.shape[0] - 1)
            mean_disc = float(np.mean(best_disc[row_px, col_px]))
            weight = dist / (mean_disc + 0.01)
            graph.add_edge(nid1, nid2, weight=weight, dist=dist, kind='connector')
            connectors.append({
                'from_corridor': ci1,
                'to_corridor': ci2,
                'from_node': nid1,
                'to_node': nid2,
                'dist_m': dist,
                'mean_disc': mean_disc,
            })

    # Ensure connectivity with MST on a fully-connected version
    if graph.number_of_nodes() > 1 and not nx.is_connected(graph):
        components = list(nx.connected_components(graph))
        for i in range(len(components) - 1):
            best_d = float('inf')
            best_pair = None
            for n1 in components[i]:
                for n2 in components[i + 1]:
                    d = np.sqrt((graph.nodes[n1]['x'] - graph.nodes[n2]['x'])**2 +
                                (graph.nodes[n1]['y'] - graph.nodes[n2]['y'])**2)
                    if d < best_d:
                        best_d = d
                        best_pair = (n1, n2)
            if best_pair:
                graph.add_edge(best_pair[0], best_pair[1],
                               weight=best_d * 10, dist=best_d, kind='bridge')

    # Find optimal route from start to goal
    optimal_route = None
    if start_enu and goal_enu:
        # Add start/goal nodes, connect to nearest graph node
        start_id = node_id
        graph.add_node(start_id, x=start_enu[0], y=start_enu[1], corridor=-1)
        node_id += 1
        goal_id = node_id
        graph.add_node(goal_id, x=goal_enu[0], y=goal_enu[1], corridor=-1)
        node_id += 1

        for special_id, (sx, sy) in [(start_id, start_enu), (goal_id, goal_enu)]:
            best_d = float('inf')
            best_n = None
            for n in graph.nodes:
                if n == start_id or n == goal_id:
                    continue
                d = np.sqrt((graph.nodes[n]['x'] - sx)**2 + (graph.nodes[n]['y'] - sy)**2)
                if d < best_d:
                    best_d = d
                    best_n = n
            if best_n is not None:
                graph.add_edge(special_id, best_n,
                               weight=best_d / 0.1, dist=best_d, kind='access')

        try:
            path = nx.dijkstra_path(graph, start_id, goal_id, weight='weight')
            xs = [graph.nodes[n]['x'] for n in path]
            ys = [graph.nodes[n]['y'] for n in path]
            # Sample disc along route
            disc_scores = []
            for n in path:
                x, y = graph.nodes[n]['x'], graph.nodes[n]['y']
                col_px = int((x + half_x) / terrain.size_x * (res - 1) / stride)
                row_px = int((half_y - y) / terrain.size_y * (res - 1) / stride)
                col_px = np.clip(col_px, 0, best_disc.shape[1] - 1)
                row_px = np.clip(row_px, 0, best_disc.shape[0] - 1)
                disc_scores.append(float(best_disc[row_px, col_px]))
            optimal_route = {'x': xs, 'y': ys, 'disc_scores': disc_scores}
            total_dist = sum(np.sqrt((xs[i+1]-xs[i])**2 + (ys[i+1]-ys[i])**2)
                             for i in range(len(xs)-1))
            print(f"Phase 3: optimal route found, {len(path)} waypoints, "
                  f"{total_dist:.0f}m, mean disc={np.mean(disc_scores):.3f}")
        except nx.NetworkXNoPath:
            print("Phase 3: no path found between start and goal")

    # Convert corridors to ENU
    corridors_enu = []
    for ci, ids in corridor_nodes:
        xs = [graph.nodes[n]['x'] for n in ids]
        ys = [graph.nodes[n]['y'] for n in ids]
        corridors_enu.append({
            'id': f'corridor_{ci}',
            'mean_disc': corridors[ci]['mean_disc'],
            'best_heading_deg': corridors[ci]['best_heading_deg'],
            'length_m': corridors[ci]['length_px'] * stride * terrain.mpp,
            'waypoints': {'x': xs, 'y': ys},
        })

    return {
        'graph': graph,
        'corridors_enu': corridors_enu,
        'connectors': connectors,
        'optimal_route': optimal_route,
    }


# ---------------------------------------------------------------------------
# Output: YAML
# ---------------------------------------------------------------------------

def save_yaml(result: dict, output_path: Path):
    """Save road network as YAML matching gps_denied_mission.yaml convention."""
    data = {
        'road_network': {
            'corridors': [],
            'connectors': [],
        }
    }

    for c in result['corridors_enu']:
        data['road_network']['corridors'].append({
            'id': c['id'],
            'mean_disc': round(c['mean_disc'], 4),
            'best_heading_deg': round(c['best_heading_deg'], 1),
            'length_m': round(c['length_m'], 1),
            'waypoints': {
                'x': [round(v, 1) for v in c['waypoints']['x']],
                'y': [round(v, 1) for v in c['waypoints']['y']],
            },
        })

    for conn in result['connectors']:
        data['road_network']['connectors'].append({
            'from': f"corridor_{conn['from_corridor']}",
            'to': f"corridor_{conn['to_corridor']}",
            'dist_m': round(conn['dist_m'], 1),
            'mean_disc': round(conn['mean_disc'], 4),
        })

    if result.get('optimal_route'):
        rt = result['optimal_route']
        data['road_network']['optimal_route'] = {
            'waypoints_x': [round(v, 1) for v in rt['x']],
            'waypoints_y': [round(v, 1) for v in rt['y']],
            'disc_scores': [round(v, 4) for v in rt['disc_scores']],
        }

    with open(output_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    print(f"Saved {output_path}")


# ---------------------------------------------------------------------------
# Output: Visualisations
# ---------------------------------------------------------------------------

def plot_disc_heatmap(
    terrain: TerrainMap,
    best_disc: np.ndarray,
    stride: int,
    output_path: Path,
):
    """Discriminability overlaid on terrain texture."""
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize

    fig, ax = plt.subplots(figsize=(10, 10))

    extent = [-terrain.size_x/2, terrain.size_x/2,
              -terrain.size_y/2, terrain.size_y/2]

    if terrain.texture is not None:
        ax.imshow(terrain.texture, extent=extent, origin='upper', alpha=0.6)

    # Upsample disc to terrain resolution for overlay
    ax.imshow(best_disc, extent=extent, origin='upper',
              cmap='RdYlGn', alpha=0.5, vmin=0, vmax=1,
              interpolation='bilinear')

    cbar = plt.colorbar(ax.images[-1], ax=ax, shrink=0.7, label='Discriminability')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('TERCOM Discriminability Heatmap')
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved {output_path}")


def plot_disc_per_heading(
    disc_all: np.ndarray,
    terrain: TerrainMap,
    n_headings: int,
    output_path: Path,
):
    """8-panel plot, one per heading direction."""
    import matplotlib.pyplot as plt

    headings = np.linspace(0, 180, n_headings, endpoint=False)
    cols = 4
    rows = (n_headings + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(16, 4 * rows))
    axes = axes.flatten()

    extent = [-terrain.size_x/2, terrain.size_x/2,
              -terrain.size_y/2, terrain.size_y/2]

    for i in range(n_headings):
        ax = axes[i]
        ax.imshow(disc_all[i], extent=extent, origin='upper',
                  cmap='RdYlGn', vmin=0, vmax=1, interpolation='bilinear')
        ax.set_title(f'{headings[i]:.1f}°')
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')

    for i in range(n_headings, len(axes)):
        axes[i].set_visible(False)

    fig.suptitle('Discriminability per Heading Direction', fontsize=14)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved {output_path}")


def plot_road_network(
    terrain: TerrainMap,
    best_disc: np.ndarray,
    result: dict,
    stride: int,
    start_enu: tuple | None,
    goal_enu: tuple | None,
    output_path: Path,
):
    """Corridors (colour=disc) + connectors (dashed) + optimal route (red)."""
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.colors import Normalize

    fig, ax = plt.subplots(figsize=(10, 10))
    extent = [-terrain.size_x/2, terrain.size_x/2,
              -terrain.size_y/2, terrain.size_y/2]

    if terrain.texture is not None:
        ax.imshow(terrain.texture, extent=extent, origin='upper', alpha=0.4)

    ax.imshow(best_disc, extent=extent, origin='upper',
              cmap='RdYlGn', alpha=0.3, vmin=0, vmax=1,
              interpolation='bilinear')

    norm = Normalize(vmin=0.3, vmax=0.9)
    cmap = plt.cm.viridis

    # Draw corridors
    for corr in result['corridors_enu']:
        xs = corr['waypoints']['x']
        ys = corr['waypoints']['y']
        color = cmap(norm(corr['mean_disc']))
        ax.plot(xs, ys, '-', color=color, linewidth=2.5, alpha=0.9)

    # Draw connectors
    graph = result['graph']
    for conn in result['connectors']:
        n1, n2 = conn['from_node'], conn['to_node']
        x1, y1 = graph.nodes[n1]['x'], graph.nodes[n1]['y']
        x2, y2 = graph.nodes[n2]['x'], graph.nodes[n2]['y']
        ax.plot([x1, x2], [y1, y2], '--', color='gray', linewidth=1, alpha=0.6)

    # Draw optimal route
    if result.get('optimal_route'):
        rt = result['optimal_route']
        ax.plot(rt['x'], rt['y'], '-', color='red', linewidth=3, alpha=0.9,
                label='Optimal Route')
        ax.legend(fontsize=11)

    # Draw start/goal
    if start_enu:
        ax.plot(start_enu[0], start_enu[1], 'g^', markersize=14, label='Start')
    if goal_enu:
        ax.plot(goal_enu[0], goal_enu[1], 'rs', markersize=14, label='Goal')

    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('TERCOM-Optimal Road Network')
    ax.legend(fontsize=11)
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved {output_path}")


def plot_best_heading(
    terrain: TerrainMap,
    best_disc: np.ndarray,
    best_heading: np.ndarray,
    n_headings: int,
    stride: int,
    output_path: Path,
):
    """Quiver plot of best flight heading per grid point."""
    import matplotlib.pyplot as plt

    headings_rad = np.linspace(0, np.pi, n_headings, endpoint=False)
    gh, gw = best_disc.shape

    fig, ax = plt.subplots(figsize=(10, 10))
    extent = [-terrain.size_x/2, terrain.size_x/2,
              -terrain.size_y/2, terrain.size_y/2]

    if terrain.texture is not None:
        ax.imshow(terrain.texture, extent=extent, origin='upper', alpha=0.4)

    # Sub-sample quiver for readability
    q_stride = max(1, gh // 32)
    rows = np.arange(0, gh, q_stride)
    cols = np.arange(0, gw, q_stride)
    ci, ri = np.meshgrid(cols, rows)

    half_x = terrain.size_x / 2.0
    half_y = terrain.size_y / 2.0
    res = terrain.res
    qx = ci * stride / (res - 1) * terrain.size_x - half_x
    qy = half_y - ri * stride / (res - 1) * terrain.size_y

    angles = headings_rad[best_heading[ri, ci]]
    disc_vals = best_disc[ri, ci]
    u = np.cos(angles) * disc_vals
    v = np.sin(angles) * disc_vals

    ax.quiver(qx, qy, u, v, disc_vals, cmap='RdYlGn', scale=25,
              width=0.003, clim=(0, 1))
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('Best TERCOM Heading per Grid Point')
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved {output_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_point(s: str) -> tuple[float, float]:
    """Parse 'x,y' string to (float, float)."""
    parts = s.split(',')
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(f"Expected x,y format, got '{s}'")
    return float(parts[0]), float(parts[1])


def main():
    parser = argparse.ArgumentParser(
        description='TERCOM-Optimal Road Network — Terrain Discriminability Analysis')
    parser.add_argument('--terrain-data', required=True,
                        help='Path to terrain_data.json')
    parser.add_argument('--output-dir', default='./road_network_output',
                        help='Output directory')
    parser.add_argument('--stride', type=int, default=4,
                        help='Grid sub-sampling stride (default: 4)')
    parser.add_argument('--disc-threshold', type=float, default=0.4,
                        help='Discriminability threshold for corridors (default: 0.4)')
    parser.add_argument('--start', type=parse_point, default=None,
                        help='Start point as x,y in ENU metres (e.g., 0,0)')
    parser.add_argument('--goal', type=parse_point, default=None,
                        help='Goal point as x,y in ENU metres (e.g., 800,800)')
    parser.add_argument('--profile-length', type=int, default=10,
                        help='Elevation samples per profile (default: 10)')
    parser.add_argument('--profile-spacing', type=float, default=12.0,
                        help='Along-track spacing in metres (default: 12)')
    parser.add_argument('--corridor-half-width', type=float, default=100.0,
                        help='Cross-track half-width in metres (default: 100)')
    parser.add_argument('--cross-spacing', type=float, default=12.0,
                        help='Cross-track offset spacing in metres (default: 12)')
    parser.add_argument('--connector-max-dist', type=float, default=500.0,
                        help='Max connector distance in metres (default: 500)')
    parser.add_argument('--altitude', type=float, default=None,
                        help='Flight altitude AGL in metres. When set, scales profile '
                        'geometry (wider spacing at higher alt) and applies a noise '
                        'floor (terrain must exceed sensor noise to be informative). '
                        'Baseline parameters are tuned for 50m AGL.')
    parser.add_argument('--show', action='store_true',
                        help='Show plots interactively')
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Load terrain
    print(f"Loading terrain from {args.terrain_data}")
    terrain = TerrainMap(args.terrain_data)
    print(f"  {terrain.res}x{terrain.res} px, {terrain.size_x:.0f}x{terrain.size_y:.0f}m, "
          f"elev {terrain.min_elev:.0f}-{terrain.min_elev + terrain.elev_range:.0f}m MSL")

    n_headings = 8

    # Phase 1: Discriminability grid
    best_disc, best_heading, disc_all = compute_discriminability_grid(
        terrain,
        stride=args.stride,
        profile_length=args.profile_length,
        profile_spacing=args.profile_spacing,
        corridor_half_width=args.corridor_half_width,
        cross_spacing=args.cross_spacing,
        n_headings=n_headings,
        altitude_agl=args.altitude,
    )

    # Phase 2: Corridor extraction
    headings_rad = np.linspace(0, np.pi, n_headings, endpoint=False)
    corridors = extract_corridors(
        best_disc, best_heading, headings_rad,
        threshold=args.disc_threshold,
    )

    # Phase 3: Road graph + optimal route
    result = build_road_graph(
        corridors, best_disc, terrain,
        stride=args.stride,
        connector_max_dist_m=args.connector_max_dist,
        start_enu=args.start,
        goal_enu=args.goal,
    )

    # Save YAML
    save_yaml(result, out_dir / 'road_network.yaml')

    # Save plots
    plot_disc_heatmap(terrain, best_disc, args.stride, out_dir / 'disc_heatmap.png')
    plot_disc_per_heading(disc_all, terrain, n_headings, out_dir / 'disc_per_heading.png')
    plot_road_network(terrain, best_disc, result, args.stride,
                      args.start, args.goal, out_dir / 'road_network.png')
    plot_best_heading(terrain, best_disc, best_heading, n_headings, args.stride,
                      out_dir / 'best_heading.png')

    if args.show:
        import matplotlib.pyplot as plt
        plt.show()

    print(f"\nAll outputs saved to {out_dir}/")


if __name__ == '__main__':
    main()
