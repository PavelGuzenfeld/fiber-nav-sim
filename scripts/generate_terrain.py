#!/usr/bin/env python3
"""Generate terrain heightmap + orthophoto texture for Gazebo Harmonic.

Downloads SRTM 30m DEM and Esri World Imagery tiles, processes them into
a Gazebo-compatible heightmap + satellite texture, and generates a world SDF
and metadata JSON.

All default parameters are loaded from:
    src/fiber_nav_bringup/config/terrain_defaults.yaml

The SDF world is generated from:
    src/fiber_nav_gazebo/worlds/terrain_world.sdf.template

Dependencies: elevation, rasterio, numpy, Pillow, requests, scipy, pyyaml

Usage:
    python3 scripts/generate_terrain.py
    python3 scripts/generate_terrain.py --lat 31.2 --lon 34.5 --radius-km 5
    python3 scripts/generate_terrain.py --config my_config.yaml

Output:
    src/fiber_nav_gazebo/terrain/
        terrain_heightmap.png   (16-bit grayscale)
        terrain_texture.png     (RGB satellite)
        flat_normal.png         (1x1 flat normal map)
        terrain_data.json       (metadata for GIS node)
    src/fiber_nav_gazebo/worlds/
        terrain_world.sdf       (Gazebo world file)
"""

import argparse
import json
import math
import os
import string
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

# Optional imports — fail gracefully with instructions
try:
    import elevation
except ImportError:
    elevation = None

try:
    import rasterio
except ImportError:
    rasterio = None

try:
    import requests
except ImportError:
    requests = None

try:
    from scipy.ndimage import zoom
except ImportError:
    zoom = None


def check_dependencies():
    missing = []
    if elevation is None:
        missing.append('elevation')
    if rasterio is None:
        missing.append('rasterio')
    if requests is None:
        missing.append('requests')
    if zoom is None:
        missing.append('scipy')
    if missing:
        print(f'Missing dependencies: {", ".join(missing)}')
        print(f'Install with: pip install {" ".join(missing)} Pillow numpy pyyaml')
        sys.exit(1)


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

def load_config(config_path: str) -> dict:
    """Load terrain defaults from YAML config file."""
    with open(config_path) as f:
        return yaml.safe_load(f)


def resolve_config_path(project_dir: Path) -> Path:
    """Find the terrain_defaults.yaml config file."""
    candidates = [
        project_dir / 'src' / 'fiber_nav_bringup' / 'config' / 'terrain_defaults.yaml',
    ]
    for c in candidates:
        if c.exists():
            return c
    return candidates[0]  # Return default path even if not found


# ---------------------------------------------------------------------------
# Geo helpers
# ---------------------------------------------------------------------------

def bbox_from_center(lat: float, lon: float, radius_km: float):
    """Compute WGS84 bounding box from center + radius."""
    km_per_deg_lat = 111.32
    km_per_deg_lon = 111.32 * math.cos(math.radians(lat))

    dlat = radius_km / km_per_deg_lat
    dlon = radius_km / km_per_deg_lon

    return {
        'south': lat - dlat,
        'north': lat + dlat,
        'west': lon - dlon,
        'east': lon + dlon,
    }


def bbox_size_meters(bbox: dict, lat: float):
    """Compute approximate size in meters of bounding box."""
    km_per_deg_lat = 111.32
    km_per_deg_lon = 111.32 * math.cos(math.radians(lat))

    size_y = (bbox['north'] - bbox['south']) * km_per_deg_lat * 1000.0
    size_x = (bbox['east'] - bbox['west']) * km_per_deg_lon * 1000.0
    return size_x, size_y


def isa_temperature(elev_m: float, cfg: dict) -> float:
    """ISA temperature at elevation (K)."""
    t0 = cfg['atmosphere']['sea_level_temperature_k']
    lapse = cfg['atmosphere']['lapse_rate_k_per_m']
    return t0 - lapse * elev_m


def isa_pressure(elev_m: float, cfg: dict) -> float:
    """ISA pressure at elevation (Pa)."""
    t0 = cfg['atmosphere']['sea_level_temperature_k']
    p0 = cfg['atmosphere']['sea_level_pressure_pa']
    exp = cfg['atmosphere']['pressure_exponent']
    return p0 * (isa_temperature(elev_m, cfg) / t0) ** exp


# ---------------------------------------------------------------------------
# DEM download + processing
# ---------------------------------------------------------------------------

def download_dem(bbox: dict, output_tif: str):
    """Download SRTM 30m DEM using the elevation package."""
    print(f'Downloading SRTM DEM for bbox: {bbox}')
    bounds = (bbox['west'], bbox['south'], bbox['east'], bbox['north'])
    elevation.clip(bounds=bounds, output=output_tif, product='SRTM3')
    elevation.clean()
    print(f'DEM saved to {output_tif}')


def process_dem(tif_path: str, target_size: int):
    """Read GeoTIFF, resize to (2^n+1)x(2^n+1), return heightmap array + stats."""
    print(f'Processing DEM: {tif_path} -> {target_size}x{target_size}')

    with rasterio.open(tif_path) as src:
        dem = src.read(1).astype(np.float64)

    nodata_mask = dem < -1000
    if nodata_mask.any():
        valid_min = dem[~nodata_mask].min()
        dem[nodata_mask] = valid_min

    min_elev = float(dem.min())
    max_elev = float(dem.max())
    elev_range = max_elev - min_elev

    print(f'  Elevation range: {min_elev:.1f} - {max_elev:.1f} m MSL '
          f'(range: {elev_range:.1f} m)')

    zoom_y = target_size / dem.shape[0]
    zoom_x = target_size / dem.shape[1]
    dem_resized = zoom(dem, (zoom_y, zoom_x), order=3)
    dem_resized = dem_resized[:target_size, :target_size]

    if elev_range > 0:
        normalized = (dem_resized - min_elev) / elev_range
    else:
        normalized = np.zeros_like(dem_resized)

    heightmap_16 = (normalized * 65535).astype(np.uint16)

    return heightmap_16, min_elev, max_elev, elev_range


# ---------------------------------------------------------------------------
# Satellite imagery download
# ---------------------------------------------------------------------------

def latlon_to_tile(lat: float, lon: float, z: int):
    """Convert lat/lon to tile x, y at zoom level z."""
    n = 2 ** z
    x = int((lon + 180.0) / 360.0 * n)
    lat_rad = math.radians(lat)
    y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return x, y


def tile_to_latlon(x: int, y: int, z: int):
    """Convert tile x, y to lat/lon (top-left corner) at zoom level z."""
    n = 2 ** z
    lon = x / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * y / n)))
    lat = math.degrees(lat_rad)
    return lat, lon


def download_satellite_tiles(bbox: dict, zoom_level: int, output_png: str,
                             target_size: int, cfg: dict):
    """Download satellite imagery tiles and merge into one image."""
    print(f'Downloading satellite imagery at zoom {zoom_level}...')

    tile_url = cfg['satellite']['tile_url']
    tile_size = cfg['satellite']['tile_size']

    x_min, y_min = latlon_to_tile(bbox['north'], bbox['west'], zoom_level)
    x_max, y_max = latlon_to_tile(bbox['south'], bbox['east'], zoom_level)

    if x_min > x_max:
        x_min, x_max = x_max, x_min
    if y_min > y_max:
        y_min, y_max = y_max, y_min

    nx = x_max - x_min + 1
    ny = y_max - y_min + 1
    total = nx * ny
    print(f'  Tiles: {nx}x{ny} = {total} tiles')

    merged = Image.new('RGB', (nx * tile_size, ny * tile_size))

    session = requests.Session()
    session.headers.update({
        'User-Agent': cfg['satellite']['user_agent'],
        'Referer': cfg['satellite']['referer'],
    })

    downloaded = 0
    for ty in range(y_min, y_max + 1):
        for tx in range(x_min, x_max + 1):
            url = tile_url.format(z=zoom_level, y=ty, x=tx)
            try:
                resp = session.get(url, timeout=30)
                resp.raise_for_status()
                tile_img = Image.open(__import__('io').BytesIO(resp.content))
                px = (tx - x_min) * tile_size
                py = (ty - y_min) * tile_size
                merged.paste(tile_img, (px, py))
                downloaded += 1
                if downloaded % 10 == 0:
                    print(f'  Downloaded {downloaded}/{total} tiles')
            except Exception as e:
                print(f'  Warning: Failed to download tile {tx},{ty}: {e}')
                gray = Image.new('RGB', (tile_size, tile_size), (128, 128, 128))
                px = (tx - x_min) * tile_size
                py = (ty - y_min) * tile_size
                merged.paste(gray, (px, py))

    print(f'  Downloaded {downloaded}/{total} tiles')

    full_tl_lat, full_tl_lon = tile_to_latlon(x_min, y_min, zoom_level)
    full_br_lat, full_br_lon = tile_to_latlon(x_max + 1, y_max + 1, zoom_level)

    img_w = nx * tile_size
    img_h = ny * tile_size

    px_west = (bbox['west'] - full_tl_lon) / (full_br_lon - full_tl_lon) * img_w
    px_east = (bbox['east'] - full_tl_lon) / (full_br_lon - full_tl_lon) * img_w
    py_north = (full_tl_lat - bbox['north']) / (full_tl_lat - full_br_lat) * img_h
    py_south = (full_tl_lat - bbox['south']) / (full_tl_lat - full_br_lat) * img_h

    crop_box = (int(px_west), int(py_north), int(px_east), int(py_south))
    cropped = merged.crop(crop_box)

    texture = cropped.resize((target_size, target_size), Image.LANCZOS)
    texture.save(output_png, 'PNG')
    print(f'  Satellite texture saved: {output_png} ({target_size}x{target_size})')


def create_flat_normal(output_png: str):
    """Create a 1x1 flat normal map (pointing straight up: 128,128,255)."""
    img = Image.new('RGB', (1, 1), (128, 128, 255))
    img.save(output_png, 'PNG')
    print(f'  Flat normal map saved: {output_png}')


# ---------------------------------------------------------------------------
# SDF world generation
# ---------------------------------------------------------------------------

def fmt_vec(vals: list) -> str:
    """Format a list of numbers as space-separated string for SDF."""
    return ' '.join(str(v) for v in vals)


def generate_world_sdf(terrain_data: dict, cfg: dict,
                       template_path: str, output_sdf: str):
    """Generate Gazebo world SDF from template + terrain data + config."""

    size_x = terrain_data['size_meters']['x']
    size_y = terrain_data['size_meters']['y']
    elev_range = terrain_data['heightmap_range_m']
    min_elev = terrain_data['min_elevation_msl']
    lat = terrain_data['center_lat']
    lon = terrain_data['center_lon']

    temperature = isa_temperature(min_elev, cfg)
    pressure = isa_pressure(min_elev, cfg)

    world_cfg = cfg['world']
    mag = cfg['magnetic_field']

    # Build substitution dictionary for template
    subs = {
        'world_name': world_cfg['name'],
        'max_step_size': world_cfg['physics']['max_step_size'],
        'real_time_factor': world_cfg['physics']['real_time_factor'],
        'real_time_update_rate': world_cfg['physics']['real_time_update_rate'],
        'collision_detector': world_cfg['physics']['collision_detector'],
        'ambient': fmt_vec(world_cfg['scene']['ambient']),
        'background': fmt_vec(world_cfg['scene']['background']),
        'shadows': str(world_cfg['scene']['shadows']).lower(),
        'sun_diffuse': fmt_vec(world_cfg['sun']['diffuse']),
        'sun_specular': fmt_vec(world_cfg['sun']['specular']),
        'sun_direction': fmt_vec(world_cfg['sun']['direction']),
        'sun_range': world_cfg['sun']['range'],
        'size_x': size_x,
        'size_y': size_y,
        'elev_range': f'{elev_range:.1f}',
        'mag_x': mag['x'],
        'mag_y': mag['y'],
        'mag_z': mag['z'],
        'latitude': lat,
        'longitude': lon,
        'min_elevation': f'{min_elev:.1f}',
        'temperature': f'{temperature:.3f}',
        'pressure': f'{pressure:.0f}',
    }

    with open(template_path) as f:
        template = string.Template(f.read())

    sdf_content = template.substitute(subs)

    with open(output_sdf, 'w') as f:
        f.write(sdf_content)
    print(f'World SDF saved: {output_sdf}')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Generate terrain heightmap + satellite texture for Gazebo'
    )
    parser.add_argument('--config', type=str, default=None,
                        help='Path to terrain config YAML (default: auto-discover)')
    parser.add_argument('--lat', type=float, default=None,
                        help='Center latitude (overrides config)')
    parser.add_argument('--lon', type=float, default=None,
                        help='Center longitude (overrides config)')
    parser.add_argument('--radius-km', type=float, default=None,
                        help='Radius in km (overrides config)')
    parser.add_argument('--resolution', type=int, default=None,
                        help='Heightmap resolution, 2^n+1 (overrides config)')
    parser.add_argument('--zoom', type=int, default=None,
                        help='Satellite imagery zoom level (overrides config)')
    parser.add_argument('--skip-dem', action='store_true',
                        help='Skip DEM download (use existing raw_dem.tif)')
    parser.add_argument('--skip-satellite', action='store_true',
                        help='Skip satellite imagery download')

    args = parser.parse_args()

    check_dependencies()

    # Resolve paths
    script_dir = Path(__file__).resolve().parent
    project_dir = script_dir.parent
    terrain_dir = project_dir / 'src' / 'fiber_nav_gazebo' / 'terrain'
    worlds_dir = project_dir / 'src' / 'fiber_nav_gazebo' / 'worlds'

    terrain_dir.mkdir(parents=True, exist_ok=True)

    # Load config
    config_path = args.config or str(resolve_config_path(project_dir))
    if not os.path.exists(config_path):
        print(f'Error: Config file not found: {config_path}')
        sys.exit(1)

    print(f'Loading config from: {config_path}')
    cfg = load_config(config_path)

    # Apply CLI overrides (CLI args take priority over config)
    lat = args.lat if args.lat is not None else cfg['location']['lat']
    lon = args.lon if args.lon is not None else cfg['location']['lon']
    radius_km = args.radius_km if args.radius_km is not None else cfg['location']['radius_km']
    resolution = args.resolution if args.resolution is not None else cfg['heightmap']['resolution']
    zoom_level = args.zoom if args.zoom is not None else cfg['satellite']['zoom_level']

    # File paths
    raw_dem = str(terrain_dir / 'raw_dem.tif')
    heightmap_png = str(terrain_dir / 'terrain_heightmap.png')
    texture_png = str(terrain_dir / 'terrain_texture.png')
    normal_png = str(terrain_dir / 'flat_normal.png')
    metadata_json = str(terrain_dir / 'terrain_data.json')
    world_sdf = str(worlds_dir / 'terrain_world.sdf')
    template_sdf = str(worlds_dir / 'terrain_world.sdf.template')

    # Compute bounding box
    bbox = bbox_from_center(lat, lon, radius_km)
    size_x, size_y = bbox_size_meters(bbox, lat)

    print(f'Target: {lat:.6f}N, {lon:.6f}E +/- {radius_km} km')
    print(f'Bounding box: {bbox}')
    print(f'Size: {size_x:.0f} x {size_y:.0f} m')
    print(f'Resolution: {resolution}x{resolution} px')
    print(f'Meters/pixel: {size_x / resolution:.2f}')
    print()

    # Step 1: Download DEM
    if not args.skip_dem:
        download_dem(bbox, raw_dem)
    else:
        print(f'Skipping DEM download (using {raw_dem})')

    # Step 2: Process DEM -> heightmap
    heightmap_16, min_elev, max_elev, elev_range = process_dem(raw_dem, resolution)

    img = Image.fromarray(heightmap_16, mode='I;16')
    img.save(heightmap_png)
    print(f'  Heightmap saved: {heightmap_png}')

    # Step 3: Download satellite imagery
    if not args.skip_satellite:
        download_satellite_tiles(bbox, zoom_level, texture_png, resolution, cfg)
    else:
        print(f'Skipping satellite download (using {texture_png})')

    # Step 4: Create flat normal map
    create_flat_normal(normal_png)

    # Step 5: Generate metadata JSON
    terrain_data = {
        'center_lat': lat,
        'center_lon': lon,
        'bbox': bbox,
        'size_meters': {'x': round(size_x, 1), 'y': round(size_y, 1)},
        'min_elevation_msl': round(min_elev, 1),
        'max_elevation_msl': round(max_elev, 1),
        'heightmap_range_m': round(elev_range, 1),
        'resolution_px': resolution,
        'meters_per_pixel': round(size_x / resolution, 2),
        'heightmap_file': 'terrain_heightmap.png',
        'texture_file': 'terrain_texture.png',
        'origin_elevation_msl': round(min_elev, 1),
        'zoom_level': zoom_level,
    }

    with open(metadata_json, 'w') as f:
        json.dump(terrain_data, f, indent=2)
    print(f'Metadata saved: {metadata_json}')

    # Step 6: Generate world SDF from template
    if os.path.exists(template_sdf):
        generate_world_sdf(terrain_data, cfg, template_sdf, world_sdf)
    else:
        print(f'Warning: SDF template not found at {template_sdf}')
        print('  Skipping SDF generation. Create the template or run from project root.')

    print()
    print('=== Generation complete ===')
    print(f'Terrain dir: {terrain_dir}')
    print(f'World SDF:   {world_sdf}')
    print()
    print('To test in Gazebo:')
    print('  ros2 launch fiber_nav_bringup simulation.launch.py '
          'world:=terrain_world world_name:=terrain_world '
          'use_px4:=true headless:=false')


if __name__ == '__main__':
    main()
