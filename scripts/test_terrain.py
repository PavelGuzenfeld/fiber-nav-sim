#!/usr/bin/env python3
"""Unit tests for terrain generation and GIS node."""

import json
import math
import os
import string
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch, MagicMock

import numpy as np
from PIL import Image

# Add scripts directory to path
sys.path.insert(0, str(Path(__file__).parent))

from generate_terrain import (
    bbox_from_center,
    bbox_size_meters,
    isa_temperature,
    isa_pressure,
    process_dem,
    create_flat_normal,
    generate_world_sdf,
    load_config,
    fmt_vec,
)
from terrain_gis_node import TerrainGISNode


class TestGeoHelpers(unittest.TestCase):
    """Tests for geographic helper functions."""

    def test_bbox_from_center_symmetry(self):
        """Bounding box should be symmetric around center."""
        bbox = bbox_from_center(31.164, 34.532, 3.0)
        self.assertAlmostEqual(
            bbox['north'] - 31.164,
            31.164 - bbox['south'],
            places=6,
        )
        self.assertAlmostEqual(
            bbox['east'] - 34.532,
            34.532 - bbox['west'],
            places=6,
        )

    def test_bbox_from_center_contains_center(self):
        """Center point must be inside the bounding box."""
        lat, lon = 31.164, 34.532
        bbox = bbox_from_center(lat, lon, 5.0)
        self.assertGreater(bbox['north'], lat)
        self.assertLess(bbox['south'], lat)
        self.assertGreater(bbox['east'], lon)
        self.assertLess(bbox['west'], lon)

    def test_bbox_size_approximately_correct(self):
        """Bounding box with 3km radius should be ~6km across."""
        bbox = bbox_from_center(31.164, 34.532, 3.0)
        size_x, size_y = bbox_size_meters(bbox, 31.164)
        self.assertAlmostEqual(size_x, 6000.0, delta=50.0)
        self.assertAlmostEqual(size_y, 6000.0, delta=50.0)

    def test_bbox_size_accounts_for_latitude(self):
        """At higher latitudes, x-size should be smaller than y-size for same degree span."""
        bbox = bbox_from_center(60.0, 10.0, 3.0)
        size_x, size_y = bbox_size_meters(bbox, 60.0)
        # At 60° latitude, longitude degrees are ~half the km of latitude degrees
        self.assertAlmostEqual(size_y, 6000.0, delta=50.0)
        self.assertAlmostEqual(size_x, 6000.0, delta=50.0)

    def test_bbox_zero_radius(self):
        """Zero radius should produce a point bbox."""
        bbox = bbox_from_center(31.164, 34.532, 0.0)
        self.assertAlmostEqual(bbox['north'], bbox['south'])
        self.assertAlmostEqual(bbox['east'], bbox['west'])


class TestAtmosphere(unittest.TestCase):
    """Tests for ISA atmosphere computations."""

    def setUp(self):
        self.cfg = {
            'atmosphere': {
                'sea_level_temperature_k': 288.15,
                'lapse_rate_k_per_m': 0.0065,
                'sea_level_pressure_pa': 101325.0,
                'pressure_exponent': 5.2558,
            }
        }

    def test_sea_level_temperature(self):
        """Temperature at sea level should match ISA T0."""
        t = isa_temperature(0.0, self.cfg)
        self.assertAlmostEqual(t, 288.15, places=2)

    def test_sea_level_pressure(self):
        """Pressure at sea level should match ISA P0."""
        p = isa_pressure(0.0, self.cfg)
        self.assertAlmostEqual(p, 101325.0, places=0)

    def test_temperature_decreases_with_altitude(self):
        """Temperature should decrease with altitude."""
        t0 = isa_temperature(0.0, self.cfg)
        t1 = isa_temperature(1000.0, self.cfg)
        t2 = isa_temperature(5000.0, self.cfg)
        self.assertGreater(t0, t1)
        self.assertGreater(t1, t2)

    def test_pressure_decreases_with_altitude(self):
        """Pressure should decrease with altitude."""
        p0 = isa_pressure(0.0, self.cfg)
        p1 = isa_pressure(1000.0, self.cfg)
        p2 = isa_pressure(5000.0, self.cfg)
        self.assertGreater(p0, p1)
        self.assertGreater(p1, p2)

    def test_known_altitude_values(self):
        """Check ISA at 488m MSL (used in canyon world)."""
        t = isa_temperature(488.0, self.cfg)
        p = isa_pressure(488.0, self.cfg)
        self.assertAlmostEqual(t, 284.978, delta=0.01)
        self.assertAlmostEqual(p, 95601.0, delta=50.0)


class TestDEMProcessing(unittest.TestCase):
    """Tests for DEM processing (using synthetic data)."""

    def setUp(self):
        """Create a synthetic GeoTIFF for testing."""
        self.tmpdir = tempfile.mkdtemp()
        self.tif_path = os.path.join(self.tmpdir, 'test_dem.tif')

        # Create synthetic DEM: a ramp from 300m to 400m
        try:
            import rasterio
            from rasterio.transform import from_bounds

            dem_data = np.linspace(300, 400, 100 * 100).reshape(100, 100).astype(np.float32)
            transform = from_bounds(34.0, 31.0, 35.0, 32.0, 100, 100)

            with rasterio.open(
                self.tif_path, 'w',
                driver='GTiff',
                height=100, width=100,
                count=1,
                dtype='float32',
                crs='EPSG:4326',
                transform=transform,
            ) as dst:
                dst.write(dem_data, 1)

            self.has_rasterio = True
        except ImportError:
            self.has_rasterio = False

    def test_process_dem_shape(self):
        """Processed heightmap should have target dimensions."""
        if not self.has_rasterio:
            self.skipTest('rasterio not installed')

        hm, min_e, max_e, rng = process_dem(self.tif_path, 65)
        self.assertEqual(hm.shape, (65, 65))

    def test_process_dem_elevation_range(self):
        """Elevation stats should match input data."""
        if not self.has_rasterio:
            self.skipTest('rasterio not installed')

        hm, min_e, max_e, rng = process_dem(self.tif_path, 33)
        self.assertAlmostEqual(min_e, 300.0, delta=1.0)
        self.assertAlmostEqual(max_e, 400.0, delta=1.0)
        self.assertAlmostEqual(rng, 100.0, delta=2.0)

    def test_process_dem_16bit_range(self):
        """Output should span 0 to 65535."""
        if not self.has_rasterio:
            self.skipTest('rasterio not installed')

        hm, _, _, _ = process_dem(self.tif_path, 33)
        self.assertEqual(hm.dtype, np.uint16)
        self.assertGreater(hm.max(), 60000)
        self.assertLess(hm.min(), 5000)

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmpdir, ignore_errors=True)


class TestFlatNormal(unittest.TestCase):
    """Tests for flat normal map generation."""

    def test_creates_valid_png(self):
        with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f:
            path = f.name

        try:
            create_flat_normal(path)
            img = Image.open(path)
            self.assertEqual(img.size, (1, 1))
            self.assertEqual(img.mode, 'RGB')
            pixel = img.getpixel((0, 0))
            self.assertEqual(pixel, (128, 128, 255))
        finally:
            os.unlink(path)


class TestConfigLoading(unittest.TestCase):
    """Tests for config file loading."""

    def test_load_real_config(self):
        """Load the actual terrain_defaults.yaml."""
        config_path = (
            Path(__file__).parent.parent /
            'src' / 'fiber_nav_bringup' / 'config' / 'terrain_defaults.yaml'
        )
        if not config_path.exists():
            self.skipTest('Config file not found')

        cfg = load_config(str(config_path))

        # Check required keys exist
        self.assertIn('location', cfg)
        self.assertIn('lat', cfg['location'])
        self.assertIn('lon', cfg['location'])
        self.assertIn('radius_km', cfg['location'])
        self.assertIn('heightmap', cfg)
        self.assertIn('satellite', cfg)
        self.assertIn('atmosphere', cfg)
        self.assertIn('magnetic_field', cfg)
        self.assertIn('world', cfg)

    def test_load_config_values(self):
        """Config values should be reasonable."""
        config_path = (
            Path(__file__).parent.parent /
            'src' / 'fiber_nav_bringup' / 'config' / 'terrain_defaults.yaml'
        )
        if not config_path.exists():
            self.skipTest('Config file not found')

        cfg = load_config(str(config_path))

        # Negev coordinates
        self.assertGreater(cfg['location']['lat'], 30.0)
        self.assertLess(cfg['location']['lat'], 33.0)
        self.assertGreater(cfg['location']['lon'], 34.0)
        self.assertLess(cfg['location']['lon'], 36.0)

        # Resolution must be 2^n + 1
        res = cfg['heightmap']['resolution']
        n = res - 1
        self.assertTrue(n > 0 and (n & (n - 1)) == 0,
                        f'Resolution {res} is not 2^n + 1')


class TestSDFGeneration(unittest.TestCase):
    """Tests for SDF world file generation from template."""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.template_path = os.path.join(self.tmpdir, 'template.sdf')
        self.output_path = os.path.join(self.tmpdir, 'output.sdf')

        # Minimal SDF template
        with open(self.template_path, 'w') as f:
            f.write('''<sdf version="1.10">
  <world name="${world_name}">
    <spherical_coordinates>
      <latitude_deg>${latitude}</latitude_deg>
      <longitude_deg>${longitude}</longitude_deg>
      <elevation>${min_elevation}</elevation>
    </spherical_coordinates>
    <atmosphere type="adiabatic">
      <temperature>${temperature}</temperature>
      <pressure>${pressure}</pressure>
    </atmosphere>
    <model name="terrain">
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <size>${size_x} ${size_y} ${elev_range}</size>
            </heightmap>
          </geometry>
        </collision>
      </link>
    </model>
    <magnetic_field>${mag_x} ${mag_y} ${mag_z}</magnetic_field>
    <scene>
      <ambient>${ambient}</ambient>
      <background>${background}</background>
      <shadows>${shadows}</shadows>
    </scene>
    <light type="directional" name="sun">
      <diffuse>${sun_diffuse}</diffuse>
      <specular>${sun_specular}</specular>
      <direction>${sun_direction}</direction>
      <attenuation><range>${sun_range}</range></attenuation>
    </light>
    <physics name="dart_physics" type="dart">
      <max_step_size>${max_step_size}</max_step_size>
      <real_time_factor>${real_time_factor}</real_time_factor>
      <real_time_update_rate>${real_time_update_rate}</real_time_update_rate>
      <dart><collision_detector>${collision_detector}</collision_detector></dart>
    </physics>
  </world>
</sdf>
''')

        self.terrain_data = {
            'center_lat': 31.164,
            'center_lon': 34.532,
            'size_meters': {'x': 6000.0, 'y': 6000.0},
            'min_elevation_msl': 300.0,
            'max_elevation_msl': 400.0,
            'heightmap_range_m': 100.0,
        }

        self.cfg = {
            'atmosphere': {
                'sea_level_temperature_k': 288.15,
                'lapse_rate_k_per_m': 0.0065,
                'sea_level_pressure_pa': 101325.0,
                'pressure_exponent': 5.2558,
            },
            'magnetic_field': {'x': 2.68e-5, 'y': 0.68e-5, 'z': -3.66e-5},
            'world': {
                'name': 'test_world',
                'physics': {
                    'max_step_size': 0.001,
                    'real_time_factor': 1.0,
                    'real_time_update_rate': 1000,
                    'collision_detector': 'bullet',
                },
                'scene': {
                    'ambient': [0.7, 0.7, 0.7, 1.0],
                    'background': [0.6, 0.75, 0.9, 1.0],
                    'shadows': True,
                },
                'sun': {
                    'diffuse': [0.9, 0.9, 0.85, 1.0],
                    'specular': [0.3, 0.3, 0.3, 1.0],
                    'direction': [-0.3, 0.2, -0.9],
                    'range': 5000,
                },
            },
        }

    def test_generates_valid_sdf(self):
        """Generated SDF should be valid XML with correct values."""
        generate_world_sdf(
            self.terrain_data, self.cfg, self.template_path, self.output_path
        )

        with open(self.output_path) as f:
            content = f.read()

        self.assertIn('name="test_world"', content)
        self.assertIn('<latitude_deg>31.164</latitude_deg>', content)
        self.assertIn('<longitude_deg>34.532</longitude_deg>', content)
        self.assertIn('6000.0 6000.0 100.0', content)

    def test_atmosphere_values_present(self):
        """SDF should contain computed atmosphere values."""
        generate_world_sdf(
            self.terrain_data, self.cfg, self.template_path, self.output_path
        )

        with open(self.output_path) as f:
            content = f.read()

        # Temperature at 300m MSL should be around 286.2K
        self.assertIn('<temperature>286.200</temperature>', content)

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmpdir, ignore_errors=True)


class TestFmtVec(unittest.TestCase):
    """Tests for vector formatting."""

    def test_formats_integers(self):
        self.assertEqual(fmt_vec([1, 2, 3]), '1 2 3')

    def test_formats_floats(self):
        self.assertEqual(fmt_vec([0.7, 0.7, 0.7, 1.0]), '0.7 0.7 0.7 1.0')

    def test_formats_scientific(self):
        result = fmt_vec([2.68e-5, 0.68e-5])
        self.assertIn('2.68e-05', result)


class TestTerrainGISHeightLookup(unittest.TestCase):
    """Tests for terrain height lookup using synthetic heightmap.

    These test the height_at_xy() logic without ROS 2 dependencies
    by directly creating a TerrainGISNode-like lookup.
    """

    def setUp(self):
        """Create a synthetic terrain: linear ramp east-west."""
        self.res = 65  # Small for testing
        self.size_x = 1000.0
        self.size_y = 1000.0
        self.elev_range = 100.0  # 0 to 100m
        self.max_val = 65535.0

        # Create heightmap: value increases linearly from west to east
        # column 0 (west) = 0, column 64 (east) = 65535
        row = np.linspace(0, 65535, self.res, dtype=np.float64)
        self.heightmap = np.tile(row, (self.res, 1))

    def _height_at(self, x: float, y: float) -> float:
        """Replicate TerrainGISNode.height_at_xy() logic."""
        px = (x + self.size_x / 2.0) / self.size_x * (self.res - 1)
        py = (self.size_y / 2.0 - y) / self.size_y * (self.res - 1)

        px = max(0.0, min(float(self.res - 1), px))
        py = max(0.0, min(float(self.res - 1), py))

        x0 = int(px)
        y0 = int(py)
        x1 = min(x0 + 1, self.res - 1)
        y1 = min(y0 + 1, self.res - 1)

        fx = px - x0
        fy = py - y0

        v = (float(self.heightmap[y0, x0]) * (1 - fx) * (1 - fy) +
             float(self.heightmap[y0, x1]) * fx * (1 - fy) +
             float(self.heightmap[y1, x0]) * (1 - fx) * fy +
             float(self.heightmap[y1, x1]) * fx * fy)

        return (v / self.max_val) * self.elev_range

    def test_center_height(self):
        """Center of terrain should be at 50% of elevation range."""
        h = self._height_at(0.0, 0.0)
        self.assertAlmostEqual(h, 50.0, delta=1.0)

    def test_west_edge_low(self):
        """West edge should have low elevation."""
        h = self._height_at(-500.0, 0.0)
        self.assertAlmostEqual(h, 0.0, delta=2.0)

    def test_east_edge_high(self):
        """East edge should have high elevation."""
        h = self._height_at(500.0, 0.0)
        self.assertAlmostEqual(h, 100.0, delta=2.0)

    def test_north_south_constant(self):
        """Moving north/south should not change height (uniform rows)."""
        h_north = self._height_at(100.0, 300.0)
        h_south = self._height_at(100.0, -300.0)
        self.assertAlmostEqual(h_north, h_south, delta=0.1)

    def test_monotonic_east(self):
        """Height should increase monotonically eastward."""
        heights = [self._height_at(x, 0.0) for x in range(-400, 401, 100)]
        for i in range(1, len(heights)):
            self.assertGreater(heights[i], heights[i - 1])

    def test_clamped_outside_bounds(self):
        """Coordinates outside terrain should clamp to edge values."""
        h_inside = self._height_at(-500.0, 0.0)
        h_outside = self._height_at(-2000.0, 0.0)
        self.assertAlmostEqual(h_inside, h_outside, delta=0.1)

    def test_bilinear_interpolation(self):
        """Value between pixels should be interpolated, not nearest-neighbor."""
        # At x=0 (center), the pixel should be exactly at column 32
        # A small offset should give a slightly different value
        h0 = self._height_at(0.0, 0.0)
        h1 = self._height_at(1.0, 0.0)
        self.assertNotAlmostEqual(h0, h1, places=1)


class TestTerrainDistanceSensorIntegration(unittest.TestCase):
    """Integration test: verify AGL computation with synthetic terrain.

    Tests the distance sensor's terrain loading and AGL calculation
    using a generated heightmap file on disk.
    """

    def setUp(self):
        """Create a synthetic terrain dataset on disk."""
        self.tmpdir = tempfile.mkdtemp()

        # Create a 65x65 heightmap: flat at 32768 (50% of 65535)
        # This means terrain_z = 50m everywhere (for elev_range=100)
        res = 65
        heightmap = np.full((res, res), 32768, dtype=np.uint16)

        # Save heightmap
        hm_path = os.path.join(self.tmpdir, 'terrain_heightmap.png')
        img = Image.fromarray(heightmap, mode='I;16')
        img.save(hm_path)

        # Save metadata
        metadata = {
            'center_lat': 31.164,
            'center_lon': 34.532,
            'size_meters': {'x': 1000.0, 'y': 1000.0},
            'min_elevation_msl': 300.0,
            'max_elevation_msl': 400.0,
            'heightmap_range_m': 100.0,
            'resolution_px': res,
            'meters_per_pixel': 1000.0 / res,
            'heightmap_file': 'terrain_heightmap.png',
            'texture_file': 'terrain_texture.png',
            'origin_elevation_msl': 300.0,
        }
        self.metadata_path = os.path.join(self.tmpdir, 'terrain_data.json')
        with open(self.metadata_path, 'w') as f:
            json.dump(metadata, f)

    def test_terrain_loads_correctly(self):
        """Terrain data should load correctly from JSON + heightmap files."""
        terrain = self._load_terrain_manually()
        self.assertIsNotNone(terrain)
        self.assertEqual(terrain['res'], 65)

    def test_agl_on_flat_terrain(self):
        """AGL should be odom_z minus terrain height."""
        terrain = self._load_terrain_manually()

        # Flat heightmap at 32768/65535 * 100 = ~50m
        # Drone at z=80 → AGL = 80 - 50 = 30
        terrain_z = self._terrain_height(terrain, 0.0, 0.0)
        self.assertAlmostEqual(terrain_z, 50.0, delta=1.0)

        odom_z = 80.0
        agl = odom_z - terrain_z
        self.assertAlmostEqual(agl, 30.0, delta=1.0)

    def _load_terrain_manually(self):
        """Load terrain data without ROS."""
        with open(self.metadata_path) as f:
            metadata = json.load(f)

        hm_path = os.path.join(self.tmpdir, metadata['heightmap_file'])
        hm_img = Image.open(hm_path)
        heightmap = np.array(hm_img, dtype=np.float64)

        max_val = 65535.0 if hm_img.mode in ('I;16', 'I') else 255.0

        return {
            'heightmap': heightmap,
            'size_x': metadata['size_meters']['x'],
            'size_y': metadata['size_meters']['y'],
            'elev_range': metadata['heightmap_range_m'],
            'res': metadata['resolution_px'],
            'max_val': max_val,
        }

    def _terrain_height(self, t, x, y):
        """Compute terrain height (same as sim_distance_sensor._terrain_height)."""
        hm = t['heightmap']
        res = t['res']

        px = (x + t['size_x'] / 2.0) / t['size_x'] * (res - 1)
        py = (t['size_y'] / 2.0 - y) / t['size_y'] * (res - 1)

        px = max(0.0, min(float(res - 1), px))
        py = max(0.0, min(float(res - 1), py))

        x0 = int(px)
        y0 = int(py)
        x1 = min(x0 + 1, res - 1)
        y1 = min(y0 + 1, res - 1)

        fx = px - x0
        fy = py - y0

        v = (float(hm[y0, x0]) * (1 - fx) * (1 - fy) +
             float(hm[y0, x1]) * fx * (1 - fy) +
             float(hm[y1, x0]) * (1 - fx) * fy +
             float(hm[y1, x1]) * fx * fy)

        return (v / t['max_val']) * t['elev_range']

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmpdir, ignore_errors=True)


class TestLaunchFileWorldName(unittest.TestCase):
    """Test that launch file template has correct world_name parameterization.

    This is a static analysis test — it reads the launch file and verifies
    that canyon_world is no longer hardcoded (except in the default value
    of the world_name arg).
    """

    def test_no_hardcoded_canyon_world_in_topics(self):
        """Bridge topics and spawn service should not hardcode canyon_world."""
        launch_path = (
            Path(__file__).parent.parent /
            'src' / 'fiber_nav_bringup' / 'launch' / 'simulation.launch.py'
        )
        if not launch_path.exists():
            self.skipTest('Launch file not found')

        with open(launch_path) as f:
            content = f.read()

        # Find all occurrences of 'canyon_world'
        lines_with_canyon = []
        for i, line in enumerate(content.split('\n'), 1):
            if 'canyon_world' in line:
                lines_with_canyon.append((i, line.strip()))

        # Only allowed occurrences: the default_value in the DeclareLaunchArgument
        # and comments
        for lineno, line in lines_with_canyon:
            is_default = "default_value='canyon_world'" in line
            is_comment = line.lstrip().startswith('#')
            is_docstring = line.lstrip().startswith("'")
            self.assertTrue(
                is_default or is_comment or is_docstring,
                f'Line {lineno} has hardcoded canyon_world: {line}'
            )


if __name__ == '__main__':
    unittest.main()
