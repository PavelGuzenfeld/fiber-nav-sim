#!/usr/bin/env python3
"""
2D TERCOM (Terrain Contour Matching) navigation node.

Accumulates rangefinder measurements along the flight path, then
cross-correlates the terrain surface profile against the stored DEM
to produce GPS-independent position fixes.

Algorithm:
1. Accumulate (dead-reckoned_pos, AGL) measurements over a sliding window
2. Compute terrain surface heights: surface_z = vehicle_z - AGL
3. Build a mean-subtracted "measured profile"
4. For each candidate position in a search grid, extract the corresponding
   DEM profile and compute normalized cross-correlation
5. The candidate with highest correlation is the position fix

Subscriptions:
  /fmu/out/vehicle_local_position_v1  - EKF position (dead reckoning source)
  /fmu/in/distance_sensor             - Rangefinder AGL

Publications:
  /tercom/position_fix    - PoseWithCovarianceStamped (NED frame)
  /tercom/diagnostics     - String (JSON: correlation, search stats)

Parameters:
  search_radius_m     - Half-width of search grid [m] (default: 300)
  search_step_m       - Grid step size [m] (default: 12, ~DEM resolution)
  window_size         - Number of measurements in sliding window (default: 50)
  min_spacing_m       - Minimum distance between measurements [m] (default: 10)
  match_interval_s    - Time between TERCOM matches [s] (default: 10)
  min_correlation     - Quality threshold for publishing fix (default: 0.5)
  min_alt_agl_m       - Minimum AGL to accept rangefinder data [m] (default: 20)
"""

import json
import math
import os
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import VehicleLocalPosition, DistanceSensor
from std_msgs.msg import String


@dataclass
class TerrainData:
    """Loaded terrain heightmap and metadata."""
    heightmap: np.ndarray       # (res, res) float64 array of pixel values
    max_val: float              # Max pixel value (65535 for 16-bit)
    elev_range_m: float         # Height range in meters
    min_elev_msl: float         # Minimum elevation MSL
    size_x: float               # Terrain extent in meters (East)
    size_y: float               # Terrain extent in meters (North)
    resolution: int             # Heightmap resolution (pixels per side)
    meters_per_pixel: float     # Spatial resolution


@dataclass
class Measurement:
    """Single terrain measurement."""
    x: float        # EKF position North [m]
    y: float        # EKF position East [m]
    z: float        # EKF altitude (negative down) [m]
    agl: float      # Rangefinder AGL [m]
    surface_z: float  # Computed terrain surface height (Gazebo Z) [m]
    timestamp: float  # ROS time [s]


def load_terrain() -> TerrainData:
    """Load terrain heightmap and metadata, auto-discovering paths."""
    search_paths = [
        os.path.join(os.path.dirname(__file__), '..', 'src', 'fiber_nav_gazebo',
                     'terrain', 'terrain_data.json'),
        '/root/ws/src/fiber-nav-sim/src/fiber_nav_gazebo/terrain/terrain_data.json',
    ]

    terrain_path = None
    for p in search_paths:
        resolved = os.path.realpath(p)
        if os.path.isfile(resolved):
            terrain_path = resolved
            break

    if terrain_path is None:
        raise FileNotFoundError(
            f"terrain_data.json not found in: {search_paths}")

    terrain_dir = os.path.dirname(terrain_path)
    with open(terrain_path) as f:
        meta = json.load(f)

    hm_path = os.path.join(terrain_dir, meta['heightmap_file'])
    hm_img = Image.open(hm_path)
    heightmap = np.array(hm_img, dtype=np.float64)

    max_val = 65535.0 if hm_img.mode in ('I;16', 'I') else 255.0

    return TerrainData(
        heightmap=heightmap,
        max_val=max_val,
        elev_range_m=meta['heightmap_range_m'],
        min_elev_msl=meta['min_elevation_msl'],
        size_x=meta['size_meters']['x'],
        size_y=meta['size_meters']['y'],
        resolution=meta['resolution_px'],
        meters_per_pixel=meta['meters_per_pixel'],
    )


def terrain_height_at(terrain: TerrainData, x_enu: float, y_enu: float) -> float:
    """Get terrain surface height (Gazebo Z) at ENU position.

    Args:
        x_enu: East position [m] from terrain center
        y_enu: North position [m] from terrain center

    Returns:
        Terrain height in Gazebo Z frame (above min_elevation_msl)
    """
    res = terrain.resolution
    px = (x_enu + terrain.size_x / 2.0) / terrain.size_x * (res - 1)
    py = (terrain.size_y / 2.0 - y_enu) / terrain.size_y * (res - 1)

    px = max(0.0, min(float(res - 1), px))
    py = max(0.0, min(float(res - 1), py))

    x0, y0 = int(px), int(py)
    x1 = min(x0 + 1, res - 1)
    y1 = min(y0 + 1, res - 1)
    fx, fy = px - x0, py - y0

    v = (terrain.heightmap[y0, x0] * (1 - fx) * (1 - fy) +
         terrain.heightmap[y0, x1] * fx * (1 - fy) +
         terrain.heightmap[y1, x0] * (1 - fx) * fy +
         terrain.heightmap[y1, x1] * fx * fy)

    return (v / terrain.max_val) * terrain.elev_range_m


def terrain_height_batch(terrain: TerrainData,
                         x_enu: np.ndarray,
                         y_enu: np.ndarray) -> np.ndarray:
    """Vectorized terrain height lookup for arrays of positions.

    Args:
        x_enu: East positions [m], any shape
        y_enu: North positions [m], same shape as x_enu

    Returns:
        Terrain heights (Gazebo Z), same shape as input
    """
    res = terrain.resolution
    px = (x_enu + terrain.size_x / 2.0) / terrain.size_x * (res - 1)
    py = (terrain.size_y / 2.0 - y_enu) / terrain.size_y * (res - 1)

    px = np.clip(px, 0.0, res - 1.0)
    py = np.clip(py, 0.0, res - 1.0)

    x0 = np.floor(px).astype(int)
    y0 = np.floor(py).astype(int)
    x1 = np.minimum(x0 + 1, res - 1)
    y1 = np.minimum(y0 + 1, res - 1)
    fx = px - x0
    fy = py - y0

    v = (terrain.heightmap[y0, x0] * (1 - fx) * (1 - fy) +
         terrain.heightmap[y0, x1] * fx * (1 - fy) +
         terrain.heightmap[y1, x0] * (1 - fx) * fy +
         terrain.heightmap[y1, x1] * fx * fy)

    return (v / terrain.max_val) * terrain.elev_range_m


class TercomNode(Node):
    """2D TERCOM navigation node."""

    def __init__(self):
        super().__init__('tercom_node')

        # Parameters
        self.declare_parameter('search_radius_m', 300.0)
        self.declare_parameter('search_step_m', 12.0)
        self.declare_parameter('window_size', 50)
        self.declare_parameter('min_spacing_m', 10.0)
        self.declare_parameter('match_interval_s', 10.0)
        self.declare_parameter('min_correlation', 0.5)
        self.declare_parameter('min_alt_agl_m', 20.0)

        self.search_radius = self.get_parameter('search_radius_m').value
        self.search_step = self.get_parameter('search_step_m').value
        self.window_size = self.get_parameter('window_size').value
        self.min_spacing = self.get_parameter('min_spacing_m').value
        self.match_interval = self.get_parameter('match_interval_s').value
        self.min_correlation = self.get_parameter('min_correlation').value
        self.min_alt_agl = self.get_parameter('min_alt_agl_m').value

        # Load terrain
        self.terrain = load_terrain()
        self.get_logger().info(
            f"TERCOM loaded DEM: {self.terrain.resolution}x{self.terrain.resolution}, "
            f"{self.terrain.meters_per_pixel:.1f} m/px, "
            f"elev range {self.terrain.elev_range_m:.0f}m "
            f"({self.terrain.min_elev_msl:.0f}-"
            f"{self.terrain.min_elev_msl + self.terrain.elev_range_m:.0f}m MSL)")

        # Pre-compute search grid offsets
        grid_1d = np.arange(-self.search_radius,
                            self.search_radius + self.search_step,
                            self.search_step)
        self.grid_x, self.grid_y = np.meshgrid(grid_1d, grid_1d)
        grid_points = self.grid_x.shape[0] * self.grid_x.shape[1]
        self.get_logger().info(
            f"TERCOM search grid: +/-{self.search_radius:.0f}m, "
            f"step={self.search_step:.0f}m, "
            f"{grid_points} candidates")

        # State
        self.measurements: deque[Measurement] = deque(maxlen=self.window_size)
        self.last_match_time = 0.0
        self.last_pos_x = None
        self.last_pos_y = None
        self.ekf_x = 0.0
        self.ekf_y = 0.0
        self.ekf_z = 0.0
        self.ekf_valid = False
        self.agl = 0.0
        self.agl_valid = False
        self.agl_timestamp = 0.0
        self.match_count = 0
        self.amsl_ref = None  # Set from first valid EKF altitude
        self.pos_cb_count = 0
        self.ds_cb_count = 0
        self.last_status_time = 0.0

        # NED → ENU conversion note:
        # PX4 VehicleLocalPosition uses NED: x=North, y=East, z=Down
        # Gazebo/terrain uses ENU: x=East, y=North, z=Up
        # Conversion: enu_x = ned_y, enu_y = ned_x, enu_z = -ned_z

        # QoS for PX4 topics
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # Subscriptions
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._on_local_position,
            px4_qos)

        self.create_subscription(
            DistanceSensor,
            '/fmu/in/distance_sensor',
            self._on_distance_sensor,
            px4_qos)

        # Publishers
        self.fix_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/tercom/position_fix', 10)
        self.diag_pub = self.create_publisher(
            String, '/tercom/diagnostics', 10)

        # Timer for periodic matching
        self.create_timer(1.0, self._check_match)

        self.get_logger().info("TERCOM node started, waiting for data...")

    def _on_local_position(self, msg: VehicleLocalPosition):
        """Handle EKF local position update."""
        self.pos_cb_count += 1
        if self.pos_cb_count == 1:
            self.get_logger().info(
                f"First VehicleLocalPosition received: xy_valid={msg.xy_valid} "
                f"z_valid={msg.z_valid} x={msg.x:.1f} y={msg.y:.1f} z={msg.z:.1f}")

        if not msg.xy_valid or not msg.z_valid:
            return

        # PX4 NED frame
        self.ekf_x = msg.x  # North
        self.ekf_y = msg.y  # East
        self.ekf_z = msg.z  # Down (negative = up)
        self.ekf_valid = True

        # Set AMSL reference from first valid reading
        if self.amsl_ref is None and msg.ref_alt > 0:
            self.amsl_ref = msg.ref_alt
            self.get_logger().info(f"TERCOM AMSL reference: {self.amsl_ref:.1f}m")

    def _on_distance_sensor(self, msg: DistanceSensor):
        """Handle rangefinder AGL update."""
        self.ds_cb_count += 1
        if self.ds_cb_count == 1:
            self.get_logger().info(
                f"First DistanceSensor received: dist={msg.current_distance:.1f}m")

        if msg.current_distance < 0.1 or msg.current_distance > 500.0:
            return
        self.agl = msg.current_distance
        self.agl_valid = True
        self.agl_timestamp = msg.timestamp / 1e6  # us → s

        # Try to add measurement
        self._try_add_measurement()

    def _try_add_measurement(self):
        """Add a measurement if conditions are met."""
        if not self.ekf_valid or not self.agl_valid:
            return

        if self.agl < self.min_alt_agl:
            return  # Too low, terrain proximity sensor unreliable

        # Check minimum spacing from last measurement
        if self.last_pos_x is not None:
            dx = self.ekf_x - self.last_pos_x
            dy = self.ekf_y - self.last_pos_y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < self.min_spacing:
                return

        # Convert NED → ENU for terrain lookup
        # PX4 NED: x=North, y=East, z=Down
        # Terrain ENU: x=East, y=North
        enu_x = self.ekf_y   # East = NED.y
        enu_y = self.ekf_x   # North = NED.x
        vehicle_alt_gazebo = -self.ekf_z  # Gazebo Z = -NED.z (positive up)

        # Surface height in Gazebo Z frame
        surface_z = vehicle_alt_gazebo - self.agl

        m = Measurement(
            x=self.ekf_x,  # Store in NED for position fix output
            y=self.ekf_y,
            z=self.ekf_z,
            agl=self.agl,
            surface_z=surface_z,
            timestamp=self.agl_timestamp,
        )
        self.measurements.append(m)
        self.last_pos_x = self.ekf_x
        self.last_pos_y = self.ekf_y

    def _check_match(self):
        """Periodically check if we should run a TERCOM match."""
        now = time.time()

        # Periodic status log (every 10s)
        if now - self.last_status_time >= 10.0:
            self.last_status_time = now
            self.get_logger().info(
                f"[status] pos_msgs={self.pos_cb_count} ds_msgs={self.ds_cb_count} "
                f"measurements={len(self.measurements)} ekf_valid={self.ekf_valid} "
                f"agl_valid={self.agl_valid} agl={self.agl:.1f}m "
                f"ekf=({self.ekf_x:.0f},{self.ekf_y:.0f},{self.ekf_z:.0f})")

        if len(self.measurements) < 10:
            return  # Need minimum measurements

        if now - self.last_match_time < self.match_interval:
            return

        self.last_match_time = now
        self._do_tercom_match()

    def _do_tercom_match(self):
        """Run 2D TERCOM cross-correlation."""
        measurements = list(self.measurements)
        n = len(measurements)

        if n < 10:
            return

        # Reference point: center of measurement window (NED)
        ref_x = measurements[n // 2].x  # NED North
        ref_y = measurements[n // 2].y  # NED East

        # Relative positions (NED) from reference
        dx_ned = np.array([m.x - ref_x for m in measurements])
        dy_ned = np.array([m.y - ref_y for m in measurements])

        # Convert to ENU for DEM lookup
        # ENU.x = NED.y (East), ENU.y = NED.x (North)
        dx_enu = dy_ned  # East offsets
        dy_enu = dx_ned  # North offsets

        # Measured surface profile (Gazebo Z)
        measured_surface = np.array([m.surface_z for m in measurements])

        # Normalize: subtract mean (removes altitude bias)
        meas_mean = np.mean(measured_surface)
        meas_norm = measured_surface - meas_mean
        meas_energy = np.sqrt(np.sum(meas_norm ** 2))

        if meas_energy < 0.1:
            # Flat terrain — no variation to match
            self._publish_diagnostics(0.0, 0.0, 0.0, n, "flat_terrain")
            return

        # Reference position in ENU: ref_enu_x = ref_ned_y, ref_enu_y = ref_ned_x
        ref_enu_x = ref_y  # East
        ref_enu_y = ref_x  # North

        # Search grid: each candidate (cx, cy) is an offset from ref position
        # Total candidate ENU positions: (ref_enu_x + grid_x, ref_enu_y + grid_y)
        grid_shape = self.grid_x.shape

        # For each measurement, compute DEM heights at all candidate positions
        # Shape: (grid_h, grid_w, n_measurements)
        dem_profiles = np.zeros((*grid_shape, n))

        for i in range(n):
            query_x = ref_enu_x + self.grid_x + dx_enu[i]
            query_y = ref_enu_y + self.grid_y + dy_enu[i]
            dem_profiles[:, :, i] = terrain_height_batch(
                self.terrain, query_x, query_y)

        # Normalize DEM profiles (subtract mean along measurement axis)
        dem_mean = np.mean(dem_profiles, axis=2, keepdims=True)
        dem_norm = dem_profiles - dem_mean
        dem_energy = np.sqrt(np.sum(dem_norm ** 2, axis=2))

        # Normalized cross-correlation
        # NCC = sum(dem_norm * meas_norm) / (dem_energy * meas_energy)
        correlation = np.sum(dem_norm * meas_norm[np.newaxis, np.newaxis, :],
                             axis=2)
        valid_mask = dem_energy > 0.1
        ncc = np.where(valid_mask,
                       correlation / (dem_energy * meas_energy),
                       -1.0)

        # Find best match
        best_idx = np.unravel_index(np.argmax(ncc), ncc.shape)
        best_ncc = ncc[best_idx]
        best_offset_enu_x = self.grid_x[best_idx]  # East offset
        best_offset_enu_y = self.grid_y[best_idx]  # North offset

        # Convert best position back to NED
        # fix_enu = ref_enu + best_offset
        fix_enu_x = ref_enu_x + best_offset_enu_x
        fix_enu_y = ref_enu_y + best_offset_enu_y

        # ENU → NED: ned_x = enu_y, ned_y = enu_x
        fix_ned_x = fix_enu_y  # North
        fix_ned_y = fix_enu_x  # East

        # Compute position correction (how far off the EKF was)
        # Use the latest measurement as reference
        latest = measurements[-1]
        # EKF position of latest measurement in ENU
        latest_enu_x = latest.y  # East
        latest_enu_y = latest.x  # North
        # TERCOM fix for latest measurement position
        latest_dx_enu = dx_enu[-1]
        latest_dy_enu = dy_enu[-1]
        fix_latest_enu_x = fix_enu_x + (latest_enu_x - ref_enu_x) - latest_dx_enu + dx_enu[-1]
        fix_latest_enu_y = fix_enu_y + (latest_enu_y - ref_enu_y) - dy_enu[-1] + dy_enu[-1]

        # Actually simpler: the fix tells us where the reference point really is.
        # The correction is: fix_pos - ekf_ref_pos
        correction_enu_x = best_offset_enu_x  # East correction
        correction_enu_y = best_offset_enu_y  # North correction
        correction_m = math.sqrt(correction_enu_x**2 + correction_enu_y**2)

        # Compute peak sharpness (ratio of best to second-best non-adjacent peak)
        # This indicates match quality — sharp peak = confident fix
        peak_sharpness = self._compute_peak_sharpness(ncc, best_idx)

        self.match_count += 1

        # Publish fix if quality is good enough
        if best_ncc >= self.min_correlation:
            self._publish_fix(fix_ned_x, fix_ned_y, best_ncc, peak_sharpness)

        self._publish_diagnostics(
            best_ncc, correction_m, peak_sharpness, n, "ok")

        self.get_logger().info(
            f"TERCOM #{self.match_count}: NCC={best_ncc:.3f} "
            f"correction=({correction_enu_x:.1f}E, {correction_enu_y:.1f}N) "
            f"= {correction_m:.1f}m, "
            f"sharpness={peak_sharpness:.2f}, "
            f"n_meas={n}")

    def _compute_peak_sharpness(self, ncc: np.ndarray,
                                 best_idx: tuple) -> float:
        """Compute peak sharpness as ratio of best to mean correlation."""
        best_val = ncc[best_idx]
        if best_val <= 0:
            return 0.0

        # Mask out a 5x5 region around the peak
        mask = ncc.copy()
        r, c = best_idx
        r_lo = max(0, r - 2)
        r_hi = min(mask.shape[0], r + 3)
        c_lo = max(0, c - 2)
        c_hi = min(mask.shape[1], c + 3)
        mask[r_lo:r_hi, c_lo:c_hi] = -1.0

        # Second best peak
        second_best = np.max(mask)
        if second_best <= 0:
            return 10.0  # Very sharp — nothing else matches

        return best_val / max(second_best, 0.01)

    def _publish_fix(self, ned_x: float, ned_y: float,
                     ncc: float, sharpness: float):
        """Publish TERCOM position fix."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'  # NED local frame

        msg.pose.pose.position.x = float(ned_x)
        msg.pose.pose.position.y = float(ned_y)
        msg.pose.pose.position.z = 0.0  # TERCOM doesn't fix altitude

        # Covariance: lower correlation → higher uncertainty
        # Base variance: search_step^2 (can't do better than grid resolution)
        base_var = self.search_step ** 2
        # Scale by inverse correlation quality
        quality = max(ncc, 0.1) * max(min(sharpness, 5.0), 0.1)
        variance = base_var / quality

        # Fill diagonal of 6x6 covariance (x, y, z, roll, pitch, yaw)
        cov = [0.0] * 36
        cov[0] = variance    # x variance
        cov[7] = variance    # y variance
        cov[14] = 1e6        # z: no altitude fix
        cov[21] = 1e6        # roll: no fix
        cov[28] = 1e6        # pitch: no fix
        cov[35] = 1e6        # yaw: no fix
        msg.pose.covariance = cov

        self.fix_pub.publish(msg)

    def _publish_diagnostics(self, ncc: float, correction_m: float,
                             sharpness: float, n_measurements: int,
                             status: str):
        """Publish diagnostics as JSON."""
        diag = {
            'match_count': self.match_count,
            'status': status,
            'ncc': round(ncc, 4),
            'correction_m': round(correction_m, 1),
            'peak_sharpness': round(sharpness, 3),
            'n_measurements': n_measurements,
            'search_radius_m': self.search_radius,
            'search_step_m': self.search_step,
            'ekf_pos_ned': [round(self.ekf_x, 1), round(self.ekf_y, 1)],
        }
        msg = String()
        msg.data = json.dumps(diag)
        self.diag_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TercomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
