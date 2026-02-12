#!/usr/bin/env python3
"""ROS 2 map bridge node for Foxglove Studio Map panel.

Bridges PX4 VehicleGlobalPosition to sensor_msgs/NavSatFix for the Foxglove
Map panel, generates terrain elevation and mission plan GeoJSON overlays,
and publishes altitude data for plotting.

Published topics:
    /vehicle/nav_sat_fix        (sensor_msgs/NavSatFix)   — vehicle dot on Map panel
    /map/terrain_overlay        (foxglove_msgs/GeoJSON)    — terrain elevation heatmap
    /map/mission_plan           (foxglove_msgs/GeoJSON)    — mission waypoints + path
    /vehicle/terrain_agl        (std_msgs/Float64)         — drone height above ground
    /vehicle/terrain_elevation  (std_msgs/Float64)         — ground height MSL under drone
    /vehicle/altitude_msl       (std_msgs/Float64)         — drone altitude MSL

Subscriptions:
    /fmu/out/vehicle_global_position  (px4_msgs/VehicleGlobalPosition)

Usage:
    python3 scripts/map_bridge_node.py
"""

import json
import math
import os

import numpy as np
from PIL import Image
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    HistoryPolicy,
    ReliabilityPolicy,
)

from px4_msgs.msg import VehicleGlobalPosition
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64

# foxglove_msgs may not be installed — fail gracefully
try:
    from foxglove_msgs.msg import GeoJSON
    HAS_GEOJSON = True
except ImportError:
    HAS_GEOJSON = False


def _elevation_color(normalized: float) -> str:
    """Map 0..1 elevation to green->yellow->red hex color."""
    r = int(min(1.0, 2.0 * normalized) * 200)
    g = int(min(1.0, 2.0 * (1.0 - normalized)) * 200)
    return f'#{r:02x}{g:02x}00'


class MapBridgeNode(Node):
    """Bridges PX4 global position to Foxglove Map panel topics."""

    def __init__(self):
        super().__init__('map_bridge_node')

        # --- Load terrain data (reuse auto-discovery from terrain_gis_node) ---
        terrain_path = self._find_terrain_data()
        self._heightmap = None
        self._metadata = None

        if terrain_path:
            self.get_logger().info(f'Loading terrain data from: {terrain_path}')
            terrain_dir = os.path.dirname(terrain_path)

            with open(terrain_path) as f:
                self._metadata = json.load(f)

            hm_path = os.path.join(terrain_dir, self._metadata['heightmap_file'])
            hm_img = Image.open(hm_path)
            self._heightmap = np.array(hm_img, dtype=np.float64)

            if hm_img.mode in ('I;16', 'I'):
                self._max_val = 65535.0
            else:
                self._max_val = 255.0

            self._res = self._metadata['resolution_px']
            self._min_elev = self._metadata['min_elevation_msl']
            self._elev_range = self._metadata['heightmap_range_m']
            self._bbox = self._metadata['bbox']
            self._size_x = self._metadata['size_meters']['x']
            self._size_y = self._metadata['size_meters']['y']
            self._center_lat = self._metadata['center_lat']
            self._center_lon = self._metadata['center_lon']
        else:
            self.get_logger().warn('No terrain data found — terrain overlay disabled')

        # --- QoS profiles ---
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Subscription ---
        self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self._global_pos_cb,
            qos_best_effort,
        )

        # --- Publishers ---
        self._fix_pub = self.create_publisher(NavSatFix, '/vehicle/nav_sat_fix', 10)
        self._agl_pub = self.create_publisher(Float64, '/vehicle/terrain_agl', 10)
        self._terrain_elev_pub = self.create_publisher(
            Float64, '/vehicle/terrain_elevation', 10
        )
        self._alt_msl_pub = self.create_publisher(
            Float64, '/vehicle/altitude_msl', 10
        )

        if HAS_GEOJSON:
            self._geojson_pub = self.create_publisher(
                GeoJSON, '/map/terrain_overlay', qos_latched
            )
            self._mission_pub = self.create_publisher(
                GeoJSON, '/map/mission_plan', qos_latched
            )
        else:
            self._geojson_pub = None
            self._mission_pub = None
            self.get_logger().warn(
                'foxglove_msgs not installed — GeoJSON overlays disabled'
            )

        # --- Publish terrain overlay once ---
        if self._heightmap is not None and self._geojson_pub is not None:
            self._publish_terrain_overlay()

        # --- Publish mission plan once ---
        if self._metadata is not None and self._mission_pub is not None:
            self._publish_mission_plan()

        self.get_logger().info('Map bridge node ready')

    def _find_terrain_data(self) -> str | None:
        """Auto-discover terrain_data.json."""
        candidates = [
            os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'src', 'fiber_nav_gazebo', 'terrain', 'terrain_data.json',
            ),
            '/root/ws/src/fiber-nav-sim/src/fiber_nav_gazebo/terrain/terrain_data.json',
        ]
        for c in candidates:
            if os.path.exists(c):
                return c
        return None

    def _find_mission_config(self) -> str | None:
        """Find mission config YAML from MISSION env var."""
        mission = os.environ.get('MISSION', '')
        if not mission:
            return None

        mission_map = {
            'vtol_terrain': 'terrain_mission.yaml',
            'vtol_canyon': 'canyon_mission.yaml',
        }
        yaml_file = mission_map.get(mission)
        if not yaml_file:
            return None

        candidates = [
            os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'src', 'fiber_nav_mode', 'config', yaml_file,
            ),
            f'/root/ws/src/fiber-nav-sim/src/fiber_nav_mode/config/{yaml_file}',
        ]
        for c in candidates:
            if os.path.exists(c):
                return c
        return None

    def _ned_to_latlon(self, x_north: float, y_east: float) -> tuple[float, float]:
        """Convert NED offset (meters) from home to lat/lon."""
        bbox = self._bbox
        lat = self._center_lat + (x_north / self._size_x) * (
            bbox['north'] - bbox['south']
        )
        lon = self._center_lon + (y_east / self._size_y) * (
            bbox['east'] - bbox['west']
        )
        return lat, lon

    def _global_pos_cb(self, msg: VehicleGlobalPosition):
        """Convert VehicleGlobalPosition -> NavSatFix + altitude data."""
        # --- NavSatFix ---
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'map'

        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS

        fix.latitude = msg.lat
        fix.longitude = msg.lon
        fix.altitude = msg.alt

        # Covariance (diagonal: east, north, up)
        eph2 = msg.eph * msg.eph
        epv2 = msg.epv * msg.epv
        fix.position_covariance = [
            eph2, 0.0, 0.0,
            0.0, eph2, 0.0,
            0.0, 0.0, epv2,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self._fix_pub.publish(fix)

        # --- Drone altitude MSL ---
        alt_msg = Float64()
        alt_msg.data = msg.alt
        self._alt_msl_pub.publish(alt_msg)

        # --- Terrain elevation + AGL ---
        if self._heightmap is not None:
            terrain_msl = self._terrain_height_msl(msg.lat, msg.lon)

            elev_msg = Float64()
            elev_msg.data = terrain_msl
            self._terrain_elev_pub.publish(elev_msg)

            agl = msg.alt - terrain_msl
            agl_msg = Float64()
            agl_msg.data = agl
            self._agl_pub.publish(agl_msg)

    def _latlon_to_pixel(self, lat: float, lon: float) -> tuple[float, float]:
        """Convert lat/lon to heightmap pixel coordinates."""
        bbox = self._bbox
        py = (bbox['north'] - lat) / (bbox['north'] - bbox['south']) * (self._res - 1)
        px = (lon - bbox['west']) / (bbox['east'] - bbox['west']) * (self._res - 1)
        return px, py

    def _pixel_to_latlon(self, px: float, py: float) -> tuple[float, float]:
        """Convert heightmap pixel coordinates to lat/lon."""
        bbox = self._bbox
        lat = bbox['north'] - (py / (self._res - 1)) * (bbox['north'] - bbox['south'])
        lon = bbox['west'] + (px / (self._res - 1)) * (bbox['east'] - bbox['west'])
        return lat, lon

    def _sample_heightmap(self, px: float, py: float) -> float:
        """Sample heightmap at pixel coords, return Gazebo Z (above min_elev)."""
        px = max(0.0, min(float(self._res - 1), px))
        py = max(0.0, min(float(self._res - 1), py))

        x0 = int(px)
        y0 = int(py)
        x1 = min(x0 + 1, self._res - 1)
        y1 = min(y0 + 1, self._res - 1)

        fx = px - x0
        fy = py - y0

        v = (float(self._heightmap[y0, x0]) * (1 - fx) * (1 - fy) +
             float(self._heightmap[y0, x1]) * fx * (1 - fy) +
             float(self._heightmap[y1, x0]) * (1 - fx) * fy +
             float(self._heightmap[y1, x1]) * fx * fy)

        return (v / self._max_val) * self._elev_range

    def _terrain_height_msl(self, lat: float, lon: float) -> float:
        """Get terrain height in MSL at given lat/lon."""
        px, py = self._latlon_to_pixel(lat, lon)
        gz_z = self._sample_heightmap(px, py)
        return self._min_elev + gz_z

    def _publish_mission_plan(self):
        """Load mission waypoints and publish as GeoJSON on the map."""
        config_path = self._find_mission_config()
        if not config_path:
            self.get_logger().info('No MISSION env var set — mission plan overlay skipped')
            return

        with open(config_path) as f:
            config = yaml.safe_load(f)

        params = config.get('vtol_navigation_node', {}).get('ros__parameters', {})
        wp_data = params.get('waypoints', {})
        wp_x = wp_data.get('x', [])
        wp_y = wp_data.get('y', [])

        if not wp_x or not wp_y or len(wp_x) != len(wp_y):
            self.get_logger().warn('Invalid waypoints in mission config')
            return

        # Convert NED waypoints to lat/lon
        home_lat, home_lon = self._center_lat, self._center_lon
        wp_coords = []
        for x, y in zip(wp_x, wp_y):
            lat, lon = self._ned_to_latlon(x, y)
            wp_coords.append((lat, lon))

        features = []

        # Home marker
        features.append({
            'type': 'Feature',
            'geometry': {
                'type': 'Point',
                'coordinates': [home_lon, home_lat],
            },
            'properties': {
                'name': 'HOME',
                'marker-color': '#00ff00',
                'marker-size': 'large',
            },
        })

        # Outbound path: home -> WP0 -> WP1 -> ... -> WPn
        outbound_coords = [[home_lon, home_lat]]
        for lat, lon in wp_coords:
            outbound_coords.append([lon, lat])

        features.append({
            'type': 'Feature',
            'geometry': {
                'type': 'LineString',
                'coordinates': outbound_coords,
            },
            'properties': {
                'name': 'Outbound',
                'stroke': '#ff8800',
                'stroke-width': 3,
                'stroke-opacity': 0.9,
            },
        })

        # Return path: WPn -> home (dashed)
        last_lat, last_lon = wp_coords[-1]
        features.append({
            'type': 'Feature',
            'geometry': {
                'type': 'LineString',
                'coordinates': [
                    [last_lon, last_lat],
                    [home_lon, home_lat],
                ],
            },
            'properties': {
                'name': 'Return',
                'stroke': '#0088ff',
                'stroke-width': 2,
                'stroke-opacity': 0.7,
                'stroke-dasharray': '8 4',
            },
        })

        # Waypoint markers
        for i, (lat, lon) in enumerate(wp_coords):
            features.append({
                'type': 'Feature',
                'geometry': {
                    'type': 'Point',
                    'coordinates': [lon, lat],
                },
                'properties': {
                    'name': f'WP{i}',
                    'marker-color': '#ff4400',
                    'marker-size': 'medium',
                },
            })

        # FW acceptance radius circles (approximate with 24-point polygons)
        fw_radius = params.get('fw_accept_radius', 60.0)
        for i, (lat, lon) in enumerate(wp_coords):
            circle_coords = []
            for a in range(25):
                angle = 2 * math.pi * a / 24
                # Approximate meters to degrees
                dlat = (fw_radius * math.cos(angle) / self._size_x) * (
                    self._bbox['north'] - self._bbox['south']
                )
                dlon = (fw_radius * math.sin(angle) / self._size_y) * (
                    self._bbox['east'] - self._bbox['west']
                )
                circle_coords.append([lon + dlon, lat + dlat])

            features.append({
                'type': 'Feature',
                'geometry': {
                    'type': 'Polygon',
                    'coordinates': [circle_coords],
                },
                'properties': {
                    'name': f'WP{i} radius',
                    'stroke': '#ff4400',
                    'stroke-width': 1,
                    'stroke-opacity': 0.5,
                    'fill': '#ff4400',
                    'fill-opacity': 0.08,
                },
            })

        geojson_obj = {
            'type': 'FeatureCollection',
            'features': features,
        }

        msg = GeoJSON()
        msg.geojson = json.dumps(geojson_obj)
        self._mission_pub.publish(msg)

        self.get_logger().info(
            f'Published mission plan: {len(wp_coords)} waypoints, '
            f'accept_radius={fw_radius}m'
        )

    def _publish_terrain_overlay(self):
        """Generate and publish terrain elevation GeoJSON overlay."""
        grid_size = 30
        bbox = self._bbox

        lat_step = (bbox['north'] - bbox['south']) / grid_size
        lon_step = (bbox['east'] - bbox['west']) / grid_size

        # Find min/max elevation for normalization
        elevations = []
        for row in range(grid_size):
            for col in range(grid_size):
                lat = bbox['north'] - (row + 0.5) * lat_step
                lon = bbox['west'] + (col + 0.5) * lon_step
                elevations.append(self._terrain_height_msl(lat, lon))

        elev_min = min(elevations)
        elev_max = max(elevations)
        elev_range = elev_max - elev_min if elev_max > elev_min else 1.0

        # Build GeoJSON FeatureCollection
        features = []
        idx = 0
        for row in range(grid_size):
            for col in range(grid_size):
                s = bbox['north'] - (row + 1) * lat_step
                n = bbox['north'] - row * lat_step
                w = bbox['west'] + col * lon_step
                e = bbox['west'] + (col + 1) * lon_step

                elev = elevations[idx]
                norm = (elev - elev_min) / elev_range
                color = _elevation_color(norm)

                feature = {
                    'type': 'Feature',
                    'geometry': {
                        'type': 'Polygon',
                        'coordinates': [[
                            [w, s], [e, s], [e, n], [w, n], [w, s],
                        ]],
                    },
                    'properties': {
                        'elevation_msl': round(elev, 1),
                        'fill': color,
                        'fill-opacity': 0.4,
                        'stroke': color,
                        'stroke-width': 0,
                        'stroke-opacity': 0,
                    },
                }
                features.append(feature)
                idx += 1

        geojson_obj = {
            'type': 'FeatureCollection',
            'features': features,
        }

        msg = GeoJSON()
        msg.geojson = json.dumps(geojson_obj)
        self._geojson_pub.publish(msg)

        self.get_logger().info(
            f'Published terrain overlay: {grid_size}x{grid_size} grid, '
            f'elev {elev_min:.0f}-{elev_max:.0f}m MSL'
        )


def main():
    rclpy.init()
    node = MapBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
