#!/usr/bin/env python3
"""ROS 2 terrain GIS service node.

Loads terrain_data.json + heightmap PNG and provides a service to query
terrain height at any (x, y) position in the Gazebo ENU frame.

Services:
    /terrain/height_at_xy (std_srvs/srv/Trigger-like, uses custom topic)

Published topics:
    /terrain/info (std_msgs/msg/String) — terrain metadata JSON, latched

Usage:
    ros2 run fiber_nav_gazebo terrain_gis_node
    # or
    python3 scripts/terrain_gis_node.py

    # Query height at (x=100, y=200):
    ros2 topic pub --once /terrain/query geometry_msgs/msg/Point "{x: 100.0, y: 200.0, z: 0.0}"
    ros2 topic echo /terrain/height  # get result
"""

import json
import os

import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, String


class TerrainGISNode(Node):
    """Provides terrain height queries from the generated heightmap."""

    def __init__(self):
        super().__init__('terrain_gis_node')

        # Declare parameters
        self.declare_parameter('terrain_data_path', '')

        terrain_path = self.get_parameter('terrain_data_path').value

        # Auto-discover terrain_data.json
        if not terrain_path:
            # Try common locations
            candidates = [
                # Installed ROS 2 package share
                self._find_package_share_path(),
                # Development workspace
                os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'src', 'fiber_nav_gazebo', 'terrain', 'terrain_data.json'
                ),
                # Docker container path
                '/root/ws/src/fiber-nav-sim/src/fiber_nav_gazebo/terrain/terrain_data.json',
            ]
            for c in candidates:
                if c and os.path.exists(c):
                    terrain_path = c
                    break

        if not terrain_path or not os.path.exists(terrain_path):
            self.get_logger().error(
                'Cannot find terrain_data.json. Set terrain_data_path parameter '
                'or generate terrain first with generate_terrain.py'
            )
            self._heightmap = None
            self._metadata = None
            return

        self.get_logger().info(f'Loading terrain data from: {terrain_path}')
        terrain_dir = os.path.dirname(terrain_path)

        # Load metadata
        with open(terrain_path) as f:
            self._metadata = json.load(f)

        # Load heightmap
        hm_path = os.path.join(terrain_dir, self._metadata['heightmap_file'])
        hm_img = Image.open(hm_path)
        self._heightmap = np.array(hm_img, dtype=np.float64)

        # Precompute scale factors
        self._size_x = self._metadata['size_meters']['x']
        self._size_y = self._metadata['size_meters']['y']
        self._min_elev = self._metadata['min_elevation_msl']
        self._elev_range = self._metadata['heightmap_range_m']
        self._res = self._metadata['resolution_px']

        # Determine heightmap bit depth for normalization
        if hm_img.mode == 'I;16':
            self._max_val = 65535.0
        elif hm_img.mode == 'I':
            self._max_val = 65535.0
        else:
            self._max_val = 255.0

        self.get_logger().info(
            f'Terrain loaded: {self._res}x{self._res}, '
            f'{self._size_x:.0f}x{self._size_y:.0f}m, '
            f'elev {self._min_elev:.0f}-{self._min_elev + self._elev_range:.0f}m MSL'
        )

        # QoS for latched info topic
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self._info_pub = self.create_publisher(
            String, '/terrain/info', qos_latched
        )
        self._height_pub = self.create_publisher(
            Float64, '/terrain/height', 10
        )

        # Subscriber for point queries
        self.create_subscription(
            Point, '/terrain/query', self._query_cb, 10
        )

        # Publish terrain info (latched)
        info_msg = String()
        info_msg.data = json.dumps(self._metadata)
        self._info_pub.publish(info_msg)

    def _find_package_share_path(self):
        """Try to find terrain_data.json via ament package share."""
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('fiber_nav_gazebo')
            return os.path.join(share, 'terrain', 'terrain_data.json')
        except Exception:
            return None

    def height_at_xy(self, x: float, y: float) -> float:
        """Get terrain height (Gazebo Z, above min_elev) at ENU position (x, y).

        Returns the terrain surface height in Gazebo frame (where z=0 is at
        min_elevation_msl). This is NOT MSL — it's the local Gazebo Z.
        """
        if self._heightmap is None:
            return 0.0

        # Convert ENU (x, y) to pixel coordinates
        # Gazebo heightmap: origin at center, x=East, y=North
        # Pixel (0,0) = top-left = (west, north) corner
        # Pixel (res-1, res-1) = bottom-right = (east, south) corner
        px = (x + self._size_x / 2.0) / self._size_x * (self._res - 1)
        py = (self._size_y / 2.0 - y) / self._size_y * (self._res - 1)

        # Clamp to valid range
        px = max(0.0, min(float(self._res - 1), px))
        py = max(0.0, min(float(self._res - 1), py))

        # Bilinear interpolation
        x0 = int(px)
        y0 = int(py)
        x1 = min(x0 + 1, self._res - 1)
        y1 = min(y0 + 1, self._res - 1)

        fx = px - x0
        fy = py - y0

        v00 = float(self._heightmap[y0, x0])
        v01 = float(self._heightmap[y0, x1])
        v10 = float(self._heightmap[y1, x0])
        v11 = float(self._heightmap[y1, x1])

        v = (v00 * (1 - fx) * (1 - fy) +
             v01 * fx * (1 - fy) +
             v10 * (1 - fx) * fy +
             v11 * fx * fy)

        # Convert pixel value to Gazebo Z (height above min_elev)
        terrain_z = (v / self._max_val) * self._elev_range
        return terrain_z

    def _query_cb(self, msg: Point):
        """Handle height query: publish terrain height at (x, y)."""
        terrain_z = self.height_at_xy(msg.x, msg.y)

        result = Float64()
        result.data = terrain_z
        self._height_pub.publish(result)


def main():
    rclpy.init()
    node = TerrainGISNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
