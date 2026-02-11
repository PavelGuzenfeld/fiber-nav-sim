#!/usr/bin/env python3
"""Simulated downward distance sensor for PX4 SITL.

Reads Gazebo ground truth odometry and publishes a distance_sensor message
to PX4, enabling the terrain estimator and dist_bottom_valid. This breaks
the flying_but_ground_contact deadlock in MulticopterPositionControl.

When terrain data is available (terrain_data.json + heightmap from
generate_terrain.py), computes true AGL by subtracting terrain height
at the current (x, y) position. On flat worlds, falls back to odom z.
"""

import json
import os

import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import DistanceSensor


class SimDistanceSensor(Node):
    def __init__(self):
        super().__init__('sim_distance_sensor')

        self.declare_parameter('terrain_data_path', '')

        # Try to load terrain data for AGL computation
        self._terrain = None
        self._load_terrain()

        qos_gz = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            Odometry,
            '/model/quadtailsitter/odometry',
            self._odom_cb,
            qos_gz,
        )
        self.pub = self.create_publisher(
            DistanceSensor, '/fmu/in/distance_sensor', qos_px4
        )

        if self._terrain:
            self.get_logger().info(
                f'Sim distance sensor started (terrain-aware AGL, '
                f'{self._terrain["res"]}x{self._terrain["res"]})'
            )
        else:
            self.get_logger().info(
                'Sim distance sensor started (flat ground, odom z = AGL)'
            )

    def _load_terrain(self):
        """Try to load terrain data for height lookups."""
        terrain_path = self.get_parameter('terrain_data_path').value

        if not terrain_path:
            # Auto-discover terrain_data.json
            candidates = [
                os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'src', 'fiber_nav_gazebo', 'terrain', 'terrain_data.json'
                ),
                '/root/ws/src/fiber-nav-sim/src/fiber_nav_gazebo/terrain/terrain_data.json',
            ]
            try:
                from ament_index_python.packages import get_package_share_directory
                share = get_package_share_directory('fiber_nav_gazebo')
                candidates.insert(0, os.path.join(share, 'terrain', 'terrain_data.json'))
            except Exception:
                pass

            for c in candidates:
                if os.path.exists(c):
                    terrain_path = c
                    break

        if not terrain_path or not os.path.exists(terrain_path):
            return

        try:
            terrain_dir = os.path.dirname(terrain_path)
            with open(terrain_path) as f:
                metadata = json.load(f)

            hm_path = os.path.join(terrain_dir, metadata['heightmap_file'])
            hm_img = Image.open(hm_path)
            heightmap = np.array(hm_img, dtype=np.float64)

            max_val = 65535.0 if hm_img.mode in ('I;16', 'I') else 255.0

            self._terrain = {
                'heightmap': heightmap,
                'size_x': metadata['size_meters']['x'],
                'size_y': metadata['size_meters']['y'],
                'elev_range': metadata['heightmap_range_m'],
                'res': metadata['resolution_px'],
                'max_val': max_val,
            }
        except Exception as e:
            self.get_logger().warn(f'Failed to load terrain data: {e}')

    def _terrain_height(self, x: float, y: float) -> float:
        """Get terrain surface height (Gazebo Z) at ENU position (x, y)."""
        t = self._terrain
        hm = t['heightmap']
        res = t['res']

        # Convert ENU (x, y) to pixel coordinates
        # Heightmap origin at center, x=East, y=North
        # Pixel (0,0) = top-left = (west, north)
        px = (x + t['size_x'] / 2.0) / t['size_x'] * (res - 1)
        py = (t['size_y'] / 2.0 - y) / t['size_y'] * (res - 1)

        px = max(0.0, min(float(res - 1), px))
        py = max(0.0, min(float(res - 1), py))

        # Bilinear interpolation
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

    def _odom_cb(self, msg):
        # Gazebo ENU: z = height above z=0 (which is min_elevation_msl for terrain worlds)
        z = msg.pose.pose.position.z

        if self._terrain:
            # Terrain-aware: AGL = odom_z - terrain_surface_z
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            terrain_z = self._terrain_height(x, y)
            height = z - terrain_z
        else:
            # Flat ground: odom z IS AGL
            height = z

        if height < 0.0:
            height = 0.0

        ds = DistanceSensor()
        ds.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        ds.min_distance = 0.1
        ds.max_distance = 50.0
        ds.current_distance = max(0.1, float(height))
        ds.variance = 0.01
        ds.signal_quality = 100
        ds.type = 0  # LASER
        ds.h_fov = 0.05
        ds.v_fov = 0.05
        ds.orientation = 25  # ROTATION_DOWNWARD_FACING
        ds.mode = 1  # MODE_ENABLED

        self.pub.publish(ds)


def main():
    rclpy.init()
    node = SimDistanceSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
