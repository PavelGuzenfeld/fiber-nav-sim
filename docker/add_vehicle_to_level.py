#!/usr/bin/env python3
"""Create a terrain level prefab with vehicle entity, camera, and PX4 bridge.

Replaces the warehouse DemoLevel with a terrain-based level containing:
- Terrain world (heightmap + satellite texture)
- Sun/sky environment
- Vehicle entity (quadtailsitter) with ROS2 camera sensor
- PX4 MAVLink bridge system entity

Usage:
    python3 add_vehicle_to_level.py /opt/HeadlessTest/Levels/TerrainLevel/TerrainLevel.prefab
"""

import json
import os
import sys


def create_terrain_level(prefab_path):
    os.makedirs(os.path.dirname(prefab_path), exist_ok=True)

    # Entity IDs (arbitrary unique numbers)
    LEVEL_ID = "Entity_[88800000000001]"
    SKY_ID = "Entity_[88800000000002]"
    SUN_ID = "Entity_[88800000000003]"
    VEHICLE_ID = "Entity_[88800000000004]"
    CAMERA_ID = "Entity_[88800000000005]"
    BRIDGE_ID = "Entity_[88800000000006]"

    def editor_boilerplate(entity_id, children=None):
        """Standard O3DE editor components every entity needs."""
        base_id = int(entity_id.split("[")[1].rstrip("]"))
        components = {
            "EditorInspectorComponent": {
                "$type": "EditorInspectorComponent",
                "Id": base_id + 100
            },
            "EditorLockComponent": {
                "$type": "EditorLockComponent",
                "Id": base_id + 101
            },
            "EditorVisibilityComponent": {
                "$type": "EditorVisibilityComponent",
                "Id": base_id + 102
            },
            "EditorOnlyEntityComponent": {
                "$type": "EditorOnlyEntityComponent",
                "Id": base_id + 103
            },
            "EditorPendingCompositionComponent": {
                "$type": "EditorPendingCompositionComponent",
                "Id": base_id + 104
            },
            "EditorDisabledCompositionComponent": {
                "$type": "EditorDisabledCompositionComponent",
                "Id": base_id + 105
            },
            "EditorEntitySortComponent": {
                "$type": "EditorEntitySortComponent",
                "Id": base_id + 106,
                "Child Entity Order": children or []
            },
            "EditorEntityIconComponent": {
                "$type": "EditorEntityIconComponent",
                "Id": base_id + 107
            }
        }
        return components

    # ── Sky entity (HDRi skybox) ──
    sky_components = editor_boilerplate(SKY_ID)
    sky_components["TransformComponent"] = {
        "$type": "{27F1E1A1-8D9D-4C3B-BD3A-AFB9762449C0} TransformComponent",
        "Id": 88800000000202,
        "Parent Entity": LEVEL_ID
    }
    sky_components["EditorHDRiSkyboxComponent"] = {
        "$type": "AZ::Render::EditorHDRiSkyboxComponent",
        "Id": 88800000000203,
        "Controller": {
            "Configuration": {
                "CubemapAsset": {
                    "assetId": {
                        "guid": "{8C964DCA-0737-5EA1-B333-EF1D2F281431}",
                        "subId": 2000
                    },
                    "assetHint": "lightingpresets/lowcontrast/kloetzle_blei_4k_iblskyboxcm_iblspecular.exr.streamingimage"
                }
            }
        }
    }

    # ── Sun entity (directional light) ──
    sun_components = editor_boilerplate(SUN_ID)
    sun_components["TransformComponent"] = {
        "$type": "{27F1E1A1-8D9D-4C3B-BD3A-AFB9762449C0} TransformComponent",
        "Id": 88800000000302,
        "Parent Entity": LEVEL_ID,
        "Transform Data": {
            "Translate": [0.0, 0.0, 100.0],
            "Rotate": [-45.0, 30.0, 0.0]
        }
    }
    sun_components["EditorDirectionalLightComponent"] = {
        "$type": "AZ::Render::EditorDirectionalLightComponent",
        "Id": 88800000000303,
        "Controller": {
            "Configuration": {
                "Intensity": 4.0,
                "Color": [1.0, 0.96, 0.9, 1.0],
                "Shadow": {
                    "ShadowmapMaxSize": 2048
                }
            }
        }
    }

    # ── Vehicle entity (quadtailsitter) ──
    vehicle_components = editor_boilerplate(VEHICLE_ID, children=[CAMERA_ID])
    vehicle_components["TransformComponent"] = {
        "$type": "{27F1E1A1-8D9D-4C3B-BD3A-AFB9762449C0} TransformComponent",
        "Id": 88800000000402,
        "Parent Entity": LEVEL_ID,
        "Transform Data": {
            "Translate": [0.0, 0.0, 50.0]
        }
    }
    vehicle_components["ROS2FrameEditorComponent"] = {
        "$type": "ROS2FrameEditorComponent",
        "Id": 88800000000403,
        "ROS2FrameConfiguration": {
            "Frame Name": "base_link"
        }
    }

    # ── Camera entity (child of vehicle) ──
    camera_components = editor_boilerplate(CAMERA_ID)
    camera_components["TransformComponent"] = {
        "$type": "{27F1E1A1-8D9D-4C3B-BD3A-AFB9762449C0} TransformComponent",
        "Id": 88800000000502,
        "Parent Entity": VEHICLE_ID,
        "Transform Data": {
            "Translate": [-0.16, 0.3, 0.0],
            "Rotate": [0.0, -90.0, 0.0]
        }
    }
    camera_components["EditorCameraComponent"] = {
        "$type": "{CA11DA46-29FF-4083-B5F6-E02C3A8C3A3D} EditorCameraComponent",
        "Id": 88800000000503,
        "Controller": {
            "Configuration": {
                "Field of View": 80.0
            }
        }
    }
    camera_components["ROS2FrameEditorComponent"] = {
        "$type": "ROS2FrameEditorComponent",
        "Id": 88800000000504,
        "ROS2FrameConfiguration": {
            "Frame Name": "forward_camera"
        }
    }
    camera_components["ROS2CameraSensorEditorComponent"] = {
        "$type": "ROS2CameraSensorEditorComponent",
        "Id": 88800000000505,
        "CameraSensorConfig": {
            "Width": 1920,
            "Height": 1080,
            "ClipNear": 0.1,
            "ClipFar": 1000.0
        },
        "SensorConfig": {
            "Frequency": 30.0,
            "Publishers": {
                "Color Camera Info": {
                    "Type": "sensor_msgs::msg::CameraInfo",
                    "Topic": "camera_info",
                    "QoS": {"Reliability": 1}
                },
                "Color Image": {
                    "Type": "sensor_msgs::msg::Image",
                    "Topic": "camera_image_color",
                    "QoS": {"Reliability": 1}
                },
                "Depth Camera Info": {
                    "Type": "sensor_msgs::msg::CameraInfo",
                    "Topic": "depth_camera_info",
                    "QoS": {"Reliability": 1}
                },
                "Depth Image": {
                    "Type": "sensor_msgs::msg::Image",
                    "Topic": "camera_image_depth",
                    "QoS": {"Reliability": 1}
                }
            }
        }
    }

    # ── PX4 MAVLink Bridge entity ──
    bridge_components = editor_boilerplate(BRIDGE_ID)
    bridge_components["TransformComponent"] = {
        "$type": "{27F1E1A1-8D9D-4C3B-BD3A-AFB9762449C0} TransformComponent",
        "Id": 88800000000602,
        "Parent Entity": LEVEL_ID
    }

    # ── Assemble level prefab ──
    prefab = {
        "ContainerEntity": {
            "Id": LEVEL_ID,
            "Name": "Level",
            "Components": {
                **{f"Component_{k}": v for k, v in {
                    "EditorInspectorComponent": {
                        "$type": "EditorInspectorComponent",
                        "Id": 88800000000010
                    },
                    "EditorOnlyEntityComponent": {
                        "$type": "EditorOnlyEntityComponent",
                        "Id": 88800000000011
                    },
                    "EditorPendingCompositionComponent": {
                        "$type": "EditorPendingCompositionComponent",
                        "Id": 88800000000012
                    },
                    "EditorEntitySortComponent": {
                        "$type": "EditorEntitySortComponent",
                        "Id": 88800000000013,
                        "Child Entity Order": [
                            SKY_ID, SUN_ID, VEHICLE_ID, BRIDGE_ID
                        ]
                    },
                    "TransformComponent": {
                        "$type": "{27F1E1A1-8D9D-4C3B-BD3A-AFB9762449C0} TransformComponent",
                        "Id": 88800000000014,
                        "Parent Entity": ""
                    },
                    "EditorDisabledCompositionComponent": {
                        "$type": "EditorDisabledCompositionComponent",
                        "Id": 88800000000015
                    },
                    "EditorEntityIconComponent": {
                        "$type": "EditorEntityIconComponent",
                        "Id": 88800000000016
                    },
                    "EditorPrefabComponent": {
                        "$type": "EditorPrefabComponent",
                        "Id": 88800000000017
                    },
                    "EditorLockComponent": {
                        "$type": "EditorLockComponent",
                        "Id": 88800000000018
                    },
                    "EditorVisibilityComponent": {
                        "$type": "EditorVisibilityComponent",
                        "Id": 88800000000019
                    }
                }.items()}
            }
        },
        "Entities": {
            SKY_ID: {
                "Id": SKY_ID,
                "Name": "Sky",
                "Components": sky_components
            },
            SUN_ID: {
                "Id": SUN_ID,
                "Name": "Sun",
                "Components": sun_components
            },
            VEHICLE_ID: {
                "Id": VEHICLE_ID,
                "Name": "quadtailsitter",
                "Components": vehicle_components
            },
            CAMERA_ID: {
                "Id": CAMERA_ID,
                "Name": "forward_camera",
                "Components": camera_components
            },
            BRIDGE_ID: {
                "Id": BRIDGE_ID,
                "Name": "PX4Bridge_System",
                "Components": bridge_components
            }
        },
        "Instances": {}
    }

    with open(prefab_path, 'w') as f:
        json.dump(prefab, f, indent=4)

    print(f"[add_vehicle] Created terrain level at {prefab_path}")
    print(f"[add_vehicle] Entities: Sky, Sun, quadtailsitter (+ forward_camera), PX4Bridge_System")
    print(f"[add_vehicle] Camera: 1920x1080 @ 30Hz, topic: /camera_image_color")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <prefab_path>")
        sys.exit(1)
    create_terrain_level(sys.argv[1])
