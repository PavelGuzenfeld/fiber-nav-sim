#!/usr/bin/env python3
"""Inject a ROS2 Camera Sensor entity into an O3DE level prefab.

This script programmatically adds a camera entity with ROS2CameraSensorEditorComponent
to the DemoLevel prefab, avoiding the need for the O3DE Editor GUI.

Usage:
    python3 add_camera_to_level.py /opt/HeadlessTest/Levels/DemoLevel/DemoLevel.prefab
"""

import json
import sys

def add_camera_entity(prefab_path):
    with open(prefab_path, 'r') as f:
        prefab = json.load(f)

    # The camera entity ID (must be unique within the prefab)
    camera_entity_id = "Entity_[99900000000001]"

    # Parent to the Level container entity
    container_id = prefab["ContainerEntity"]["Id"]

    # Camera entity with ROS2CameraSensorEditorComponent + ROS2FrameEditorComponent
    camera_entity = {
        "Id": camera_entity_id,
        "Name": "Phase0_Camera",
        "Components": {
            "TransformComponent": {
                "$type": "{27F1E1A1-8D9D-4C3B-BD3A-AFB9762449C0} TransformComponent",
                "Id": 99900000000001,
                "Parent Entity": container_id,
                "Transform Data": {
                    "Translate": [0.0, -3.0, 2.0],
                    "Rotate": [-15.0, 0.0, 0.0]
                }
            },
            "EditorInspectorComponent": {
                "$type": "EditorInspectorComponent",
                "Id": 99900000000002
            },
            "EditorLockComponent": {
                "$type": "EditorLockComponent",
                "Id": 99900000000003
            },
            "EditorVisibilityComponent": {
                "$type": "EditorVisibilityComponent",
                "Id": 99900000000004
            },
            "EditorOnlyEntityComponent": {
                "$type": "EditorOnlyEntityComponent",
                "Id": 99900000000005
            },
            "EditorPendingCompositionComponent": {
                "$type": "EditorPendingCompositionComponent",
                "Id": 99900000000006
            },
            "EditorDisabledCompositionComponent": {
                "$type": "EditorDisabledCompositionComponent",
                "Id": 99900000000007
            },
            "EditorEntitySortComponent": {
                "$type": "EditorEntitySortComponent",
                "Id": 99900000000008,
                "Child Entity Order": []
            },
            "EditorEntityIconComponent": {
                "$type": "EditorEntityIconComponent",
                "Id": 99900000000009
            },
            "EditorCameraComponent": {
                "$type": "{CA11DA46-29FF-4083-B5F6-E02C3A8C3A3D} EditorCameraComponent",
                "Id": 99900000000010,
                "Controller": {
                    "Configuration": {
                        "Field of View": 60.0
                    }
                }
            },
            "ROS2FrameEditorComponent": {
                "$type": "ROS2FrameEditorComponent",
                "Id": 99900000000011,
                "ROS2FrameConfiguration": {
                    "Frame Name": "phase0_camera"
                }
            },
            "ROS2CameraSensorEditorComponent": {
                "$type": "ROS2CameraSensorEditorComponent",
                "Id": 99900000000012,
                "CameraSensorConfig": {
                    "Width": 1920,
                    "Height": 1080,
                    "ClipNear": 0.1,
                    "ClipFar": 100.0
                },
                "SensorConfig": {
                    "Frequency": 30.0,
                    "Publishers": {
                        "Color Camera Info": {
                            "Type": "sensor_msgs::msg::CameraInfo",
                            "Topic": "camera_info",
                            "QoS": {
                                "Reliability": 1
                            }
                        },
                        "Color Image": {
                            "Type": "sensor_msgs::msg::Image",
                            "Topic": "camera_image_color",
                            "QoS": {
                                "Reliability": 1
                            }
                        }
                    }
                }
            }
        }
    }

    # Add entity to prefab
    prefab["Entities"][camera_entity_id] = camera_entity

    # Add to Level's child entity order
    for comp_key, comp in prefab["ContainerEntity"]["Components"].items():
        if comp.get("$type") == "EditorEntitySortComponent":
            if "Child Entity Order" in comp:
                comp["Child Entity Order"].append(camera_entity_id)
                break

    # Write back
    with open(prefab_path, 'w') as f:
        json.dump(prefab, f, indent=4)

    print(f"[add_camera] Added Phase0_Camera entity to {prefab_path}")
    print(f"[add_camera] Camera publishes to: /camera_image_color (1920x1080, 30 Hz)")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <prefab_path>")
        sys.exit(1)
    add_camera_entity(sys.argv[1])
