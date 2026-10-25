# Mode B YCB scene load

Date: 2026-10-25 07:24:15
Layout: `data/scenes/indoor_objects/ycb/layouts/seed42.json` (not regenerated)
only_missing: 1

## Verify
- ok: **True**
- n_exist: 13 / 13
- n_targets_exist: 11 / 11
- n_pose_ok: 13

## Spawn
- n_ok: 13 / 13
- n_fail: 0

## Visibility diag
```json
{
  "gazebo_master_uri": "http://127.0.0.1:11355",
  "ros_master_uri": "http://localhost:11311",
  "sim_time": 82911.11,
  "n_world_models": 94,
  "agx_xy": [
    -2.1366641627667002,
    -5.903461427561879
  ],
  "table_xy_expected": [
    -3.9,
    -7.5
  ],
  "d455": {
    "rgb": true,
    "depth": true
  },
  "n_layout": 13,
  "n_present": 13,
  "n_with_issues": 0,
  "models": [
    {
      "gazebo_name": "mo_table_01",
      "in_world": true,
      "pose": {
        "x": -3.9,
        "y": -7.5,
        "z": 0.0
      },
      "geom_names": [
        "top",
        "leg1",
        "leg2",
        "leg3",
        "leg4"
      ],
      "visual_hint": {
        "body_names": [
          "link"
        ]
      },
      "issues": []
    },
    {
      "gazebo_name": "mo_obj_00",
      "in_world": true,
      "pose": {
        "x": -3.77729,
        "y": -7.41405,
        "z": 0.759
      },
      "geom_names": [
        "collision"
      ],
      "visual_hint": {
        "body_names": [
          "link"
        ]
      },
      "issues": []
    },
    {
      "gazebo_name": "mo_obj_01",
      "in_world": true,
      "pose": {
        "x": -3.95802,
        "y": -7.54694,
        "z": 0.741
      },
      "geom_names": [
        "collision"
      ],
      "visual_hint": {
        "body_names": [
          "link"
        ]
      },
      "issues": []
    },
    {
      "gazebo_name": "mo_obj_02",
      "in_world": true,
      "pose": {
        "x": -4.04352,
        "y": -7.37396,
        "z": 0.741
      },
      "geom_names": [
        "collision"
      ],
      "visual_hint": {
        "body_names": [
          "link"
        ]
      },
      "issues": []
    },
    {
      "gazebo_name": "mo_obj_03",
      "in_world": true,
      "pose": {
        "x": -3.72152,
        "y": -7.62665,
        "z": 0.768
      },
      "geom_names": [
        "collision"
      ],
      "visual_hint": {
        "body_names": [
          "link"
        ]
      },
      "issues": []
    },
    {
      "gazebo_name": "mo_obj_04",
      "in_world": true,
      "pose": {
        "x": -4.07224,
        "y": -7.59021,
        "z": 0.741
      },
      "geom_names": [
        "collision"
      ],
      "visual_hint": {
        "body_names": [
          "link"
        ]
      },
      "issues": []
    },
    {
      "gazebo_name": "mo_obj_05",
      "in_world": true,
      "pose": {
        "x": -3.88666,
        "y": -7.32854,
        "z": 0.741
      },
      "geom_names": [
        "collision"
      ],
      "visual_hint": {
        "body_names": [
          "link"
        ]
      },
      "issues": []
    },
    {
      "gazebo_name": "mo_obj_06",
      "in_world": true,
      "pose": {
        "x": -3.69193,
        "y": -7.4836,
        "z": 0.778
      },
      "geom_names": [
        "collision"
      ],
      "visual_hint": {
        "body_names": [
          "link"
        ]
      },
      "issues": []
    },
    {
      "gazebo_name": "mo_obj_07",
      "in_world": true,
      "pose": {
        "x": -4.12468,
        "y": -7.45706,
        "z": 0.741
      },
      "geom_names": [
        "collision"
      ],
      "visual_hint": {
        "body_names": [
          "link"
        ]
      },
      "issues": []
    },
    {
      "gazebo_name": "mo_obj_08",
      "in_world": true,
      "pose": {
        "x": -3.45122,
        "y": -7.91126,
        "z": 0.001
      },
      "geom_names": [
        "collision"
      ],
      "visual_hint": {
        "body_names": [
          "link"
        ]
      },
      "issues": []
    },
    {
      "gazebo_name": "mo_obj_09",
      "in_world": true,
      "pose": {
        "x": -4.41264,
        "y": -7.8062,
        "z": 0.001
      },
      "geom_names": [
        "collision"
      ],
      "visual_hint": {
        "body_names": [
          "link"
        ]
      },
      "issues": []
    },
    {
      "gazebo_name": "mo_obj_10",
      "in_world": true,
      "pose": {
        "x": -3.18844,
        "
```

## Shots
- `reports/mode_b_ycb_shots/`

LOAD_YCB_SCENE_OK
