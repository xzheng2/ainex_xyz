#!/usr/bin/env python3
"""Blackboard key constants and ROSA topic mapping.

Single source of truth for all BB key names.

Convention:
  BB.*_KEY   — short/relative key, used in register_key() with a /latched/ client
  BB.*       — absolute path, used in BB_LOG_KEYS, bb_writes dicts, ROSA_TOPIC_MAP
"""


class BB:
    # /latched/ — written by adapters/ every tick, read by L1/L2 nodes
    LATCHED_NS            = '/latched'

    # Short keys (relative) — used with register_key() on /latched/ clients
    ROBOT_STATE_KEY        = 'robot_state'
    TRACKED_OBJECTS_KEY    = 'tracked_objects'
    DETECTION_SOURCE_KEY   = 'detection_source'
    APRILTAG_TURN_BIAS_KEY = 'apriltag_turn_bias'
    APRILTAG_DIRECTION_KEY = 'apriltag_direction'
    APRILTAG_TAG_ID_KEY    = 'apriltag_tag_id'
    HEADING_TARGET_KEY     = 'heading_target'
    SERVO_POSITIONS_KEY    = 'servo_positions'
    NAV_U9_KEY             = 'nav_u9'
    NAV_U10_KEY            = 'nav_u10'
    NAV_CX_KEY             = 'nav_cx'
    NAV_FX_KEY             = 'nav_fx'
    NAV_START_BX_KEY       = 'nav_start_bx'
    NAV_START_BY_KEY       = 'nav_start_by'
    NAV_HAS_PATH_KEY       = 'nav_has_path'
    NAV_OBSTACLE_NEAR_KEY  = 'nav_obstacle_near'
    NAV_OBSTACLE_DETECTED_KEY = 'nav_obstacle_detected'
    NAV_IMU_HEADING_KEY    = 'nav_imu_heading'
    NAV_IMU_HEADING_MIN_KEY = 'nav_imu_heading_min'
    NAV_IMU_HEADING_MAX_KEY = 'nav_imu_heading_max'

    # Absolute paths — used in BB_LOG_KEYS, bb_writes, ROSA_TOPIC_MAP
    ROBOT_STATE       = LATCHED_NS + '/' + ROBOT_STATE_KEY        # '/latched/robot_state'
    TRACKED_OBJECTS   = LATCHED_NS + '/' + TRACKED_OBJECTS_KEY    # '/latched/tracked_objects'
    DETECTION_SOURCE  = LATCHED_NS + '/' + DETECTION_SOURCE_KEY   # '/latched/detection_source'
    APRILTAG_TURN_BIAS = LATCHED_NS + '/' + APRILTAG_TURN_BIAS_KEY # '/latched/apriltag_turn_bias'
    APRILTAG_DIRECTION = LATCHED_NS + '/' + APRILTAG_DIRECTION_KEY # '/latched/apriltag_direction'
    APRILTAG_TAG_ID   = LATCHED_NS + '/' + APRILTAG_TAG_ID_KEY    # '/latched/apriltag_tag_id'
    HEADING_TARGET    = LATCHED_NS + '/' + HEADING_TARGET_KEY     # '/latched/heading_target'
    SERVO_POSITIONS   = LATCHED_NS + '/' + SERVO_POSITIONS_KEY    # '/latched/servo_positions'
    NAV_U9            = LATCHED_NS + '/' + NAV_U9_KEY             # '/latched/nav_u9'
    NAV_U10           = LATCHED_NS + '/' + NAV_U10_KEY            # '/latched/nav_u10'
    NAV_CX            = LATCHED_NS + '/' + NAV_CX_KEY             # '/latched/nav_cx'
    NAV_FX            = LATCHED_NS + '/' + NAV_FX_KEY             # '/latched/nav_fx'
    NAV_START_BX      = LATCHED_NS + '/' + NAV_START_BX_KEY       # '/latched/nav_start_bx'
    NAV_START_BY      = LATCHED_NS + '/' + NAV_START_BY_KEY       # '/latched/nav_start_by'
    NAV_HAS_PATH      = LATCHED_NS + '/' + NAV_HAS_PATH_KEY       # '/latched/nav_has_path'
    NAV_OBSTACLE_NEAR = LATCHED_NS + '/' + NAV_OBSTACLE_NEAR_KEY  # '/latched/nav_obstacle_near'
    NAV_OBSTACLE_DETECTED = LATCHED_NS + '/' + NAV_OBSTACLE_DETECTED_KEY  # '/latched/nav_obstacle_detected'
    NAV_IMU_HEADING   = LATCHED_NS + '/' + NAV_IMU_HEADING_KEY    # '/latched/nav_imu_heading'
    NAV_IMU_HEADING_MIN = LATCHED_NS + '/' + NAV_IMU_HEADING_MIN_KEY  # '/latched/nav_imu_heading_min'
    NAV_IMU_HEADING_MAX = LATCHED_NS + '/' + NAV_IMU_HEADING_MAX_KEY  # '/latched/nav_imu_heading_max'

    # /perception/ — reserved for future L3 use; nothing writes or reads these today.
    # (The former /latched/ DETECTED_OBJECTS + DETECTED_COUNT keys were removed
    #  Aug 2026 — superseded by TRACKED_OBJECTS, and dead by then.)
    PERCEPTION_NS     = '/perception'
    TARGET_PIXEL_X_KEY = 'target_pixel_x'   # short key for /perception/ client
    PERCEPTION_DETECTED_OBJECTS = '/perception/detected_objects'
    TARGET_PIXEL_X    = '/perception/target_pixel_x'
    TARGET_PIXEL_Y    = '/perception/target_pixel_y'
    FACE_DETECTED     = '/perception/face_detected'
    GESTURE_LABEL     = '/perception/gesture_label'
    TARGET_WORLD_POS  = '/perception/target_world_pos'
    COLOR_LAB_MIN     = '/perception/color_lab_min'
    COLOR_LAB_MAX     = '/perception/color_lab_max'
    COLOR_DETECT_TYPE = '/perception/color_detect_type'
    DETECT_ROI        = '/perception/detect_roi'

    # /locomotion/ (ROBOT_STATE moved to /latched/)
    #
    # 废弃 Key（Legacy — 禁止在新代码中使用）:
    #   '/locomotion/robot_state' → 已迁移至 /latched/，用 BB.ROBOT_STATE
    #   line 专用扁平键（line_data / last_line_x / camera_lost_count /
    #     line_error_x / line_center_x / last_line_error_x）→ Aug 2026 全部删除。
    #     line 现在是 TRACKED_OBJECTS 里的一个普通目标（shape='line'）：
    #       line_error_x       → tracked_objects['line']['error_x']
    #       camera_lost_count  → tracked_objects['line']['lost_count']
    #       last_line_error_x  → 同一个 error_x（丢失时 adapter 保留旧值，天然粘滞）
    #       line_center_x / last_line_x → 删除前已无任何消费方
    # 新节点、新 adapter、新项目 behaviours 禁止读写上述废弃 key；在代码中发现须重构。
    # 下面 /locomotion/ 其余 key（GAIT_ENABLED、WALK_* 等）保留供未来 L2/L3 扩展使用，
    # 当前没有任何 input adapter 写入它们，也不在 ROSA_TOPIC_MAP 中。
    GAIT_ENABLED      = '/locomotion/gait_enabled'
    WALK_X            = '/locomotion/walk_x'
    WALK_Y            = '/locomotion/walk_y'
    WALK_ANGLE        = '/locomotion/walk_angle'
    WALK_BODY_HEIGHT  = '/locomotion/walk_body_height'

    # /manipulation/
    GRIPPER_STATE     = '/manipulation/gripper_state'
    APPROACH_DONE     = '/manipulation/approach_done'

    # /mission/
    CURRENT_TASK       = '/mission/current_task'
    SESSION_ID         = '/mission/session_id'
    TARGET_COLOR       = '/mission/target_color'
    ACTION_NAME        = '/mission/action_name'
    AVAILABLE_ACTIONS  = '/mission/available_actions'

    # ROSA mirror topic map
    ROSA_TOPIC_MAP = {
        ROBOT_STATE:       '/bt/bb/latched/robot_state',
        TRACKED_OBJECTS:   '/bt/bb/latched/tracked_objects',
        DETECTION_SOURCE:  '/bt/bb/latched/detection_source',
        APRILTAG_TURN_BIAS: '/bt/bb/latched/apriltag_turn_bias',
        APRILTAG_DIRECTION: '/bt/bb/latched/apriltag_direction',
        APRILTAG_TAG_ID:   '/bt/bb/latched/apriltag_tag_id',
        HEADING_TARGET:    '/bt/bb/latched/heading_target',
        SERVO_POSITIONS:   '/bt/bb/latched/servo_positions',
        NAV_U9:            '/bt/bb/latched/nav_u9',
        NAV_U10:           '/bt/bb/latched/nav_u10',
        NAV_START_BX:      '/bt/bb/latched/nav_start_bx',
        NAV_START_BY:      '/bt/bb/latched/nav_start_by',
        NAV_CX:            '/bt/bb/latched/nav_cx',
        NAV_FX:            '/bt/bb/latched/nav_fx',
        NAV_HAS_PATH:      '/bt/bb/latched/nav_has_path',
        NAV_OBSTACLE_NEAR: '/bt/bb/latched/nav_obstacle_near',
        NAV_OBSTACLE_DETECTED: '/bt/bb/latched/nav_obstacle_detected',
        NAV_IMU_HEADING:   '/bt/bb/latched/nav_imu_heading',
        NAV_IMU_HEADING_MIN: '/bt/bb/latched/nav_imu_heading_min',
        NAV_IMU_HEADING_MAX: '/bt/bb/latched/nav_imu_heading_max',
    }
