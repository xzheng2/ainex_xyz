#!/usr/bin/env python3
"""Blackboard key constants and ROSA topic mapping.

Single source of truth for all BB key names.

Convention:
  BB.*_KEY   — short/relative key, used in register_key() with a /latched/ client
  BB.*       — absolute path, used in BB_LOG_KEYS, bb_writes dicts, ROSA_TOPIC_MAP
  BB.HEAD_PAN_POS — absolute path (root-ns), used directly in register_key()
"""


class BB:
    # /latched/ — written by input_adapters/ every tick, read by L1/L2 nodes
    LATCHED_NS            = '/latched'

    # Short keys (relative) — used with register_key() on /latched/ clients
    ROBOT_STATE_KEY        = 'robot_state'
    LINE_DATA_KEY          = 'line_data'
    LAST_LINE_X_KEY        = 'last_line_x'
    CAMERA_LOST_COUNT_KEY  = 'camera_lost_count'
    LINE_ERROR_X_KEY       = 'line_error_x'
    LINE_CENTER_X_KEY      = 'line_center_x'
    LAST_LINE_ERROR_X_KEY  = 'last_line_error_x'
    DETECTED_OBJECTS_KEY   = 'detected_objects'
    DETECTED_COUNT_KEY     = 'detected_count'

    # Absolute paths — used in BB_LOG_KEYS, bb_writes, ROSA_TOPIC_MAP
    ROBOT_STATE       = LATCHED_NS + '/' + ROBOT_STATE_KEY        # '/latched/robot_state'
    LINE_DATA         = LATCHED_NS + '/' + LINE_DATA_KEY          # '/latched/line_data'
    LAST_LINE_X       = LATCHED_NS + '/' + LAST_LINE_X_KEY        # '/latched/last_line_x'
    CAMERA_LOST_COUNT = LATCHED_NS + '/' + CAMERA_LOST_COUNT_KEY  # '/latched/camera_lost_count'
    LINE_ERROR_X      = LATCHED_NS + '/' + LINE_ERROR_X_KEY       # '/latched/line_error_x'
    LINE_CENTER_X     = LATCHED_NS + '/' + LINE_CENTER_X_KEY      # '/latched/line_center_x'
    LAST_LINE_ERROR_X = LATCHED_NS + '/' + LAST_LINE_ERROR_X_KEY  # '/latched/last_line_error_x'
    DETECTED_OBJECTS  = LATCHED_NS + '/' + DETECTED_OBJECTS_KEY   # '/latched/detected_objects'
    DETECTED_COUNT    = LATCHED_NS + '/' + DETECTED_COUNT_KEY     # '/latched/detected_count'
    HEAD_PAN_POS      = '/head_pan_pos'  # root-ns; used in register_key() directly

    # /perception/ — future L3 use (DETECTED_OBJECTS is /latched/ — see above)
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
        LINE_DATA:         '/bt/bb/latched/line_data',
        LAST_LINE_X:       '/bt/bb/latched/last_line_x',
        CAMERA_LOST_COUNT: '/bt/bb/latched/camera_lost_count',
        LINE_ERROR_X:      '/bt/bb/latched/line_error_x',
        LINE_CENTER_X:     '/bt/bb/latched/line_center_x',
        LAST_LINE_ERROR_X: '/bt/bb/latched/last_line_error_x',
        DETECTED_OBJECTS:  '/bt/bb/latched/detected_objects',
        DETECTED_COUNT:    '/bt/bb/latched/detected_count',
    }
