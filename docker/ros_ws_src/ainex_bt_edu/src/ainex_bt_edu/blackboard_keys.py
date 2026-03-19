#!/usr/bin/env python3
"""Blackboard key constants and ROSA topic mapping."""


class BB:
    # /perception/
    DETECTED_OBJECTS  = '/perception/detected_objects'
    LINE_DATA         = '/perception/line_data'
    TARGET_PIXEL_X    = '/perception/target_pixel_x'
    TARGET_PIXEL_Y    = '/perception/target_pixel_y'
    FACE_DETECTED     = '/perception/face_detected'
    GESTURE_LABEL     = '/perception/gesture_label'
    TARGET_WORLD_POS  = '/perception/target_world_pos'
    COLOR_LAB_MIN     = '/perception/color_lab_min'
    COLOR_LAB_MAX     = '/perception/color_lab_max'
    COLOR_DETECT_TYPE = '/perception/color_detect_type'
    DETECT_ROI        = '/perception/detect_roi'

    # /locomotion/
    ROBOT_STATE       = '/locomotion/robot_state'
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
        ROBOT_STATE:      '/bt/bb/locomotion/robot_state',
        GAIT_ENABLED:     '/bt/bb/locomotion/gait_enabled',
        WALK_X:           '/bt/bb/locomotion/walk_x',
        LINE_DATA:        '/bt/bb/perception/line_data',
        DETECTED_OBJECTS: '/bt/bb/perception/detected_objects',
        TARGET_PIXEL_X:   '/bt/bb/perception/target_pixel_x',
        TARGET_PIXEL_Y:   '/bt/bb/perception/target_pixel_y',
        FACE_DETECTED:    '/bt/bb/perception/face_detected',
        GESTURE_LABEL:    '/bt/bb/perception/gesture_label',
        GRIPPER_STATE:    '/bt/bb/manipulation/gripper_state',
        CURRENT_TASK:     '/bt/bb/mission/current_task',
    }
