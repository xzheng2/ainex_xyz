#!/usr/bin/env python3
"""Marathon-specific condition behaviours.

IsHeadCentered has been promoted to ainex_bt_edu as L1_Head_IsHeadCentered.
Re-exported here as an alias so existing code continues to work unchanged.
"""
from ainex_bt_edu.behaviours.L1_perception.L1_Head_IsHeadCentered import (
    L1_Head_IsHeadCentered as IsHeadCentered,
)

__all__ = ['IsHeadCentered']
