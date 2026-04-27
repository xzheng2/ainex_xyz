#!/usr/bin/env python3
"""L3 Mission: PID-based walk toward detected object using ApproachObject."""
import py_trees
from py_trees.common import Status
from xyz_bt_edu.base_node import XyzBTNode
from xyz_bt_edu.blackboard_keys import BB


class L3_Gait_ApproachObject(XyzBTNode):
    LEVEL = 'L3'
    BB_LOG_KEYS = [BB.DETECTED_OBJECTS, BB.APPROACH_DONE]

    def __init__(self, approach_object, name='L3_Gait_ApproachObject'):
        """
        Args:
            approach_object: ainex_example.approach_object.ApproachObject instance
        """
        super().__init__(name)
        self._approach = approach_object

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def initialise(self):
        py_trees.blackboard.Blackboard.storage[
            BB.APPROACH_DONE] = False

    def update(self):
        storage = py_trees.blackboard.Blackboard.storage
        objects = storage.get(BB.DETECTED_OBJECTS)

        if objects is None or len(objects.data) == 0:
            return Status.RUNNING

        obj = objects.data[0]
        done = self._approach.process(
            max_y=obj.y,
            center_x=obj.x,
            angle=getattr(obj, 'angle', 0),
            x_stop=10,
            y_stop=10,
            yaw_stop=5,
            width=obj.width,
            height=obj.height,
        )

        if done:
            storage[BB.APPROACH_DONE] = True
            return Status.SUCCESS
        return Status.RUNNING
