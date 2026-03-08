#!/usr/bin/env python3
# encoding: utf-8
import time
import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty
from ros_robot_controller.msg import BuzzerState
from ainex_interfaces.msg import AppWalkingParam
from ainex_interfaces.srv import SetWalkingCommand

OLD_AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
# OLD_BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''

OLD_BUTTON_MAP = 'A', 'B', '', 'X', 'Y', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''

NEW_AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'hat_x', 'hat_y'
NEW_BUTTON_MAP = 'Y', 'B', 'A', 'X', 'l1', 'r1', 'l2', 'r2', 'select', 'start', 'hat_yu', 'hat_yd', 'mode'

AXES_MAP = NEW_AXES_MAP
BUTTON_MAP = NEW_BUTTON_MAP

class ButtonState():
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3

class JoystickController:
    def __init__(self):
        rospy.init_node('joystick_control', anonymous=True)
        self.time_stamp_ry = 0 
        self.init_z_offset = 0.025
        self.count_stop = 0 
        self.status = 'stop'
        self.update_height = False
        self.update_param = False
        self.last_axes = None
        self.last_buttons = None
        self.mode = 0

        rospy.sleep(0.2)
        self.count = 0
        self.stand = False
        self.param_pub = rospy.Publisher('/app/set_walking_param', AppWalkingParam, queue_size=1)
        self.buzzer_pub = rospy.Publisher('ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

    def axes_callback(self, axes):
        msg = AppWalkingParam()
        msg.speed = 3
        if axes['ly'] > 0.3:
            self.update_param = True
            msg.x = 0.02
        elif axes['ly'] < -0.3:
            self.update_param = True
            msg.x = -0.02
        if axes['lx'] > 0.3:
            self.update_param = True
            msg.y = 0.015
        elif axes['lx'] < -0.3:
            self.update_param = True
            msg.y = -0.015

        if axes['rx'] > 0.3:
            self.update_param = True
            msg.angle = 5
        elif axes['rx'] < -0.3:
            self.update_param = True
            msg.angle = -5
         
        if self.update_param:
            msg.height = self.init_z_offset
            rospy.ServiceProxy('/walking/command', SetWalkingCommand)('enable_control')
            rospy.ServiceProxy('/walking/command', SetWalkingCommand)('start')
            self.param_pub.publish(msg)
        if self.status == 'stop' and self.update_param:
            self.status = 'move'
        elif self.status == 'move' and not self.update_param:
            self.status = 'stop'
            rospy.ServiceProxy('/walking/command', SetWalkingCommand)('stop')
        self.update_param = False


    def callback(self, axes):
        if rospy.get_time() > self.time_stamp_ry:
            self.update_height = False
            if axes['ry'] < -0.5:
                self.update_height = True
                self.init_z_offset += 0.005
                if self.init_z_offset > 0.06:
                    self.update_height = False
                    self.init_z_offset = 0.06
            elif axes['ry'] > 0.5:
                self.update_height = True
                self.init_z_offset += -0.005
                if self.init_z_offset < 0.025:
                    self.update_height = False
                    self.init_z_offset = 0.025
            if self.update_height and not self.update_param:
                msg = AppWalkingParam()
                msg.speed = 3
                msg.height = self.init_z_offset
                self.param_pub.publish(msg)
                self.time_stamp_ry = rospy.get_time() + 0.05

    def select_callback(self, new_state):
        print('select')
        pass

    def l1_callback(self, new_state):
        print('l1')
        pass

    def l2_callback(self, new_state):
        print('l2')
        pass

    def r1_callback(self, new_state):
        print('r1')
        pass

    def r2_callback(self, new_state):
        print('r2')
        pass

    def X_callback(self, new_state):
        print('X')
        pass 

    def A_callback(self, new_state):
        print('A')
        pass

    def B_callback(self, new_state):
        print('B')
        pass 

    def Y_callback(self, new_state):
        print('Y')
        pass 

    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            msg = BuzzerState()
            msg.freq = 3000
            msg.on_time = 0.1
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)

    def hat_xl_callback(self, new_state):
        print('hat_xl')
        pass

    def hat_xr_callback(self, new_state):
        print('hat_xr')
        pass

    def hat_yd_callback(self, new_state):
        print('hat_yd')
        pass

    def hat_yu_callback(self, new_state):
        print('hat_yu')
        pass

    def joy_callback(self, joy_msg):
        if len(joy_msg.axes) == 8:
            AXES_MAP = OLD_AXES_MAP
            BUTTON_MAP = OLD_BUTTON_MAP
        else:
            AXES_MAP = NEW_AXES_MAP
            BUTTON_MAP = NEW_BUTTON_MAP
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False
        buttons = dict(zip(BUTTON_MAP, joy_msg.buttons))
        self.callback(axes)
        if buttons['start']:
            if not self.stand:
                self.count += 1
                if self.count >= 10:
                    self.count = 0
                    self.stand = True
                    msg = BuzzerState()
                    msg.freq = 2000
                    msg.on_time = 0.3
                    msg.off_time = 0.01
                    msg.repeat = 1
                    self.buzzer_pub.publish(msg)
        else:
            if self.stand:
                self.init_z_offset = 0.025
                rospy.ServiceProxy('/walking/init_pose', Empty)()
                self.stand = False
            self.count = 0
 
        for key, value in axes.items(): # 轴的值被改变
            if key != 'ry':
                if self.last_axes is not None: 
                    if self.last_axes[key] != value:
                        axes_changed = True
        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                rospy.logerr(str(e))
        for key, value in buttons.items():
            new_state = ButtonState.Normal
            if self.last_buttons is not None:
                if value != self.last_buttons[key]:
                    new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
                else:
                    new_state = ButtonState.Holding if value > 0 else ButtonState.Normal
                callback = "".join([key, '_callback'])
                if new_state != ButtonState.Normal:
                    # rospy.loginfo(key + ': ' + str(new_state))
                    if  hasattr(self, callback):
                        try:
                            getattr(self, callback)(new_state)
                        except Exception as e:
                            rospy.logerr(str(e))

        self.last_buttons = buttons
        self.last_axes = axes

if __name__ == "__main__":
    node = JoystickController()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

