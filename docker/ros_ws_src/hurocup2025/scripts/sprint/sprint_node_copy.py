#!/usr/bin/env python3
# encoding: utf-8
# @data:2025/07/24
# @author:caicai
# apriltag追踪(apriltag tracking)
import time
import rospy
import signal
from ainex_sdk import common, pid
from ainex_example.apriltag_common import Common
from apriltag_ros.msg import AprilTagDetectionArray
from ainex_kinematics.gait_manager import GaitManager
from std_msgs.msg import Bool


class ApriltagTrackNode(Common):
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True
        self.object_info = []
        self.image_process_size = [160, 120]
        self.head_pan_init = 500  # 左右舵机的初始值(the initial values of the left and right servos)
        self.head_tilt_init = 500  # 上下舵机的初始值(the initial values of the up and down servos)

        self.turn = 0
        self.area = 0

        self.gait_manager = GaitManager()
        # self.gait_param = gait_manager.get_gait_param()
        self.gait_param_dsp = [270, 0.2, 0.02]
        self.gait_param = self.gait_manager.get_gait_param()
        self.gait_param.update({
            # Basic offsets
            'init_x_offset': 0.0,
            'init_y_offset': 0.002,
            'init_z_offset': 0.005,

            # Gait parameters
            'step_fb_ratio': 0.00,
            'z_swap_amplitude': 0.006,
            'pelvis_offset': 3,

            # Movement amplitudes
            'x_move_amplitude': 0.0,  # Fixed typo from original ('x_move_amplitud')
            'y_move_amplitude': 0.0,
            'z_move_amplitude': 0.02,

            # Other parameters
            'angle_move_amplitude': 0,
            'arm_swing_gain': 0.6,
            'hip_pitch_offset': 19.0,

            # Note: gait_param_dsp is kept separate as it's a list
            # gait_param_dsp = [300, 0.2, 0.02]
        })

        super().__init__(name, self.head_pan_init, self.head_tilt_init)
        signal.signal(signal.SIGINT, self.shutdown)

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.apriltag_callback)
        self.motion_manager.run_action('stand')

        if rospy.get_param('~start', True):
            self.enter_func(None)
            self.start_srv_callback(None)
            common.loginfo('start track')

        self.run_signal = False  # 控制是否开始跑步
        rospy.Subscriber('/start_running', Bool, self.start_callback)

    def start_callback(self, msg):
        if msg.data:
            print("收到启动信号，开始跑步")
            self.run_signal = True

    def shutdown(self, signum, frame):
        self.running = False
        common.loginfo('%s shutdown' % self.name)

    def apriltag_callback(self, msg):
        if msg.detections != []:
            self.object_info = msg.detections[0].corners

    def process(self, corners):
        x = int((corners[0].x + corners[2].x) / 2)
        y = int((corners[0].y + corners[2].y) / 2)
        # self.turn = 0
        self.object_info = []
        
        # if abs(x - 320) < 30:
        #     x = 320
        #     self.turn = 0
        # if abs(y - 240) < 30:
        #     y = 240
        # if x - 320 > 30:
        #     self.turn = -1
        # if x - 320 < -30:
        #     self.turn = 1
        k = 10 #调节拐弯档数（如果是6档，即正负四个节点，也就是k = 4，以此类推。八档就是k = 5）
        self.turn = - int((x - 320)/320*k)  

        self.area = 0.0
        n = len(corners)
        for i in range(n):
            x_i = corners[i].x
            y_i = corners[i].y
            x_j = corners[(i + 1) % n].x
            y_j = corners[(i + 1) % n].y
            self.area += (x_i * y_j) - (x_j * y_i)
        self.area = abs(self.area) / 2.0

	#change x_amplitude under here !!!
        self.gait_manager.set_step(self.gait_param_dsp, 0.03, 0, self.turn, self.gait_param, arm_swap=30, step_num=0)

    def run(self):
        print("Wait for start signal...")
        while not self.run_signal and self.running:
            time.sleep(0.1)

        start_time = time.time()
        while self.running:
            if self.start:
                if self.object_info != []:
                    print(self.object_info)
                    self.process(self.object_info)
                    print(int(self.turn))
                    print(int(self.area))

                if self.area > 33000:
                    print("stop")
                    break
                time.sleep(0.02)
            else:
                time.sleep(0.01)
        #倒退动作加到这
        end_time = time.time()
        print("Time: %s" %(end_time - start_time))
        self.gait_param.update({
            'hip_pitch_offset': 15.0,
        })
        self.gait_manager.set_step(self.gait_param_dsp, 0, 0, 0, self.gait_param, arm_swap=30, step_num=2)
        self.gait_manager.set_step(self.gait_param_dsp, -0.025, 0, 0, self.gait_param, arm_swap=30, step_num=15)
        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')


if __name__ == "__main__":
    ApriltagTrackNode('apriltag_track').run()
