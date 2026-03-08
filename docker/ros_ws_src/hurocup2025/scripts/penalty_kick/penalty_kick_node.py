#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/11
# @author:aiden
# 自动踢球(autonomous ball kicking)
import time
import math
import rospy
import signal
import numpy as np

from std_msgs.msg import Float64, String
from ainex_sdk import pid, misc, common
from ainex_example.color_common import Common
from ainex_example.pid_track import PIDTrack
from approach_object import ApproachObject
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect, ColorsDetect
from ainex_kinematics.motion_manager import MotionManager

from ainex_sdk import Board



class PenaltyKickBallNode(Common):
    # 左右踢球的动作名(action names for kicking the ball left and right)
    left_shot_action_name = 'left_shot_long'
    right_shot_action_name = 'right_shot_long'
    right_side_shot_action_name = 'right_side_kick_1'
    side_move_right = 'move_right'
    side_move_left = 'move_left'
    
    # 图像处理大小(image processing size)
    image_process_size = [160, 120]

    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True
        # 存储检测结果(store detection result)
        self.objects_info = []

        self.ignore_ball = False

        # 初始化头部位置(initialize head position)
        self.servo_control = Board()
        self.head_pan_init = 450  # 左右舵机的初始值(initial value of left-right servo)
        self.head_tilt_init = 500 # 上下舵机的初始值(initial value of up-down servo)
        self.head_time_stamp = rospy.get_time()
        # 初始化父类(initialize parent class)
        super().__init__(name, self.head_pan_init, self.head_tilt_init)
        self.calib_config = common.get_yaml_data('/home/ubuntu/ros_ws/src/ainex_example/config/calib.yaml')
        # 存放检测结果(store detection result)
        self.rl_dis = None
        self.ud_dis = None
        self.gap_x = None
        self.ball_x = None
        self.ball_data = None
        self.ball_still_check_count = 0
        self.max_ball_still_check_count = 10
        self.max_ball_history = 10
        self.ball_move_threshold = 2.0
        self.ball_still_frame_required = 5
        self.ball_positions = []
        #self.recored_ball_servo_position = []
        #self.start_seeking_walls = False
        self.start_index = 0
       #self.start_find_ball = False
        # self.find_ball_position = [[450, 500, 1000], 
        #                            [400, 400, 1000], 
        #                            [350, 350, 1000], 
        #                            [300, 300, 1000],
        #                            ]
        #self.find_ball_position = [[350, 500, 2000],
        #                           [350, 300, 2000],
        #                          [650, 300, 2000], 
        #                           [650, 500, 2000], 
        #                           [500, 500, 2000], 
        #                           ]
        self.find_ball_position = [[500, 400, 1000],[300, 450, 1500],
                                   [300, 350, 1500],
                                   [700, 350, 1500], 
                                   [700, 500, 1500], 
                                   [500, 450, 1500], 
                                   ]
                                   
        self.status = 'idle'
        # 初始化接近控制(initialize approaching control)
        self.gait_param = self.gait_manager.get_gait_param()
        self.gait_param['hip_pitch_offset'] = 20
        # self.gait_param['step_fb_ratio'] = 0.025
        self.gait_param['init_y_offset'] = 0.0
        self.gait_param['step_height'] = 0.015
        self.gait_param['pelvis_offset'] = 3.0

        self.approach_object = ApproachObject(self.gait_manager, step_mode=0, debug = False)
        self.approach_object.update_gait(dsp=[400, 0.2, 0.02], walking_param=self.gait_param)
        self.approach_object.update_stop_count(1)
        self.approach_object.update_gait_range(x_range=[-0.013, 0.013])
        self.approach_object.update_approach_stop_value(40, 0, 3)
        signal.signal(signal.SIGINT, self.shutdown)
        
        # 头部的pid追踪(head PID tracking)
        self.head_pan_range = [300, 700]  # 左右转动限制在这个范围， 125为右(limit the left and right rotation in this range, with 125 as right)
        self.head_tilt_range = [270, 500]  # 上下限制在这个范围， 250为下(limit the up and down in this range, with 250 as down)
        self.pid_rl = pid.PID(0.1, 0.0, 0.001)
        self.pid_ud = pid.PID(0.1, 0.0, 0.001)
        self.head_pan_init = 480  # 左右舵机的初始值(initial value of left-right servo)
        self.head_tilt_init = 500 # 上下舵机的初始值(initial value of up-down servo)
        self.rl_track = PIDTrack(self.pid_rl, self.head_pan_range, self.head_pan_init)
        self.ud_track = PIDTrack(self.pid_ud, self.head_tilt_range, self.head_tilt_init)
        
        # 躯体的追踪参数(body tracking parameter)
        self.yaw_stop = 40  # 躯体停止转动时头部左右舵机脉宽值和中位500的差值(When body stops rotating, the difference between the pulse width of the left and right head servo and the neutral position 500)

        # 订阅颜色识别结果(subscribe color recognition result)
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        #rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色(set color)
        # 初始化站姿(initialize standing posture)
        self.motion_manager = MotionManager('/home/ubuntu/software/ainex_controller/ActionGroups/penalty_kick')
        self.motion_manager.run_action('walk_ready')
        # 是否自动开始(whether to start automatically)
        if rospy.get_param('~start', True):
            # 通知颜色识别准备，此时只显示摄像头原画(Notify the color recognition to prepare, at this time only display the camera original image)
            target_color = rospy.get_param('~color', 'blue')  # 设置识别蓝色(set to recognize blue)
            self.enter_func(None)
            self.set_color_srv_callback(String(target_color))  
            self.start_srv_callback(None)  # 开启颜色识别(enable color recognition)
            common.loginfo('start kick %s ball' % target_color)
    # 退出回调(exit callback)
    def shutdown(self, signum, frame):
        with self.lock:
            self.running = False
            common.loginfo('%s shutdown' % self.name)
    # 设置颜色服务回调(set color service callback)
    def set_color_srv_callback(self, msg):
        # 设置追踪颜色(set color to be tracked)
        param1 = ColorDetect()
        param1.color_name = msg.data
        param1.detect_type = 'circle'
        param1.use_name = True
        param1.image_process_size = self.image_process_size
        param1.min_area = 20
        param1.max_area = 128*128/16

        param3 = ColorDetect()
        param3.color_name = 'kick_ball'
        param3.detect_type = 'circle'
        param3.use_name = True
        param3.image_process_size = self.image_process_size
        param3.min_area = 20
        param3.max_area = 128*128/16

        # param2 = ColorDetect()
        # param2.color_name = 'blue'
        # param2.detect_type = 'rect'
        # param2.use_name = True
        # param2.image_process_size = self.image_process_size
        # param2.min_area = 100
        # param2.max_area = self.image_process_size[0]*self.image_process_size[1]

        #self.detect_pub.publish([param1, param2, param3])
        # self.detect_pub.publish([param1, param2])
        self.detect_pub.publish([param1])
        
        common.loginfo('%s set_color' % self.name)
        return [True, 'set_color']

    def get_color_callback(self, msg):
        # 获取颜色识别结果(obtain color recognition result)
        self.objects_info = msg.data
        # 获取识别结果，根据结果计算距离(Obtain recognition result, and calculate distance based on the result)
        rect_num = 0
        rect_list = []
        ball_list = []
        for object_info in self.objects_info:
            if object_info.type == 'circle' :
                object_info.x = object_info.x - self.calib_config['center_x_offset']

                #if self.status != 'wait_for_next_kick':
                if self.ignore_ball is not True:
                    self.ball_x = object_info.x
                    self.ball_width = object_info.width
                    self.ball_height = object_info.height
                    ball_list.append(object_info)
                    print("Get ball_x {}".format(self.ball_x))

                    self.rl_dis, self.ud_dis = self.head_track_process(object_info)
                    
                
            if object_info.type == 'rect':
                rect_list.append([object_info.x, object_info.y, object_info.width, object_info.height])
                
                
                #self.gait_manager.set_step(dsp, 0, 0, -int(yaw_output), gait_param, arm_swap=self.arm_swap, step_num=self.step_mode)
                
                if self.status == 'start_chasing_ball':
                    gait_param = self.gait_manager.get_gait_param()
                    gait_param['pelvis_offset'] = 3
                    gait_param['step_height'] = 0.02
                    gait_param['z_swap_amplitude'] = 0.006
                    dsp = [300, 0.2, 0.02]

                    #image size is at 640*480, we use 320 as left/right division
                    # if object at the left side, turn right
                    print('rect info [x, w]')
                    print(object_info.x, object_info.width)
                    if object_info.x < 320 and object_info.x > 200:
                        print('obstacle at left, turn right')
                        self.gait_manager.set_step(dsp, 0.01, 0, 1, gait_param)

                    # if object at the right side, turn left
                    if object_info.x > 320 and object_info.x < 440:
                        print('obstacle at right, turn left')
                        self.gait_manager.set_step(dsp, 0.01, 0, -1, gait_param)                

        if len(rect_list) == 2:
            self.gap_x = (rect_list[0][0] + rect_list[1][0])/2
            print("Get ball_x {}, gap_x {}".format(self.ball_x, self.gap_x))

            #restore to ball tracking
            #if len(ball_list) < 1 and self.start_seeking_walls :
            #    print("restore recored ball servo position")
            #    self.start_seeking_walls = False
            #    #self.recored_ball_servo_position = [self.rl_dis, self.ud_dis]
            #    self.motion_manager.set_servos_position(20, [[23, int(self.recored_ball_servo_position[0])], [24, int(self.recored_ball_servo_position[1])]])



    # 头部PID跟踪(head PID tracking)
    def head_track_process(self, object_info):
        # 头部追踪(head tracking)
        if abs(object_info.x - object_info.width/2) < 10:
            object_info.x = object_info.width/2
        if abs(object_info.y - object_info.height/2) < 10:
            object_info.y = object_info.height/2
        rl_dis = self.rl_track.track(object_info.x, object_info.width/2)
        ud_dis = self.ud_track.track(object_info.y, object_info.height/2)
        self.motion_manager.set_servos_position(20, [[23, int(rl_dis)], [24, int(ud_dis)]])

        #record ball positions
        self.ball_positions.append((rl_dis,ud_dis))
        if len(self.ball_positions) > self.max_ball_history:
            self.ball_positions.pop(0)

        return rl_dis, ud_dis
    # 躯体追踪(body tracking)
    def body_track_process(self, rl_dis, ud_dis):                                    
        # 左右根据头部左右舵机位置值进行调整(adjust left and right based on the position value of the left and right head servos)
        if self.ball_x is not None:
            yaw_stop = 500 + math.copysign(self.yaw_stop, rl_dis - 500)
            yaw_stop_ = (yaw_stop - rl_dis)/9
            if 15 < abs(yaw_stop - rl_dis) < 27:
                yaw_stop_ = math.copysign(4, yaw_stop - rl_dis)
            if self.approach_object.process(500 - ud_dis, 0 + self.calib_config['center_x_offset'], yaw_stop_, self.head_tilt_range[0], 0, 0, self.ball_width, self.ball_height):          
                self.gait_manager.disable()
                if rl_dis > 500:
                    self.motion_manager.run_action(self.left_shot_action_name)  # 左脚踢(kick with left foot)
                    # self.motion_manager.run_action(self.right_shot_action_name)
                else:
                    self.motion_manager.run_action(self.right_shot_action_name)  # 右脚踢(kick with right foot)
                # After kicking, reset the status
                #self.status = 'waitting_ball_still'
                #self.ball_x = None
                #print('reset head position')
                #print('wait_for_next_kick TO find_ball_process')
                gait_param = self.gait_manager.get_gait_param()
                gait_param['pelvis_offset'] = 5
                gait_param['step_height'] = 0.035
                dsp = [250, 0.2, 0.02]
                self.gait_manager.set_step(dsp, 0.02, 0, 0, gait_param, step_num=5)

                #self.servo_control.bus_servo_set_position(0.5, [[23, 450]])
                self.status = 'wait_for_next_kick'
                self.wait_for_next_kick_count = 0
                self.ignore_ball = True
                self.ball_x = None

    def is_ball_still(self):
        if len(self.ball_positions) < self.max_ball_history:
            return False
        
        displacements = []
        for i in range(1, len(self.ball_positions)):
            prev = np.array(self.ball_positions[i-1])
            curr = np.array(self.ball_positions[i])
            dist = np.linalg.norm(curr-prev)
            displacements.append(dist)

        still_count = sum(d < self.ball_move_threshold for d in displacements[-self.ball_still_frame_required:])
        return still_count == self.ball_still_frame_required\

    def find_ball_process(self):
        # 根据预设的扫描点进行找球(find ball based on the preset scanning points)
        if rospy.get_time() > self.head_time_stamp:
            if self.start_index > len(self.find_ball_position) - 1:
                self.start_index = 0
            rl_dis = self.find_ball_position[self.start_index][0]
            ud_dis = self.find_ball_position[self.start_index][1]
            self.rl_track.update_position(rl_dis)  # pid的输出值要跟着更新(update output value of PID)
            self.ud_track.update_position(ud_dis)
            self.motion_manager.set_servos_position(self.find_ball_position[self.start_index][2], [[23, rl_dis], [24, ud_dis]])

            self.head_time_stamp = rospy.get_time() + self.find_ball_position[self.start_index][2]/1000.0
            self.start_index += 1

    def run(self):
        self.motion_manager.run_action(self.right_side_shot_action_name)
        self.status = 'waitting_ball_still'
        wait_ball_still_count = 0

        while self.running:
            # 状态判断(determine state)
            if self.start:
                #ball_data = None 
                
                # 在检测到的对象中查找球(search for the ball in the detected objects)
                #for object_info in self.objects_info:
                #    if object_info.type == 'circle':
                #        ball_data = object_info

                if self.status == 'waitting_ball_still': 
                    if self.ball_x is None:
                        print('go find_ball_process')
                        self.find_ball_process()
                        time.sleep(0.5)
                    else:
                        if self.is_ball_still():
                            self.status = 'try_side_walk'
                        else:
                            if wait_ball_still_count > 10:
                                print("Wait over 10. times, go side walk")
                                self.status = 'try_side_walk'
                            else:
                                print("Ball is moving")
                                time.sleep(0.5)
                                wait_ball_still_count += 1
                            continue

                #side move the robot to get a good direction to approach the ball
                if self.status == 'try_side_walk'  : 
                    #print("rl_dis {}, ball_x {}, gap_x {}".format(self.rl_dis, self.ball_x, self.gap_x))
                    #if self.gap_x is not None and self.ball_x is not None:
                    #if ball_data is not None:
                    if True:
                        #if self.rl_dis < 500 and self.ball_x > self.gap_x:
                        gait_param = self.gait_manager.get_gait_param()
                        gait_param['pelvis_offset'] = 3
                        gait_param['step_height'] = 0.035
                        dsp = [220, 0.2, 0.02]

                        if self.rl_dis < 498:
                            #print('side moving right')
                            self.gait_manager.set_step(dsp, 0.005, -0.02, 0, gait_param)
                        elif self.rl_dis > 502:
                            #print('side moving left')
                            self.gait_manager.set_step(dsp, 0.005, 0.02, 0, gait_param)
                        else:
                            self.gait_manager.disable()
                            self.status = 'start_chasing_ball'


                # 如果识别到球就进行躯体追踪(If ball is recognized, perform body tracking) 
                if self.rl_dis is not None and self.ball_x is not None and self.status == 'start_chasing_ball':
                    #print(self.rl_dis, self.ud_dis)  
                    #print('body_track_process')
                    self.body_track_process(self.rl_dis, self.ud_dis)
                #else:
                #    print('IDLE body_track_process')

                if self.status == 'wait_for_next_kick':
                    if self.ball_x is None:
                    #if ball_data is None:
                        if self.wait_for_next_kick_count > 5:
                            print('wait_for_next_kick no ball position, TO find_ball_process')
                            #gait_param = self.gait_manager.get_gait_param()
                            #gait_param['pelvis_offset'] = 5
                            #gait_param['step_height'] = 0.035
                            #dsp = [250, 0.2, 0.02]
                            #self.gait_manager.set_step(dsp, 0.02, 0, 0, gait_param, step_num=5)
                            self.wait_for_next_kick_count = 0
                            self.ignore_ball = False
                            self.find_ball_process()
                            #self.status = 'find_ball_process'
                            
                        else:
                            #print('find_ball_process')
                            print('wait_for_next_kick no ball position add count')
                            self.wait_for_next_kick_count += 1
                            time.sleep(0.5)
                    else:
                        print('wait_for_next_kick get ball data, now change to  start_chasing_ball')
                        #self.gait_manager.set_step(dsp, 0.02, 0, 0, gait_param, step_num=7)
                        #time.sleep(5)
                        self.status = 'start_chasing_ball'

                
                time.sleep(0.01)
            else:
                time.sleep(0.01)

        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    PenaltyKickBallNode('penalty_kick_ball').run()
