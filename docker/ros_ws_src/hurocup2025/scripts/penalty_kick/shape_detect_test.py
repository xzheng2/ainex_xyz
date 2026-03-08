#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/11
# @author:aiden
# 自动踢球(autonomous ball kicking)
import time
import math
import rospy
import signal
from std_msgs.msg import Float64, String
from ainex_sdk import pid, misc, common
from ainex_example.color_common import Common
from ainex_example.pid_track import PIDTrack
from ainex_example.approach_object import ApproachObject
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect, ColorsDetect
import time
from ainex_sdk import Board

servo_control = Board()
#print('id23 serial servo move between 400 - 600')

class ShapeDetectNode(Common):
    # 左右踢球的动作名(action names for kicking the ball left and right)
    left_shot_action_name = 'left_shot'
    right_shot_action_name = 'right_shot'
    
    # 图像处理大小(image processing size)
    image_process_size = [160, 120]

    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True

        print('ShapeDetectNode init')
        # 存储检测结果(store detection result)
        self.objects_info = []
        self.count_miss = 0
        # 找球相关标志位(flag related to find ball)
        self.start_index = 0
        self.start_find_ball = False
        # 初始化头部位置(initialize head position)
        self.head_pan_init = 500  # 左右舵机的初始值(initial value of left-right servo)
        self.head_tilt_init = 300 # 上下舵机的初始值(initial value of up-down servo)
        self.head_time_stamp = rospy.get_time()
        # 初始化父类(initialize parent class)
        super().__init__(name, self.head_pan_init, self.head_tilt_init)
        self.calib_config = common.get_yaml_data('/home/ubuntu/ros_ws/src/ainex_example/config/calib.yaml')
        # 存放检测结果(store detection result)
        self.rl_dis = None
        self.ud_dis = None
        # 初始化接近控制(initialize approaching control)
        self.approach_object = ApproachObject(self.gait_manager, step_mode=0)
        self.approach_object.update_gait(dsp=[400, 0.2, 0.02])
        self.approach_object.update_stop_count(1)
        self.approach_object.update_gait_range(x_range=[-0.013, 0.013])
        self.approach_object.update_approach_stop_value(30, 0, 3)
        signal.signal(signal.SIGINT, self.shutdown)
        
        # 头部的pid追踪(head PID tracking)
        self.head_pan_range = [125, 875]  # 左右转动限制在这个范围， 125为右(limit the left and right rotation in this range, with 125 as right)
        self.head_tilt_range = [260, 500]  # 上下限制在这个范围， 250为下(limit the up and down in this range, with 250 as down)
        self.pid_rl = pid.PID(0.1, 0.0, 0.001)
        self.pid_ud = pid.PID(0.1, 0.0, 0.001)
        self.head_pan_init = 500  # 左右舵机的初始值(initial value of left-right servo)
        self.head_tilt_init = 300 # 上下舵机的初始值(initial value of up-down servo)
        self.rl_track = PIDTrack(self.pid_rl, self.head_pan_range, self.head_pan_init)
        self.ud_track = PIDTrack(self.pid_ud, self.head_tilt_range, self.head_tilt_init)
        
        # 躯体的追踪参数(body tracking parameter)
        self.yaw_stop = 40  # 躯体停止转动时头部左右舵机脉宽值和中位500的差值(When body stops rotating, the difference between the pulse width of the left and right head servo and the neutral position 500)
        
        # 找球时头部经过的5个位置，左右舵机，上下舵机，时间ms(When searching for the ball, the robot head will pass through 5 positions, with specific pulse width values and time (in milliseconds) for the left-right and up-down servos at each position)
        # left_down, left_up, center_up, right_up, right_down
        self.find_ball_position = [[650, 300, 1000], 
                                   [650, 500, 1000], 
                                   [500, 500, 1000], 
                                   [350, 500, 1000],
                                   [350, 300, 1000]
                                   ]

        # 订阅颜色识别结果(subscribe color recognition result)
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色(set color)
        # 初始化站姿(initialize standing posture)
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
        print('set_color_srv_callback called')
        param = ColorDetect()
        #param.color_name = msg.data
        param.color_name = 'white'
        param.detect_type = 'circle'
        param.use_name = True
        param.image_process_size = self.image_process_size
        param.min_area = 20
        param.max_area = 128*128/16
        #param.max_area = self.image_process_size[0]*self.image_process_size[1]
        #self.detect_pub.publish([param])

        param2 = ColorDetect()
        #param.color_name = msg.data
        param2.color_name = 'kick_ball'
        param2.detect_type = 'circle'
        param2.use_name = True
        param2.image_process_size = self.image_process_size
        param2.min_area = 20
        param2.max_area = 128*128/16
        
        common.loginfo('%s set_color' % self.name)

        param1 = ColorDetect()
        param1.color_name = msg.data
        param1.detect_type = 'rect'
        param1.use_name = True
        param1.image_process_size = self.image_process_size
        param1.min_area = 20
        param1.max_area = self.image_process_size[0]*self.image_process_size[1]
        #self.detect_pub.publish([param])
        self.detect_pub.publish([param, param2, param1])
        

        return [True, 'set_color']

    def get_color_callback(self, msg):
        # 获取颜色识别结果(obtain color recognition result)
        self.objects_info = msg.data
        # 获取识别结果，根据结果计算距离(Obtain recognition result, and calculate distance based on the result)
        #for object_info in self.objects_info:
        #    if object_info.type == 'circle':
        #        object_info.x = object_info.x - self.calib_config['center_x_offset']
        #        self.rl_dis, self.ud_dis = self.head_track_process(object_info)
    
    #def setServoPostion(servo_id, position)
    #servo_id = 23  # 舵机id(0-253)(servo ID (0-253))
    #position = 400  # 位置(0-1000)(position (0-1000))
    #duration = 0.5  # 时间(0.02-30s)(time (0.02-30))
    #servo_control.bus_servo_set_position(duration, [[servo_id, position]])
    #time.sleep(duration)  

   
    def run(self):
        #while self.running:
        print('Run')
        while True:
            # 状态判断(determine state)
            #if self.start:
            if True:
                time.sleep(0.1)
            else:
                time.sleep(0.01)

        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    ShapeDetectNode('ShapeDetect').run()
