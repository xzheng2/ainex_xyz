#!/usr/bin/python3
# coding=utf8
# 独立按键节点(standalone board-button node)
# 第一次点击:锁定腿部舵机(1st click: lock leg servos 1-12) == servo_lock.sh
# 第二次点击:全部舵机掉电(2nd click: power down all servos) == servo_powerdown.desktop
# 长按:关机(long-press: shutdown via .halt.txt)
import time, subprocess
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from ros_robot_controller.msg import BuzzerState, RGBState, RGBsState
from ainex_interfaces.srv import SetWalkingCommand


class ButtonNode:
    SERVO_LOCK_IDS  = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]     # 锁定的舵机(mirrors servo_lock.sh)
    SERVO_LOCK_FILE = '/home/ubuntu/share/src/.servo_lock.txt'    # 容器内路径(container view of host /home/pi/docker/src/.servo_lock.txt)
    HALT_FILE       = '/home/ubuntu/share/src/.halt.txt'
    POWERDOWN_CWD   = '/home/ubuntu/software/ainex_controller'

    def __init__(self):
        rospy.init_node('button', anonymous=False)
        self.button_state = 0
        self.button_pressed = False
        self.count_button_press = 0
        self.count_button_release = 0
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        self.rgb_pub    = rospy.Publisher('/ros_robot_controller/set_rgb', RGBsState, queue_size=1)
        rospy.wait_for_service('/sensor/button/enable')
        # 开启按钮发布(enable button publishing)
        rospy.ServiceProxy('/sensor/button/enable', SetBool)(True)
        # 订阅按钮状态(subscribe to button state)
        rospy.Subscriber('/sensor/button/get_button_state', Bool, self.button_callback)
        rospy.loginfo('button_node ready: 1st click=lock legs, 2nd click=power down all')

    def _beep(self, repeat=1):
        # 板载蜂鸣器提示(onboard buzzer feedback via /ros_robot_controller/set_buzzer)
        self.buzzer_pub.publish(BuzzerState(freq=1900, on_time=0.1, off_time=0.06, repeat=repeat))

    def _stop_gait(self):
        # 尽力停止步态,不依赖 GaitManager(best-effort gait stop; no GaitManager dependency)
        try:
            rospy.wait_for_service('walking/command', timeout=0.5)
            rospy.ServiceProxy('walking/command', SetWalkingCommand)('stop')
        except Exception:
            pass

    def lock_servos(self):
        # 锁定舵机(lock servos) — mirrors servo_lock.sh
        with open(self.SERVO_LOCK_FILE, 'w') as f:
            f.write("\n".join(str(i) for i in self.SERVO_LOCK_IDS) + "\n")
        rospy.loginfo('Locked servos: %s' % self.SERVO_LOCK_IDS)

    def powerdown_servos(self):
        # 与 servo_powerdown.desktop 完全一致:以 root 运行同一段 Board 代码
        # (identical to servo_powerdown.desktop — run the same Board payload as ROOT.
        #  docker exec's default user is root; the button is already inside the container,
        #  so `sudo` reaches the same root-in-container state as the desktop shortcut.)
        powerdown_code = (
            "from ros_robot_controller_sdk import Board\n"
            "board = Board()\n"
            "for sid in range(1, 25):\n"
            "    board.bus_servo_enable_torque(sid, 1)\n"
        )
        subprocess.Popen(['sudo', 'python3', '-c', powerdown_code], cwd=self.POWERDOWN_CWD)
        rospy.loginfo('Powered down (limp) all servos 1-24 (as root, like servo_powerdown.desktop).')

    def button_callback(self, msg):
        # rospy.loginfo('button_state: %s'%msg.data)
        if msg.data == 0:
            self.count_button_release = 0
            self.count_button_press += 1
        else:
            if self.button_pressed:
                self.count_button_release += 1
            self.count_button_press = 0
        if self.button_pressed:
            if self.count_button_release >= 5:
                self.count_button_release = 0
                self.count_button_press = 0
                self.button_pressed = False
                self.button_state += 1
                if self.button_state == 1:
                    # 第一次点击:锁定舵机(1st click: lock servos) — 蜂鸣一声(1 beep)
                    self.lock_servos()
                    self.rgb_pub.publish(RGBsState([RGBState(1, 0, 0, 255)]))
                    self._beep(repeat=1)
                elif self.button_state == 2:
                    # 第二次点击:舵机掉电(2nd click: power down servos) — 蜂鸣两声(2 beeps)
                    self.powerdown_servos()
                    self.rgb_pub.publish(RGBsState([RGBState(1, 255, 0, 0)]))
                    self._beep(repeat=2)
                    self.button_state = 0
            elif self.count_button_press > 200:
                self.buzzer_pub.publish(BuzzerState(freq=3000, on_time=0.5, off_time=0.01, repeat=1))
                time.sleep(0.5)
                self.count_button_press = 0
                self.count_button_release = 0
                with open(self.HALT_FILE, 'w') as f:
                    f.write('1')
                # os.system('sudo halt')
                time.sleep(2)
        else:
            if self.count_button_press >= 10:
                self.count_button_press = 10
                self.button_pressed = True
                # 蜂鸣器提示(buzzer sounds)
                self.buzzer_pub.publish(BuzzerState(freq=3000, on_time=0.1, off_time=0.01, repeat=1))


if __name__ == '__main__':
    ButtonNode()
    rospy.spin()
