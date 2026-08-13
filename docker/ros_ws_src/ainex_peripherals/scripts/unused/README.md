# scripts/unused/ — 归档：不在日常运行路径上的文件

这里的文件**没有删除，仍然可用**。放进来只是为了让 `scripts/` 根目录只剩当前
真正在跑的节点，避免下次读这个目录时把过时脚本误当成活代码。

## roslaunch 仍然找得到这里的脚本

`roslib.packages._find_resource` 对整个 package 目录做 `os.walk`（只跳过点开头的
目录），再按可执行位过滤。所以 `<node pkg="ainex_peripherals" type="xxx.py">` 里的
`type` 属性**不需要写路径**，移进本目录后照样解析得到。

代价：**必须保留可执行位**。若哪天 `chmod -x` 了，`_executable_filter` 会把它过滤掉，
launch 会报 `cannot launch node ... type not found`。用 `git mv` 移动文件即可保住 mode。

## 各文件为什么在这里

### `99-usb-cam.rules` — 已失效
四组 VID/PID（`090c:b371`、`32e6:9005/9251/9221`）对应老的 V4L USB 摄像头。当前相机是
Orbbec Gemini 305（`2bc5:0840`），走 libuvc + `/dev/shm` 零拷贝，主机上由
`/etc/udev/rules.d/99-obsensor-libusb.rules` 负责。这条规则匹配不到任何在位设备，
`/dev/usb_cam` 符号链接也不存在。

**何时拿回去**：换回老的 UVC USB 摄像头时。

### `create_udev_rules` — 半废
用 `rospack find` 定位包路径，所以只能在 `ainex` 容器里跑；但 udev 在**主机**上，
容器内 `service udev restart` 无效。规则实际是手工拷到主机 `/etc/udev/rules.d/` 的。
详见脚本顶部注释。

**何时拿回去**：改成主机侧脚本（去掉 `rospack`，写死路径）之后。

### `joystick_control.py` — 闲置，缺硬件
`/dev/input/js0` 不存在（没接手柄）。除 `launch/joystick_control.launch` 外无任何引用。
代码本身没问题，插上手柄即可用：

    roslaunch ainex_peripherals joystick_control.launch

### `tf_broadcaster_imu.py` — 调试专用
只在 `launch/imu.launch` 的 `debug:=true` 分支下启动（默认 false），配 `rviz/imu.rviz`
可视化板载 IMU 姿态。日常 bringup 不需要它。

    roslaunch ainex_peripherals imu.launch debug:=true

## 不在这里的：两个 IMU 都是活的

`scripts/` 根目录下的两个 IMU 节点**不是新旧关系，是两颗不同的物理传感器**：

- `imu_odometry_node.py` — 板载 RRC IMU。`/ros_robot_controller/imu_raw` → `imu_calib`
  → `imu_complementary_filter` → `/imu` → `/odom` + `odom→base_link` TF。
  由 `imu.launch` ← `ainex_bringup/base.launch` ← `xyz_bringup.launch` 拉起，开机即在跑。
- `imu_gui_node.py` — 外接 WitMotion 10 轴 USB 串口 IMU（`/dev/ttyUSB0`）。发
  `/imu_gui`（6 轴陀螺积分航向角），供 `xyz_perception` 的 `depth_nav_node.py` 做航向
  修正。**不由 bringup 启动**，需手动 `roslaunch ainex_peripherals imu_gui.launch`。
