#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ainex IMU visualization GUI + /imu_gui publisher -- NEW serial WitMotion IMU.

This node reads a standalone WitMotion 10-axis IMU wired directly to the Pi over a
USB-serial (CH340) adapter -- NOT the RRC-board IMU that ros_robot_controller_node.py
owns. The sensor speaks the WitMotion 0x55 frame protocol at 9600 baud (8N1, 11-byte
frames, last byte = low 8 bits of the sum of the first 10). Frames:
  0x51 accel  (/32768*16   -> g)
  0x52 gyro   (/32768*2000 -> deg/s)   [integrated for the 6-axis heading]
  0x53 angle  (/32768*180  -> deg)     roll/pitch/yaw, the module's own 9-axis fusion
  0x54 mag    (raw counts)
The parser is ported from newimureference/imu_gui_mac.py (Hiwonder wit_ros2_imu proto).

Heading convention: the robot's real yaw/heading is the ROLL axis (index 0) -- set by
HEADING_AXIS (IMU mounted so sensor ROLL aligns with robot vertical/yaw). Green needle /
"9ax" = the module's fused angle (drift-free reference); red = 6ax gyro (drifts).

The node publishes the 6-axis gyro-integrated angles (deg) as std_msgs/Float64 scalars:
  /imu_gui        heading axis (HEADING_AXIS) -- depth_nav_node.py subscribes for goal-correction
  /imu_gui_pitch  gyro_ang[1]  -- pitch (web viewer)
  /imu_gui_roll   gyro_ang[2]  -- roll  (web viewer)

Run inside the ``ainex`` container as user ubuntu (device /dev/ttyUSB0 is /dev-mounted
and readable by the dialout group). With SHOW_GUI=True also pass the X env vars:
  docker exec -e DISPLAY=:1 -e LIBGL_ALWAYS_SOFTWARE=1 -u ubuntu ainex \
    bash -lc 'source ~/ros_ws/devel/setup.bash && rosrun ainex_peripherals imu_gui_node.py'
Or: roslaunch ainex_peripherals imu_gui.launch  (headless by default; see SHOW_GUI).
"""

import sys
import os
import glob
import math
import time
import threading
import termios

import matplotlib
matplotlib.use("Qt5Agg")                     # PyQt5 backend (container has no macosx)
matplotlib.rcParams["font.family"] = "monospace"
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from matplotlib.patches import Circle

import rospy
from std_msgs.msg import Float64

# Master switch for the on-screen GUI window. True = show the window; False = run
# headless (no window) while still reading the IMU and publishing /imu_gui.
SHOW_GUI = False

G = 9.80665  # m/s^2 per g (accel already comes in g from the WitMotion frame)

# Which gyro axis is the robot's real heading/yaw. Flip this one constant to re-map it:
#   0 = ROLL (current -- IMU mounted so sensor roll aligns with robot yaw)
#   1 = PITCH, 2 = YAW.
HEADING_AXIS = 0  # 0=roll, 1=pitch, 2=yaw
HEADING_NAME = ("ROLL", "PITCH", "YAW")[HEADING_AXIS]  # label follows HEADING_AXIS

# Manual trim added to the heading rate before 6-axis integration, to null a residual
# constant gyro bias on the heading axis (HEADING_AXIS).
HEADING_GYRO_OFFSET_DPS = -0.0  # deg/s (applied to the heading axis)

# Stable by-id glob for the CH340 USB-serial adapter the WitMotion IMU uses.
# Resolved before falling back to number-based /dev/ttyUSB* enumeration.
IMU_DEVICE_ID_GLOB = "/dev/serial/by-id/usb-1a86_USB_Serial*"
DEFAULT_BAUD = 9600

BAUD_MAP = {9600: termios.B9600, 115200: termios.B115200, 230400: termios.B230400,
            57600: termios.B57600, 38400: termios.B38400, 19200: termios.B19200}


def find_port():
    """Resolve the WitMotion IMU serial port, preferring the stable by-id path."""
    by_id = sorted(glob.glob(IMU_DEVICE_ID_GLOB))
    if by_id:
        return by_id[0]
    cands = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    return cands[0] if cands else None


def open_serial(port, baud):
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = a[1] = a[3] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[4] = a[5] = BAUD_MAP.get(baud, termios.B9600)
    termios.tcsetattr(fd, termios.TCSANOW, a)
    time.sleep(0.2)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def s16(lo, hi):
    v = (hi << 8) | lo
    return v - 65536 if v > 32767 else v


# ---------------------------------------------------------------- serial reader
class SerialImuReader(threading.Thread):
    """Background thread: reads /dev/ttyUSB0, parses WitMotion 0x55 frames, keeps the
    latest sample and the running 6-axis gyro integration. Publishes the heading-axis
    6-axis integrated angle (deg) on /imu_gui every gyro frame.

    Exposes the same interface the GUI's App class consumes (snapshot /
    align_gyro_to_fused / connected / stop) so the visualization is reused unchanged.
    """

    IMU_GUI_TOPIC = "/imu_gui"              # gyro_ang[HEADING_AXIS] -- heading (depth_nav)
    IMU_GUI_PITCH_TOPIC = "/imu_gui_pitch"  # gyro_ang[1] = pitch
    IMU_GUI_ROLL_TOPIC = "/imu_gui_roll"    # gyro_ang[2] = roll

    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port, self.baud = port, baud
        self.lock = threading.Lock()
        self.running = True
        self.connected = False
        self.err = ""
        self.acc = [0.0, 0.0, 0.0]        # g
        self.gyro = [0.0, 0.0, 0.0]       # deg/s
        self.ang = [0.0, 0.0, 0.0]        # deg (roll, pitch, yaw) -- module 9-axis fusion
        self.mag = [0, 0, 0]
        self.gyro_ang = [0.0, 0.0, 0.0]   # deg, pure 6-axis gyro integration per axis
        self.last_t = None

        # publish the computed 6-axis heading (deg) for depth_nav to subscribe to,
        # plus the other two gyro-integrated axes (pitch, yaw) for the web viewer
        self.imu_gui_pub = rospy.Publisher(self.IMU_GUI_TOPIC, Float64, queue_size=10)
        self.imu_gui_pitch_pub = rospy.Publisher(self.IMU_GUI_PITCH_TOPIC, Float64, queue_size=10)
        self.imu_gui_roll_pub = rospy.Publisher(self.IMU_GUI_ROLL_TOPIC, Float64, queue_size=10)

    # -- serial read loop ---------------------------------------------------
    def run(self):
        try:
            fd = open_serial(self.port, self.baud)
        except Exception as e:
            self.err = "open %s failed: %s" % (self.port, e)
            rospy.logerr("imu_gui: %s", self.err)
            return
        self.connected = True
        rospy.loginfo("imu_gui: reading WitMotion IMU on %s @ %d baud", self.port, self.baud)
        buf = bytearray()
        while self.running and not rospy.is_shutdown():
            try:
                chunk = os.read(fd, 512)
                if chunk:
                    buf.extend(chunk)
            except BlockingIOError:
                pass
            except OSError as e:
                self.err = "serial disconnected: %s" % e
                rospy.logerr("imu_gui: %s", self.err)
                break
            i = 0
            while i <= len(buf) - 11:
                if buf[i] == 0x55 and 0x50 <= buf[i + 1] <= 0x5a:
                    p = buf[i:i + 11]
                    if (sum(p[:10]) & 0xff) == p[10]:
                        self._handle(p)
                        i += 11
                        continue
                i += 1
            del buf[:i]
            time.sleep(0.002)
        os.close(fd)
        self.connected = False

    def _handle(self, p):
        t, x, y, z = p[1], s16(p[2], p[3]), s16(p[4], p[5]), s16(p[6], p[7])
        pub_snap = None
        with self.lock:
            if t == 0x51:                                   # accel -> g
                self.acc = [x / 32768 * 16, y / 32768 * 16, z / 32768 * 16]
            elif t == 0x52:                                 # gyro -> deg/s + integrate
                self.gyro = [x / 32768 * 2000, y / 32768 * 2000, z / 32768 * 2000]
                now = time.time()
                if self.last_t is not None:
                    dt = now - self.last_t
                    if 0.0 < dt < 0.5:                      # ignore first sample / stalls
                        for k in range(3):
                            rate = self.gyro[k]
                            if k == HEADING_AXIS:
                                rate += HEADING_GYRO_OFFSET_DPS  # bias-trim the heading axis
                            self.gyro_ang[k] += rate * dt
                self.last_t = now
                pub_snap = list(self.gyro_ang)              # [roll(=heading), pitch, yaw] (deg)
            elif t == 0x53:                                 # angle -> deg (9-axis fusion)
                self.ang = [x / 32768 * 180, y / 32768 * 180, z / 32768 * 180]
            elif t == 0x54:                                 # mag -> raw counts
                self.mag = [x, y, z]

        # publish the three 6-axis gyro-integrated angles (deg), every gyro frame:
        # /imu_gui is the heading (unchanged, for depth_nav); pitch/yaw are the other two axes
        if pub_snap is not None:
            self.imu_gui_pub.publish(Float64(pub_snap[HEADING_AXIS]))   # heading, unchanged
            self.imu_gui_pitch_pub.publish(Float64(pub_snap[1]))
            self.imu_gui_roll_pub.publish(Float64(pub_snap[2]))
            rospy.loginfo_throttle(1.0, "imu_gui: heading=%+.1f pitch=%+.1f roll=%+.1f deg"
                                   % (pub_snap[HEADING_AXIS], pub_snap[1], pub_snap[2]))

    # -- GUI-facing interface ----------------------------------------------
    def snapshot(self):
        with self.lock:
            return (list(self.acc), list(self.gyro), list(self.ang),
                    list(self.mag), list(self.gyro_ang))

    def align_gyro_to_fused(self):
        with self.lock:
            self.gyro_ang = list(self.ang)

    def stop(self):
        self.running = False
        rospy.signal_shutdown("gui closed")


# ---------------------------------------------------------------- GUI (matplotlib)
class App:
    # axis index -> (label, plot color). All three axes are shown; YAW is the heading.
    AXES = (("ROLL", "#e5a23b"), ("PITCH", "#4aa3df"), ("YAW", "#c678dd"))

    def __init__(self, reader):
        self.reader = reader
        self.t0 = time.time()
        # time-series buffers, one per axis (roll/pitch/yaw)
        self.hist_t = []
        self.hist_g = [[], [], []]        # 6-axis gyro-integrated (already continuous)
        self.hist_f = [[], [], []]        # absolute ref (fused), continuized for plotting
        self._fcont = [None, None, None]  # running continuized fused value per axis

        self.fig = plt.figure(f"Ainex IMU - {HEADING_NAME} axis = actual heading", figsize=(12, 6.8))
        self.fig.patch.set_facecolor("#0e1116")
        # single centered compass gauge: the heading axis is the robot's real heading
        self.ax_head = self.fig.add_axes([0.34, 0.42, 0.32, 0.52])
        # bottom: per-axis drift-vs-time plot
        self.ax_t = self.fig.add_axes([0.07, 0.10, 0.88, 0.26])
        self.ax_head.set_facecolor("#0e1116")
        self.ax_t.set_facecolor("#0e1116")
        for spine in self.ax_t.spines.values():
            spine.set_color("#2b3542")
        self.ax_t.tick_params(colors="#6b7d8f", labelsize=8)

        # buttons
        def mkbtn(x, w, label, cb):
            b = Button(self.fig.add_axes([x, 0.005, w, 0.045]),
                       label, color="#1b2430", hovercolor="#2b3a4d")
            b.label.set_color("#c9d5e0"); b.label.set_fontsize(10)
            b.on_clicked(cb)
            return b
        # re-sync the 6-axis integrators to the current fused reference
        self.b_align = mkbtn(0.40, 0.2, "Align 6ax to ref",
                             lambda e: self.reader.align_gyro_to_fused())

        # HUD text objects created once, content updated each frame
        self.hud_txt = self.fig.text(0.02, 0.995, "", color="#c9d5e0",
                                     family="monospace", fontsize=10, va="top")
        self.stat_txt = self.fig.text(0.66, 0.995, "", family="monospace",
                                      fontsize=10, va="top")
        # visible title stating the axis mapping for this upright-mounted IMU
        self.fig.text(0.5, 0.985,
                      f"{HEADING_NAME} = the robot's actual heading  (WitMotion serial IMU)"
                      f"   |   {HEADING_NAME.lower()}(heading) 6ax offset {HEADING_GYRO_OFFSET_DPS:+.1f} deg/s",
                      color="#e5a23b", ha="center", va="top", fontsize=12, fontweight="bold")

        self.anim = FuncAnimation(self.fig, self.update, interval=50,
                                  blit=False, cache_frame_data=False)

    # ---- one circular gauge: red = 6-axis (gyro), green = absolute ref (fused)
    def _draw_gauge(self, ax, label, fused_deg, gyro_deg, ref_label="9ax"):
        ax.cla()
        ax.set_aspect("equal"); ax.set_xlim(-1.35, 1.35); ax.set_ylim(-1.6, 1.45)
        ax.axis("off")
        ax.add_patch(Circle((0, 0), 1.0, fill=False, ec="#2b3542", lw=2))
        for lbl, a in (("N", 0), ("E", 90), ("S", 180), ("W", 270)):
            r = math.radians(a)
            ax.text(0.86 * math.sin(r), 0.86 * math.cos(r), lbl, color="#6b7d8f",
                    ha="center", va="center", fontsize=11, fontweight="bold")

        def arrow(deg, ln, color, lw):
            r = math.radians(deg)
            ax.annotate("", xy=(ln * math.sin(r), ln * math.cos(r)), xytext=(0, 0),
                        arrowprops=dict(arrowstyle="-|>", color=color, lw=lw))
        arrow(gyro_deg, 0.82, "#e5484d", 2.5)     # red = 6-axis gyro
        arrow(fused_deg, 0.82, "#30a46c", 3.5)    # green = absolute ref (fused)
        ax.add_patch(Circle((0, 0), 0.04, color="#c9d5e0"))
        drift = ((gyro_deg - fused_deg + 180) % 360) - 180
        ax.text(0, 1.28, label, color="#c9d5e0", ha="center", fontsize=13, fontweight="bold")
        # signed continuous (no %360) so 6ax matches the HUD value + imu_accum's convention
        ax.text(0, -1.24, f"{ref_label} {fused_deg:+6.1f}   6ax {gyro_deg:+7.1f}",
                color="#8aa0b4", ha="center", fontsize=10)
        ax.text(0, -1.44, f"drift {drift:+.1f} deg", color="#e5a23b", ha="center", fontsize=10)

    # ---- per-frame refresh
    def update(self, _frame):
        acc, gyro, ang, mag, gyro_ang = self.reader.snapshot()
        h = HEADING_AXIS

        # ---------- heading gauge (the HEADING_AXIS) ----------
        self._draw_gauge(self.ax_head, f"{HEADING_NAME} = HEADING", ang[h], gyro_ang[h],
                         ref_label="9ax")

        # ---------- per-axis drift-vs-time ----------
        WIN = 30.0                                   # show only the last 30 s
        t = time.time() - self.t0
        self.hist_t.append(t)
        for i in range(3):
            # continuize fused angle so it never jumps at the +/-180 wrap
            if self._fcont[i] is None:
                self._fcont[i] = ang[i]
            else:
                self._fcont[i] += ((ang[i] - self._fcont[i] + 180) % 360) - 180
            self.hist_f[i].append(self._fcont[i])
            self.hist_g[i].append(gyro_ang[i])       # gyro integration is already continuous
        while self.hist_t and self.hist_t[0] < t - WIN:
            self.hist_t.pop(0)
            for i in range(3):
                self.hist_f[i].pop(0); self.hist_g[i].pop(0)

        ax = self.ax_t
        ax.cla(); ax.set_facecolor("#0e1116")
        for i, (label, color) in enumerate(self.AXES):
            ax.plot(self.hist_t, self.hist_f[i], color=color, lw=1.0, alpha=0.5)  # abs ref
            ax.plot(self.hist_t, self.hist_g[i], color=color, lw=1.8,
                    label=f"{label.lower()} 6-axis")                              # 6-axis drift
        ax.set_xlim(max(0, t - WIN), max(WIN, t))
        ax.set_ylabel("angle (deg)", color="#8aa0b4", fontsize=9)
        ax.tick_params(colors="#6b7d8f", labelsize=8)
        for spine in ax.spines.values():
            spine.set_color("#2b3542")
        ax.grid(True, color="#171d25", lw=0.6)
        ax.legend(loc="upper left", ncol=3, fontsize=8, facecolor="#1b2430",
                  edgecolor="#2b3542", labelcolor="#c9d5e0")
        ax.set_title("6-axis gyro-integrated (solid) vs 9-axis fused (faint) per axis   "
                     f"-- {HEADING_NAME} is the real heading",
                     color="#8aa0b4", fontsize=10)

        # ---------- HUD ----------
        hud = (f"Acc  g     X{acc[0]:+5.2f} Y{acc[1]:+5.2f} Z{acc[2]:+5.2f}\n"
               f"Gyro deg/s X{gyro[0]:+6.1f} Y{gyro[1]:+6.1f} Z{gyro[2]:+6.1f}\n"
               f"9ax deg    R{ang[0]:+6.1f} P{ang[1]:+6.1f} Y{ang[2]:+6.1f}\n"
               f"6ax deg    R{gyro_ang[0]:+6.1f} P{gyro_ang[1]:+6.1f} Y{gyro_ang[2]:+6.1f}\n"
               f"Mag        X{mag[0]:+6d} Y{mag[1]:+6d} Z{mag[2]:+6d}")
        st = (f"! {self.reader.err}" if self.reader.err else
              (f"connected {self.reader.port} @ {self.reader.baud}"
               if self.reader.connected else f"connecting {self.reader.port} ..."))
        self.hud_txt.set_text(hud)
        self.stat_txt.set_text(st)
        self.stat_txt.set_color("#e5484d" if self.reader.err else "#8aa0b4")


def main():
    argv = rospy.myargv(argv=sys.argv)          # drop ROS remap args (__name:=, __log:=)
    port = argv[1] if len(argv) > 1 else (find_port() or IMU_DEVICE_ID_GLOB)
    baud = int(argv[2]) if len(argv) > 2 else DEFAULT_BAUD
    rospy.init_node("imu_gui", anonymous=True, disable_signals=True)
    reader = SerialImuReader(port, baud)
    reader.start()
    rospy.loginfo("imu_gui: WitMotion serial IMU %s @ %d; publishing /imu_gui (6ax %s)",
                  port, baud, HEADING_NAME.lower())
    if SHOW_GUI:
        app = App(reader)
        try:
            plt.show()
        finally:
            reader.stop()
    else:
        # headless: no window, keep reading + publishing /imu_gui until shutdown
        try:
            rospy.spin()
        finally:
            reader.stop()


if __name__ == "__main__":
    main()
