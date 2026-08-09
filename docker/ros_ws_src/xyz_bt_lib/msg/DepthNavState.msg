Header  header
float32 nav_u9        # path block-centre pixel-x at grid row 9 (NaN if path misses it); valid only when has_path=True
float32 nav_u10       # path block-centre pixel-x at grid row 10 (NaN if path misses it); valid only when has_path=True
float32 cx            # camera principal point x (px) — for the consumer's bearing calc atan((u-cx)/fx)
float32 fx            # camera focal length x (px)    — for the consumer's bearing calc atan((u-cx)/fx)
int16   start_bx      # A* start block column (from the YOLO start-object centre, else default 8)
int16   start_by      # A* start block row    (from the YOLO start-object centre, else default 15)
bool    has_path      # True when A* found a path from start to a goal row
bool    obstacle_near # True when an obstacle block lies in the near-front window
bool    obstacle_detected # True when any obstacle (depth+YOLO) is in the fused mask, anywhere (not distance/near-window gated)
float32 imu_heading   # in-process /imu-integrated heading (deg), GUI cal-6ax logic; 0 at start
float32 imu_heading_min  # per-run max RIGHT (-) heading pose reached this run (deg); 0 at start
float32 imu_heading_max  # per-run max LEFT  (+) heading pose reached this run (deg); 0 at start
