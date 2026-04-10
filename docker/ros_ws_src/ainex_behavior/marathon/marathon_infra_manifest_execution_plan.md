# Marathon 基础设施通信参照文件执行方案

## 1. 目标

为 `marathon` 项目新增一份最小基础设施通信参照文件，用来明确标记：

- 哪些 topic / service / lifecycle action 属于 node 基础设施通信
- 这些通信不属于 BT 决策业务通信
- 这些通信不纳入 `Project Semantic Facade + Generic ROS Facade` 的业务日志范围

本次实现只做静态清单输出，不做运行时自动拦截。

输出文件：

- `ainex_xyz-master/docker/ros_ws_src/ainex_behavior/marathon/log/infra_comm_manifest_lastrun.json`

---

## 2. 范围

纳入参照文件的基础设施通信包括：

1. `tree_publisher.py`
2. `bb_ros_bridge.py`
3. `bt_exec_controller.py`
4. `marathon_bt_node.py` 中不属于 BT 决策业务链的固定通信

明确排除：

- `actions.py -> semantic_facade.py -> comm_facade.py` 这条业务通信链
- `Generic ROS Facade` 业务出站日志
- `BTDebugVisitor` / `bt_debug_recent.jsonl` 的 BT 生命周期日志

---

## 3. 设计原则

1. 不修改现有业务日志主链
2. 不对 ROS runtime 做 monkey patch
3. 不做动态 ROS 图发现
4. 只输出一份稳定、可读、机器可解析的静态参照文件
5. 每次 node 启动时覆盖写一次 `infra_comm_manifest_lastrun.json`

---

## 4. 文件改动

## 4.1 新增 `infra_manifest.py`

新增文件：

- `ainex_xyz-master/docker/ros_ws_src/ainex_behavior/marathon/infra_manifest.py`

职责：

- 定义基础设施通信清单的数据结构
- 提供构建清单的方法
- 提供写入 JSON 文件的方法

建议接口：

```python
def build_infra_manifest(node_name: str) -> list:
    ...

def write_infra_manifest(path: str, items: list) -> None:
    ...
```

建议每条记录字段：

```json
{
  "component": "TreeROSPublisher",
  "kind": "topic_pub",
  "name": "~log/tree",
  "resolved_name": "/marathon_bt/log/tree",
  "msg_or_srv_type": "py_trees_msgs/BehaviourTree",
  "purpose": "publish tree snapshot for visualization",
  "bt_decision_related": false,
  "excluded_from_generic_ros_facade": true
}
```

最小必填字段：

- `component`
- `kind`
- `name`
- `msg_or_srv_type`
- `purpose`
- `bt_decision_related`
- `excluded_from_generic_ros_facade`

可选字段：

- `resolved_name`
- `notes`

---

## 4.2 修改 `marathon_bt_node.py`

文件：

- `ainex_xyz-master/docker/ros_ws_src/ainex_behavior/marathon/marathon_bt_node.py`

新增职责：

- 组装固定基础设施通信清单
- 在 node 初始化阶段写出 `infra_comm_manifest_lastrun.json`

建议导入：

```python
from infra_manifest import build_infra_manifest, write_infra_manifest
```

建议写入时机：

- 在以下对象创建完成之后立刻写文件：
  - `self._tree_publisher`
  - `self._bb_bridge`
  - `self._exec_ctrl`
  - `/imu` 和 `/object/pixel_coords` subscribers
  - `self.buzzer_pub`

建议路径：

```python
os.path.join(_LOG_DIR, 'infra_comm_manifest_lastrun.json')
```

建议逻辑：

```python
manifest = build_infra_manifest(self.name)
write_infra_manifest(
    os.path.join(_LOG_DIR, 'infra_comm_manifest_lastrun.json'),
    manifest,
)
```

---

## 5. 清单内容建议

建议至少列出以下接口。

## 5.1 `tree_publisher.py`

- `~log/tree` — `topic_pub` — `py_trees_msgs/BehaviourTree`
- `~ascii/snapshot` — `topic_pub` — `std_msgs/String`
- `~tip` — `topic_pub` — `py_trees_msgs/Behaviour`

purpose:

- 树可视化与调试输出

## 5.2 `bb_ros_bridge.py`

- `/bt/marathon/bb/robot_state` — `topic_pub` — `std_msgs/String`
- `/bt/marathon/bb/line_data` — `topic_pub` — `std_msgs/String`
- `/bt/marathon/bb/last_line_x` — `topic_pub` — `std_msgs/String`
- `/bt/marathon/bb/camera_lost_count` — `topic_pub` — `std_msgs/String`
- `/bt/marathon/bb/tick_id` — `topic_pub` — `std_msgs/String`

purpose:

- 对外镜像 blackboard 只读视图

## 5.3 `bt_exec_controller.py`

- `~bt/mode` — `topic_pub` — `std_msgs/String`
- `~bt/run` — `service_server` — `std_srvs/Empty`
- `~bt/pause` — `service_server` — `std_srvs/Empty`
- `~bt/step` — `service_server` — `std_srvs/Empty`

purpose:

- BT tick 执行控制

## 5.4 `marathon_bt_node.py`

建议纳入以下固定基础设施通信：

- `/imu` — `topic_sub` — `sensor_msgs/Imu`
- `/object/pixel_coords` — `topic_sub` — `ainex_interfaces/ObjectsInfo`
- `/ros_robot_controller/set_buzzer` — `topic_pub` — `ros_robot_controller/BuzzerState`
- `detect_pub` 对应颜色检测配置发布接口 — `topic_pub` — `ainex_interfaces/ColorDetect[]`
- `walk_ready` — `lifecycle_action`
- `stand` — `lifecycle_action`

说明：

- `/ros_robot_controller/set_buzzer` 同时会被业务恢复动作使用，但 publisher 本体属于 node 级基础设施对象；业务日志仍由 `comm_facade.py` 负责
- `walk_ready` 和 `stand` 是启动/关机生命周期动作，不属于 BT 决策链

---

## 6. 输出格式

文件格式：

- JSON
- 顶层为数组
- 每项一条基础设施通信记录

建议示例：

```json
[
  {
    "component": "BTExecController",
    "kind": "service_server",
    "name": "~bt/pause",
    "msg_or_srv_type": "std_srvs/Empty",
    "purpose": "pause BT ticking",
    "bt_decision_related": false,
    "excluded_from_generic_ros_facade": true
  }
]
```

---

## 7. 实施步骤

1. 新增 `infra_manifest.py`
2. 在 `infra_manifest.py` 中实现固定清单构建函数
3. 在 `infra_manifest.py` 中实现 JSON 覆盖写函数
4. 修改 `marathon_bt_node.py`，在初始化阶段调用 manifest 写出逻辑
5. 启动 node 后检查 `log/infra_comm_manifest_lastrun.json` 是否生成
6. 校验文件内容是否覆盖约定的基础设施接口

---

## 8. 验收标准

- [ ] `infra_manifest.py` 已新增
- [ ] `marathon_bt_node.py` 启动时会生成 `log/infra_comm_manifest_lastrun.json`
- [ ] 每次启动会覆盖旧文件
- [ ] 文件中包含 `tree_publisher.py` 的 3 个 topic
- [ ] 文件中包含 `bb_ros_bridge.py` 的 BB 镜像 topic
- [ ] 文件中包含 `bt_exec_controller.py` 的 mode topic 和 3 个 service
- [ ] 文件中包含 `marathon_bt_node.py` 的固定基础设施输入/输出
- [ ] 每条记录包含 `bt_decision_related=false`
- [ ] 每条记录包含 `excluded_from_generic_ros_facade=true`

---

## 9. 非目标

本次不做：

- 不做动态 `rosnode info` / `rostopic list` 扫描
- 不做基础设施通信的实时日志流
- 不做 node 级全量通信拦截
- 不把基础设施通信并入 `Generic ROS Facade`
- 不修改现有 `bt_ros_comm_debug_recent.jsonl` / `bt_debug_recent.jsonl` 语义

---

## 10. 后续可选扩展

如果后续需要，可以在此基础上继续加：

1. `infra_comm_manifest_recent.json`
2. `infra_comm_manifest.md` 人类可读版本
3. 启动时校验 manifest 与实际代码接口是否一致
4. 为 ROSA/LLM 增加“基础设施通信白名单”过滤逻辑
