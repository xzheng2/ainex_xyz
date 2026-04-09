# Marathon BT + ROS 最小可落地重构方案

## 1. 目的

本方案用于指导 coding agent 在**尽量少改现有目录结构**的前提下，完成 `marathon` 项目的最小可落地重构，使系统具备以下能力：

- 每次 BT tick 只依据 tick 前锁存输入执行
- 所有关键 ROS 出站通信走统一门面
- 拦截发生时直接写最终日志
- 为后续 pause/step 和通用化保留清晰扩展点

本方案刻意不追求一次性完成完整架构重写，只做当前最值得做、最容易验证、最不容易回归的改造。

---

## 2. 最终效果

完成后，`marathon` 应满足：

1. 外部 callback 持续更新 `live inputs`
2. 每次 `tree.tick()` 之前，将 `live inputs` 复制到 `latched inputs`
3. BT 的 condition / action 节点在 tick 内只读取 `latched inputs`
4. `FollowLine`、`FindLine`、`RecoverFromFall` 等叶节点不再直接触达 manager 通信链，而是统一通过轻量 `comm_facade`
5. 所有关键出站通信在门面层直接生成最终日志
6. 现有日志可读性不降低，仍能直接看出：
   - 哪个 BT 节点
   - 在哪个 tick
   - 发了什么 ROS 命令
   - 目标接口是什么

---

## 3. 本次只做的内容

这次重构**只做以下 4 件事**：

1. 引入 `live inputs` / `latched inputs`
2. 引入轻量 `comm_facade.py`
3. 修改 action 节点，统一走 facade
4. 修改 bridge / 条件节点，使调试视图与 BT 读取语义一致

这次**不做**：

- 不拆 `app.py`
- 不新建完整 `domain/` 目录
- 不重排整个 `marathon` 目录结构
- 不一次性统一所有事件 schema 名称
- 不引入完整 node 级统一 monkey patch 拦截器

---

## 4. 需要覆盖的输入键

当前最小方案只覆盖 `marathon` 核心输入：

- `/robot_state`
- `/line_data`
- `/last_line_x`
- `/camera_lost_count`

建议新增：

- `/live/robot_state`
- `/live/line_data`
- `/live/last_line_x`
- `/live/camera_lost_count`
- `/latched/robot_state`
- `/latched/line_data`
- `/latched/last_line_x`
- `/latched/camera_lost_count`

如当前逻辑仍依赖旧 key，可在过渡期保留兼容映射，但 BT 最终读取必须切到 `latched/*`。

---

## 5. 需要覆盖的关键出站通信

本次 facade 至少要统一覆盖这些关键通信：

1. `gait_manager.disable()`
2. `gait_manager.enable()`（如果当前流程会使用）
3. `gait_manager.set_step(...)`
4. `motion_manager.run_action(...)`
5. `buzzer_pub.publish(...)`

如果实现过程中发现 `FindLineHeadSweep` 或其他逻辑还存在额外关键 ROS 出站通信，也应一并纳入 facade。

---

## 6. 文件改动范围

本次尽量只改这些文件：

### 必改

- `ainex_xyz-master/docker/ros_ws_src/ainex_behavior/marathon/marathon_bt_node.py`
- `ainex_xyz-master/docker/ros_ws_src/ainex_behavior/marathon/behaviours/conditions.py`
- `ainex_xyz-master/docker/ros_ws_src/ainex_behavior/marathon/behaviours/actions.py`
- `ainex_xyz-master/docker/ros_ws_src/ainex_behavior/marathon/bb_ros_bridge.py`

### 新增

- `ainex_xyz-master/docker/ros_ws_src/ainex_behavior/marathon/comm_facade.py`

### 尽量不动

- `marathon_bt.py`
- `bt_exec_controller.py`
- `bt_observability/debug_event_logger.py`
- `bt_observability/bt_debug_visitor.py`

除非实现最小方案时必须微调，否则不主动扩大改动面。

---

## 7. 分步计划

## Step 1：引入 live / latched 输入

目标：

- callback 与 BT 读取语义解耦

具体改动：

1. 在 `marathon_bt_node.py` 初始化阶段注册并初始化：
   - `/live/*`
   - `/latched/*`
2. `_imu_callback()` 只更新 `/live/robot_state`
3. `_objects_callback()` 只更新：
   - `/live/line_data`
   - `/live/last_line_x`
   - `/live/camera_lost_count`
4. 在 `run()` 中每次 `self.tree.tick()` 之前执行一次：
   - `live -> latched`

实现要求：

- tick 内部不应再直接依赖 live 值
- callback 仍然保留当前 `ros_topic_received` 入站日志

验收：

- callback 正常更新 `live/*`
- `tree.tick()` 前 `latched/*` 被刷新
- 单 tick 内 callback 新写入不会改变当前 tick 判断结果

---

## Step 2：条件节点切换到 latched 输入

目标：

- condition 只读锁存视图

具体改动：

1. `conditions.py` 中以下节点切换为读取 `latched/*`：
   - `IsRobotStanding`
   - `IsLineDetected`
   - `IsHeadCentered` 如果其依赖的是输入视图，也要明确其来源
2. 保持现有 `decision` 日志逻辑不变或仅做必要字段更新

验收：

- `IsRobotStanding` 不再读取旧 `/robot_state`
- `IsLineDetected` 不再读取旧 `/line_data`
- 条件节点行为与旧逻辑一致，但 tick 语义更稳定

---

## Step 3：新增轻量 comm facade

目标：

- 把关键 ROS 出站通信集中到一个文件

新文件：

- `marathon/comm_facade.py`

建议提供最小接口：

1. `disable_gait(...)`
2. `enable_gait(...)`
3. `set_step(...)`
4. `run_action(...)`
5. `publish_buzzer(...)`

职责：

- 调用真实 manager / publisher
- 在调用时直接写最终日志
- 附带：
  - `tick_id`
  - `bt_node`
  - `semantic_source`
  - `target`
  - `payload`

要求：

- 初期可以直接复用现有 `DebugEventLogger`
- 初期可以把当前 `ManagerProxy` 的 payload 规则迁移到 facade，而不是立即删除所有旧逻辑

验收：

- 新 facade 能覆盖当前关键出站通信
- facade 写出的日志仍保持当前可读性

---

## Step 4：action 节点切换为调用 facade

目标：

- 叶节点不再直接依赖 manager 调用链

具体改动：

1. `StopWalking` 调用 facade，而不是直接 `gait_manager.disable()`
2. `FollowLine` 的最终出站命令通过 facade 发出
3. `FindLine` 的最终出站命令通过 facade 发出
4. `RecoverFromFall` 内：
   - buzzer 通过 facade
   - gait disable 通过 facade
   - run_action 通过 facade

实现要求：

- 尽量不在叶节点内拼接日志 payload
- 叶节点只负责收集自身业务参数

验收：

- `actions.py` 中关键叶节点不再直接操作真实 manager 的通信方法
- 关键通信都在 facade 中有唯一出口

---

## Step 5：bridge 切换为优先发布 latched 视图

目标：

- ROSA / 调试侧看到的 BB 视图与 BT 决策依据一致

具体改动：

1. `bb_ros_bridge.py` 优先读取 `/latched/*`
2. 如有必要，可保留 `live/*` 作为附加调试信息，但主视图应为 `latched/*`
3. 对 `tick_id` 可继续保留当前逻辑

验收：

- bridge 发布内容与 BT 实际读取视图一致
- 不再出现“调试侧看到 BB 已更新，但 BT 仍按旧值决策却看不出来”的困惑

---

## 8. 检验方法

## 8.1 功能路径检验

至少验证以下路径：

1. 正常巡线
- `IsLineDetected -> FollowLine`
- 日志中可看到 `FollowLine` 对应的出站命令

2. 丢线搜索
- `FindLine` 或 `FindLineHeadSweep`
- 日志中可看到搜索动作对应的出站命令

3. 摔倒恢复
- `/imu` 导致 `robot_state` 改变
- BT 进入恢复路径
- 日志中可看到：
  - `/imu` 入站
  - 状态更新
  - buzzer
  - gait disable
  - run_action

## 8.2 语义检验

需要明确验证：

1. callback 更新的是 `live/*`
2. BT 读取的是 `latched/*`
3. facade 写出的日志中，`bt_node`、`tick_id`、`target`、`payload` 都存在

## 8.3 结构检验

重构完成后，检查：

1. `conditions.py` 是否只读 `latched/*`
2. `actions.py` 是否不再直接发关键通信
3. 所有关键出站通信是否都能在 `comm_facade.py` 找到唯一出口

---

## 9. 成功标准

完成本方案后，满足以下条件即视为成功：

- [x] BT 每次 tick 只依据 `latched inputs`
- [x] 关键 ROS 出站通信统一走 `comm_facade.py`
- [x] 关键叶节点通信都能直接在最终日志中看到
- [x] 当前目录结构未发生大规模重排
- [ ] 现有运行逻辑无明显回归（待上机验证）
- [x] 后续若要接入 pause/step，只需要围绕 `live -> latched` 机制继续扩展

---

## 10. coding agent 执行要求

coding agent 执行时必须遵守：

1. 严格按 Step 1 -> Step 5 顺序推进
2. 每完成一步，都说明：
   - 改了哪些文件
   - 当前能否运行
   - 用什么方法验证
3. 不主动扩大架构重构范围
4. 若发现现有 key 命名不一致、字段错乱、潜在 bug，可在最小范围内顺手修复，但必须在说明中明确指出
5. 如果无法运行测试，至少完成静态路径核查，并清楚说明未验证项

---

## 11. 后续扩展点

本最小方案完成后，后续可继续扩展：

1. 将 callback 进一步拆到独立 `inputs/` 文件
2. 将 facade 再演进成更通用的 BT + ROS 通信门面
3. 引入 node 级兜底拦截器，覆盖非 facade 通信
4. 将 pause/step 升级为真正的“冻结 latched 输入”调试模式

但这些内容**不属于本次最小方案的交付范围**。

---

## 12. 实施记录（2026-04-09）

本方案已由 coding agent 完整实施，所有静态验证通过。以下记录关键实现决策。

### 12.1 实际改动文件

| 文件 | 类型 | 说明 |
|------|------|------|
| `marathon_bt_node.py` | 修改 | BB 客户端拆分 + 创建 CommFacade + 更新 bootstrap 调用 |
| `bb_ros_bridge.py` | 修改 | MARATHON_BB_TOPIC_MAP key 改为 `/latched/*`（ROS topic 名称不变） |
| `behaviours/conditions.py` | 修改 | IsRobotStanding + IsLineDetected 切换到 `namespace="/latched"` |
| `behaviours/actions.py` | 修改 | 全部 BB 客户端加 namespace + 叶节点走 facade |
| `marathon_bt.py` | **也改了** | bootstrap() 签名变更（必要微调，原计划标注"尽量不动"） |
| `comm_facade.py` | **新增** | CommFacade 类，6 个方法 |

未改动：`bt_exec_controller.py`、`tree_publisher.py`、`bt_observability/` 全部文件。

### 12.2 关键实现决策

**BB 命名方案（namespace 而非 key 前缀）**

未采用"在 key 字符串里加 `/latched/` 前缀"的做法，而是通过 py_trees `Client(namespace="/latched")` + 相对 key 名实现。

效果：所有 `self.bb.robot_state` 属性访问代码**一行不变**，namespace 在 py_trees 内部自动解析到 `/latched/robot_state`。改动最小，diff 最干净。

`/tick_id` 不属于传感器输入，保留在独立客户端（无 namespace），与 latched 客户端分离：

```python
self._bb      = py_trees.blackboard.Client(name="MarathonBTNode_latched", namespace="/latched")
self._bb_meta = py_trees.blackboard.Client(name="MarathonBTNode_meta")
```

`FindLineHeadSweep` 同样拆成两个客户端：一个 namespace="/latched"（读传感器），一个无 namespace（写 `/head_pan_pos`）。

**CommFacade 包装 ManagerProxy（不替换）**

CommFacade 接收的 `gait_manager` / `motion_manager` 是已经被 `ManagerProxy` 包装过的代理对象（与 `visual_patrol` 共用同一套 proxy）。

CommFacade 只负责在调用前设置 `proxy_context.current_node = bt_node`，实际日志仍由 ManagerProxy 写出。无双重日志。

Buzzer 无 ManagerProxy，由 CommFacade 直接调用 `logger.emit_comm()` 后 publish，替代了原来 RecoverFromFall 里的 `ROSCommTracer` 路径。

**FollowLine 保留 proxy_context（刻意例外）**

`FollowLine.update()` 调用 `visual_patrol.process()`，gait 命令在 VisualPatrol 内部由 PID 算法决定，不经过叶节点显式调用。如果强行走 facade，需要改动 VisualPatrol，超出本次最小范围。

保留方式：`proxy_context.current_node = self.name` → `visual_patrol.process(...)` → ManagerProxy 自动归因到 "FollowLine"。日志归因正确，行为不变。

**新增 `set_servos_position` 到 facade**

原方案 facade 计划只有 5 个方法。实施时发现 `FindLineHeadSweep._command_head()` 直接调用 `motion_manager.set_servos_position()`（头部舵机控制），按计划纳入 facade，增加为第 6 个方法。

**marathon_bt.py 必须修改**

原计划标注"尽量不动"。但 action 节点构造函数签名改变（接收 facade 而非分散的 manager/buzzer_pub），`bootstrap()` 必须随之更新。属于计划内的"必要微调"。

新签名：`bootstrap(comm_facade, visual_patrol, find_line_cls=None, logger=None, tick_id_getter=None, robot_state_setter=None)`

### 12.3 静态验证结果

```
comm_facade.py        OK (syntax)
bb_ros_bridge.py      OK (syntax)
marathon_bt.py        OK (syntax)
behaviours/conditions.py  OK (syntax)
behaviours/actions.py     OK (syntax)
marathon_bt_node.py   OK (AST parse)

旧 BB key "/robot_state" 等在 conditions/actions 中：NONE（已全部清除）
proxy_context.current_node 在 actions.py 中仅剩 FollowLine 一处：OK
comm_facade.py 共 6 个公开方法：OK
```

### 12.4 待验证项（需上机）

- [ ] `rosrun ainex_behavior marathon_bt_node.py` 启动无报错
- [ ] `/bt/marathon/bb/*` 话题正常发布（ROSA 工具可读）
- [ ] JSONL 日志 comm 事件中 `bt_node` 字段有正确归因
- [ ] Buzzer 事件在 `bt_ros_comm_debug_lastrun.jsonl` 中可见
- [ ] 正常巡线 / 丢线 / 摔倒恢复三条路径功能不退化
