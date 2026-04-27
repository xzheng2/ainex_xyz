# XYZ RuntimeFacade 文件级改造清单

本文档重写并收敛前面的改造计划，使其与当前确认的 `XyzBTFacade` public contract 完全一致。

## 1. 当前确认的 public contract

`XyzBTFacade` 名称保持不变，但语义改为 BT-facing Runtime 接口。

### Primitive methods

- `disable_gait()`
- `enable_gait()`
- `stop_gait()`
- `set_step(...)`
- `run_action(...)`
- `set_servos_position(...)`
- `publish_buzzer(...)`

### Convenience wrappers

- `go_step(...)`
- `turn_step(...)`
- `move_head(...)`

规则：

- `set_step(...)` 是 gait 的核心 primitive，参数形式与底层执行形态一致。
- `go_step(...)` / `turn_step(...)` 是基于 profile base cfg + override 的 wrapper。
- `move_head(...)` 是基于 `set_servos_position(...)` 的 wrapper。
- `follow_line(...)`、`recover_from_fall(...)`、`gait_step(...)` 不再属于 public contract。

---

## 2. 目标层次

目标结构：

```text
L1
  -> 只读 BB，纯判别

L2
  -> 只依赖 XyzBTFacade
  -> 决定什么时候做、当前 tick 要求什么动作

RuntimeFacade
  -> 实现 XyzBTFacade
  -> profile/base cfg + override 合成
  -> convenience wrapper
  -> 组合动作逻辑

_RuntimeIO
  -> 唯一 raw ROS / manager 出口
  -> 唯一 ros_out 记录出口
  -> 完整保留当前 CommFacade._emit() 强制边界
```

约束：

- 禁止 `bt_node` / L2 / project code 直接绕过 `RuntimeFacade` 调 `gait_manager` / `motion_manager`
- 禁止除 `_RuntimeIO` 外的执行层代码直接发 outbound ROS

---

## 3. `xyz_bt_edu` 公共基础层改造

### 3.1 `docker/ros_ws_src/xyz_bt_edu/src/xyz_bt_edu/base_facade.py`

必须修改：

1. 保留类名 `XyzBTFacade`
2. 改类级 docstring
   - `SemanticFacade` 改为 `RuntimeFacade`
   - `CommFacade` 改为 `_RuntimeIO`
   - 明确 `_RuntimeIO` 是唯一 raw ROS / manager 出口
3. 删除 abstract methods
   - `follow_line(...)`
   - `recover_from_fall(...)`
   - `gait_step(...)`
4. 保留或新增 abstract methods
   - `disable_gait(...)`
   - `enable_gait(...)`
   - `stop_gait(...)`
   - `set_step(...)`
   - `run_action(...)`
   - `set_servos_position(...)`
   - `publish_buzzer(...)`
   - `go_step(...)`
   - `turn_step(...)`
   - `move_head(...)`
5. 写清楚 `stop_gait()` 与 `disable_gait()` 的语义区别
   - `stop_gait()`：停止当前 gait motion，但 controller 仍可继续接受命令
   - `disable_gait()`：关闭 gait controller，需要 `enable_gait()` 才能重新工作

建议的 docstring 方向：

```text
Primitive runtime methods:
  disable_gait / enable_gait / stop_gait / set_step / run_action /
  set_servos_position / publish_buzzer

Convenience wrappers:
  go_step / turn_step / move_head
```

### 3.2 `docker/ros_ws_src/xyz_bt_edu/src/xyz_bt_edu/base_node.py`

必须修改：

1. 保留 `call_facade()` 方法名，避免大面积破坏现有 L2
2. 改 `XyzL2ActionNode` docstring
   - 不再写 “through SemanticFacade”
   - 改成 “through injected XyzBTFacade runtime interface”
3. 可以增加兼容别名：
   - `call_runtime = call_facade`

不做的事：

- 暂不改类名
- 暂不改方法名

### 3.3 `docker/ros_ws_src/xyz_bt_edu/src/xyz_bt_edu/base_adapter.py`

只改文档，不改行为：

1. 明确 input adapters 是 inbound ROS 入口
2. 明确 outbound 执行副作用不走 adapter，而统一走 `_RuntimeIO`

---

## 4. 现有标准 L2 节点改造

### 4.1 `.../behaviours/L2_locomotion/L2_Gait_Stop.py`

必须修改：

1. `FACADE_CALLS`
   - 从 `['stop_walking']`
   - 改为 `['stop_gait']`
2. `update()`
   - 从 `self.call_facade('stop_walking')`
   - 改为 `self.call_facade('stop_gait')`
3. docstring 同步改成 `stop_gait()`

### 4.2 `.../behaviours/L2_locomotion/L2_Head_FindLineSweep.py`

必须修改：

1. `FACADE_CALLS`
   - 从 `['stop_walking', 'move_head', 'turn_step']`
   - 改为 `['stop_gait', 'move_head', 'turn_step']`
2. 所有 `self.call_facade('stop_walking')`
   - 改为 `self.call_facade('stop_gait')`
3. docstring 中所有 `stop_walking()` 说明改掉

### 4.3 `.../behaviours/L2_locomotion/L2_Balance_RecoverFromFall.py`

这是现有 L2 中改动最大的一项。

必须修改：

1. `FACADE_CALLS`
   - 从 `['recover_from_fall']`
   - 改为 `['publish_buzzer', 'disable_gait', 'run_action']`
2. 删除对 public `recover_from_fall(...)` 的依赖
3. 把原本 semantic 层里的恢复流程组合移回 L2：
   - `publish_buzzer(...)`
   - `time.sleep(pre_action_delay_s)`
   - `disable_gait(...)`
   - `run_action(recovery_action)`
   - `time.sleep(post_action_delay_s)`
   - 写 `BB.ROBOT_STATE = 'stand'`
4. 把恢复动作名和 buzzer/等待时间放入 `CONFIG_DEFAULTS`
   - `lie_action`
   - `recline_action`
   - `buzzer_freq`
   - `buzzer_on_time`
   - `buzzer_off_time`
   - `buzzer_repeat`
   - `pre_action_delay_s`
   - `post_action_delay_s`
5. 保留 `robot_state_setter`，继续同步 live adapter store

说明：

- 这个节点不再依赖高层语义 contract
- 它直接使用 public primitive 和 project-independent wrapper 组合出恢复流程

### 4.4 `.../behaviours/L2_locomotion/L2_Gait_FollowLine.py`

必须修改：

1. `FACADE_CALLS` 保持：
   - `['move_head', 'go_step', 'turn_step']`
2. docstring 删除任何 “SemanticFacade” 表述
3. 若文件里还提 `follow_line` facade 兼容方法，全部删掉相关表述

### 4.5 `.../behaviours/L2_locomotion/L2_Gait_FindLine.py`

必须修改：

1. `FACADE_CALLS` 保持 `['turn_step']`
2. docstring 改为 `RuntimeFacade wrapper`

### 4.6 `.../behaviours/L2_locomotion/L2_Head_MoveTo.py`

必须修改：

1. `FACADE_CALLS` 保持 `['move_head']`
2. docstring 明确 `move_head()` 是 convenience wrapper over `set_servos_position(...)`

---

## 5. 项目模板层（废弃旧 skill，重建新 skill）

这一层不再按“在现有 `xyz-bt-project` 活动模板上改造”的思路执行。  
`ainex_xyz-master/.claude/skills/xyz-bt-project` 处于**随时可删除**状态，不再作为后续模板演化基准。

新的活动模板体系应单独新建：

```text
ainex_xyz-master/.claude/skills/xyz-bt-facade-project/
```

后续项目模板只以这个新 skill 为唯一正确生成基准。

### 5.1 旧 skill `xyz-bt-project` 的处理规则

处理规则：

- 从活动模板语义上视为**废弃**
- 不再继续重命名、修补、兼容式演化
- 不再作为生成新项目或修改模板的参考基线
- 可以保留作历史归档
- 也可以直接删除；删除不会影响新模板体系设计

旧 skill 内这些文件不再继续维护为活动模板：

- `assets/templates/semantic_facade.py.tpl`
- `assets/templates/comm_facade.py.tpl`
- `assets/templates/bt_node.py.tpl`
- `assets/templates/project_bt.py.tpl`
- `assets/templates/actions.py.tpl`
- `assets/templates/infra_manifest.py.tpl`

### 5.2 新 skill `xyz-bt-facade-project` 的目录结构

新的 skill 目录建议为：

```text
ainex_xyz-master/.claude/skills/xyz-bt-facade-project/
  SKILL.md
  assets/
    templates/
      runtime_facade.py.tpl
      _runtime_io.py.tpl
      bt_node.py.tpl
      project_bt.py.tpl
      actions.py.tpl
      infra_manifest.py.tpl
```

这个 skill 的职责是：

- 生成基于 `RuntimeFacade / _RuntimeIO` 的新项目骨架
- 不再生成 `SemanticFacade / CommFacade` 范式文件
- 把 Runtime contract 作为唯一活动模板范式

### 5.3 `runtime_facade.py.tpl` 重建要求

新文件位置：

- `ainex_xyz-master/.claude/skills/xyz-bt-facade-project/assets/templates/runtime_facade.py.tpl`

类名：

- `{{PROJECT_CLASS}}RuntimeFacade`

职责：

- 实现 `XyzBTFacade`
- 提供 BT-facing public runtime contract
- 负责 profile/base cfg + override 合成
- 提供 convenience wrapper
- 允许组合多个 `_RuntimeIO` primitive 调用
- 不直接调用 raw ROS / publisher / service / manager

必须实现的 public methods：

#### Primitive passthrough

- `disable_gait(...)`
- `enable_gait(...)`
- `stop_gait(...)`
- `set_step(...)`
- `run_action(...)`
- `set_servos_position(...)`
- `publish_buzzer(...)`

#### Convenience wrapper

- `go_step(...)`
- `turn_step(...)`
- `move_head(...)`

规则：

- `set_step(...)` 接收完整最终参数，不做 profile 推断
- `go_step(...)` 基于 `go_cfg` 合成完整 `dsp / gait_param / arm_swap / step_num`
- `turn_step(...)` 基于 `turn_cfg` 合成完整参数
- `move_head(...)` 用 `HEAD_PAN_SERVO` 与 `HEAD_MOVE_DELAY_MS` 映射到 `set_servos_position(...)`
- 不再实现：
  - `follow_line(...)`
  - `recover_from_fall(...)`
  - `gait_step(...)`

### 5.4 `_runtime_io.py.tpl` 重建要求

新文件位置：

- `ainex_xyz-master/.claude/skills/xyz-bt-facade-project/assets/templates/_runtime_io.py.tpl`

类名：

- `_RuntimeIO`

职责：

- 唯一 raw ROS / manager 出口
- 唯一 `ros_out` 记录出口
- 完整承接当前 `CommFacade._emit() -> raw execution` 的结构性强制边界

必须实现的方法：

- `disable_gait(...)`
- `enable_gait(...)`
- `stop_gait(...)`
- `set_step(...)`
- `run_action(...)`
- `set_servos_position(...)`
- `publish_buzzer(...)`

规则：

- `_RuntimeIO` 才允许持有：
  - `gait_manager`
  - `motion_manager`
  - ROS publisher
  - ROS service client
- `_RuntimeIO` 只接收完整参数
- `_RuntimeIO` 不负责：
  - profile/base cfg 选择
  - override 合成
  - 项目业务语义
- `stop_gait(...)` 需要作为新 primitive 明确加入

### 5.5 `bt_node.py.tpl` 重建要求

新文件位置：

- `ainex_xyz-master/.claude/skills/xyz-bt-facade-project/assets/templates/bt_node.py.tpl`

必须满足：

1. Input adapters 继续在这里创建
2. `_RuntimeIO` 在这里创建
3. `RuntimeFacade` 在这里创建
4. `bootstrap()` 只接收 `RuntimeFacade`
5. 不允许直接调用：
   - `self.gait_manager.xxx(...)`
   - `self.motion_manager.xxx(...)`
   - raw publisher/service

新的执行链必须是：

```text
bt_node
  -> _RuntimeIO(...)
  -> RuntimeFacade(runtime_io=...)
  -> bootstrap(runtime_facade)
```

启动/收尾中的 direct manager 行为也必须改走 `RuntimeFacade`，例如：

- `walk_ready`
- `stand`

### 5.6 `project_bt.py.tpl` 重建要求

新文件位置：

- `ainex_xyz-master/.claude/skills/xyz-bt-facade-project/assets/templates/project_bt.py.tpl`

要求：

1. 参数名统一为 `runtime_facade`
2. 注释中明确：
   - “All leaf nodes receive only the Project RuntimeFacade (`XyzBTFacade`).”
3. 所有 L2 继续通过：
   - `facade=runtime_facade`
4. 模板中不再出现 `semantic_facade`

### 5.7 `actions.py.tpl` 重建要求

新文件位置：

- `ainex_xyz-master/.claude/skills/xyz-bt-facade-project/assets/templates/actions.py.tpl`

规则：

- 只展示基于 `XyzBTFacade` 的 project-level action 模式
- 不示范任何 direct manager 调用
- 不示范任何 raw ROS publish/service
- 所有 project action 如需执行副作用，也必须通过 `RuntimeFacade`

### 5.8 `infra_manifest.py.tpl` 重建要求

新文件位置：

- `ainex_xyz-master/.claude/skills/xyz-bt-facade-project/assets/templates/infra_manifest.py.tpl`

规则：

- `runtime_facade.py` 是 public BT-facing runtime layer
- `_runtime_io.py` 是 sole raw ROS egress layer
- 不再出现：
  - `semantic_facade.py`
  - `comm_facade.py`

---

## 6. 模板输出路径调整

新 skill 生成的项目输出结构推荐为：

```text
{{PROJECT}}/
  runtime/
    runtime_facade.py
    _runtime_io.py
  tree/
    {{PROJECT}}_bt.py
  app/
    {{PROJECT}}_bt_node.py
  infra/
    ...
```

不再生成：

```text
{{PROJECT}}/semantics/semantic_facade.py
{{PROJECT}}/comm/comm_facade.py
```

所有 import 路径从一开始就按新结构书写，不保留旧范式兼容层。

---

## 7. skill 文档改造

### 7.1 `ainex_xyz-master/.claude/skills/xyz-bt-edu-extend/SKILL.md`

必须修改：

1. 全文把 `SemanticFacade` 改成 `RuntimeFacade`
2. 全文把 `CommFacade` 改成 `_RuntimeIO`
3. 更新 facade contract 列表，使其与当前 public contract 一致：
   - `disable_gait`
   - `enable_gait`
   - `stop_gait`
   - `set_step`
   - `run_action`
   - `set_servos_position`
   - `publish_buzzer`
   - `go_step`
   - `turn_step`
   - `move_head`
4. 删除 contract 列表中的：
   - `follow_line`
   - `recover_from_fall`
   - `gait_step`
5. L2 规则增加一句：
   - `_RuntimeIO` 是唯一 raw ROS / manager 层

### 7.2 `ainex_xyz-master/.claude/skills/xyz-bt-facade-project/SKILL.md`

这是新建 skill，不是在旧 `xyz-bt-project/SKILL.md` 上继续演化。

必须满足：

1. 明确写成：
   - `xyz-bt-facade-project` 是唯一活动项目模板 skill
   - `xyz-bt-project` 已废弃，可归档，可删除
2. 架构图改成：
   - `L2 -> RuntimeFacade -> _RuntimeIO -> runtime/managers/ROS`
3. 输出文件映射改成新的 `runtime/` 结构
4. 明确声明：
   - 旧的 `SemanticFacade / CommFacade` 模板不再作为生成或修改基准

---

## 8. 验证项

### 8.1 Public contract 验证

检查 `base_facade.py` 中只存在以下 public methods：

- `disable_gait`
- `enable_gait`
- `stop_gait`
- `set_step`
- `run_action`
- `set_servos_position`
- `publish_buzzer`
- `go_step`
- `turn_step`
- `move_head`

不应再出现：

- `follow_line`
- `recover_from_fall`
- `gait_step`

### 8.2 执行边界验证

```bash
rg -n "gait_manager|motion_manager|\\.publish\\(|ServiceProxy|rospy\\.Publisher" \
  /Users/xingyuzheng/Desktop/HuroCup机器人创业项目/学习笔记/AINEX+CC/ainex_xyz-master/docker/ros_ws_src/xyz_bt_edu/src/xyz_bt_edu \
  /Users/xingyuzheng/Desktop/HuroCup机器人创业项目/学习笔记/AINEX+CC/ainex_xyz-master/.claude/skills/xyz-bt-project/assets/templates
```

期望：

- 只有 `_RuntimeIO` 模板和 project wiring 构造处出现执行对象
- L2 / tree / RuntimeFacade 本身不直接碰 manager / raw ROS

### 8.3 Observability 验证

```bash
rg -n "emit_comm" \
  /Users/xingyuzheng/Desktop/HuroCup机器人创业项目/学习笔记/AINEX+CC/ainex_xyz-master/docker/ros_ws_src/xyz_bt_edu/src/xyz_bt_edu \
  /Users/xingyuzheng/Desktop/HuroCup机器人创业项目/学习笔记/AINEX+CC/ainex_xyz-master/.claude/skills/xyz-bt-project/assets/templates
```

期望：

- `_RuntimeIO` 中存在 raw outbound `ros_out`
- `XyzInputAdapter` 中存在 inbound `ros_in` / `input_state`
- L1 / L2 / RuntimeFacade 不直接 `emit_comm`

---

## 9. 推荐执行顺序

第一批必须改：

1. `base_facade.py`
2. `runtime_facade.py.tpl`
3. `_runtime_io.py.tpl`
4. `bt_node.py.tpl`
5. `project_bt.py.tpl`
6. `L2_Gait_Stop.py`
7. `L2_Head_FindLineSweep.py`
8. `L2_Balance_RecoverFromFall.py`

第二批同步改：

1. `actions.py.tpl`
2. `infra_manifest.py.tpl`
3. `xyz-bt-edu-extend/SKILL.md`
4. `xyz-bt-project/SKILL.md`
5. 相关 references / docs

---

## 10. 当前方案的收敛结果

这版方案与当前接口集合完全一致：

- `XyzBTFacade` 名称保留
- `RuntimeFacade` 是 public BT-facing 实现层
- `_RuntimeIO` 是唯一 raw ROS / manager 层
- gait 的核心 primitive 是 `set_step(...)`
- `go_step(...)` / `turn_step(...)` / `move_head(...)` 保留为 convenience wrapper
- `recover_from_fall(...)` 不再属于 public contract
- `follow_line(...)`、`gait_step(...)` 不再属于 public contract
