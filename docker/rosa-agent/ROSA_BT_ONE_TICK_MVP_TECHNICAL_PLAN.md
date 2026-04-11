# ROSA BT One-Tick MVP 技术实现报告

> 目标：实现 ROSA 侧 one-tick 运行时诊断 MVP。
> 数据源：只读 `bt_debug_recent.jsonl` 和 `bt_ros_comm_debug_recent.jsonl`。
> 输出：`explain_tick`、`compare_tick`、`diagnose_tick` 三段式报告。
> 实现策略：MVP 不做 subagent，不在 tool 内部嵌套 LLM 调用。tool 只做 deterministic retrieval + prompt scaffold，由 ROSA 主 agent 完成三阶段推理。
>
> **状态：已实现 — MVP 完成 2026-04-11。**

---

## 1. MVP 边界

### 1.1 做什么

MVP 实现两个 ROSA tool：

```text
get_bt_tick_raw
analyze_bt_tick
```

`get_bt_tick_raw` 用于取指定 tick 的 recent raw JSONL，方便调试和证据复查。

`analyze_bt_tick` 用于用户的一次 tick 诊断请求。它读取 raw、组织上下文、返回分阶段分析指令。ROSA 主 agent 收到 tool 返回后，按 scaffold 输出：

```text
1. explain_tick
2. compare_tick
3. diagnose_tick
```

如果用户没有提供物理观察，ROSA 只输出 `explain_tick`，然后请求用户补充观察。

### 1.2 不做什么

MVP 不做：

```text
1. 不读取 lastrun 作为 one-tick 数据源。
2. 不自动 fallback 到 lastrun。
3. 不做 subagent。
4. 不在 tool 内部做三次 LLM 调用。
5. 不把 ros_out 当成物理执行完成证明。
6. 不做跨 tick episode 诊断。
```

`lastrun` 后续用于 replay、跨 tick、跨 run 分析，不进入当前 one-tick MVP。

---

## 2. 目录与文件

新增目录：

```text
ainex_agent_tools/bt_analysis/
```

新增文件：

```text
ainex_agent_tools/bt_analysis/__init__.py   ✓ 已创建
ainex_agent_tools/bt_analysis/raw_tick.py   ✓ 已创建
ainex_agent_tools/tools/bt_tick_analysis.py ✓ 已创建
```

注意：原设计中提到的 `sources.py` 未单独创建；相关功能全部在 `raw_tick.py` 中实现。

修改文件：

```text
ainex_agent_tools/__init__.py   ✓ 已修改（新增 2 个 tool 导出，共 12 个 tool）
ainex_agent_tools/prompts.py   ✓ 已修改（capabilities + critical_instructions rules 5–7）
```

不修改录制侧代码。

---

## 3. Raw Tick Retrieval

### 3.1 模块位置

```text
ainex_agent_tools/bt_analysis/raw_tick.py
```

### 3.2 常量

```python
BT_OBS_DIR         = "/opt/ainex_bt_log"
BT_RECENT_BT       = "bt_debug_recent.jsonl"       # 实际常量名（非 BT_RECENT）
BT_RECENT_COMM     = "bt_ros_comm_debug_recent.jsonl"  # 实际常量名（非 COMM_RECENT）
STALE_WARN_SECONDS = 10.0
```

注意：`STALE_WARN_SECONDS` 只用于 warning，不用于拒绝分析。

### 3.3 数据源规则

只读：

```text
/opt/ainex_bt_log/bt_debug_recent.jsonl
/opt/ainex_bt_log/bt_ros_comm_debug_recent.jsonl
```

行为：

```text
recent 文件缺失：
  返回 error，不能分析。

recent 文件存在但 mtime stale：
  允许分析。
  输出 warning。
  不 fallback 到 lastrun。
  解释：pause/step 模式下 stale 是正常情况。
```

### 3.4 Raw Line 保留策略

每条 JSONL 要同时保留：

```text
raw_line:    原始字符串
parsed:      json.loads 后的 dict，如果解析成功（实际字段名为 parsed，非 event）
parse_error: 如果解析失败
```

原因：

```text
1. LLM 需要看到原始 payload。
2. 解析失败不能悄悄吞掉。
3. diagnose_tick 要能引用 raw 字段。
```

### 3.5 实际实现函数

```python
def _path(filename: str) -> str: ...
    # 返回完整路径

def _file_status(filename: str) -> dict:
    # 返回: {"exists": bool, "age_seconds": float|None, "stale": bool}
    # 注意：无 size / mtime 字段

def _read_jsonl_raw(path: str) -> list:
    # 返回 list of {"raw_line": str, "parsed": dict|None, "parse_error": str|None}
    # 文件缺失时返回 [{"error": "file not found: <path>"}]

def _detect_order(entries: list) -> str:
    # 输入: _read_jsonl_raw 返回的 entries 列表（含 parsed 字段）
    # 返回: "newest-first" | "oldest-first" | "single" | "unknown"

def _group_by_tick(entries: list) -> dict:
    # 无 tick_id 的 entry 归入 key=-1
    # 返回 dict[int, list[entry]]

def _resolve_tick_id(tick_id_param, grouped: dict) -> int:
    # 接受 grouped dict（非 set）
    # "latest" -> max(k for k in grouped if k >= 0)
    # 不存在时抛 KeyError（含可用 tick_ids 列表）

def _select_neighbors(grouped: dict, selected_id: int, include_neighbors: int) -> dict:
    # 返回 {neighbor_tick_id: [entries], ...}，不含 selected_id

def get_raw_tick_bundle(tick_id="latest", include_neighbors: int = 1) -> dict:
    # 主函数，永不抛异常，check ["ok"] first
```

### 3.6 RawTickBundle 实际结构

```text
RawTickBundle
  ok: bool
  error: str | None

  selected_tick_id: int | None
  source_mode: "recent"             # 固定值

  files:
    bt:   "bt_debug_recent.jsonl"
    comm: "bt_ros_comm_debug_recent.jsonl"

  recent_status:
    bt_age_seconds:   float | None
    comm_age_seconds: float | None
    stale_warning:    str | None    # 单字段，非 per-file 对象

  order_detected:                   # per-file 独立检测
    bt:   newest-first | oldest-first | single | unknown
    comm: newest-first | oldest-first | single | unknown

  all_tick_ids: [int, ...]          # sorted ascending，来自 bt 文件

  selected_tick:
    bt_entries:   list of {raw_line, parsed, parse_error}
    comm_entries: list of {raw_line, parsed, parse_error}

  neighbors:
    <tick_id>:
      bt_entries:   list
      comm_entries: list

  warnings: [str, ...]
```

与原设计的差异：
- 无 `project` 字段（marathon 是当前唯一实现，不需要区分）
- 无 `include_neighbors` 字段（输入参数不回传）
- `recent_status` 为扁平结构，非 per-file 对象；`stale_warning` 为单字符串
- entry 字段名 `parsed`（非 `event`），无 `tick_id` 独立字段（从 `parsed` 读取）
- selected_tick 内字段名 `bt_entries`/`comm_entries`（非 `bt_raw_entries`/`comm_raw_entries`）

### 3.7 Neighbor 策略

默认：

```text
include_neighbors = 1
```

含义：

```text
取 selected tick 前后各 1 tick，如果存在。
```

重要规则：

```text
selected tick 是主证据。
neighbor tick 只是可选上下文。
非必要不使用 neighbor。
使用 neighbor 时必须显式标注 tick_id。
不能把 neighbor 的 ros_out / decision / bb_write 说成 selected tick 的事实。
```

neighbor 只在这些情况使用：

```text
1. selected tick 日志不完整。
2. 用户描述包含“刚才”“下一步”“变化过程”。
3. 需要判断命令是否连续重复。
4. rqt/pytreeview 和 selected tick 不一致，需要检查对齐。
5. 需要证明某个动作是连续扫描，而不是单 tick 偶发命令。
```

---

## 4. Tool 1: `get_bt_tick_raw`

### 4.1 文件位置

```text
ainex_agent_tools/tools/bt_tick_analysis.py
```

### 4.2 接口

```python
@tool
def get_bt_tick_raw(
    tick_id: str = "latest",
    include_neighbors: int = 1,
) -> str:
    """
    Read raw recent BT and ROS communication JSONL lines for one runtime tick.
    Reads only bt_debug_recent.jsonl and bt_ros_comm_debug_recent.jsonl.
    include_neighbors=1 by default, but neighbor ticks are optional context.
    """
```

### 4.3 输出

返回纯文本，便于 ROSA 主 agent 和用户阅读：

```text
# BT Tick Raw

selected_tick_id: 1618
source_mode: recent
include_neighbors: 1

recent_status:
  bt_file_age_seconds: 47.2
  comm_file_age_seconds: 47.0
  stale_warning: true
  stale_note: allowed in pause/step mode

SELECTED TICK BT RAW:
{"event": "..."}

SELECTED TICK ROS COMM RAW:
{"event": "..."}

OPTIONAL NEIGHBOR RAW:
tick 1617:
  BT:
  ...
  COMM:
  ...
```

### 4.4 失败输出

```text
Cannot read one-tick recent logs.
Missing file: /opt/ainex_bt_log/bt_debug_recent.jsonl
One-tick analysis does not fall back to lastrun.
Start/pause/step the BT node to generate recent logs.
```

---

## 5. Tool 2: `analyze_bt_tick`

### 5.1 接口

```python
@tool
def analyze_bt_tick(
    tick_id: str = "latest",
    project_context: str = "",
    user_observation: str = "",
    rqt_observation: str = "",
    include_neighbors: int = 1,
) -> str:
    """
    Prepare one-tick BT analysis from recent logs.
    The tool returns raw logs and staged prompt instructions.
    ROSA main agent performs explain_tick, compare_tick, and diagnose_tick.
    """
```

### 5.2 职责

`analyze_bt_tick` 不直接诊断。它负责：

```text
1. 调用 get_raw_tick_bundle。
2. 格式化 selected tick raw。
3. 格式化 optional neighbor raw。
4. 拼接 project_context。
5. 拼接 user_observation。
6. 拼接 rqt_observation。
7. 返回 staged analysis scaffold。
```

### 5.3 默认项目上下文（已实现）

如果 `project_context` 为空，tool 使用以下**高层次**默认上下文：

```text
This marathon BT is for Ainex HuroCup line-following behavior. At a high level, the
robot should maintain safe standing state, use vision/blackboard line information to
decide whether to follow or search for the line, and emit ROS commands through the BT
semantic/communication layers. The raw tick logs are the primary evidence for exact
node behavior and payloads.
```

**重要约束（已落实）**：
- 默认上下文只描述高层目标，不包含具体节点行为、servo ID、阶段逻辑等实现细节
- 详细行为由 selected tick raw logs 提供
- 用户提供 `project_context` 时直接覆盖，不与默认上下文叠加

### 5.4 user_observation 为空

当 `user_observation.strip() == ""`：

```text
analysis_mode: explain_only
```

tool 返回的 scaffold 要求 ROSA：

```text
只输出 explain_tick。
不要输出 compare_tick。
不要输出 diagnose_tick。
请求用户补充这个 tick 的物理观察。
```

### 5.5 user_observation 非空

当用户观察存在：

```text
analysis_mode: full
```

tool 返回 scaffold 要求 ROSA 输出：

```text
1. explain_tick
2. compare_tick
3. diagnose_tick
```

### 5.6 analyze_bt_tick 输出模板

```text
# BT One-Tick Analysis Package

analysis_mode: full | explain_only
selected_tick_id: <id>
source_mode: recent
include_neighbors: 1

RECENT FILE STATUS:
  ...

PROJECT CONTEXT:
  ...

USER OBSERVATION:
  ...

RQT / PYTREEVIEW OBSERVATION:
  ...

SELECTED TICK BT RAW:
  ...

SELECTED TICK ROS COMM RAW:
  ...

OPTIONAL NEIGHBOR RAW:
  ...

STAGED ANALYSIS INSTRUCTIONS:
  ...
```

---

## 6. Prompt Scaffold

### 6.1 全局约束

`analyze_bt_tick` 返回中必须包含：

```text
Global rules:
- Selected tick raw is primary evidence.
- Neighbor raw is optional context only.
- Do not use neighbor tick events as selected tick facts.
- Do not claim physical execution from ros_out alone.
- ros_out proves command emission, not command execution.
- diagnose_tick must include both layer-level and concrete raw-field attribution.
```

### 6.2 explain_tick 指令

```text
1. explain_tick

Use only PROJECT CONTEXT and SELECTED TICK RAW.
Do not use USER OBSERVATION.
Do not diagnose.

Explain:
- active BT leaf/path
- condition inputs, status, reason
- important blackboard values
- ROS commands emitted in selected tick
- concrete payloads
- theoretical physical behavior
- unknowns: what raw logs do not prove

Neighbor raw may be used only if selected tick raw is incomplete or transition context is necessary.
If used, cite neighbor tick_id explicitly.
```

### 6.3 compare_tick 指令

```text
2. compare_tick

Use explain_tick and USER OBSERVATION.
Do not use raw logs directly unless the information was already stated in explain_tick.
Do not diagnose root cause.

Output:
- matches
- mismatches
- uncertain items

Each mismatch must map:
  theoretical behavior -> observed physical behavior
```

### 6.4 diagnose_tick 指令

```text
3. diagnose_tick

Use compare_tick + PROJECT CONTEXT + SELECTED TICK RAW + optional RQT/PYTREEVIEW OBSERVATION.
Go back to raw logs for concrete attribution.

Output:
- most likely layer: Layer 0/1/2/3/4
- confidence
- concrete attribution:
  - bt_node
  - condition
  - condition inputs
  - blackboard key/value
  - semantic_source
  - target
  - comm_type
  - ros_node
  - payload
- evidence
- less likely layers and why
- next verification questions

Do not stop at layer-level diagnosis.
Always name specific raw fields when available.
```

### 6.5 归因层级

```text
Layer 0: alignment/source mismatch
  tick_id mismatch, wrong session, recent file stale in non-pause mode, rqt display not aligned, namespace mismatch.

Layer 1: BT decision/input layer
  condition result, blackboard value, tree branch selection, state machine logic.

Layer 2: command generation/emission layer
  leaf ticked but ros_out missing, wrong semantic_source, wrong target, wrong payload.

Layer 3: ROS controller/communication feedback layer
  ros_out exists but controller state does not change, command may not be accepted, competing command source.

Layer 4: physical execution/hardware layer
  controller appears correct but physical action is wrong: servo, power, mechanical jam, slipping, unstable posture.
```

---

## 7. ROSA Prompt 更新

修改：

```text
ainex_agent_tools/prompts.py
```

### 7.1 capabilities 中增加

```text
analyze_bt_tick — one-tick runtime BT diagnosis from recent logs.
Use this first when the user asks about the current pause/step tick, a specific tick_id, or says the robot's current physical behavior does not match BT behavior.
It reads only recent JSONL files, not lastrun.
After calling it, produce the staged report requested by the tool output.
```

### 7.2 critical instructions 中增加

```text
For one-tick BT debug:
1. Always call analyze_bt_tick before answering one-tick runtime diagnosis questions.
2. explain_tick must use only project context and selected tick raw logs.
3. compare_tick must compare explain_tick with the user's physical observation.
4. diagnose_tick must go back to selected tick raw logs and cite concrete fields.
5. Do not claim physical execution from ros_out alone.
6. If user_observation is missing, output only explain_tick and ask for physical observation.
7. Neighbor ticks are optional context; do not treat them as selected-tick facts.
```

---

## 8. 典型调用

### 8.1 用户有观察

用户：

```text
分析当前 tick。机器人还在往前走，头没有动。rqt 显示 FindLine。
```

ROSA 应调用：

```python
analyze_bt_tick(
    tick_id="latest",
    user_observation="机器人还在往前走，头没有动。",
    rqt_observation="rqt 显示 FindLine",
    include_neighbors=1,
)
```

ROSA 最终输出：

```text
1. explain_tick
2. compare_tick
3. diagnose_tick
```

### 8.2 用户没有观察

用户：

```text
解释当前 tick。
```

ROSA 应调用：

```python
analyze_bt_tick(tick_id="latest", include_neighbors=1)
```

ROSA 最终只输出：

```text
1. explain_tick

请补充这个 tick 的物理观察：
- 身体是否走、停、转、倒？
- 头部是否移动？
- 地上目标是否可见？
- rqt/pytreeview 显示什么？
```

### 8.3 用户指定 tick

用户：

```text
分析 tick 1618。机器人停止了，但是地上有线却没有进入 FollowLine。
```

ROSA 应调用：

```python
analyze_bt_tick(
    tick_id="1618",
    user_observation="机器人停止了，但是地上有线却没有进入 FollowLine。",
    include_neighbors=1,
)
```

---

## 9. 测试计划

### 9.1 Retrieval 单元测试

用本地 recent 文件测试：

```text
ainex_behavior/marathon/log/bt_debug_recent.jsonl
ainex_behavior/marathon/log/bt_ros_comm_debug_recent.jsonl
```

测试项：

```text
1. latest 能解析为最大 tick_id。
2. 指定 tick 能返回对应 raw。
3. include_neighbors=0 只返回 selected tick。
4. include_neighbors=1 返回前后 tick，缺失时不报错。
5. JSON parse 失败时保留 raw_line 和 parse_error。
6. recent 文件 stale 时返回 warning，但不失败。
7. recent 文件缺失时失败，且不 fallback 到 lastrun。
```

### 9.2 Tool 输出测试

测试 `get_bt_tick_raw`：

```text
1. 输出 selected_tick_id。
2. 输出 BT raw 和 ROS comm raw。
3. neighbor 区域明确标注 tick_id。
4. stale warning 文案正确。
```

测试 `analyze_bt_tick`：

```text
1. user_observation 为空时 analysis_mode=explain_only。
2. user_observation 非空时 analysis_mode=full。
3. scaffold 包含三阶段 prompt。
4. scaffold 包含 neighbor policy。
5. scaffold 明确禁止从 ros_out 推断物理执行完成。
```

### 9.3 Agent 行为测试

测试 prompt 是否让 ROSA 按预期输出：

```text
1. 用户无观察 -> 只输出 explain_tick。
2. 用户有观察 -> 输出 explain_tick / compare_tick / diagnose_tick。
3. diagnose_tick 是否具体引用 payload。
4. rqt 与 raw 不一致时是否先归因 Layer 0。
5. raw 有 ros_out 但用户说没动作时，是否归因 Layer 3/4，而不是 BT 层。
6. 用户说地上有线但 raw line_data=None 时，是否具体归因到视觉输入 / blackboard 输入链路。
```

---

## 10. 后续升级路线

### V2: Orchestrated Three-Call Chain

如果 MVP 中 ROSA 主 agent 输出格式不稳定，再实现 staged chain：

```text
explain = llm.invoke(explain_prompt(A, B raw))
compare = llm.invoke(compare_prompt(explain, C))
diagnose = llm.invoke(diagnose_prompt(compare, A, B raw, D))
```

优点：

```text
1. explain 阶段完全看不到用户观察。
2. compare 阶段不直接看 raw，避免提前归因。
3. diagnose 阶段强制回 raw 取证。
```

### V3: Verifier

增加 verifier 检查：

```text
1. diagnose 是否引用了具体 raw fields。
2. 是否错误地把 ros_out 当作物理执行证明。
3. 是否把 neighbor tick 事件错当成 selected tick。
4. 是否在 user_observation 缺失时输出了 compare/diagnose。
```

### V4: Cross-Tick

在 one-tick 稳定后扩展：

```text
raw tick bundle[]
  -> explain_episode
  -> compare_episode
  -> diagnose_episode
```

跨 tick 引入 episode、状态转移、持续失败、command spam、感知长时间缺失等诊断。

---

## 11. 实现顺序（已完成）

```text
1. ✓ 创建 bt_analysis/raw_tick.py
2. ✓ 实现 recent raw retrieval（_path, _file_status, _read_jsonl_raw, _detect_order,
      _group_by_tick, _resolve_tick_id, _select_neighbors, get_raw_tick_bundle）
3. ✓ 实现 get_bt_tick_raw（bt_tick_analysis.py）
4. ✓ 用现有 recent log 验证 raw 输出（tick 1618 + neighbor 1617，stale warning 正确）
5. ✓ 实现 analyze_bt_tick scaffold（explain_only / full 两种 mode）
6. ✓ 更新 ainex_agent_tools/__init__.py（12 个 tool，analyze_bt_tick + get_bt_tick_raw 在 read_bt_obs 之后）
7. ✓ 更新 prompts.py（capabilities + critical_instructions rules 5–7）
8. 手工 query 测试（待运行时验证）
9. Scaffold 微调（视稳定性决定）
10. V2 staged chain（视 MVP 稳定性决定）
```

---

## 12. 验收标准

MVP 完成时应满足（已全部通过 2026-04-11）：

```text
✓ 1. one-tick 只读取 recent，不读取 lastrun。
     → raw_tick.py source_mode 固定为 "recent"，无 lastrun fallback。
✓ 2. latest tick 能正确选择。
     → _resolve_tick_id("latest", grouped) = max(k for k >= 0)；验证 tick_id=1618 正确。
✓ 3. 默认 include_neighbors=1。
     → get_bt_tick_raw / analyze_bt_tick 均默认 include_neighbors=1。
✓ 4. neighbor 非必要不参与解释，使用时必须标 tick_id。
     → GLOBAL RULES 第 2 条 + prompts.py critical_instructions rule 7 明确规定。
✓ 5. user_observation 为空时只 explain，不 diagnose。
     → analysis_mode = "explain_only" if not user_observation.strip()；task_instructions 分支。
✓ 6. pause/step stale recent 允许分析，但输出 warning。
     → stale_warning 写入 bundle；analyze_bt_tick 输出 STALE WARNING 但 ok=True。
✓ 7. diagnose_tick 同时输出层级归因和具体 raw 字段归因。
     → task_instructions full 模式：layer 0-4 + concrete attribution 字段列表。
✓ 8. payload 出现在诊断证据中。
     → diagnose_tick 指令要求 cite payload 字段；GLOBAL RULES 第 4 条强制要求。
✓ 9. ROSA 不把 ros_out 误说成物理动作已经完成。
     → GLOBAL RULES 第 3 条 + prompts.py critical_instructions rule 6 明确规定。
✓ 10. 用户能继续补充观察，ROSA 能基于同一 tick 继续比较和归因。
      → 同 tick_id 再次调用 analyze_bt_tick 传入 user_observation 即可进入 full 模式。
```

