# BT 基础设施通信参照文件框架 — ADR

> 适用范围：所有基于 `py_trees` + ROS 的 BT 项目
> 最后更新：2026-04-17
> **完整规则同步维护于 skill**: `~/.claude/skills/ainex-bt-project/references/bt_infra_manifest_rules.md`

---

## 决策背景

每次 BT node 启动时，将所有**非业务、基础设施级别** ROS 通信接口记录到静态 JSON 文件，
作为诊断、维护、ROSA 查询的参照依据。清单"启动即覆盖"，反映真实初始化后的实际接口。

---

## 核心约束（4 条）

1. **清单写入时机**：所有 infra 对象创建完毕后，在 `__init__()` 末尾调用
   `write_infra_manifest(path, build_infra_manifest(self.name))`

2. **传感器订阅双重性**：
   - subscriber **接口注册** → 属于 infra → 写入本清单
   - subscriber **callback 收到消息** → 属于 business_in → 写入 `bt_ros_comm_debug`

3. **父类接口必须显式记录**：继承链（如 `color_common.Common`）创建的接口在子类代码中
   不可见，必须在 `infra_manifest.py` 的 `notes` 字段注明父类和源文件路径

4. **节点组合约束**（与 bt_observability_framework.md 一致）：
   `package.xml` 必须声明 `<exec_depend>ainex_bt_edu</exec_depend>`

---

## 1. 目标

每次 BT node 启动时，将该 node 所有**非业务、基础设施级别**的 ROS 通信接口
记录到一份静态 JSON 文件，作为诊断、维护、ROSA 查询的参照依据。

核心原则：
- **"真实启动"即参照**：清单内容必须反映 node 完整初始化后的实际 ROS 接口
- **启动即覆盖**：每次 node 启动时覆盖写一次，不做增量追加
- **手动维护，源锚约束**：清单为静态文件，通过注释显式绑定到源码位置，防止漂移

---

## 2. 核心概念

### 2.1 三类通信边界

| 类型 | 定义 | 示例 | 记录位置 |
|---|---|---|---|
| **infra（基础设施通信）** | 服务 node 运行框架本身，不代表 BT 决策意图 | 树可视化 topic、BB 镜像、exec 控制 service、生命周期 service、传感器 subscriber 接口注册 | 本清单（`infra_comm_manifest_lastrun.json`） |
| **business_out（业务出站）** | 由 BT leaf node 触发，经 behaviours → semantics → comm → runtime 发出 | 步态指令、蜂鸣器、颜色检测配置 | `comm/comm_facade.py`（`ros_out` / `ros_result`） |
| **business_in（业务入站）** | 传感器或外部输入 callback 收到的消息事件（输入适配） | 摄像头 line_data callback、IMU callback 收到数据 | `ainex_bt_edu/input_adapters/`（`ros_in` / `input_state`） |

**传感器订阅的双重性**（重要区分）：

| 维度 | 归属 | 记录到哪里 |
|---|---|---|
| subscriber **接口注册**（topic 名称、消息类型、是否存在） | infra | 本清单 |
| subscriber **callback 收到消息**的采样事件（每 tick 记一次） | business_in | `ainex_bt_edu/input_adapters/` → `bt_ros_comm_debug_*.jsonl`（`ros_in` / `input_state`） |

infra 通信**不纳入** `bt_ros_comm_debug` 业务日志。business_in（`ros_in` / `input_state`）**纳入** `bt_ros_comm_debug`。

### 2.2 "真实启动"的含义

BT node 的 ROS 接口不仅来自其自身 `__init__()`，还来自：

- **继承链**：父类 `__init__()` 创建的 publisher / service server（常见：`color_common.Common`）
- **组合对象**：`TreeROSPublisher`、`MarathonBBBridge`、`BTExecController` 等 infra 组件
- **按需代理**：`rospy.ServiceProxy` 在 lifecycle 方法（`enter_func` / `exit_func`）中按需创建

**维护陷阱**：只读子类代码、忽略父类 `__init__()` 是清单漏记的头号原因。

---

## 3. 通用目录结构

新 BT 项目建议采用以下布局：

```
<bt_package>/
  <project_name>/
    infra/
      __init__.py
      infra_manifest.py       ← 清单定义 + 写文件
      tree_publisher.py
      bb_ros_bridge.py
      bt_exec_controller.py
    app/
      <project>_bt_node.py    ← node 入口，__init__ 末尾调用 write_infra_manifest
    log/
      infra_comm_manifest_lastrun.json   ← 启动时生成
```

---

## 4. `infra_manifest.py` 代码模板

```python
#!/usr/bin/env python3
"""Infrastructure communication manifest for <project> BT node.

每次 node 启动时生成一份静态 JSON，列出所有基础设施级别 ROS 接口。
这些接口被排除在 Generic ROS Facade 业务日志之外。

输出：<project>/log/infra_comm_manifest_lastrun.json
"""
import json
import os
import time


def build_infra_manifest(node_name: str) -> list:
    """构建基础设施接口清单。

    Args:
        node_name: ROS node 名称（如 'marathon_bt'），用于解析以 '~' 开头的相对名称。

    Returns:
        接口记录列表，每条记录字段见下方模板。
    """
    def resolve(name):
        if name.startswith('~'):
            return '/' + node_name + '/' + name[1:]
        return name

    records = [
        # ── <ComponentA> ──────────────────────────────────────────────────
        # 说明：来自哪个类 / 文件，方便维护时溯源
        {
            "component": "<ComponentA>",
            "kind": "topic_pub",           # topic_pub | topic_sub | service_server | service_client | lifecycle_action
            "name": "~some/topic",
            "resolved_name": resolve("~some/topic"),
            "msg_or_srv_type": "std_msgs/String",
            "purpose": "简要说明用途",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
        },

        # ── <BtNode> 继承自 <BaseClass> ───────────────────────────────────
        # IMPORTANT: <BtNode> 继承 <BaseClass>。
        # <BaseClass>.__init__() 创建以下接口，在 <BtNode> 代码中不可见。
        # 任何修改 <BaseClass>.__init__() 的操作必须同步更新此清单。
        # Source: <package>/src/<package>/<base_class>.py
        {
            "component": "<BtNode>",
            "kind": "service_server",
            "name": "~enter",
            "resolved_name": resolve("~enter"),
            "msg_or_srv_type": "std_srvs/Empty",
            "purpose": "lifecycle enter — created by <BaseClass>.__init__()",
            "bt_decision_related": False,
            "excluded_from_generic_ros_facade": True,
            "notes": "created by <BaseClass>.__init__(); see <package>/src/<package>/<base_class>.py",
        },
        # ... 继续列出父类创建的其余接口
    ]
    return records


def write_infra_manifest(path: str, items: list) -> None:
    """将清单写入 JSON 文件（每次调用覆盖）。"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    doc = {
        "schema_version": "1.0",
        "generated_ts": time.time(),
        "description": "Static <project> BT infrastructure communication manifest. "
                       "All items are excluded from the Generic ROS Facade business log.",
        "interfaces": items,
    }
    with open(path, 'w') as f:
        json.dump(doc, f, indent=2)
```

---

## 5. node `__init__()` 集成模板

在 BT node 的 `__init__()` 末尾（所有 infra 对象创建完成后）插入：

```python
# ── Infra comm manifest（须在所有 infra 对象创建完毕后调用）──────────
from <project>.infra.infra_manifest import build_infra_manifest, write_infra_manifest

manifest_path = os.path.join(_LOG_DIR, 'infra_comm_manifest_lastrun.json')
write_infra_manifest(manifest_path, build_infra_manifest(self.name))
```

写入时机要求（缺一则清单不完整）：

| 前置条件 | 原因 |
|---|---|
| `TreeROSPublisher` 已创建 | 树可视化 topic 已注册 |
| `MarathonBBBridge` / BB bridge 已创建 | BB 镜像 topic 已注册 |
| `BTExecController` 已创建 | exec 控制 service 已注册 |
| `/imu`、`/object/pixel_coords` 等 subscriber 已创建 | 传感器 subscriber **接口注册**已完成（纳入 infra 清单） |
| `super().__init__()` 已执行 | 父类接口已全部注册 |

> subscriber 接口写入 infra manifest 记录的是接口**是否存在**。
> subscriber callback 实际收到的消息事件属于 business_in，由 `ainex_bt_edu/input_adapters/` 的 `write_snapshot()` 以 `ros_in` / `input_state` 写入 `bt_ros_comm_debug`，不在本清单记录。

---

## 6. 接口记录规范

### 6.1 必填字段

| 字段 | 说明 |
|---|---|
| `component` | 创建该接口的类名（便于追溯） |
| `kind` | `topic_pub` / `topic_sub` / `service_server` / `service_client` / `lifecycle_action` |
| `name` | ROS 名称（可使用 `~` 相对名） |
| `msg_or_srv_type` | 消息或服务类型（如 `std_msgs/String`） |
| `purpose` | 一句话说明用途 |
| `bt_decision_related` | 通常为 `false`（基础设施通信） |
| `excluded_from_generic_ros_facade` | 通常为 `true` |

### 6.2 可选字段

| 字段 | 说明 |
|---|---|
| `resolved_name` | `~` 名称解析后的绝对路径（建议填写，便于 ROSA 查询） |
| `notes` | 补充说明，**继承自父类的接口必须填写，标明父类和源文件路径** |

---

## 7. 维护核查清单

每次新增或修改 BT node 时，检查以下项：

- [ ] 是否修改了 BT node 的继承链？→ 重新审查父类 `__init__()` 创建的全部接口
- [ ] 是否新增了 infra 组件（publisher / subscriber / service）？→ 在 `infra_manifest.py` 对应 section 新增记录
- [ ] 是否有按需创建的 `ServiceProxy`（lifecycle 方法中）？→ 以 `service_client` 类型纳入清单
- [ ] 清单中所有 `notes` 是否仍指向有效的源文件路径？→ 文件改名时同步更新

---

## 8. 静态验证命令模板

无需启动 node，在 host 直接验证清单内容：

```bash
python3 -c "
import sys
sys.path.insert(0, '/home/pi/docker/ros_ws_src/<bt_package>')
from <project>.infra.infra_manifest import build_infra_manifest
items = build_infra_manifest('<node_name>')
names = [r['name'] for r in items]
# 按需添加断言
assert '<expected_topic>' in names, 'missing <expected_topic>'
assert any(r['name'] == '<svc>' and r['kind'] == 'service_server' for r in items), 'missing <svc>'
print('OK —', len(items), 'interfaces')
for r in items:
    print(' ', r['kind'].ljust(16), r['name'])
"
```

运行时验证（node 启动后）：

```bash
docker exec <container> python3 -c "
import json
with open('/home/ubuntu/ros_ws/src/<bt_package>/<project>/log/infra_comm_manifest_lastrun.json') as f:
    d = json.load(f)
print('schema_version:', d['schema_version'])
print('interfaces:', len(d['interfaces']))
for r in d['interfaces']:
    print(' ', r['kind'].ljust(16), r['name'])
"
```

---

## 9. 常见漏记根因

| 根因 | 表现 | 预防措施 |
|---|---|---|
| 只读子类，未读父类 `__init__()` | 父类创建的 publisher / service 缺失 | 在清单中为每条父类接口写 `notes`，注明源文件 |
| 按需 `ServiceProxy` 被忽略 | lifecycle 调用的 service client 未记录 | 审查 `enter_func`、`exit_func`、`start_srv_callback`、`stop_srv_callback` 等方法 |
| 参考旧名称猜测接口名 | topic 名称或消息类型错误 | 必须读源码确认，不依赖记忆或文档推断 |
| infra 组件改名后未更新清单 | 清单与实际不符 | 改名时同步触发清单审查（用 `notes` 中的源文件路径追溯） |

---

## 10. 参考实现

- `infra_manifest.py`：`ainex_behavior/marathon/infra/infra_manifest.py`
- node 集成示例：`ainex_behavior/marathon/app/marathon_bt_node.py`，`__init__()` 末尾
- 完整规则（10 节，含验证命令）：`~/.claude/skills/ainex-bt-project/references/bt_infra_manifest_rules.md`
- 新项目脚手架：`/ainex-bt-project <project_name>` (Claude Code skill)
