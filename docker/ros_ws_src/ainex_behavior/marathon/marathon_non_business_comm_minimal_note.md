# Marathon 非业务通信最简说明

## 1. 范围

非业务通信指：

- 不属于 `BT leaf -> Project Semantic Facade -> Generic ROS Facade` 业务链的 `ROS` 通信

当前按 `marathon_infra_manifest_execution_plan.md` 的思路处理。

包括：

- `tree_publisher.py`
- `bb_ros_bridge.py`
- `bt_exec_controller.py`
- `marathon_bt_node.py` 中启动/关机固定动作
- `marathon_bt_node.py` 中固定 publisher / subscriber / service server

---

## 2. 处理原则

非业务通信不并入 `Generic ROS Facade`。

处理方式：

1. 不走业务 facade
2. 不要求纳入业务 comm jsonl 主链
3. 用一份静态基础设施参照文件单独描述

输出文件：

- `log/infra_comm_manifest_lastrun.json`

---

## 3. 与业务通信的边界

业务通信：

- 由 `actions.py` 触发
- 经过 `semantic_facade.py`
- 最终由 `comm_facade.py` 发出并记录

非业务通信：

- 由基础设施组件自行发出
- 不参与 BT 业务决策归因
- 仅在 manifest 中声明其存在和用途

---

## 4. 最小落地要求

1. 维持 `marathon_infra_manifest_execution_plan.md` 中的 manifest 方案
2. 不额外引入 node 级全量拦截
3. 不要求实时记录非业务通信流
4. 只要求静态说明：
   - 接口名
   - 类型
   - 所属组件
   - 用途
   - `excluded_from_generic_ros_facade=true`

---

## 5. 当前建议

默认做法：

- 业务通信：继续推进 `Project Semantic Facade + Generic ROS Facade`
- 非业务通信：只维护 `infra_comm_manifest_lastrun.json`

这样可以保证：

- 业务链清晰
- infra 不混入业务日志
- 范围边界稳定
