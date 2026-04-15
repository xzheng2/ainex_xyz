# ROSA-Ainex Agent — 部署说明与验证清单

## 概述

`rosa-agent` 是一个独立的 Docker 容器，运行 NASA JPL ROSA，通过 ROS_MASTER_URI 以**只读方式**接入现有 `ainex` 容器的 ROS graph，对机器人进行诊断和状态查询。

```
┌─────────────────────────┐    host network (共享宿主机网络栈)   ┌───────────────────────┐
│   ainex (已有容器)        │ ◄────── ROS topics ──────────────► │  rosa-agent (新容器)  │
│   Ubuntu 20.04           │        /services                    │  Python 3.9 + ROSA    │
│   ROS Noetic             │        /rosout_agg                  │  ainex_agent_tools    │
│   硬件驱动 + 步态 + 视觉  │   ROS_MASTER_URI →                  │  read-only            │
│   roscore                │   http://127.0.0.1:11311            │  no hardware access   │
└─────────────────────────┘                                     └───────────────────────┘
```

> **网络模式说明**：`ainex` 以 `--net host` 运行，无法加入 bridge 网络。
> `rosa-agent` 也使用 `network_mode: host`，两容器共享宿主机网络栈，直接通过 `127.0.0.1` 通信。

---

## 目录结构

```
docker/
├── docker-compose.yml              # 启动配置（管理 rosa-agent，ainex 手动管理）
└── rosa-agent/
    ├── Dockerfile                  # rosa-agent 镜像构建
    ├── requirements.txt            # Python 依赖
    ├── setup.py                    # ainex_agent_tools 安装
    ├── .env.example                # 环境变量模板（复制为 .env）
    ├── ainex_agent.py              # 主入口（CLI / ROS 节点双模式）
    ├── llm_config.py               # LLM 初始化（ollama/openai/azure）
    ├── ainex_agent_tools/          # 只读工具层
    │   ├── __init__.py
    │   ├── prompts.py              # Ainex 专用 ROSA 提示词
    │   ├── bt_analysis/
    │   │   └── raw_tick.py         # deterministic recent JSONL retrieval
    │   └── tools/
    │       ├── health.py           # get_robot_health
    │       ├── walking.py          # get_walking_state
    │       ├── bt_monitor.py       # get_bt_status
    │       ├── bt_tick_analysis.py # analyze_bt_tick, get_bt_tick_raw
    │       └── disabled.py         # stop_current_behavior, stand_safe (禁用)
    ├── config/
    │   ├── readonly.yaml           # 只读模式配置
    │   └── blacklist.yaml          # ROSA 内置工具黑名单
    └── vendor/                     # （可选）ROSA 本地源码，见步骤 A
```

---

## 步骤 A：（可选）下载 ROSA 本地源码

本地源码允许定制 ROSA 框架，不依赖 PyPI 版本。如果不需要定制，可跳过此步骤（将从 PyPI 安装）。

```bash
# 在 /home/pi/docker/rosa-agent/ 下执行
mkdir -p vendor
git clone https://github.com/nasa-jpl/rosa vendor/rosa
```

如果存在 `vendor/rosa/`，Dockerfile 会优先从本地安装；否则从 PyPI 安装 `jpl-rosa`。

---

## 步骤 B：配置 LLM

```bash
cd /home/pi/docker/rosa-agent
cp .env.example .env
# 编辑 .env，选择 LLM 提供商
nano .env
```

**推荐方案（无需 API key）：Ollama 本地部署**

```bash
# 在宿主机安装 Ollama（不在容器内）
curl -fsSL https://ollama.ai/install.sh | sh
ollama pull llama3.2    # 或其他模型

# .env 中设置（host 网络模式下用 localhost，不用 host.docker.internal）
LLM_PROVIDER=ollama
OLLAMA_BASE_URL=http://localhost:11434
OLLAMA_MODEL=llama3.2
```

**替代方案：OpenAI API**

```
LLM_PROVIDER=openai
OPENAI_API_KEY=sk-...
OPENAI_MODEL=gpt-4o
```

---

## 步骤 C：构建 rosa-agent 镜像

```bash
cd /home/pi/docker
docker compose build rosa-agent
```

首次构建约 5–10 分钟（ROS Noetic + Python 3.9 + 依赖）。

构建成功后验证：
```bash
docker images | grep ainex-rosa-agent
```

---

## 步骤 E：启动 rosa-agent

确保 `ainex` 容器正在运行，roscore 已启动：
```bash
# 确认 ainex 容器运行中
docker ps | grep ainex

# 启动 rosa-agent（交互式 CLI 模式）
cd /home/pi/docker
docker compose run --rm rosa-agent
```

或以后台服务方式运行（非交互）：
```bash
docker compose up -d rosa-agent
# 进入交互
docker exec -it rosa-agent python3.9 /opt/rosa-agent/ainex_agent.py
```

---

## 步骤 F：单查询模式

```bash
docker compose run --rm rosa-agent python3.9 ainex_agent.py \
  --query "What is the robot's battery voltage?"
```

---

## 验证清单

运行以下命令验证部署正确性（不会触发任何机器人运动）：

### 1. 网络连通性（host 模式：验证 roscore 端口可达）
```bash
# roscore 监听 11311，直接检查端口
docker compose run --rm rosa-agent bash -c \
  "curl -s --max-time 3 http://127.0.0.1:11311 && echo 'Port OK' || echo 'roscore not reachable'"
```
预期：显示 XML-RPC 响应或 `Port OK`（roscore 在宿主机 11311 端口监听）

### 2. ROS Master 连接
```bash
docker compose run --rm rosa-agent python3.9 -c "
import rosgraph
m = rosgraph.Master('/test')
print('ROS Master PID:', m.getPid())
print('Connected: OK')
"
```
预期：打印 Master PID，无异常

### 3. ROS 话题可见性
```bash
docker compose run --rm rosa-agent bash -c "
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
rostopic list | head -20
"
```
预期：看到 `/ros_robot_controller/*`、`/walking/*` 等话题

### 4. 只读工具测试
```bash
docker compose run --rm rosa-agent python3.9 -c "
import os, rospy
os.environ['ROS_MASTER_URI'] = 'http://ainex:11311'
rospy.init_node('test', anonymous=True)
from ainex_agent_tools.tools.health import get_robot_health
print(get_robot_health.invoke({}))
"
```
预期：打印电池电压、舵机温度、IMU 数据（或超时提示，不抛出异常）

### 5. 禁用工具验回
```bash
docker compose run --rm rosa-agent python3.9 -c "
from ainex_agent_tools.tools.disabled import stop_current_behavior, stand_safe
print(stop_current_behavior.invoke({}))
print(stand_safe.invoke({}))
"
```
预期：两个工具均返回 `"This operation is disabled in read-only mode."`

### 6. 黑名单验证
```bash
docker compose run --rm rosa-agent python3.9 -c "
import yaml
with open('/opt/rosa-agent/config/blacklist.yaml') as f:
    cfg = yaml.safe_load(f)
print('Blacklisted tools:', cfg['blacklist'])
assert 'rosparam_set' in cfg['blacklist']
assert 'rosservice_call' in cfg['blacklist']
assert 'roslaunch' in cfg['blacklist']
assert 'rosnode_kill' in cfg['blacklist']
print('Blacklist check: PASS')
"
```
预期：打印黑名单列表，断言通过

### 7. 完整 Agent 对话测试
```bash
docker compose run --rm rosa-agent python3.9 ainex_agent.py \
  --query "List the currently running ROS nodes"
```
预期：返回节点列表（来自 rosnode_list 工具），无错误

### 8. 写操作拒绝验证
```bash
docker compose run --rm rosa-agent python3.9 ainex_agent.py \
  --query "Please stop the robot and make it stand"
```
预期：Agent 解释自己处于只读模式，不尝试调用任何写操作

---

## 已禁用的写操作（完整列表）

| 操作 | 原因 | 禁用方式 |
|------|------|---------|
| `stop_current_behavior` | 调用 `/walking/command stop` | 工具存根，返回 disabled 消息 |
| `stand_safe` | 调用 `/walking/command stand` + 舵机位置 | 工具存根，返回 disabled 消息 |
| `rosparam_set` | 写入 ROS 参数服务器 | ROSA blacklist |
| `rosservice_call` | 通用服务调用，可触发执行器 | ROSA blacklist |
| `roslaunch` | 启动 ROS 节点 | ROSA blacklist |
| `rosnode_kill` | 终止 ROS 节点 | ROSA blacklist |

---

## 风险与未完成项

### 当前风险
1. **Python 3.9 + ROS Noetic 兼容性**：ROS Noetic 的 Python 包在 Ubuntu 20.04 上编译为 Python 3.8。通过 PYTHONPATH 传递给 Python 3.9 的纯 Python 包（rospy, rosgraph, std_msgs 等）可正常工作；含 C 扩展的包（如 roslz4）可能有版本不匹配，但这些包不在关键路径上。
2. **ainex_interfaces 消息**：walking.py 中的 `from ainex_interfaces.srv import ...` 需要 ainex_interfaces 安装到 rosa-agent 的 PYTHONPATH 中。当前 Dockerfile 不包含此步骤——若该工具报 ImportError，需要将 ainex_interfaces 的 Python 包拷贝进容器或通过卷挂载。

### 未完成项（Phase 2）
- [ ] 将 ainex_interfaces 消息定义挂载/复制到 rosa-agent 容器
- [ ] 为 ainex_interfaces 消息添加自动生成步骤
- [ ] 实现 `stop_current_behavior` 和 `stand_safe` 的真实逻辑（需 Phase 2 写权限审计）
- [ ] 添加 ROS 节点模式的 launch 文件
- [ ] 添加 vision streaming（/camera/image_raw 截图诊断）

---

## 快速参考

```bash
# 构建
cd /home/pi/docker && docker compose build rosa-agent

# 启动交互式 CLI
docker compose run --rm rosa-agent

# 后台运行
docker compose up -d rosa-agent

# 查看日志
docker compose logs -f rosa-agent

# 进入容器 shell
docker exec -it rosa-agent bash

# 停止
docker compose stop rosa-agent
```
