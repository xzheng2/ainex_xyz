# Claude Code 开发环境配置

`ainex_behavior` BT 开发环境中生效的 Claude Code 扩展：`ainex-bt-project` skill + BT节点守护 hooks。

---

## 1. Skill — `ainex-bt-project`

**文件**: `~/.claude/skills/ainex-bt-project/SKILL.md`

### 触发时机

| 用户意图 | 示例 |
|---|---|
| 新建 BT 项目 | `新建BT项目 ball_kick`、`scaffold bt project sprint` |
| 给已有项目加节点 | `给 marathon 加条件节点`、`add bt node to fall_recovery` |

### 两种模式

**Mode 1 — 新建项目** (11步工作流)

1. 加载 `references/` 规范文档
2. 创建目录结构 (`app/` `tree/` `behaviours/` `semantics/` `comm/` `infra/` `log/`)
3. 渲染模板文件（替换 `{{PROJECT}}` / `{{PROJECT_CLASS}}`）
4. 节点组合检查（必须引用 ainex_bt_edu 标准节点）
5. 接口检查（SemanticFacade 6个抽象方法）
6. 更新 `package.xml`
7. 输出合规清单（自动满足项 vs TODO 项）
8. `chmod +x` 节点脚本
9. 更新 `CMakeLists.txt`
10. 创建 `.launch` 文件
11. `catkin build` + 验证

**Mode 2 — 添加/修改节点**

1. 先查 `ainex_bt_edu/` 是否已有等价节点
2. 若无，生成 `behaviours/` 新节点（遵循 AinexBTNode 规范）
3. 给出 `tree/<project>_bt.py` 的精确 diff
4. 检查 SemanticFacade / comm_facade 是否需要新方法
5. 检查 `infra_manifest.py` 是否需要更新
6. 提示运行 `check_imports.py`

### AinexBTNode 节点规范（四项强制）

```python
from ainex_bt_edu.base import AinexBTNode

class MyNode(AinexBTNode):       # ① 继承 AinexBTNode
    LEVEL = 'L2'                  # ② 类变量，声明节点等级 L1/L2/L3
    BB_LOG_KEYS = ['robot_state'] # ③ 类变量，声明 BB 可观测键

    def update(self):
        # ④ 不直接调用 rospy.*() — 通过 facade/comm_facade
        return self.SUCCESS
```

---

## 2. Hooks — BT节点守护

两个 Claude Code hook 守护 `ainex_behavior/.../behaviours/*.py` 的写入行为。
配置文件: `~/.claude/settings.json`

### Hook 1 — 事前提醒 (PreToolUse)

**脚本**: `~/.claude/hooks/bt_node_pre_guard.py`
**触发**: Write 或 Edit 工具写入前
**路径匹配**: `ainex_behavior/.+/behaviours/[^/]+\.py`

行为：路径匹配时向 Claude 注入 skill 使用提醒，提示在写代码前先调用
`ainex-bt-project` skill 获取规范模板。不阻断写入（exit 0）。

### Hook 2 — 事后合规检查 (PostToolUse)

**脚本**: `~/.claude/hooks/bt_node_guard.py`
**触发**: Write 或 Edit 工具写入后
**路径匹配**: `ainex_behavior/.+/behaviours/[^/]+\.py`

写入完成后读取文件内容，检查四项规范（剥离字符串字面量和注释后检查，避免误报）：

| 检查项 | 规则 |
|---|---|
| ① AinexBTNode | `class Foo(AinexBTNode)` 模式存在 |
| ② LEVEL | 类体内 `LEVEL = ...` 存在（4空格缩进） |
| ③ BB_LOG_KEYS | 类体内 `BB_LOG_KEYS = ...` 存在（4空格缩进） |
| ④ rospy 隔离 | 无 `rospy.*()` 调用 |

有违规时注入违规列表触发自纠正；全部合规则静默（无输出）。不阻断写入（exit 0）。

### settings.json 配置摘要

```json
{
  "hooks": {
    "PreToolUse": [
      {
        "matcher": "Write|Edit",
        "hooks": [{"type": "command",
                   "command": "python3 ~/.claude/hooks/bt_node_pre_guard.py",
                   "timeout": 5}]
      }
    ],
    "PostToolUse": [
      {
        "matcher": "Write|Edit",
        "hooks": [{"type": "command",
                   "command": "python3 ~/.claude/hooks/bt_node_guard.py",
                   "timeout": 5}]
      }
    ]
  }
}
```

> 非 `behaviours/` 路径的写入完全不受影响。

---

## 3. 验证命令

```bash
# 准备测试目录
mkdir -p /tmp/ainex_behavior/marathon/behaviours

# ① Pre hook 有提醒
echo '{"tool_name":"Write","tool_input":{"file_path":"/tmp/ainex_behavior/marathon/behaviours/test.py"}}' \
  | python3 ~/.claude/hooks/bt_node_pre_guard.py

# ② Post hook 非合规 → 违规列表
echo 'class Foo: pass' > /tmp/ainex_behavior/marathon/behaviours/bad.py
echo '{"tool_name":"Write","tool_input":{"file_path":"/tmp/ainex_behavior/marathon/behaviours/bad.py"}}' \
  | python3 ~/.claude/hooks/bt_node_guard.py

# ③ Post hook 合规 → 无输出
cat > /tmp/ainex_behavior/marathon/behaviours/good.py << 'EOF'
from ainex_bt_edu.base import AinexBTNode
class GoodNode(AinexBTNode):
    LEVEL = 'L2'
    BB_LOG_KEYS = ['robot_state']
    def update(self):
        return self.SUCCESS
EOF
echo '{"tool_name":"Write","tool_input":{"file_path":"/tmp/ainex_behavior/marathon/behaviours/good.py"}}' \
  | python3 ~/.claude/hooks/bt_node_guard.py; echo "exit=$?"
```

---

## 4. 文件索引

| 文件 | 说明 |
|---|---|
| `~/.claude/skills/ainex-bt-project/SKILL.md` | Skill 主文件（触发条件 + 完整工作流） |
| `~/.claude/skills/ainex-bt-project/references/` | 规范文档（observability + infra manifest rules） |
| `~/.claude/skills/ainex-bt-project/assets/templates/` | 代码模板（`*.py.tpl`） |
| `~/.claude/hooks/bt_node_pre_guard.py` | PreToolUse hook 脚本 |
| `~/.claude/hooks/bt_node_guard.py` | PostToolUse hook 脚本 |
| `~/.claude/settings.json` | Hook 注册配置 |
