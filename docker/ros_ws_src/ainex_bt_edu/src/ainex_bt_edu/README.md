1. Input Adapter 模板

INPUT: ROS topic message                         # 例：/object/pixel_coords 收到 ObjectsInfo
INPUT: lock                                      # 例：self.lock，保证多个 adapter 同 tick 快照一致
INPUT: logger                                    # 例：self._obs_logger；None 时不产生日志
INPUT: tick_id_getter                            # 例：lambda: self._tick_id

__init__()                                      # 初始化 adapter
  store lock/logger/tick_id_getter              # 保存运行时依赖
  init self._live_* fields                      # 例：_live_line_data = None
  init self._received_count = 0                 # 记录上次 latch 后收到多少条 ROS msg
  create py_trees BB client                     # 例：namespace=BB.LATCHED_NS
  register BB write keys                        # 例：BB.LINE_DATA_KEY, Access.WRITE
  write initial BB defaults                     # 例：line_data=None，避免节点第一次读 KeyError
  rospy.Subscriber(topic, msg_type, callback)   # 唯一允许创建 Subscriber 的位置

_callback(msg)                                  # ROS callback 线程入口
  extracted = _extract_xxx(msg)                 # 从 ROS msg 提取原始字段；例：找 obj.type == "line"
  classified = _classify_xxx(extracted)         # 传感器层判别；例：计算 line_error_x
  bb_writes = _make_bb_writes(classified)       # 映射成 BB 写入集合；例：{BB.LINE_DATA: line}
  with lock                                     # 只保护 live state 更新
    _apply_live_writes(bb_writes)               # 把 BB 写入集合复制到 self._live_* 字段
    received_count += 1                         # 标记这个 tick 周期内收到过输入

snapshot_and_reset()                            # app 层在 lock 内调用
  snap = copy self._live_* fields               # 例：{'line_data': ..., 'line_error_x': ...}
  snap['received_count'] = received_count       # 带上消息计数
  received_count = 0                            # 为下个 tick 周期清零
  OUTPUT: snap                                  # 输出冻结快照

write_snapshot(snap, tick_id)                   # app 层释放 lock 后调用
  if snap.received_count > 0                    # 如果本 tick 周期有 ROS 输入
    emit_comm('ros_in')                         # 例：source="/object/pixel_coords", count=3
  write snap values to BB                       # 例：self._bb.line_data = snap['line_data']
  emit_comm('input_state')                      # 每 tick 都记录 BT 实际读到的 BB 输入

OUTPUT: BB facts                                # 例：/latched/line_data, /latched/line_error_x
OUTPUT: observability events                    # ros_in + input_state
---

2. L1 Node 模板

INPUT: BB values                                # 例：BB.ROBOT_STATE, BB.LINE_DATA
INPUT: constructor defaults                     # 例：expected_state='stand', threshold=30
INPUT: logger/tick_id_getter                    # 只用于 decision 日志

__init__()                                      # 初始化条件节点
  inherit AinexBTNode                           # 让 BTDebugVisitor / node event 正常识别
  store logger/tick_id_getter                   # 不直接 import bt_observability
  store default judgement settings              # 例：self._expected_state = 'stand'

setup()                                         # py_trees setup 阶段
  attach BB client                              # 例：namespace=BB.LATCHED_NS
  register BB read keys                         # 只能 Access.READ；例：BB.ROBOT_STATE_KEY

update()                                        # 每次 tick 执行
  value = read BB                               # 例：state = self._bb.robot_state
  passed, reason = _evaluate(value)             # 调用纯判别函数
  status = SUCCESS if passed else FAILURE       # L1 正常只返回 SUCCESS/FAILURE
  emit_bt('decision') if logger exists          # 例：inputs={'robot_state': 'stand'}
  OUTPUT: status                                # 给 BT tree selector/sequence 使用

_evaluate(value)                                # 无副作用判别函数
  compare with constructor defaults             # 例：state == self._expected_state
  OUTPUT: (passed, reason)                      # 例：(True, "robot_state == stand")

OUTPUT: Status.SUCCESS / Status.FAILURE         # 不写 BB，不调用 facade，不发 ROS
OUTPUT: optional decision event                 # 只走 emit_bt，不走 emit_comm
---

3. L2 Node 模板

INPUT: RUNTIME DATA: BB values                 # 每 tick 读取的数据
INPUT: CONFIG: constructor defaults            # 项目树实例化时覆盖的参数
INPUT: DEPENDENCY: facade                      # 外部注入的动作派发接口
INPUT: DEPENDENCY: logger/tick_id_getter       # 外部注入的观测与 tick 归因


__init__()                                      # 初始化动作/策略节点
  inherit AinexBTNode                           # 标准 BT 节点身份
  store facade                                  # 所有硬件/ROS 副作用都通过 facade
  store logger/tick_id_getter                   # 日志和 tick 归因
  store strategy defaults                       # 例：self._yaw_limit = yaw_limit

setup()                                         # 注册 BB 访问
  attach BB client                              # 通常 namespace=BB.LATCHED_NS
  register read/write keys                      # 例：READ line_error_x；必要时 WRITE 状态 key

initialise()                                    # 节点从 IDLE 进入 RUNNING 时调用
  emit_bt('action_intent') if useful            # 例：准备开始 follow_line
  never emit ros_out                            # ros_out 只能由 comm_facade 发

update()                                        # 每 tick 执行动作策略
  value = read BB                               # 例：err = self._bb.line_error_x
  method, kwargs, reason = _select_action(value)# 选择 facade 方法和参数
  getattr(facade, method)(**kwargs, bt_node, tick_id)  # 例：turn_step(x=0,y=0,yaw=-5)
  optional documented BB write                  # 例：写 /head_pan_pos 给 L1 读取
  status = documented return                    # 例：RUNNING 持续找线；SUCCESS 单步完成
  emit_bt('decision') if logger exists          # 记录输入、选择、结果
  OUTPUT: status                                # 反馈给 BT tree

_select_action(values)                          # 无副作用策略函数
  compute params from BB + defaults             # 例：err > deadband -> yaw
  choose existing facade method                 # 例：'go_step' 或 'turn_step'
  OUTPUT: (method_name, kwargs, reason)         # 例：('turn_step', {'x':0,'y':0,'yaw':-5}, 'line right')

OPTIONAL _compute_command()                     # 只有复杂算法才拆出来
  compute command dict only                     # 例：{'profile':'turn','yaw':-5,'x':0}
  called by _select_action()                    # 避免简单节点产生多余空方法

OUTPUT: facade dispatch                         # 真正 ROS 通信在 comm_facade 记录 ros_out
OUTPUT: optional BB write                       # 仅限文档声明的动作状态/协调 key
OUTPUT: optional action_intent/decision         # 只走 emit_bt
