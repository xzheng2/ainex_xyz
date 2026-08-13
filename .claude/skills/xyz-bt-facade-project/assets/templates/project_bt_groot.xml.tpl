<?xml version="1.0" encoding="UTF-8"?>
<!--
  Groot BT visualization for {{PROJECT}}.
  Keep in sync with tree/{{PROJECT}}_bt.py (see xyz-bt-facade-project SKILL.md § Groot XML sync).
  Param values are auto-refreshed one-way from bt.py by tools/sync_groot_xml.py (matches
  nodes by name=), which writes {{PROJECT}}_groot.synced.xml — do not hand-tune params here;
  every node must carry a unique name= matching its bt.py instance so the sync can match it.

  The scaffold tree below names SIX CONCRETE NODE TYPES from xyz_bt_lib.behaviours,
  the same six that project_bt.py.tpl imports. On a robot whose node library differs,
  substitute the node types that exist there — in BOTH files, together. What the
  template teaches is the structure (ID= type + name= label, TreeNodesModel port
  schema, one-for-one correspondence with bt.py), not the node selection.

  Three-part structure:
    <BehaviorTree>   — tree instances: each leaf uses ID="ClassName" (type) + name="label"
                       Attribute values are the CONFIG_DEFAULTS overrides from _bt.py.
    <TreeNodesModel> — port schema for EVERY node type in the tree.
                       Without an entry here, Groot shows the node box but no port fields.
                       Must include: Sequence/Fallback (memory port), all L1/L2 library nodes,
                       ALL project-local action/condition nodes, and decorators.
                       Omit dict-type params (e.g. gait_param) — not Groot-representable.
-->
<root main_tree_to_execute="{{PROJECT}}">

  <!-- TODO: Replace this scaffold tree with the actual {{PROJECT}} tree structure. -->
  <BehaviorTree ID="{{PROJECT}}">
    <Sequence name="{{PROJECT_CLASS}}BT" memory="false">
      <Fallback name="SafetyGate" memory="false">
        <Decorator ID="LatchedDwellDecorator" name="StandConfirmed"
          required_ticks="5" state_key="safety_stand_confirmed">
          <Condition ID="L1_Balance_IsStanding" name="IsStanding"
            expected_stand_label="stand"/>
        </Decorator>
        <Sequence name="Recovery" memory="false">
          <Action ID="L2_Motion_StopGait" name="StopGait_recovery"/>
          <Action ID="L2_Balance_RecoverFromFall" name="RecoverFromFall"
            lie_action="lie_to_stand"
            recline_action="recline_to_stand"
            buzzer_freq="1900"
            buzzer_on_time="0.1"
            buzzer_off_time="0.01"
            buzzer_repeat="1"
            pre_action_delay_s="2.0"
            post_action_delay_s="0.5"/>
        </Sequence>
      </Fallback>
      <Fallback name="TaskControl" memory="false">
        <Sequence name="Approach" memory="false">
          <Condition ID="L1_Vision_IsObjectDetected" name="IsObjectDetected"
            target_id="ball"
            lost_count_threshold="0"/>
          <Action ID="L2_Gait_VisionToObject" name="ApproachObject"
            target_id="ball"
            align_x="320"
            align_y="360"
            x_error_threshold="30"
            y_error_threshold="30"
            x_speed="0.010"/>
        </Sequence>
        <Sequence name="Search" memory="false">
          <Action ID="L2_Motion_StopGait" name="StopGait_search"/>
          <Action ID="L2_Head_SearchSweep" name="SearchSweep"
            target_id="ball"
            state_key="target_search"
            sweep_left_pos="700"
            sweep_right_pos="300"
            sweep_step="10"/>
        </Sequence>
      </Fallback>
    </Sequence>
  </BehaviorTree>

  <TreeNodesModel>
    <Control ID="Sequence">
      <input_port name="memory"/>
    </Control>
    <Control ID="Fallback">
      <input_port name="memory"/>
    </Control>
    <!-- Decorators need a model entry too, or Groot hides their ports. -->
    <Decorator ID="LatchedDwellDecorator">
      <input_port name="required_ticks"/>
      <input_port name="state_key"/>
    </Decorator>
    <Condition ID="L1_Balance_IsStanding">
      <input_port name="expected_stand_label"/>
    </Condition>
    <Condition ID="L1_Vision_IsObjectDetected">
      <input_port name="target_id"/>
      <input_port name="lost_count_threshold"/>
    </Condition>
    <Action ID="L2_Motion_StopGait"/>
    <Action ID="L2_Balance_RecoverFromFall">
      <input_port name="lie_action"/>
      <input_port name="recline_action"/>
      <input_port name="buzzer_freq"/>
      <input_port name="buzzer_on_time"/>
      <input_port name="buzzer_off_time"/>
      <input_port name="buzzer_repeat"/>
      <input_port name="pre_action_delay_s"/>
      <input_port name="post_action_delay_s"/>
    </Action>
    <Action ID="L2_Gait_VisionToObject">
      <input_port name="target_id"/>
      <input_port name="align_x"/>
      <input_port name="align_y"/>
      <input_port name="x_error_threshold"/>
      <input_port name="y_error_threshold"/>
      <input_port name="max_size_threshold"/>
      <input_port name="x_speed"/>
      <input_port name="y_speed"/>
      <input_port name="kp"/>
      <input_port name="max_yaw_deg"/>
      <input_port name="deadband"/>
      <input_port name="min_yaw_deg"/>
      <input_port name="steer_axis"/>
      <input_port name="lateral_kp"/>
      <input_port name="max_y_speed"/>
      <input_port name="min_y_speed"/>
      <input_port name="pan_default"/>
      <input_port name="tilt_default"/>
      <input_port name="lost_count_threshold"/>
    </Action>
    <Action ID="L2_Head_SearchSweep">
      <input_port name="target_id"/>
      <input_port name="state_key"/>
      <input_port name="sweep_left_pos"/>
      <input_port name="sweep_right_pos"/>
      <input_port name="sweep_step"/>
      <input_port name="sweep_pause_ticks"/>
      <input_port name="tilt_sweep_min"/>
      <input_port name="tilt_sweep_max"/>
      <input_port name="tilt_step"/>
    </Action>
  </TreeNodesModel>

</root>
