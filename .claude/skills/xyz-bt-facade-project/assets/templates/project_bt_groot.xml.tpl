<?xml version="1.0" encoding="UTF-8"?>
<!--
  Groot BT visualization for {{PROJECT}}.
  Keep in sync with tree/{{PROJECT}}_bt.py (see xyz-bt-facade-project SKILL.md § Groot XML sync).
  Param values are auto-refreshed one-way from bt.py by tools/sync_groot_xml.py (matches
  nodes by name=), which writes {{PROJECT}}_groot.synced.xml — do not hand-tune params here;
  every node must carry a unique name= matching its bt.py instance so the sync can match it.

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
        <Condition ID="L1_Balance_IsStanding" name="IsStanding"
          expected_stand_label="stand"/>
        <Sequence name="Recovery" memory="false">
          <Action ID="L2_Gait_Stop" name="L2_Gait_Stop_recovery"/>
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
      <Fallback name="PatrolControl" memory="false">
        <Sequence name="LineFollowing" memory="false">
          <Condition ID="L1_Vision_IsLineDetected" name="IsLineDetected"/>
          <Action ID="L2_Gait_FollowLine" name="FollowLine"
            x_range="[0, 0.015]"
            yaw_range="[-8, 10]"
            deadband_px="10"
            go_turn_threshold="4"
            head_pan_center="500"
            hi_yaw_threshold="6"
            x_hi_yaw="0.008"/>
        </Sequence>
        <Action ID="L2_Gait_FindLine" name="FindLine"
          base_turn_deg="3"
          max_turn_deg="7"
          count_scale_at="30"
          default_turn_deg="3"
          right_turn_deg="5"/>
        <Action ID="L2_Gait_Stop" name="L2_Gait_Stop_fallback"/>
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
    <Condition ID="L1_Balance_IsStanding">
      <input_port name="expected_stand_label"/>
    </Condition>
    <Condition ID="L1_Vision_IsLineDetected"/>
    <Action ID="L2_Gait_Stop"/>
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
    <Action ID="L2_Gait_FollowLine">
      <input_port name="x_range"/>
      <input_port name="yaw_range"/>
      <input_port name="deadband_px"/>
      <input_port name="go_turn_threshold"/>
      <input_port name="head_pan_center"/>
      <input_port name="hi_yaw_threshold"/>
      <input_port name="x_hi_yaw"/>
    </Action>
    <Action ID="L2_Gait_FindLine">
      <input_port name="base_turn_deg"/>
      <input_port name="max_turn_deg"/>
      <input_port name="count_scale_at"/>
      <input_port name="default_turn_deg"/>
      <input_port name="right_turn_deg"/>
    </Action>
  </TreeNodesModel>

</root>
