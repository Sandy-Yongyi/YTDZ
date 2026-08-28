# 伺服二维点云范围报文实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将左右 `out_2d_servo` 改为Y轴承载窗口点云Y最大/最小值、X轴执行最小X定位的无状态按帧规划。

**Architecture:** 规划器独立扫描左右Z窗口并生成Y点云载荷与X运动命令；统一轴映射层只为 `out_2d_servo/y` 绕过机械限位并复核Int16。外层协调器在自动模式调用窗口规划，在关闭、强停、手动和清理模式调用专用零数据/零X目标命令。

**Tech Stack:** Python、dataclass、TOML、现有 `FrameSearchHelper` / `MachineAxisMap` / `MotionToTarget` / PLC `AxisData`。

**Spec:** `docs/superpowers/specs/2026-08-28-out-2d-servo-point-range-payload-design.md`

## Global Constraints

- 本计划取代 `2026-08-27-out-2d-servo-frame-motion.md`，不得保留六枪选枪、Y往复、链条开枪、柜体识别或慢进慢退逻辑。
- 轴数、索引和86字节报文结构不变；SN1使用4/5，SN3使用10/11。
- Y固定发送 `Pos=Y_MAX, Speed=Y_MIN, Status=0`，无Y数据或Int16越界发送0/0/0。
- Y是点云载荷，不执行机械位置或速度限位；X仍执行规划器和最终写入层双重限位。
- X固定发送 `Pos=X_MIN-x_position-out_front_x_offset, Speed=x_pos_speed, Status=0`；无X数据时用定位速度回0。
- X和Y独立统计；任一方无数据不使另一方失效。
- 非自动路径发送Y 0/0/0和X目标0、定位速度、Status0，只根据X判断到位。
- TOML旧Y/枪字段保留但不读取；界面隐藏这些字段。
- 规划器返回 `(axis_cmds, False)` 并保证异常不外逸。
- 按项目规则不新增或运行单元测试；使用不落盘场景、编译、导入、TOML和报文验证。

---

### Task 1: 重写无状态点云范围规划器

**Files:**
- Modify: `model/motionplan/MotionOut2DServoFramePlanning.py`

**Interfaces:**
- Consumes: `auto_out_2d_servo_move(machine_cfg, runtime_cfg, plc_data, frame_queue_manager)`。
- Produces: `(axis_cmds: dict[str, AxisData], stop_chain: bool)`。
- Produces: `build_zero_commands(machine_cfg, runtime_cfg, plc_data) -> tuple[dict[str, AxisData], bool]`，布尔值只表示X是否到达目标0。

- [x] **Step 1: 缩减配置与结果数据结构**

用 `Servo2DConfig(sn, x_position, x_front_offset, x_pos_speed, z_front_offset, z_after_offset)` 和 `Servo2DWindowResult(y_min, y_max, x_min)` 取代六枪/Y往复字段；构造器只加载 `ReadDataConfig.z_threshold` 并保留 `MotionToTarget` 供X回零到位判断。

- [x] **Step 2: 实现独立窗口统计**

固定使用：

```python
if h_axis != 0:
    y_min = h_axis if y_min is None else min(y_min, h_axis)
    y_max = h_axis if y_max is None else max(y_max, h_axis)
if v_axis_min != 0:
    x_min = v_axis_min if x_min is None else min(x_min, v_axis_min)
```

窗口使用 `floor((z_position-front)/z_threshold)` 到 `floor((z_position+after)/z_threshold)`，并按安装方向读取 left/right。

- [x] **Step 3: 构建Y点云载荷**

当 `y_min/y_max` 都存在且位于 `[-32768, 32767]` 时返回 `AxisData(Pos=y_max, Speed=y_min, Status=0)`；无数据返回全零；越界记录SN及两个边界后返回全零。

- [x] **Step 4: 构建X运动命令和零数据命令**

有X时计算 `x_min - x_position - x_front_offset`，无X时目标0；两者都使用 `x_pos_speed` 和Status0，并通过X轴位置/速度限制。`build_zero_commands()` 返回Y全零、X目标0，并复用 `move_x_axes_to_target()` 的X到位判断。

- [x] **Step 5: 封闭异常路径**

配置或扫描异常记录SN后返回Y全零和可解析定位速度的X目标0；若X限位或速度配置也无效，最终返回X/Y均0/0/0，方法不得抛出。

- [x] **Step 6: 运行语法检查**

Run: `python -m py_compile model/motionplan/MotionOut2DServoFramePlanning.py`

Expected: exit code 0。

### Task 2: 接入点云载荷语义和非自动零命令

**Files:**
- Modify: `model/motionplan/MachineAxisMap.py`
- Modify: `model/motionplan/MotionFrameByFramePlanning.py`
- Modify: `view/MachineConfigFrame.py`

**Interfaces:**
- Consumes: Task 1 的二元自动返回值和 `build_zero_commands()`。
- Produces: 最终AxisList中的专用Y载荷校验、X正常限位和所有模式一致的零数据行为。

- [x] **Step 1: 增加伺服二维Y专用最终校验**

在 `_limit_axis_command()` 最前面识别 `machine_cfg["type"] == "out_2d_servo" and axis_name == "y"`。将Pos/Speed转换为整数，任一值越过Int16时记录SN和载荷值并返回 `AxisData()`，随后继续处理同设备的X轴；合法时强制返回 `AxisData(Pos=pos, Speed=speed, Status=0)`。其他轴继续走原有限位代码。

- [x] **Step 2: 删除无效状态复位**

删除外层关闭、清理、手动分支中的 `out_2d_servo.reset_motion_state(sn)`，保留 `xn_updown4` 复位逻辑和普通二维 `_handle_out_lift`。

- [x] **Step 3: 接入非自动零数据命令**

设备关闭/强停和回安全分支、手动分支、清理分支遇到 `out_2d_servo` 时调用 `build_zero_commands()`，不再调用通用Y回安全命令；只用返回的X到位结果维护 `device_returning_to_origin` 和 `device_origin_complete`。

- [x] **Step 4: 隐藏无效界面字段**

把 `OUT_2D_SERVO_PARAM_KEYS` 精确缩减为：

```python
[
    "out_front_x_offset",
    "x_pos_speed",
    "out_z_front_offset",
    "out_z_after_offset",
]
```

不删除 MachineConfig TOML 旧字段。

- [x] **Step 5: 检查关键导入**

Run: `python -c "from model.motionplan.MotionFrameByFramePlanning import MotionFrameByFramePlanning; MotionFrameByFramePlanning(); print('ok')"`

Expected: 输出 `ok`，exit code 0。

### Task 3: 点云载荷、模式与报文验证

**Files:**
- Verify: `model/motionplan/MotionOut2DServoFramePlanning.py`
- Verify: `model/motionplan/MachineAxisMap.py`
- Verify: `model/motionplan/MotionFrameByFramePlanning.py`
- Verify: `view/MachineConfigFrame.py`

**Interfaces:**
- Consumes: Task 1～2 的完整实现。
- Produces: 可复查的运行断言和静态检查结果。

- [x] **Step 1: 运行内存窗口场景**

用 `SimpleNamespace`、`AxisFrameData` 和点云行构造左右帧栈，断言多帧结果、零值过滤、只有Y、只有X、均无数据、X公式、X无数据定位速度、Status全零以及返回的 `stop_chain is False`。

- [x] **Step 2: 验证Int16和最终限位分流**

断言Y的合法负值和超过1000的点云坐标不会被机械Y限位修改；`32767/-32768` 可发送，任一越界使本周期Y归零；X目标和速度仍被规划器及 `MachineAxisMap` 限制。

- [x] **Step 3: 验证非自动零数据路径**

用最小 `proc` 数据分别触发设备关闭/回安全、手动和清理分支，断言Y始终0/0/0，X始终目标0、定位速度、Status0，并且只根据X反馈更新到位状态。

- [x] **Step 4: 验证左右索引与报文**

通过 `apply_device_axes_to_list()` 写入左右命令，断言索引4/5和10/11；序列化 `SendMovingFrameData` 后断言长度86字节。

- [x] **Step 5: 执行最终静态检查**

Run: `python -m compileall -q model/motionplan model/plc view/MachineConfigFrame.py`

Run: `git diff --check`

Run: `rg -n "gun_mask|GUN_COUNT|_y_directions|reset_motion_state.*out_2d_servo|frame_idle_y_reciprocate_enabled|frame_x_slow_in_out_enabled" model/motionplan/MotionOut2DServoFramePlanning.py model/motionplan/MotionFrameByFramePlanning.py`

Expected: 编译和差异检查 exit code 0；残留搜索在伺服二维实现中无命中。
