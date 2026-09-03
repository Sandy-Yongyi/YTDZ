# xn_updown2 两枪顶底设备按帧运动 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (\`- [ ]\`) syntax for tracking.

**Goal:** 让按帧策略的左右 \`xn_updown2\` 设备为 \`y1/x1\` 与 \`y2/x2\` 两组轴独立执行顶底点云定位、连续 X 往复、Y 大跨度互锁和两阶段安全返回。

**Architecture:** 新增无状态搜索助手，只根据方向帧栈和 Z 窗口输出原始几何量；新增规划器以每个 SN 的两个组状态保存目标、往复方向和安全返回阶段。外层按帧规划器在自动、禁用、强制停机、清理模式和手动模式边界委派新规划器，最终仍通过 \`MachineAxisMap.apply_device_axes_to_list\` 做 PLC 写入前限位。

**Tech Stack:** Python 3、dataclasses、现有 TOML 配置、\`FrameSearchHelper\`、\`AxisLimits\`、PLC \`AxisData\`。

**Spec:** \`docs/superpowers/specs/2026-08-29-xn-updown2-frame-motion-design.md\`

## Global Constraints

- 仅支持 \`MachineConfig1.toml\` 中 SN 0/2 的 \`xn_updown2\`，轴顺序为 \`y1,x1,y2,x2\`，AxisList 索引分别为 0-3 与 6-9。
- 原始 \`raw_x_max - raw_x_min > ProcessConfig.x_range\` 必须在坐标换算和 X 限位前判断；限位后不得撤销资格。
- 使用 \`out_z_front_offset/out_z_after_offset\`、\`out_front_x_offset/out_after_x_offset\`、\`out_down_y_offset/out_up_y_offset\`、\`x_pos_speed/x_recip_speed/y_pos_speed\`、\`spray_pos_tolerance\` 与现有位置/速度/安全限位；合法 0 不得被默认值覆盖。
- 不使用 \`outside_total_cycles\`；只要几何仍有效，X 就持续往复。
- 每组安全返回必须独立执行 \`return_safe_x -> return_safe_y\`；清理模式自身的既有动作不变。
- 不新增或运行单元测试；使用 Python 编译/导入、TOML 解析、不落盘模拟、\`git diff --check\`、180 字符检查和残留搜索验证。
- 不修改现有 \`XNUpdown4FrameSearchHelper.py\` 的用户修改，也不改 12 轴顺序、86 字节报文、界面、点云采集或其他设备规划器。

---

### Task 1: 新增按帧几何搜索助手

**Files:**
- Create: \`model/motionplan/motionutil/XNUpdown2FrameSearchHelper.py\`
- Read: \`model/motionplan/motionutil/FrameSearchHelper.py\`

**Interfaces:**
- Consumes: \`machine_cfg\`、\`runtime_cfg\`、\`frame_queue_manager\`，以及帧行的 \`H_Axis\`、\`V_Axis_Min\`、\`V_Axis_Max\`。
- Produces: \`XNUpdown2FrameGeometry(has_data, raw_x_min, raw_x_max, raw_y_min, band_y_max)\` 和 \`get_geometry(machine_cfg, runtime_cfg, frame_queue_manager)\`。

- [ ] **Step 1: 建立不可变几何结果和帧栈访问**

\`\`\`python
@dataclass(frozen=True)
class XNUpdown2FrameGeometry:
    has_data: bool = False
    raw_x_min: int | None = None
    raw_x_max: int | None = None
    raw_y_min: int | None = None
    band_y_max: int | None = None
\`\`\`

以 \`FrameSearchHelper.get_side_frames()\` 选择 left/right 帧栈，复用其 \`row_has_data()\` 与 \`z_threshold\` 语义。

- [ ] **Step 2: 按设计计算 Z 窗口并汇总有效点**

将 \`z_position + out_z_front_offset\` 与 \`z_position - out_z_after_offset\` 变换为索引，排序后夹在帧列表范围内。遍历窗口内的有效行，汇总所有行的原始 \`V_Axis_Min\` 最小值、\`V_Axis_Max\` 最大值、\`H_Axis\` 最小值；仅在 \`origin_pos[1] <= H_Axis <= origin_pos[1] + max_limit_pos[0]\` 的行中汇总 \`band_y_max\`。

- [ ] **Step 3: 明确无数据和错误输入的返回**

窗口无效、无有效行或点云 X 几何不完整时返回默认几何；不在助手中保存喷涂状态、修改帧队列或做轴限位。

- [ ] **Step 4: 静态检查助手**

Run: \`python -m py_compile model/motionplan/motionutil/XNUpdown2FrameSearchHelper.py\`

Expected: 退出码 0；助手未引用 \`outside_total_cycles\`、未写入 PLC。

### Task 2: 新增两组独立按帧状态机

**Files:**
- Create: \`model/motionplan/MotionXNUpdown2FramePlanning.py\`
- Read: \`model/motionplan/MotionXNUpdown4FramePlanning.py\`, \`model/motionplan/MotionToTarget.py\`, \`model/motionplan/MachineAxisMap.py\`

**Interfaces:**
- Consumes: \`auto_xn_updown2_move(machine_cfg, runtime_cfg, plc_data, frame_queue_manager)\`，\`request_safe_return(machine_cfg, runtime_cfg, plc_data)\`，\`reset_motion_state(sn, preserve_safe_return=False)\`。
- Produces: 四轴 \`{y1, x1, y2, x2}\` 的限位 \`AxisData\` 命令，以及安全返回接口的 \`(axis_cmds, all_ready)\`。

- [ ] **Step 1: 定义每 SN、每组的状态与不变量**

为每组保存 \`phase\`、\`x_direction\`、\`x_min_target\`、\`x_max_target\`、\`y_target\`。状态仅允许 \`return_safe_x\`、\`return_safe_y\`、\`positioning\`、\`reciprocating\`、\`retract_for_y\`、\`reposition_y\`；任何 \`Status=1\` 只能由 \`reciprocating\` 或 \`retract_for_y\` 产生。

- [ ] **Step 2: 实现配置、反馈、限位与故障收敛助手**

读取 \`origin_pos\`、Y/X 限位与安全位、PLC 实际位置和速度配置；所有可转换错误、轴映射错误、反馈缺失、原点长度不足与几何缺失都记录带 SN 的错误，关闭该组喷枪并进入该组两阶段安全返回。X/Y 目标分别用 \`clamp_to_limit_yx\`，速度用 \`build_axis(..., max_speed)\`；保持显式 0。

- [ ] **Step 3: 从原始几何构建组目标和资格**

先以 \`raw_x_max - raw_x_min > x_range\` 判定两组共享的 X 资格，再计算并限位：

\`\`\`python
x_min = raw_x_min - out_front_x_offset - x_position
x_max = raw_x_max - out_after_x_offset - x_position
y1 = raw_y_min - out_down_y_offset - origin_pos[0]
y2 = band_y_max + out_up_y_offset - origin_pos[1]
\`\`\`

Y2 缺少 band 数据或绝对目标大于 \`origin_pos[1] + max_limit_pos[0]\` 时只让第二组安全返回，不影响第一组。

- [ ] **Step 4: 实现首次定位和持续往复**

首次有效目标进入 \`positioning\`：X 去最小目标、Y 去计算目标，均为 \`Status=0\`；两轴均在 \`spray_pos_tolerance\` 内后进入 \`reciprocating\`。往复状态到端点切换方向，使用 \`x_recip_speed\` 和 \`Status=1\`；Y 以 \`y_pos_speed\` 随新目标更新且不阻断 X。每组独立判定与输出。

- [ ] **Step 5: 实现大跨度 Y 互锁**

仅在原状态为 \`reciprocating\` 且新目标变更时，Y1 使用 \`current_y1 > next_y1 + 2 * out_down_y_offset\`，Y2 使用 \`current_y2 < next_y2 + 2 * out_up_y_offset\`。触发后在 \`retract_for_y\` 让 Y 保持、X 带粉以往复速度回最小值；该周期 X 到位即关闭粉并切入 \`reposition_y\`；Y 到位才恢复往复与开粉。

- [ ] **Step 6: 实现独立两阶段安全返回和复位**

每组先在 \`return_safe_x\` 关闭粉、Y 保持，X 用 \`x_pos_speed\` 到 X 安全位；仅该组 X 到位后进入 \`return_safe_y\`，X 保持安全位、Y 用 \`y_pos_speed\` 到 Y 安全位。两组都到位才返回 \`all_ready=True\` 并清空设备状态；复位只能清除旧往复/喷涂锁存，保留正在进行的安全阶段。

- [ ] **Step 7: 静态检查规划器**

Run: \`python -m py_compile model/motionplan/MotionXNUpdown2FramePlanning.py\`

Expected: 退出码 0；代码中不存在基于 \`outside_total_cycles\` 的停止条件。

### Task 3: 接入按帧外层与模式边界

**Files:**
- Modify: \`model/motionplan/MotionFrameByFramePlanning.py\`
- Read: \`model/motionplan/MotionCleaningPlanning.py\`

**Interfaces:**
- Consumes: 新规划器的自动、安全返回与复位接口。
- Produces: \`xn_updown2\` 在自动模式走专用规划；禁用、强制停机、清理结束、手动模式和关闭下降沿走专用安全返回或复位。

- [ ] **Step 1: 初始化并在自动模式调度专用规划器**

导入 \`MotionXNUpdown2FramePlanning\` 并在构造函数初始化。自动模式中 \`machine_type == "xn_updown2"\` 时调用 \`auto_xn_updown2_move()\`，不再落入通用 \`move_to_origin_safe()\`。

- [ ] **Step 2: 接入禁用与外层安全返回**

在 \`not device_operate_enabled or should_return_safe\` 分支中，对 \`xn_updown2\` 调用 \`request_safe_return()\`，并将其 \`all_ready\` 写入 \`device_returning_to_origin\` 与 \`device_origin_complete\`。关闭下降沿、总使能、雷达异常、采数超时与伺服异常均通过现有外层分支到达此接口。

- [ ] **Step 3: 接入清理和手动边界复位**

清理模式本身仍调用既有 \`cleaning_planner.build_device_axis_cmds()\`；进入清理和手动模式时通知新规划器清理旧喷涂状态，清理结束时由外层安全返回分支执行专用两阶段返回。手动模式的 \`xn_updown2\` 不产生专用运动，走安全返回。

- [ ] **Step 4: 静态检查接入点**

Run: \`python -m py_compile model/motionplan/MotionFrameByFramePlanning.py\`

Expected: 退出码 0；\`xn_updown2\` 自动路径只调用新规划器，清理动作本身未被替换。

### Task 4: 按设计进行无落盘验证

**Files:**
- Read: \`model/tomls/MachineConfig1.toml\`, \`model/tomls/ProcessConfig.toml\`, \`model/tomls/SprayConfig.toml\`

**Interfaces:**
- Consumes: 构造的帧、PLC \`AxisList\`、SN0/SN2 配置。
- Produces: 不写文件的断言结果和静态检查输出。

- [ ] **Step 1: 解析全部涉及 TOML 并编译、导入目标模块**

Run: \`python -c "from model.utils.TomlLoader import TomlLoader; [TomlLoader.load(p) for p in ('model/tomls/MachineConfig1.toml','model/tomls/ProcessConfig.toml','model/tomls/SprayConfig.toml')]; from model.motionplan.MotionXNUpdown2FramePlanning import MotionXNUpdown2FramePlanning; from model.motionplan.motionutil.XNUpdown2FrameSearchHelper import XNUpdown2FrameSearchHelper"\`

Expected: 退出码 0。

- [ ] **Step 2: 执行不落盘状态机冒烟推演**

使用内存中的 fake frame/row/queue/plc 构造左右设备的有效几何、短 X 行程、缺失 Y2 区域与两个组不同反馈。检查：(a) 原始 X 条件即使两个限位目标重合仍允许定位；(b) 两组首定位互不等待；(c) 互锁时 X 带粉回最小、到位关粉、Y 到位再开粉；(d) X1/X2 各自到安全位后才放行 Y1/Y2；(e) reset 后重新进入 positioning。

- [ ] **Step 3: 执行差异、行宽与残留检查**

Run: \`git diff --check\`

Run: \`python -c "from pathlib import Path; files=['model/motionplan/MotionXNUpdown2FramePlanning.py','model/motionplan/motionutil/XNUpdown2FrameSearchHelper.py','model/motionplan/MotionFrameByFramePlanning.py']; bad=[f'{p}:{i}' for p in files for i,line in enumerate(Path(p).read_text(encoding='utf-8').splitlines(),1) if len(line)>180]; print('\\n'.join(bad)); raise SystemExit(bool(bad))"\`

Run: \`rg -n "outside_total_cycles|xn_updown2" model/motionplan/MotionXNUpdown2FramePlanning.py model/motionplan/MotionFrameByFramePlanning.py\`

Expected: diff 检查和行宽检查退出码 0；残留搜索显示 \`xn_updown2\` 专用调度，且新规划器不把 \`outside_total_cycles\` 作为结束条件。

## Plan Self-Review

### Spec coverage

- 设备 SN/方向/轴映射：全局约束、Task 3 和既有 \`MachineAxisMap\` 最终校验。
- Z 窗口、原始 X 资格、Y1/Y2 几何：Task 1 与 Task 2 Step 3。
- 两组独立首次定位、往复、Y 跟随和大跨度互锁：Task 2 Step 1、4、5。
- 每组 X 后 Y 的安全返回及设备级 all_ready：Task 2 Step 6 与 Task 3 Step 2。
- 关闭、强制停机、清理、手动、模式切换、重开复位：Task 3 Step 2-3。
- 限位、显式 0、异常日志、无单测验证和实机未验证边界：全局约束、Task 2 Step 2、Task 4。

未发现未覆盖的设计要求。

### Placeholder scan

已检查本文；不含 \`TBD\`、\`TODO\`、\`implement later\`、\`fill in details\` 或“适当处理”一类占位说明。

### Type consistency

Task 1 的 \`XNUpdown2FrameGeometry\` 是 Task 2 的唯一几何输入；Task 2 的三个公开接口是 Task 3 的唯一委派接口；所有 PLC 输出都为按轴名的 \`AxisData\` 字典，最终由既有 \`apply_device_axes_to_list()\` 写入 \`AxisList\`。

## Execution Handoff

用户已明确选择 inline execution：下一步使用 \`superpowers:executing-plans\` 逐项实施本计划，不创建子代理或 worktree。

