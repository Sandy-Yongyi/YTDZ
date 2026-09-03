# xn_updown2 两枪顶底设备按帧运动设计

## 目标与范围

为按帧策略中的左右 `xn_updown2` 设备实现独立运动规划。每台设备包含两组互不阻塞的顶底喷枪轴对：`y1/x1` 与 `y2/x2`。规划器在设备 Z 工作窗口内持续读取点云，动态计算 Y 定位目标和 X 往复范围，并处理首次定位、连续往复、Y 大跨度互锁以及所有安全返回路径。

本设计只新增 `xn_updown2` 的点云搜索与按帧运动规划，并接入现有 `MotionFrameByFramePlanning`。不修改 12 轴顺序、86 字节 PLC 报文、设备界面、其他设备规划器、点云采集流程或清理模式本身的既有动作。

## 设备与配置合同

| SN | 方向 | 轴顺序 | AxisList 索引 |
| --- | --- | --- | --- |
| 0 | left | `y1, x1, y2, x2` | `0, 1, 2, 3` |
| 2 | right | `y1, x1, y2, x2` | `6, 7, 8, 9` |

规划使用现有配置：

- `MachineConfig1.toml`：设备定位、原点、Z/X/Y 偏移、定位与往复速度、位置与速度限位及安全位置。
- `ProcessConfig.toml`：`z_threshold` 与 `x_range`。
- `SprayConfig.toml`：`spray_pos_tolerance`。

`xn_updown2` 的 `max_limit_pos` 按 `[Y1, X1, Y2, X2]` 配置；程序仍兼容旧的 `[Y, X]` 共用格式。`y2_max_limit` 在新格式下读取 `max_limit_pos[2]`，旧格式下回退读取 `max_limit_pos[0]`。`outside_total_cycles` 不参与 `xn_updown2` 按帧规划；点云条件有效期间持续往复。

## 组件边界

新增 `model/motionplan/motionutil/XNUpdown2FrameSearchHelper.py`：

- 读取设备方向对应的帧栈。
- 计算 Z 工作窗口。
- 汇总窗口内有效点的原始 `Xmin`、`Xmax`、`Ymin`。
- 在 Y2 专用 Y 区间内汇总 `Ymax`。
- 返回不含运动状态的几何结果。

新增 `model/motionplan/MotionXNUpdown2FramePlanning.py`：

- 为每台 SN 保存两组独立状态。
- 根据几何结果和 PLC 实际位置生成四轴命令。
- 管理首次定位、X 往复、Y 大跨度互锁、两阶段安全返回及状态复位。
- 提供自动规划、请求安全返回和按 SN 复位的明确入口。

修改 `model/motionplan/MotionFrameByFramePlanning.py`：

- 初始化并调度 `xn_updown2` 规划器。
- 自动模式下把 `xn_updown2` 从通用安全位置分支改为专用规划入口。
- 外层需要设备回安全位置时调用专用两阶段安全返回。
- 设备关闭下降沿、强制停机和模式切换时通知规划器清理旧喷涂状态。

旧 `xn_updown4` 规划器及其搜索助手保持不变。

## 点云窗口与几何计算

### Z 工作窗口

沿用现有按帧索引语义，通过 `z_threshold` 把以下绝对 Z 范围转换成帧索引，并限制在当前方向帧列表内：

```text
z_position + out_z_front_offset
z_position - out_z_after_offset
```

遍历窗口内全部有效行。左设备读取 `frame_stack["left"]`，右设备读取 `frame_stack["right"]`。

### 原始 X 工作资格

汇总窗口内所有有效行：

```text
raw_x_min = min(V_Axis_Min)
raw_x_max = max(V_Axis_Max)
raw_y_min = min(H_Axis)
```

只有原始点云满足下式时，设备才具备顶底往复资格：

```text
raw_x_max - raw_x_min > ProcessConfig.x_range
```

资格判断必须发生在设备坐标换算和轴限位之前。换算或限位后的 X 行程即使小于 `x_range`，也不得撤销已经成立的工作资格。

### X 往复目标

两组枪共用同一原始 X 范围，并分别生成相同的设备轴目标：

```text
x_min_target = raw_x_min - out_front_x_offset - x_position
x_max_target = raw_x_max - out_after_x_offset - x_position
```

两个目标分别经过 X 位置限位。`x_pos_speed` 用于首次定位和安全返回，`x_recip_speed` 用于正常往复及大跨度互锁中的带粉回到最小值。速度经过 X 最大速度限制。

### Y1 目标

```text
y1_target = raw_y_min - out_down_y_offset - origin_pos[0]
```

目标经过 Y 位置限位，运动使用 `y_pos_speed` 并经过 Y 最大速度限制。

### Y2 区域与目标

Y2 只读取以下绝对点云 Y 区间：

```text
origin_pos[1] - out_up_y_offset <= H_Axis <= origin_pos[1] + y2_max_limit
```

区间内存在有效数据时计算：

```text
y2_absolute_target = band_y_max + out_up_y_offset
y2_target = y2_absolute_target - origin_pos[1]
```

若区间内无有效数据，或满足下式，则 Y2 不允许进枪，`y2/x2` 独立进入安全返回：

```text
y2_absolute_target > origin_pos[1] + y2_max_limit
```

Y2 不可进枪不影响仍满足条件的 Y1/X1。

## 每组独立状态机

每台设备为 `y1/x1` 和 `y2/x2` 分别保存状态、当前往复方向、目标和安全返回进度。一组状态不得阻塞另一组。

| 状态 | X 输出 | Y 输出 | Status | 离开条件 |
| --- | --- | --- | --- | --- |
| `return_safe_x` | X 安全位置，定位速度 | 保持当前 Y | 0 | X 到安全位置 |
| `return_safe_y` | 保持 X 安全位置 | Y 安全位置，定位速度 | 0 | Y 到安全位置 |
| `positioning` | `x_min_target`，定位速度 | 当前计算目标，定位速度 | 0 | 本组 X/Y 均到位 |
| `reciprocating` | 在最小值和最大值之间按当前方向运行，往复速度 | 正常时动态跟随计算目标 | 1 | 到达端点反向，或触发退出/互锁 |
| `retract_for_y` | `x_min_target`，往复速度 | 保持当前 Y | 1 | X 到最小值 |
| `reposition_y` | 保持 `x_min_target` | 新 Y 目标，定位速度 | 0 | Y 到新目标 |

所有到位判断使用 `spray_pos_tolerance`。

### 首次启动

设备或单组轴从安全状态首次获得有效喷涂目标时进入 `positioning`。X 定位到 `x_min_target`，Y 定位到计算目标，期间保持关粉。本组 X/Y 都到位后即可独立进入 `reciprocating`，不等待另一组。

### 连续往复与动态目标

X 在原始点云条件持续有效期间一直往复，不按次数停止。每次到达当前端点后切换方向。几何结果可逐周期更新；未触发 Y 大跨度互锁时，Y 使用定位速度跟随新目标，X 保持当前往复方向并持续出粉。

### Y 大跨度互锁

Y1 触发条件：

```text
current_y1 > next_y1_target + 2 * out_down_y_offset
```

Y2 触发条件：

```text
current_y2 < next_y2_target + 2 * out_up_y_offset
```

仅在已经喷涂且目标更新时判断大跨度互锁。首次启动始终走关粉的 `positioning`。

触发后执行：

```text
reciprocating -> retract_for_y -> reposition_y -> reciprocating
```

`retract_for_y` 中 Y 保持当前位置，X 继续出粉并以往复速度回到 `x_min_target`。检测到 X 到位的当前周期关闭出粉并切换到 `reposition_y`。Y 到新目标后重新开粉并恢复 X 往复。

## 安全返回

任何需要本组轴回安全位置的路径都必须执行：

```text
return_safe_x -> return_safe_y
```

先关闭出粉并保持 Y，X 使用定位速度回到 X 安全位置；X 到位后保持 X，Y 才使用定位速度回到 Y 安全位置。`y1/x1` 与 `y2/x2` 独立执行：X1 到位即可释放 Y1，X2 到位即可释放 Y2，不互相等待。

安全返回覆盖：

- 设备关闭或刚关闭。
- 总使能关闭、雷达异常、采数超时或伺服异常造成的强制停机。
- Z 窗口无有效点云。
- 原始 `raw_x_max - raw_x_min` 不大于 `x_range`。
- Y2 区域无有效数据或 Y2 不可进枪；此时只返回第二组。
- 手动模式中 `xn_updown2` 没有专用手动运动。
- 清理模式结束后需要回安全位置。

清理模式本身保留既有清理动作，不改成安全返回；只有外层决定返回安全时才使用上述顺序。

设备级 `all_ready` 只有两组 X/Y 都到安全位置时为真。单组完成安全返回不会阻塞另一组继续喷涂或完成自己的返回过程。

## 复位与模式边界

- 设备关闭下降沿、强制停机或队列失效时，清除旧往复方向、待定位目标、出粉状态和喷涂锁存。
- 清理旧喷涂状态后仍保留当前安全返回阶段，直到 PLC 反馈确认对应轴到位。
- 两组均安全到位后，设备状态回到初始等待；重新开启必须从 `positioning` 开始，不能恢复旧往复方向。
- 点云失效或工件离开窗口不清空共享帧队列，也不影响其他设备状态。
- 规划器不请求停链，链条控制继续由外层既有逻辑负责。

## 异常与降级处理

以下情况记录包含设备 SN 和原因的错误日志，关闭对应喷枪并进入安全返回：

- `origin_pos` 少于两个值。
- Z 窗口无效或点云几何不完整。
- X/Y 配置无法转换为整数。
- 轴映射、实际位置反馈或安全位置读取失败。

允许显式配置值 0，不使用 `value or default` 覆盖合法的 0。所有规划结果在规划器内按轴限位，写入 PLC `AxisList` 前仍经过现有 `MachineAxisMap` 最终校验。

## 验证范围

不新增或运行单元测试。实现后执行：

- TOML 解析和目标模块 Python 编译/导入。
- 使用不落盘构造数据推演左右设备、两组独立状态及边界输入。
- 检查原始 X 资格判断发生在坐标换算和限位之前。
- 检查所有回安全路径均为每组独立的先 X 后 Y。
- 检查 `Status=1` 只出现在 `reciprocating` 和 `retract_for_y`。
- 检查设备关闭及重新开启不会恢复旧往复状态。
- 执行 `git diff --check`、180 字符检查和 `xn_updown2` 调度残留搜索。

静态检查不能替代 PLC 通信、点云现场数据、轴方向、出粉时序和实机运动验证。
