# 伺服二维点云范围报文设计

## 1. 目标与替代关系

重新定义按帧策略中左右 `out_2d_servo` 设备的 Y/X 数据语义。每台设备仍占用一个 Y 轴和一个 X 轴，12轴顺序和86字节报文结构保持不变。

本设计取代 `2026-08-27-out-2d-servo-frame-motion-design.md` 中以下伺服二维规则：

- 六枪区间选择和 X.Status 选枪掩码
- Y 轴固定范围往复及方向状态
- `frame_idle_y_reciprocate_enabled` 对伺服二维 Y 的控制
- 链条状态对伺服二维开枪状态的控制
- 柜体识别和 X 慢进慢退

旧规格中的设备轴顺序、标准 AxisList 写入和规划器永不请求停链仍然有效。

## 2. 设备与报文合同

全局轴索引不变：

- 左伺服二维 SN1：`y=4, x=5`
- 右伺服二维 SN3：`y=10, x=11`

每台伺服二维固定发送两个 `AxisData`：

```text
Y.Pos    = Y_MAX
Y.Speed  = Y_MIN
Y.Status = 0

X.Pos    = X目标位置
X.Speed  = X定位速度
X.Status = 0
```

Y轴字段承载点云范围数据，不代表伺服运动位置或速度。X轴仍是普通运动命令。

规划器固定返回 `(axis_cmds, False)`；第二项是 `stop_chain`，伺服二维永远不请求停链。

## 3. Z 窗口与方向

设备没有 Z 运动轴，窗口中心固定为设备 `z_position`：

```text
start_index = floor((z_position - out_z_front_offset) / z_threshold)
end_index   = floor((z_position + out_z_after_offset) / z_threshold)
```

窗口索引限制在当前方向帧列表范围内：

- 左设备读取 `frame_stack["left"]`
- 右设备读取 `frame_stack["right"]`

遍历窗口内全部帧和全部行。Y与X独立统计，任一方缺少数据不影响另一方输出有效结果。

## 4. Y 点云范围载荷

Y统计只使用非零 `H_Axis`：

```text
Y_MAX = max(all non-zero H_Axis in window)
Y_MIN = min(all non-zero H_Axis in window)
```

有有效Y数据且两个结果都在有符号 Int16 范围 `[-32768, 32767]` 时：

```text
Y = AxisData(Pos=Y_MAX, Speed=Y_MIN, Status=0)
```

下列任一情况使本周期Y发送全零：

- 窗口内没有非零 `H_Axis`
- `Y_MAX` 或 `Y_MIN` 超出 Int16 范围

越界时记录包含设备 SN 和越界值的错误日志：

```text
Y = AxisData(Pos=0, Speed=0, Status=0)
```

Y点云载荷不得经过机械 Y 位置限位或 Y 最大速度限制。最终 AxisList 写入层只复核 Pos/Speed 都是合法 Int16，Status 必须归零。

## 5. X 运动命令

X统计只使用非零 `V_Axis_Min`：

```text
X_MIN = min(all non-zero V_Axis_Min in window)
```

有有效X数据时：

```text
x_target = X_MIN - x_position - out_front_x_offset
X = AxisData(Pos=x_target, Speed=x_pos_speed, Status=0)
```

没有有效X数据时：

```text
X = AxisData(Pos=0, Speed=x_pos_speed, Status=0)
```

X目标和速度在规划器中分别经过X位置限位、X最大速度限制；写入最终 AxisList 时再执行一次相同限制。若X位置最小限位大于0，回0目标按普通运动轴规则限制到合法最小位置。

X不再发送任何喷枪选择或开关状态。链条运行、停止、反向或异常均不改变 `X.Status=0`，也不改变本设计的X定位计算。

X不识别柜体，不读取 `ProcessConfig.toml` 的柜体阈值，也不读取或执行 `frame_x_slow_in_out_enabled`。

## 6. 配置与状态

伺服二维规划器只读取下列设备参数：

- `x_position`
- `z_position`
- `out_front_x_offset`
- `out_z_front_offset`
- `out_z_after_offset`
- `x_pos_speed`
- X轴位置、速度限位

以及公共 `ReadDataConfig.z_threshold`。

伺服二维界面只显示：

- `out_front_x_offset`
- `x_pos_speed`
- `out_z_front_offset`
- `out_z_after_offset`

MachineConfig TOML 中原有的 `spray_num`、`origin_pos`、`y_move_min`、`y_move_max`、`y_pos_speed`、`y_recip_speed` 保留以兼容现有配置文件，但伺服二维规划器完全不读取这些字段。

规划器不再保存任何Y方向或工件状态，因此删除伺服二维的状态复位接口和外层关闭、回安全、手动、清理路径中的对应复位调用。普通二维 `_handle_out_lift` 保持现状。

伺服二维在设备关闭、强制停机、手动模式和清理模式不得调用通用 Y 回安全命令，因为 Y 字段已经不是运动轴语义。这些路径统一使用专用零数据命令：

```text
Y = AxisData(Pos=0, Speed=0, Status=0)
X = AxisData(Pos=0, Speed=x_pos_speed, Status=0)
```

X继续执行位置和速度限制，并只根据X实际位置与现有到位公差判断伺服二维回零是否完成；Y点云载荷不参与到位判断。

## 7. 错误与安全行为

- 无帧栈或窗口无数据：Y发送0/0/0，X使用定位速度回0且Status为0。
- 设备关闭、强停、手动或清理模式：使用专用零数据命令，禁止通用回安全逻辑把 `safe_pos/y_pos_speed` 写入Y载荷。
- Y Int16 越界：仅Y发送0/0/0并记录错误，X仍按自身数据独立计算。
- X数据缺失：仅X回0，Y仍发送有效范围。
- 配置或扫描异常：记录设备 SN 和原因；Y发送0/0/0，X使用可解析的定位速度回0。
- 若X安全命令所需配置也无效：X退化为 Pos=0、Speed=0、Status=0，规划器不得让异常外逸到控制循环。
- 最终 AxisList 层再次保证伺服二维 Y 的点云载荷是合法 Int16，并保证其他运动轴继续执行原有位置和速度限制。

## 8. 修改范围

本阶段修改：

- `model/motionplan/MotionOut2DServoFramePlanning.py`
- `model/motionplan/MotionFrameByFramePlanning.py`
- `model/motionplan/MachineAxisMap.py`
- `view/MachineConfigFrame.py`

不修改：

- `model/plc/MovingFrameData.py` 的 AxisData 字段、12轴数量和86字节结构
- `model/tomls/MachineConfig1.toml` 中保留的旧Y/枪配置字段
- 普通二维 `out_lift` 逻辑
- 其他设备的轴限位和喷枪状态逻辑

## 9. 验证范围

按项目规则不新增或运行单元测试。使用不落盘小数据验证：

- 左右设备分别读取正确方向并写入索引4/5和10/11
- 多帧、多行Y最大最小值和X最小值
- `H_Axis=0`、`V_Axis_Min=0` 的独立过滤
- 只有Y、只有X、X/Y均无数据三种组合
- Y Int16 正负边界和越界归零日志路径
- X公式减去 `x_position` 与 `out_front_x_offset`
- X有数据和无数据时都使用 `x_pos_speed`，Status始终为0
- Y绕过机械位置/速度限位，X仍经过双重限位
- 规划器异常最终不会从控制循环外逸
- 报文仍为12轴和86字节

完成后执行 Python 语法与关键导入、TOML解析、内存场景断言、`git diff --check`、180字符检查和旧六枪/Y往复/慢进慢退残留搜索。上述验证不等同于 PLC 或实机调试。
