# 伺服二维按帧运动设计

## 1. 目标与范围

为按帧策略中的左右 `out_2d_servo` 设备实现独立运动规划。每台设备只有一个 Y 轴和一个 X 轴，Y 轴机械安装六把从下到上排列的喷枪；六把枪共用 X/Y 运动，但通过 X 轴命令的 16 位 `Status` 分别控制开关。

本阶段只修改：

- `model/motionplan/MotionOut2DServoFramePlanning.py`
- `model/motionplan/MotionFrameByFramePlanning.py`
- `model/tomls/MachineConfig1.toml`

界面继续使用用户已更新的 `OUT_2D_SERVO_PARAM_KEYS`，不恢复自动计算 Y 范围所需的上下偏移参数，也不修改普通二维 `out_lift` 路径。

## 2. 设备与协议合同

左右伺服二维使用相同枪数和机械原点：

```toml
spray_num = 6
origin_pos = [1000, 1300, 1600, 1900, 2200, 2500]
axis_type = ["y", "x"]
```

全局轴索引保持不变：

- 左伺服二维 SN1：`y=4, x=5`
- 右伺服二维 SN3：`y=10, x=11`

`AxisData.Status` 仍为 `Int16`。只有 X 轴的 `Status` 承载喷枪掩码：

- bit0～bit5：从下到上的第1～6把枪
- `1` 表示开枪，`0` 表示关枪
- bit6～bit15固定为0
- Y轴 `Status` 固定为0

规划器固定返回 `(axis_cmds, False)`，第二项为 `stop_chain`；伺服二维规划器永远不请求停链。

## 3. 配置来源

运动参数优先读取 `runtime_cfg`，未提供时读取 `machine_cfg`：

- `y_move_min`、`y_move_max`：界面直接输入的固定 Y 往复范围
- `y_recip_speed`：Y 往复速度
- `y_pos_speed`：Y 回安全位置速度
- `x_pos_speed`：X 定位和回安全位置速度
- `out_front_x_offset`：X 前定位偏移
- `out_z_front_offset`、`out_z_after_offset`：Z 点云窗口

公共配置来自现有 TOML：

- `ReadDataConfig.z_threshold`：Z 坐标到帧索引的换算步长
- `SprayConfig.spray_pos_tolerance`：Y 端点到位公差
- `SprayConfig.frame_idle_y_reciprocate_enabled`：无有效工作数据时 Y 是否继续往复

本方案不再根据点云 `ymin/ymax` 计算 Y 行程，也不使用 `scan_span`、`gun_distance`、`out_up_y_offset` 或 `out_down_y_offset` 生成往复范围。

## 4. Z 窗口与点云统计

设备没有 Z 运动轴，窗口中心固定为设备 `z_position`：

```text
start_index = floor((z_position - out_z_front_offset) / z_threshold)
end_index   = floor((z_position + out_z_after_offset) / z_threshold)
```

窗口索引必须限制在当前方向帧列表范围内。左设备读取 `frame_stack["left"]`，右设备读取 `frame_stack["right"]`。

遍历窗口内全部帧时：

- `H_Axis == 0` 不参与喷枪 Y 区间命中
- `V_Axis_Min == 0` 不参与全局 X 最小值统计
- 其他有效 `V_Axis_Min` 的最小值作为窗口全局 `xmin`

## 5. 六枪选择

Y 轴往复范围由界面固定输入，并先按 Y 轴位置限位解析为实际端点：

```text
y_target_min = clamp_y(y_move_min)
y_target_max = clamp_y(y_move_max)
```

枪区间筛选和实际 Y 往复必须使用同一组限位后端点。第 `i` 把枪的绝对点云搜索区间为：

```text
gun_y_min[i] = origin_pos[i] + y_target_min
gun_y_max[i] = origin_pos[i] + y_target_max
```

只要 Z 窗口内存在非零 `H_Axis` 落入闭区间 `[gun_y_min[i], gun_y_max[i]]`，就设置：

```text
gun_mask |= 1 << i
```

六把枪独立判断，不要求命中枪连续。最终掩码必须限制为 `gun_mask & 0x003F`。

有效工作数据必须同时满足：

1. 窗口内存在有效全局 `xmin`；
2. `gun_mask != 0`。

只存在 `xmin` 但六个 Y 区间均未命中，或只命中 Y 区间但不存在有效 `xmin`，都按无有效工作数据处理。

## 6. 有数据时的轴命令

X 目标为：

```text
x_target = xmin - out_front_x_offset - x_position
```

其中 `x_position` 读取设备配置。X 使用 `x_pos_speed`，目标和速度分别经过 X 位置限位、X 最大速度限制。

Y 始终在限位后的 `y_target_min` 与 `y_target_max` 之间往复，使用 `y_recip_speed`。每台 SN 独立保存方向状态 `to_max` 或 `to_min`：

- 当前 Y 小于 `y_target_min - spray_pos_tolerance`：目标为 `y_target_max`
- 当前 Y 大于 `y_target_max + spray_pos_tolerance`：目标为 `y_target_min`
- 当前 Y 在范围内且状态尚未初始化：首次目标为 `y_target_max`
- `to_max` 且距离上限不超过公差：立即反向到 `y_target_min`
- `to_min` 且距离下限不超过公差：立即反向到 `y_target_max`

喷枪不等待 X 或 Y 到位。有效工作数据存在时：

- `ChainStatus == "moving_forward"`：`X.Status = gun_mask`
- 链条停止、反向、未知或异常：`X.Status = 0`

链条状态只控制喷枪掩码，不停止 X 定位或 Y 往复。

## 7. 无数据与配置错误

无有效工作数据时：

- X 使用安全位置和 `x_pos_speed`，`X.Status = 0`
- `frame_idle_y_reciprocate_enabled == 1`：Y 按第6节相同方向规则继续在固定范围内往复
- `frame_idle_y_reciprocate_enabled == 0`：Y 使用安全位置和 `y_pos_speed`，并清除该 SN 的 Y 方向状态

下列情况属于配置错误：

- `spray_num != 6`
- `origin_pos` 数量不是6
- `origin_pos` 未严格从低到高排列
- Y 限位后的 `y_target_min >= y_target_max`
- `frame_idle_y_reciprocate_enabled` 不是0或1

配置错误必须记录错误日志、关闭全部喷枪，并让 X/Y 都回安全位置；不得按空闲往复开关继续运动。

## 8. 状态复位与外层集成

`MotionOut2DServoFramePlanning` 提供 `reset_motion_state(sn=None)`：

- 指定 SN 时只删除该设备方向状态
- 未指定 SN 时清空全部伺服二维状态

`MotionFrameByFramePlanning` 在以下边界重置对应伺服二维状态：

- 设备操作位关闭或强制停机
- 设备进入回安全位置流程
- 手动模式
- 清理模式

自动模式调用：

```python
axis_cmds, device_stop_chain = self.out_2d_servo_planner.auto_out_2d_servo_move(...)
```

外层继续通过 `apply_device_axes_to_list()` 写入标准12轴报文，并执行最终位置、速度限制。普通二维 `_handle_out_lift` 保持现状。

## 9. 验证范围

按项目规则不新增或运行单元测试。使用不落盘小数据验证：

- 六把枪分别单独命中、组合命中和全部命中，掩码只使用 bit0～bit5
- 有 `xmin` 但 `gun_mask == 0` 时 X 回安全位置并关枪
- 有效数据且链条正向时开枪，停止或反向时只关枪、轴继续运动
- Y 初始方向、越界方向和端点公差反向
- 无数据且空闲往复开关为0/1时的两种 Y 行为
- 配置错误时 X/Y 回安全位置并关枪
- 左右设备分别读取正确帧方向并写入索引4/5和10/11

完成后执行 Python 语法与关键导入、TOML 解析、86字节报文断言、`git diff --check`、180字符检查和残留搜索。上述静态与冒烟验证不等同于 PLC 或实机运动验证。
