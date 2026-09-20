from model.motionplan.MachineAxisMap import apply_device_axes_to_list, get_axis_map
from model.motionplan.MotionCleaningPlanning import MotionCleaningPlanning
from model.motionplan.MotionOut2DServoFramePlanning import MotionOut2DServoFramePlanning
from model.motionplan.MotionToTarget import MotionToTarget
from model.motionplan.MotionXNUpdown2FramePlanning import MotionXNUpdown2FramePlanning
from model.plc.MovingFrameData import AxisData, SendMovingFrameData, create_axis_list
from model.utils.LoggerUtil import logger


class MotionFrameByFramePlanning:
    """frame_by_frame 模式运动执行。"""

    def __init__(self):
        self.motion_to_target = MotionToTarget()
        self.out_2d_servo_planner = MotionOut2DServoFramePlanning(self.motion_to_target)
        self.xn_updown2_planner = MotionXNUpdown2FramePlanning()
        self.cleaning_planner = MotionCleaningPlanning()
        self._last_safety_fault_mask = 0

    def build_moving_frame(self, proc) -> SendMovingFrameData:
        moving_frame = SendMovingFrameData()
        plc_enable = (proc.plc_data.Operate & 0x01) == 1
        axis_list = create_axis_list()
        servo_alarm = proc.plc_data.Status != 1
        lidar_abnormal = int(getattr(proc, "lidar_status", 0) or 0) in (1, 2, 3)
        raw_data_timeout = bool(getattr(proc, "raw_data_timeout_active", False))
        safety_fault = servo_alarm or lidar_abnormal or raw_data_timeout
        safety_fault_mask = self._build_safety_fault_mask(servo_alarm, lidar_abnormal, raw_data_timeout)
        stop_chain = lidar_abnormal or raw_data_timeout

        if not plc_enable:
            axis_list = self._build_hard_stop_axis_list(proc)
            enable_value = 0
            self._last_safety_fault_mask = 0
            if (int(getattr(proc, "last_operate_state", 0) or 0) & 0x01) != 0:
                logger.info("PLC总使能关闭：所有设备硬停并保持当前位置")
        else:
            if safety_fault and safety_fault_mask != self._last_safety_fault_mask:
                print(f"{self._format_safety_faults(servo_alarm, lidar_abnormal, raw_data_timeout)}：所有设备安全回零")
            self._last_safety_fault_mask = safety_fault_mask

            clean_mode_enabled, clean_mode_ready = self._resolve_clean_mode_state(proc, safety_fault)
            clean_mode_just_closed = self._is_clean_mode_just_closed(proc, clean_mode_enabled)
            if clean_mode_enabled and clean_mode_ready:
                stop_chain = True

            if clean_mode_enabled:
                enable_value = self._build_clean_mode_enable_and_axes(proc, clean_mode_ready, axis_list)
            elif self._is_manual_mode_enabled(proc) and not safety_fault:
                enable_value = self._build_manual_mode_enable_and_axes(proc, axis_list)
            else:
                enable_value, auto_stop_chain = self._build_auto_mode_enable_and_axes(
                    proc=proc,
                    force_return_safe=safety_fault,
                    clean_mode_just_closed=clean_mode_just_closed,
                    axis_list=axis_list,
                )
                stop_chain = stop_chain or auto_stop_chain

        proc.last_operate_state = proc.plc_data.Operate

        moving_frame.AxisList = axis_list
        moving_frame.Enable = enable_value
        moving_frame.Gun_Cont1 = 0
        moving_frame.Gun_Cont2 = 0
        moving_frame.HeartBeat = proc.plc_data.HeartBeat
        moving_frame.Operate = 0 if stop_chain else 0x02
        # bit15 反映全部帧队列中有无非零点云，与设备使能和停链状态无关。
        if self.cleaning_planner.has_any_frame_data(proc.frame_queue_manager):
            moving_frame.Operate |= 1 << 15
        return moving_frame

    def _build_auto_mode_enable_and_axes(self, proc, force_return_safe, clean_mode_just_closed, axis_list):
        """自动模式逐设备规划；故障时通过关闭有效设备位进入安全返回。"""
        enable_value = 0
        stop_chain = False
        effective_operate = 0 if force_return_safe else proc.plc_data.Operate

        for sn in range(proc.num_devices):
            machine_cfg = proc.machine_config.get(str(sn))
            if not machine_cfg or machine_cfg.get("type", "") == "out_lift":
                continue

            runtime_cfg = proc.runtime_machine_config.get(sn, {})
            machine_type = machine_cfg.get("type", "")
            device_bit = sn + 1
            device_operate_enabled = (effective_operate & (1 << device_bit)) != 0
            last_device_operate = (proc.last_operate_state & (1 << device_bit)) != 0
            device_just_closed = (not force_return_safe) and last_device_operate and not device_operate_enabled
            should_return_safe = force_return_safe or self._should_return_safe_before_idle(
                device_operate_enabled=device_operate_enabled,
                device_just_closed=device_just_closed,
                clean_mode_just_closed=clean_mode_just_closed,
                device_returning=proc.device_returning_to_origin[sn],
            )

            if not device_operate_enabled and not should_return_safe:
                self._apply_hold_device_axes(proc, machine_cfg, axis_list)
                proc.device_origin_complete[sn] = True
                proc.device_returning_to_origin[sn] = False
                continue

            if not device_operate_enabled or should_return_safe:
                if device_just_closed:
                    logger.info(f"SN[{sn}] 设备由开启变为关闭：开始安全回零")
                axis_cmds, all_ready = self._build_inactive_device_axes(
                    proc, sn, machine_cfg, runtime_cfg, machine_type, should_return_safe
                )
                proc.device_origin_complete[sn] = all_ready
                device_enable = not all_ready
            else:
                axis_cmds, device_stop_chain = self._build_active_device_axes(
                    proc, sn, machine_cfg, runtime_cfg, machine_type
                )
                stop_chain = stop_chain or device_stop_chain
                device_enable = True

            if axis_cmds:
                apply_device_axes_to_list(proc.machine_config, sn, axis_cmds, axis_list)
            if device_enable:
                enable_value |= 1 << device_bit

        if not force_return_safe:
            enable_value |= 0x01
        return enable_value, stop_chain

    def _build_hard_stop_axis_list(self, proc):
        axis_list = create_axis_list()
        for sn in range(proc.num_devices):
            machine_cfg = proc.machine_config.get(str(sn))
            if not machine_cfg or machine_cfg.get("type") == "out_lift":
                continue
            self._apply_hold_device_axes(proc, machine_cfg, axis_list)
        return axis_list

    @staticmethod
    def _apply_hold_device_axes(proc, machine_cfg, axis_list):
        machine_type = machine_cfg.get("type", "")
        orientation = machine_cfg.get("install_orietation", "left")
        try:
            axis_map = get_axis_map(machine_type, orientation)
        except Exception as exc:
            logger.error(f"SN[{machine_cfg.get('sn', '?')}] 保持当前位置失败，相关轴发送零值: {exc}")
            return
        feedback_axes = getattr(proc.plc_data, "AxisList", None)
        for axis_name, axis_index in axis_map.items():
            if machine_type == "out_2d_servo" and axis_name == "y":
                axis_list[axis_index] = AxisData()
                continue
            try:
                current_pos = 0
                if isinstance(feedback_axes, (list, tuple)) and axis_index < len(feedback_axes):
                    feedback = feedback_axes[axis_index]
                    if hasattr(feedback, "Pos"):
                        current_pos = int(getattr(feedback, "Pos", 0) or 0)
                    elif isinstance(feedback, (list, tuple)) and feedback:
                        current_pos = int(feedback[0] or 0)
                    elif isinstance(feedback, dict):
                        current_pos = int(feedback.get("Pos", 0) or 0)
            except (TypeError, ValueError) as exc:
                logger.error(
                    f"SN[{machine_cfg.get('sn', '?')}] {axis_name}反馈位置无效，当前轴发送零值: {exc}"
                )
                current_pos = 0
            axis_list[axis_index] = AxisData(Pos=current_pos, Speed=0, Status=0)

    @staticmethod
    def _build_safety_fault_mask(servo_alarm, lidar_abnormal, raw_data_timeout):
        return (1 if servo_alarm else 0) | (2 if lidar_abnormal else 0) | (4 if raw_data_timeout else 0)

    @staticmethod
    def _format_safety_faults(servo_alarm, lidar_abnormal, raw_data_timeout):
        reasons = []
        if servo_alarm:
            reasons.append("伺服异常")
        if lidar_abnormal:
            reasons.append("雷达异常")
        if raw_data_timeout:
            reasons.append("采数超时")
        return "、".join(reasons)

    def _build_inactive_device_axes(self, proc, sn, machine_cfg, runtime_cfg, machine_type, should_return_safe):
        """为关闭或正在安全返回的设备生成命令。"""
        if machine_type == "xn_updown2":
            self.xn_updown2_planner.reset_motion_state(sn, preserve_safe_return=True)
            axis_cmds, all_ready = self.xn_updown2_planner.request_safe_return(
                machine_cfg, runtime_cfg, proc.plc_data
            )
            proc.device_returning_to_origin[sn] = not all_ready
        elif machine_type == "out_2d_servo":
            axis_cmds, all_ready = self.out_2d_servo_planner.build_zero_commands(
                machine_cfg, runtime_cfg, proc.plc_data
            )
            proc.device_returning_to_origin[sn] = not all_ready
        elif should_return_safe:
            axis_cmds, all_ready = self.motion_to_target.move_to_origin_safe(
                machine_cfg, runtime_cfg, proc.plc_data
            )
            proc.device_returning_to_origin[sn] = not all_ready
        else:
            axis_cmds = self.motion_to_target.hold_current_position(machine_cfg, proc.plc_data)
            all_ready = proc.device_origin_complete.get(sn, False)

        return axis_cmds, all_ready

    def _build_active_device_axes(self, proc, sn, machine_cfg, runtime_cfg, machine_type):
        """为已开启设备分发对应的自动运动规划器。"""
        proc.device_returning_to_origin[sn] = False
        proc.device_origin_complete[sn] = False

        if machine_type == "xn_updown2":
            axis_cmds = self.xn_updown2_planner.auto_xn_updown2_move(
                machine_cfg, runtime_cfg, proc.plc_data, proc.frame_queue_manager
            )
            device_stop_chain = False
        elif machine_type == "out_2d_servo":
            axis_cmds, device_stop_chain = self.out_2d_servo_planner.auto_out_2d_servo_move(
                machine_cfg, runtime_cfg, proc.plc_data, proc.frame_queue_manager
            )
        else:
            # 尚无自动轨迹的设备统一回安全位置。
            axis_cmds, all_ready = self.motion_to_target.move_to_origin_safe(
                machine_cfg, runtime_cfg, proc.plc_data
            )
            proc.device_returning_to_origin[sn] = not all_ready
            proc.device_origin_complete[sn] = all_ready
            device_stop_chain = False

        return axis_cmds, device_stop_chain

    def _build_clean_mode_enable_and_axes(self, proc, clean_mode_ready: bool, axis_list: list) -> int:
        enable_value = 0x01 | self.cleaning_planner.CLEAN_MODE_BIT
        for sn in range(proc.num_devices):
            machine_cfg = proc.machine_config.get(str(sn))
            if not machine_cfg or machine_cfg.get("type") == "out_lift":
                continue

            runtime_cfg = proc.runtime_machine_config.get(sn, {})
            enable_value = self._handle_clean_mode_device(
                proc=proc,
                sn=sn,
                machine_cfg=machine_cfg,
                runtime_cfg=runtime_cfg,
                clean_mode_ready=clean_mode_ready,
                axis_list=axis_list,
                enable_value=enable_value,
            )

        return enable_value

    def _resolve_clean_mode_state(self, proc, force_disable_all):
        clean_mode_enabled = (not force_disable_all) and self.cleaning_planner.is_clean_mode_enabled(proc.plc_data.Operate)
        clean_mode_ready = clean_mode_enabled and not self.cleaning_planner.has_any_frame_data(proc.frame_queue_manager)
        if clean_mode_enabled and not clean_mode_ready:
            self.cleaning_planner.log_clean_mode_blocked("当前按帧队列中仍有点云，请关闭清理模式")
        return clean_mode_enabled, clean_mode_ready

    def _handle_clean_mode_device(self, proc, sn, machine_cfg, runtime_cfg, clean_mode_ready, axis_list, enable_value):
        if machine_cfg.get("type") == "xn_updown2":
            self.xn_updown2_planner.reset_motion_state(sn)
        if machine_cfg.get("type") == "out_2d_servo":
            axis_cmds, all_ready = self.out_2d_servo_planner.build_zero_commands(
                machine_cfg, runtime_cfg, proc.plc_data
            )
            proc.device_returning_to_origin[sn] = not all_ready
            proc.device_origin_complete[sn] = all_ready
        else:
            proc.device_returning_to_origin[sn] = False
            proc.device_origin_complete[sn] = False
            axis_cmds = self.cleaning_planner.build_device_axis_cmds(machine_cfg, runtime_cfg, clean_mode_ready)
        if axis_cmds:
            apply_device_axes_to_list(proc.machine_config, sn, axis_cmds, axis_list)
        return enable_value | (1 << (sn + 1))

    def _is_clean_mode_just_closed(self, proc, clean_mode_enabled):
        last_clean_mode_enabled = self.cleaning_planner.is_clean_mode_enabled(proc.last_operate_state)
        return last_clean_mode_enabled and not clean_mode_enabled

    @staticmethod
    def _should_return_safe_before_idle(device_operate_enabled, device_just_closed, clean_mode_just_closed, device_returning):
        if device_returning:
            return True
        if device_just_closed:
            return True
        if clean_mode_just_closed:
            return True
        return False

    def _is_manual_mode_enabled(self, proc) -> bool:
        spray_mode = int(proc.mode_config.get("spray_mode", 0) or 0)
        return spray_mode == 1

    def _build_manual_mode_enable_and_axes(self, proc, axis_list: list) -> int:
        enable_value = 0x01
        for sn in range(proc.num_devices):
            machine_cfg = proc.machine_config.get(str(sn))
            if not machine_cfg:
                continue
            if machine_cfg.get("type") == "out_lift":
                continue
            runtime_cfg = proc.runtime_machine_config.get(sn, {})
            machine_type = machine_cfg.get("type", "")
            device_bit = sn + 1
            device_operate_enabled = (proc.plc_data.Operate & (1 << device_bit)) != 0
            last_device_operate = (proc.last_operate_state & (1 << device_bit)) != 0
            device_just_closed = last_device_operate and not device_operate_enabled
            should_return_safe = device_just_closed or proc.device_returning_to_origin[sn]
            if not device_operate_enabled and not should_return_safe:
                self._apply_hold_device_axes(proc, machine_cfg, axis_list)
                proc.device_returning_to_origin[sn] = False
                proc.device_origin_complete[sn] = True
                continue
            if device_just_closed:
                logger.info(f"SN[{sn}] 手动模式设备由开启变为关闭：开始安全回零")
            if machine_type == "xn_updown2":
                self.xn_updown2_planner.reset_motion_state(sn, preserve_safe_return=True)
                axis_cmds, all_ready = self.xn_updown2_planner.request_safe_return(
                    machine_cfg, runtime_cfg, proc.plc_data
                )
                proc.device_returning_to_origin[sn] = not all_ready
                proc.device_origin_complete[sn] = all_ready
                if axis_cmds:
                    apply_device_axes_to_list(proc.machine_config, sn, axis_cmds, axis_list)
                if not all_ready:
                    enable_value |= 1 << device_bit
                continue

            if machine_type == "out_2d_servo":
                axis_cmds, all_ready = self.out_2d_servo_planner.build_zero_commands(
                    machine_cfg, runtime_cfg, proc.plc_data
                )
                proc.device_returning_to_origin[sn] = not all_ready
                proc.device_origin_complete[sn] = all_ready
                if axis_cmds:
                    apply_device_axes_to_list(proc.machine_config, sn, axis_cmds, axis_list)
                if not all_ready:
                    enable_value |= 1 << device_bit
                continue

            axis_cmds, all_ready = self.motion_to_target.move_to_origin_safe(machine_cfg, runtime_cfg, proc.plc_data)
            proc.device_returning_to_origin[sn] = not all_ready
            proc.device_origin_complete[sn] = all_ready
            if axis_cmds:
                apply_device_axes_to_list(proc.machine_config, sn, axis_cmds, axis_list)
            if not all_ready:
                enable_value |= 1 << device_bit

        return enable_value
