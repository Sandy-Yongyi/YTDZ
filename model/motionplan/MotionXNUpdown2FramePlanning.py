import os
from dataclasses import dataclass, field

from model.motionplan.MachineAxisMap import (
    get_axis_map,
    get_axis_position_limits,
    get_axis_safe_pos,
    get_axis_speed_limit,
)
from model.motionplan.motionutil.AxisLimits import build_axis, clamp_to_limit_yx
from model.motionplan.motionutil.XNUpdown2FrameSearchHelper import (
    XNUpdown2FrameGeometry,
    XNUpdown2FrameSearchHelper,
)
from model.utils.LoggerUtil import logger
from model.utils.TomlLoader import TomlLoader


SAFE_PHASES = {"return_safe_x", "return_safe_y"}


@dataclass(frozen=True)
class XNUpdown2GunTarget:
    y_target: int
    x_min_target: int
    x_max_target: int


@dataclass
class XNUpdown2GroupState:
    phase: str = "return_safe_x"
    x_direction: str = "to_max"
    target: XNUpdown2GunTarget | None = None


@dataclass
class XNUpdown2DeviceState:
    groups: dict[int, XNUpdown2GroupState] = field(
        default_factory=lambda: {1: XNUpdown2GroupState(), 2: XNUpdown2GroupState()}
    )


class MotionXNUpdown2FramePlanning:
    """xn_updown2 两组顶底喷枪的独立按帧运动规划。"""

    def __init__(self, process_cfg=None, spray_cfg=None):
        config_dir = os.path.join(os.getcwd(), "model", "tomls")
        self.process_cfg = process_cfg if process_cfg is not None else TomlLoader.load(os.path.join(config_dir, "ProcessConfig.toml"))
        self.spray_cfg = spray_cfg if spray_cfg is not None else TomlLoader.load(os.path.join(config_dir, "SprayConfig.toml"))
        self.z_threshold = self._get_mapping_int(self.process_cfg, "z_threshold", 10)
        self.x_range = self._get_mapping_int(self.process_cfg, "x_range", 0)
        self.tolerance = self._get_mapping_int(self.spray_cfg, "spray_pos_tolerance", 10)
        if self.z_threshold <= 0:
            raise ValueError(f"z_threshold 必须大于 0，当前值: {self.z_threshold}")
        self.search_helper = XNUpdown2FrameSearchHelper(z_threshold=self.z_threshold)
        self._states: dict[int, XNUpdown2DeviceState] = {}

    def reset_motion_state(self, sn=None, preserve_safe_return=False):
        """清除旧喷涂锁存；保留安全返回时不重启已经进行的返回阶段。"""
        if sn is None:
            if preserve_safe_return:
                for device_state in self._states.values():
                    self._reset_device_state(device_state, preserve_safe_return=True)
            else:
                self._states.clear()
            return

        sn = int(sn)
        if not preserve_safe_return:
            self._states.pop(sn, None)
            return
        self._reset_device_state(self._states.setdefault(sn, XNUpdown2DeviceState()), preserve_safe_return=True)

    def auto_xn_updown2_move(self, machine_cfg, runtime_cfg, plc_data, frame_queue_manager):
        """根据当前点云为两组枪生成独立定位、往复或安全返回命令。"""
        sn = self._get_machine_int(machine_cfg, "sn", 0)
        try:
            state = self._states.setdefault(sn, XNUpdown2DeviceState())
            geometry = self.search_helper.get_geometry(machine_cfg, runtime_cfg, frame_queue_manager)
            targets = self._build_targets(machine_cfg, runtime_cfg, geometry)
            if targets is None:
                return self._request_safe_return_for_state(machine_cfg, runtime_cfg, plc_data, state)[0]

            for group_id in (1, 2):
                group_target = targets.get(group_id)
                group_state = state.groups[group_id]
                if group_target is None:
                    self._start_safe_return(group_state)
                    continue
                self._update_group_target(machine_cfg, runtime_cfg, plc_data, group_id, group_state, group_target)

            return self._build_all_group_commands(machine_cfg, runtime_cfg, plc_data, state)[0]
        except Exception as exc:
            logger.error(f"SN[{sn}] xn_updown2 按帧规划异常，关闭喷枪并请求安全返回: {exc}")
            return self._fail_closed_commands(machine_cfg, runtime_cfg, plc_data, sn)

    def request_safe_return(self, machine_cfg, runtime_cfg, plc_data):
        """请求两组枪各自先 X 后 Y 回安全位，返回命令及设备级完成状态。"""
        sn = self._get_machine_int(machine_cfg, "sn", 0)
        try:
            state = self._states.setdefault(sn, XNUpdown2DeviceState())
            return self._request_safe_return_for_state(machine_cfg, runtime_cfg, plc_data, state)
        except Exception as exc:
            logger.error(f"SN[{sn}] xn_updown2 请求安全返回异常: {exc}")
            return self._fail_closed_commands(machine_cfg, runtime_cfg, plc_data, sn), False

    def _request_safe_return_for_state(self, machine_cfg, runtime_cfg, plc_data, state):
        for group_state in state.groups.values():
            if group_state.phase not in SAFE_PHASES:
                self._start_safe_return(group_state)

        axis_cmds, group_ready = self._build_all_group_commands(machine_cfg, runtime_cfg, plc_data, state)
        all_ready = all(group_ready.values())
        if all_ready:
            sn = self._get_machine_int(machine_cfg, "sn", 0)
            self._states.pop(sn, None)
        return axis_cmds, all_ready

    def _build_targets(self, machine_cfg, runtime_cfg, geometry: XNUpdown2FrameGeometry):
        if not geometry.has_data or None in (geometry.raw_x_min, geometry.raw_x_max, geometry.raw_y_min):
            return None

        raw_x_min = int(geometry.raw_x_min or 0)
        raw_x_max = int(geometry.raw_x_max or 0)
        # 顶底资格只能使用原始点云范围，不能由坐标换算或限位结果撤销。
        if raw_x_max - raw_x_min <= self.x_range:
            return None

        origin_pos = self._get_int_list(machine_cfg, "origin_pos")
        if len(origin_pos) < 2:
            raise ValueError("origin_pos 至少需要 y1/y2 两个值")

        x_position = self._get_runtime_int(machine_cfg, runtime_cfg, "x_position", 0)
        front_offset = self._get_runtime_int(machine_cfg, runtime_cfg, "out_front_x_offset", 100)
        after_offset = self._get_runtime_int(machine_cfg, runtime_cfg, "out_after_x_offset", 100)
        down_offset = self._get_runtime_int(machine_cfg, runtime_cfg, "out_down_y_offset", 100)
        up_offset = self._get_runtime_int(machine_cfg, runtime_cfg, "out_up_y_offset", 100)

        raw_x_min_target = raw_x_min - front_offset - x_position
        raw_x_max_target = raw_x_max - after_offset - x_position
        x_targets = {}
        for group_id in (1, 2):
            x_name = f"x{group_id}"
            x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, x_name)
            x_targets[group_id] = (
                clamp_to_limit_yx(raw_x_min_target, x_min_limit, x_max_limit),
                clamp_to_limit_yx(raw_x_max_target, x_min_limit, x_max_limit),
            )

        y1_target = self._clamp_target(machine_cfg, "y1", int(geometry.raw_y_min or 0) - down_offset - origin_pos[0])

        targets: dict[int, XNUpdown2GunTarget | None] = {
            1: XNUpdown2GunTarget(y1_target, *x_targets[1]),
            2: None,
        }
        if geometry.band_y_max is None:
            return targets

        y2_absolute_target = int(geometry.band_y_max) + up_offset
        _, y2_max_limit = get_axis_position_limits(machine_cfg, "y2")
        y2_limit = origin_pos[1] + y2_max_limit
        if y2_absolute_target > y2_limit:
            return targets

        y2_target = self._clamp_target(machine_cfg, "y2", y2_absolute_target - origin_pos[1])
        targets[2] = XNUpdown2GunTarget(y2_target, *x_targets[2])
        return targets

    def _update_group_target(self, machine_cfg, runtime_cfg, plc_data, group_id, state, next_target):
        previous_target = state.target
        if state.phase in SAFE_PHASES or previous_target is None:
            state.phase = "positioning"
            state.x_direction = "to_max"
            state.target = next_target
            return

        if state.phase == "reciprocating" and previous_target.y_target != next_target.y_target:
            # 已喷涂时才允许进入带粉收 X、关粉重定位 Y 的互锁路径。
            current_y = self._get_axis_pos(machine_cfg, plc_data, f"y{group_id}")
            down_offset = self._get_runtime_int(machine_cfg, runtime_cfg, "out_down_y_offset", 100)
            up_offset = self._get_runtime_int(machine_cfg, runtime_cfg, "out_up_y_offset", 100)
            if group_id == 1 and current_y > next_target.y_target + 2 * down_offset:
                state.phase = "retract_for_y"
            elif group_id == 2 and current_y < next_target.y_target + 2 * up_offset:
                state.phase = "retract_for_y"

        state.target = next_target

    def _build_all_group_commands(self, machine_cfg, runtime_cfg, plc_data, state):
        axis_cmds = {}
        group_ready = {}
        for group_id in (1, 2):
            group_cmds, group_ready[group_id] = self._build_group_commands(
                machine_cfg,
                runtime_cfg,
                plc_data,
                group_id,
                state.groups[group_id],
            )
            axis_cmds.update(group_cmds)
        return axis_cmds, group_ready

    def _build_group_commands(self, machine_cfg, runtime_cfg, plc_data, group_id, state):
        x_name = f"x{group_id}"
        y_name = f"y{group_id}"
        current_x = self._get_axis_pos(machine_cfg, plc_data, x_name)
        current_y = self._get_axis_pos(machine_cfg, plc_data, y_name)

        if state.phase in SAFE_PHASES or state.target is None:
            return self._build_safe_group_commands(machine_cfg, runtime_cfg, group_id, state, current_x, current_y)

        x_pos_speed = self._get_runtime_int(machine_cfg, runtime_cfg, "x_pos_speed", 300)
        x_recip_speed = self._get_runtime_int(machine_cfg, runtime_cfg, "x_recip_speed", 100)
        y_pos_speed = self._get_runtime_int(machine_cfg, runtime_cfg, "y_pos_speed", 100)
        target = state.target

        if state.phase == "positioning":
            axis_cmds = {
                x_name: self._build_axis(machine_cfg, x_name, target.x_min_target, x_pos_speed, 0),
                y_name: self._build_axis(machine_cfg, y_name, target.y_target, y_pos_speed, 0),
            }
            if self._has_arrived(current_x, target.x_min_target) and self._has_arrived(current_y, target.y_target):
                state.phase = "reciprocating"
                state.x_direction = "to_max"
            return axis_cmds, False

        if state.phase == "retract_for_y":
            axis_cmds = {
                y_name: self._build_axis(machine_cfg, y_name, current_y, 0, 0),
                x_name: self._build_axis(machine_cfg, x_name, target.x_min_target, x_recip_speed, 1),
            }
            if self._has_arrived(current_x, target.x_min_target):
                state.phase = "reposition_y"
                axis_cmds[x_name] = self._build_axis(machine_cfg, x_name, target.x_min_target, 0, 0)
            return axis_cmds, False

        if state.phase == "reposition_y":
            axis_cmds = {
                x_name: self._build_axis(machine_cfg, x_name, target.x_min_target, x_pos_speed, 0),
                y_name: self._build_axis(machine_cfg, y_name, target.y_target, y_pos_speed, 0),
            }
            if self._has_arrived(current_y, target.y_target):
                state.phase = "reciprocating"
                state.x_direction = "to_max"
            return axis_cmds, False

        if state.phase != "reciprocating":
            raise ValueError(f"未知组状态: {state.phase}")

        if state.x_direction == "to_max" and self._has_arrived(current_x, target.x_max_target):
            state.x_direction = "to_min"
        elif state.x_direction == "to_min" and self._has_arrived(current_x, target.x_min_target):
            state.x_direction = "to_max"

        x_target = target.x_max_target if state.x_direction == "to_max" else target.x_min_target
        return {
            x_name: self._build_axis(machine_cfg, x_name, x_target, x_recip_speed, 1),
            y_name: self._build_axis(machine_cfg, y_name, target.y_target, y_pos_speed, 0),
        }, False

    def _build_safe_group_commands(self, machine_cfg, runtime_cfg, group_id, state, current_x, current_y):
        x_name = f"x{group_id}"
        y_name = f"y{group_id}"
        x_safe_target = self._safe_target(machine_cfg, x_name)
        y_safe_target = self._safe_target(machine_cfg, y_name)
        x_pos_speed = self._get_runtime_int(machine_cfg, runtime_cfg, "x_pos_speed", 300)
        y_pos_speed = self._get_runtime_int(machine_cfg, runtime_cfg, "y_pos_speed", 100)

        if state.phase == "return_safe_x":
            # 每组独立完成 X 安全位后，才允许本组 Y 离开当前位置。
            if self._has_arrived(current_x, x_safe_target):
                state.phase = "return_safe_y"
            else:
                return {
                    x_name: self._build_axis(machine_cfg, x_name, x_safe_target, x_pos_speed, 0),
                    y_name: self._build_axis(machine_cfg, y_name, current_y, 0, 0),
                }, False

        y_arrived = self._has_arrived(current_y, y_safe_target)
        return {
            x_name: self._build_axis(machine_cfg, x_name, x_safe_target, 0, 0),
            y_name: self._build_axis(machine_cfg, y_name, y_safe_target, 0 if y_arrived else y_pos_speed, 0),
        }, y_arrived

    def _fail_closed_commands(self, machine_cfg, runtime_cfg, plc_data, sn):
        try:
            state = self._states.setdefault(sn, XNUpdown2DeviceState())
            for group_state in state.groups.values():
                self._start_safe_return(group_state)
            return self._build_all_group_commands(machine_cfg, runtime_cfg, plc_data, state)[0]
        except Exception as fallback_exc:
            logger.error(f"SN[{sn}] xn_updown2 安全返回兜底失败，保持无轴命令: {fallback_exc}")
            return {}

    @staticmethod
    def _start_safe_return(state):
        state.phase = "return_safe_x"
        state.x_direction = "to_max"
        state.target = None

    @staticmethod
    def _reset_device_state(device_state, preserve_safe_return):
        for group_state in device_state.groups.values():
            if preserve_safe_return and group_state.phase in SAFE_PHASES:
                group_state.x_direction = "to_max"
                group_state.target = None
            else:
                MotionXNUpdown2FramePlanning._start_safe_return(group_state)

    def _get_axis_pos(self, machine_cfg, plc_data, axis_name):
        axis_map = get_axis_map(machine_cfg.get("type", ""), machine_cfg.get("install_orietation", "left"))
        if axis_name not in axis_map:
            raise ValueError(f"轴映射缺少 {axis_name}")
        axis_list = getattr(plc_data, "AxisList", None)
        axis_index = axis_map[axis_name]
        if not isinstance(axis_list, (list, tuple)) or axis_index >= len(axis_list):
            raise ValueError(f"AxisList 缺少 {axis_name} 的反馈")
        axis_item = axis_list[axis_index]
        if hasattr(axis_item, "Pos"):
            return self._to_int(getattr(axis_item, "Pos", 0), axis_name)
        if isinstance(axis_item, (list, tuple)) and axis_item:
            return self._to_int(axis_item[0], axis_name)
        if isinstance(axis_item, dict):
            return self._to_int(axis_item.get("Pos", 0), axis_name)
        raise ValueError(f"{axis_name} 的反馈格式无效")

    def _build_axis(self, machine_cfg, axis_name, target, speed, status):
        min_limit, max_limit = get_axis_position_limits(machine_cfg, axis_name)
        limited_target = clamp_to_limit_yx(int(target), min_limit, max_limit)
        return build_axis(
            limited_target,
            int(speed),
            int(status),
            get_axis_speed_limit(machine_cfg, axis_name),
        )

    def _clamp_target(self, machine_cfg, axis_name, target):
        min_limit, max_limit = get_axis_position_limits(machine_cfg, axis_name)
        return clamp_to_limit_yx(int(target), min_limit, max_limit)

    def _safe_target(self, machine_cfg, axis_name):
        return self._clamp_target(machine_cfg, axis_name, get_axis_safe_pos(machine_cfg, axis_name))

    def _has_arrived(self, current, target):
        return abs(int(current) - int(target)) <= self.tolerance

    @staticmethod
    def _get_runtime_int(machine_cfg, runtime_cfg, key, default):
        if isinstance(runtime_cfg, dict) and key in runtime_cfg:
            value = runtime_cfg[key]
        else:
            value = machine_cfg.get(key, default)
        return MotionXNUpdown2FramePlanning._to_int(value if value is not None else default, key)

    @staticmethod
    def _get_machine_int(machine_cfg, key, default):
        return MotionXNUpdown2FramePlanning._to_int(machine_cfg.get(key, default), key)

    @staticmethod
    def _get_mapping_int(mapping, key, default):
        return MotionXNUpdown2FramePlanning._to_int(mapping.get(key, default), key)

    @staticmethod
    def _get_int_list(machine_cfg, key):
        values = machine_cfg.get(key, [])
        if not isinstance(values, (list, tuple)):
            raise ValueError(f"{key} 必须为列表")
        return [MotionXNUpdown2FramePlanning._to_int(value, key) for value in values]

    @staticmethod
    def _to_int(value, key):
        try:
            return int(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{key} 必须为整数，当前值: {value}") from exc
