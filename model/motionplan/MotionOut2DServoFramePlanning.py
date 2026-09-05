import math
import os
from dataclasses import dataclass
from model.motionplan.MachineAxisMap import get_axis_position_limits, get_axis_speed_limit
from model.motionplan.MotionToTarget import MotionToTarget
from model.motionplan.motionutil.AxisLimits import build_axis, clamp_to_limit_yx
from model.motionplan.motionutil.FrameSearchHelper import FrameSearchHelper
from model.motionplan.motionutil.FrameXMotionHelper import FrameXMotionHelper
from model.plc.MovingFrameData import AxisData
from model.utils.LoggerUtil import logger
from model.utils.TomlLoader import TomlLoader


INT16_MIN = -32768
INT16_MAX = 32767


@dataclass(frozen=True)
class Servo2DConfig:
    sn: int
    x_position: int
    x_front_offset: int
    x_pos_speed: int
    z_front_offset: int
    z_after_offset: int


@dataclass(frozen=True)
class Servo2DWindowResult:
    y_min: int | None = None
    y_max: int | None = None
    x_min: int | None = None
    start_signature: bool = False
    end_signature: bool = False
    center_has_data: bool = False
    window_empty: bool = True
    x_offset: int = 0


class MotionOut2DServoFramePlanning:
    """伺服二维按帧点云范围报文规划。（底座是伺服二维，需要返回ymax,ymin，X运动目标位置实时计算）"""

    def __init__(self, motion_to_target=None, read_data_cfg=None, spray_cfg=None):
        config_dir = os.path.join(os.getcwd(), "model", "tomls")
        self.read_data_cfg = read_data_cfg if read_data_cfg is not None else TomlLoader.load(os.path.join(config_dir, "ReadDataConfig.toml"))
        self.motion_to_target = motion_to_target if motion_to_target is not None else MotionToTarget()
        self.spray_cfg = spray_cfg if spray_cfg is not None else TomlLoader.load(os.path.join(config_dir, "SprayConfig.toml"))
        self._stages: dict[int, str] = {}

        try:
            self.z_threshold = int(self.read_data_cfg.get("z_threshold", 10) or 0)
            if self.z_threshold <= 0:
                raise ValueError(f"z_threshold 必须大于 0，当前值: {self.z_threshold}")
            self._startup_config_error = ""
        except (TypeError, ValueError) as exc:
            self.z_threshold = 10
            self._startup_config_error = str(exc)

        self.frame_helper = FrameSearchHelper(z_threshold=self.z_threshold)

    def auto_out_2d_servo_move(self, machine_cfg, runtime_cfg, plc_data, frame_queue_manager):
        """返回Y点云载荷、X定位命令和固定为False的停链状态。"""
        sn = self._to_int(machine_cfg.get("sn", 0), 0)
        try:
            if self._startup_config_error:
                raise ValueError(self._startup_config_error)
            config = self._resolve_config(machine_cfg, runtime_cfg)
            slow_enabled = self._get_machine_int(self.spray_cfg, "frame_x_slow_in_out_enabled", 0)
            if slow_enabled not in (0, 1):
                raise ValueError("frame_x_slow_in_out_enabled 必须为0或1")
            if not slow_enabled:
                self.reset_motion_state(sn)
            elif self._stages.get(sn) == "return_safe":
                return self._return_safe(machine_cfg, runtime_cfg, plc_data, sn), False

            result = self._scan_window(machine_cfg, frame_queue_manager, config, slow_enabled=bool(slow_enabled))
            if slow_enabled:
                return self._build_staged_commands(machine_cfg, runtime_cfg, plc_data, config, result), False
            return {
                "y": self._build_y_payload(config.sn, result),
                "x": self._build_x_axis(machine_cfg, config, result.x_min),
            }, False
        except Exception as exc:
            self.reset_motion_state(sn)
            logger.error(f"SN[{sn}] out_2d_servo 配置或规划错误，发送零数据命令: {exc}")
            return self._build_fail_closed_commands(machine_cfg, runtime_cfg, sn), False

    def build_zero_commands(self, machine_cfg, runtime_cfg, plc_data):
        """非自动路径发送Y零载荷和X回零命令，只按X反馈判断到位。"""
        self.reset_motion_state(self._to_int(machine_cfg.get("sn", 0), 0))
        return self._build_zero_commands(machine_cfg, runtime_cfg, plc_data)

    def reset_motion_state(self, sn=None):
        """关闭设备或切换模式时丢弃该设备的旧工件阶段。"""
        if sn is None:
            self._stages.clear()
        else:
            self._stages.pop(int(sn), None)

    def _return_safe(self, machine_cfg, runtime_cfg, plc_data, sn):
        axis_cmds, ready = self._build_zero_commands(machine_cfg, runtime_cfg, plc_data)
        if ready:
            self.reset_motion_state(sn)
        return axis_cmds

    def _build_staged_commands(self, machine_cfg, runtime_cfg, plc_data, config, result):
        stage = self._stages.get(config.sn, "idle")
        transitions = {
            "idle": (result.start_signature, "start"),
            "start": (result.center_has_data, "middle"),
            "middle": (result.end_signature, "end"),
            "end": (result.window_empty, "return_safe"),
        }
        should_advance, next_stage = transitions[stage]
        if should_advance:
            stage = next_stage
            logger.info(f"SN[{config.sn}] out_2d_servo frame stage={stage}")
        # 每周期最多前进一个阶段，刚进入的阶段先输出一次命令。
        self._stages[config.sn] = stage
        if stage == "return_safe":
            return self._return_safe(machine_cfg, runtime_cfg, plc_data, config.sn)
        return {
            "y": self._build_y_payload(config.sn, result),
            "x": self._build_x_axis(machine_cfg, config, None if stage == "idle" else result.x_min, result.x_offset),
        }

    def _build_zero_commands(self, machine_cfg, runtime_cfg, plc_data):
        """共用回零命令；自动安全返回保留阶段直到X实际到位。"""
        sn = self._to_int(machine_cfg.get("sn", 0), 0)
        try:
            x_speed = self._get_config_int(machine_cfg, runtime_cfg, "x_pos_speed", 0)
            x_cmds, x_ready = self.motion_to_target.move_x_axes_to_target(machine_cfg=machine_cfg, plc_data=plc_data, target=0, speed=x_speed, status=0)
            if "x" not in x_cmds:
                raise ValueError("out_2d_servo 缺少X轴命令")
            return {"y": AxisData(), "x": x_cmds["x"]}, bool(x_ready)
        except Exception as exc:
            logger.error(f"SN[{sn}] out_2d_servo X回零命令错误，使用零状态零速兜底: {exc}")
            return {"y": AxisData(), "x": AxisData()}, False

    def _resolve_config(self, machine_cfg, runtime_cfg):
        return Servo2DConfig(
            sn=self._get_machine_int(machine_cfg, "sn", 0),
            x_position=self._get_machine_int(machine_cfg, "x_position", 0),
            x_front_offset=self._get_config_int(machine_cfg, runtime_cfg, "out_front_x_offset", 0),
            x_pos_speed=self._get_config_int(machine_cfg, runtime_cfg, "x_pos_speed", 0),
            z_front_offset=self._get_config_int(machine_cfg, runtime_cfg, "out_z_front_offset", 0),
            z_after_offset=self._get_config_int(machine_cfg, runtime_cfg, "out_z_after_offset", 0),
        )

    def _scan_window(self, machine_cfg, frame_queue_manager, config, slow_enabled=False):
        frame_stack = getattr(frame_queue_manager, "frame_stack", {}) or {}
        direction = self.frame_helper.get_side_direction(machine_cfg)
        frames = frame_stack.get(direction, []) or []
        if not frames:
            return Servo2DWindowResult()

        z_position = self._get_machine_int(machine_cfg, "z_position", 0)
        window = self.frame_helper.create_window(
            math.floor((z_position - config.z_front_offset) / self.z_threshold),
            math.floor(z_position / self.z_threshold),
            math.floor((z_position + config.z_after_offset) / self.z_threshold),
            len(frames),
        )

        y_min = None
        y_max = None
        x_min = None
        for frame_index in self.frame_helper.iter_window_indices(window.start, window.end):
            frame = self.frame_helper.get_frame_by_index(frames, frame_index)
            for row in getattr(frame, "FrameData", None) or []:
                h_axis = self._to_int(getattr(row, "H_Axis", 0), 0)
                v_axis_min = self._to_int(getattr(row, "V_Axis_Min", 0), 0)
                if h_axis != 0:
                    y_min = h_axis if y_min is None else min(y_min, h_axis)
                    y_max = h_axis if y_max is None else max(y_max, h_axis)
                if v_axis_min != 0:
                    x_min = v_axis_min if x_min is None else min(x_min, v_axis_min)

        if not slow_enabled:
            return Servo2DWindowResult(y_min=y_min, y_max=y_max, x_min=x_min)
        return self._build_stage_geometry(frames, window, config, y_min, y_max, x_min)

    def _build_stage_geometry(self, frames, window, config, y_min, y_max, x_min):
        detect_count = self._get_machine_int(self.spray_cfg, "stage_detect_frame_count", 8)
        if not 0 < detect_count < window.end - window.start + 1:
            raise ValueError("stage_detect_frame_count 必须大于0且小于Z窗口帧数")
        populated = [index for index in range(window.start, window.end + 1)
                     if self.frame_helper.frame_has_data(self.frame_helper.get_frame_by_index(frames, index))]
        offset = 0
        if populated:
            offset = FrameXMotionHelper.resolve_slow_offset(
                max(populated) * self.z_threshold, min(populated) * self.z_threshold,
                window.center * self.z_threshold, config.z_front_offset, config.z_after_offset, config.x_front_offset,
            )
        return Servo2DWindowResult(
            y_min=y_min, y_max=y_max, x_min=x_min,
            start_signature=self.frame_helper.has_start_signature(frames, window, detect_count),
            end_signature=self.frame_helper.has_end_signature(frames, window, detect_count),
            center_has_data=self.frame_helper.frame_has_data(self.frame_helper.get_frame_by_index(frames, window.center)),
            window_empty=not populated, x_offset=offset,
        )

    @staticmethod
    def _build_y_payload(sn, result):
        if result.y_min is None or result.y_max is None:
            return AxisData()
        if not (
            INT16_MIN <= result.y_min <= INT16_MAX
            and INT16_MIN <= result.y_max <= INT16_MAX
        ):
            logger.error(
                f"SN[{sn}] out_2d_servo Y点云范围超出Int16，"
                f"本周期发送零值: y_min={result.y_min}, y_max={result.y_max}"
            )
            return AxisData()
        return AxisData(Pos=result.y_max, Speed=result.y_min, Status=0)

    @staticmethod
    def _build_x_axis(machine_cfg, config, x_min, x_offset=None):
        # None使用固定前偏移；动态偏移0是有效值，不能回退为默认偏移。
        offset = config.x_front_offset if x_offset is None else x_offset
        x_target = 0 if x_min is None else int(x_min) - config.x_position - offset
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
        if x_min_limit > x_max_limit:
            raise ValueError(f"X轴位置限位无效: {x_min_limit} > {x_max_limit}")
        limited_target = clamp_to_limit_yx(x_target, x_min_limit, x_max_limit)
        return build_axis(
            limited_target,
            config.x_pos_speed,
            0,
            get_axis_speed_limit(machine_cfg, "x"),
        )

    def _build_fail_closed_commands(self, machine_cfg, runtime_cfg, sn):
        try:
            config = Servo2DConfig(
                sn=sn,
                x_position=0,
                x_front_offset=0,
                x_pos_speed=self._to_int(
                    runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 0)),
                    0,
                ),
                z_front_offset=0,
                z_after_offset=0,
            )
            return {"y": AxisData(), "x": self._build_x_axis(machine_cfg, config, None)}
        except Exception as exc:
            logger.error(f"SN[{sn}] out_2d_servo 零数据命令配置无效，使用全零兜底: {exc}")
            return {"y": AxisData(), "x": AxisData()}

    @staticmethod
    def _get_machine_int(machine_cfg, key, default=0):
        value = machine_cfg.get(key, default)
        try:
            return int(value if value is not None else default)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{key} 必须为整数，当前值: {value}") from exc

    @staticmethod
    def _get_config_int(machine_cfg, runtime_cfg, key, default=0):
        value = runtime_cfg.get(key, machine_cfg.get(key, default))
        try:
            return int(value if value is not None else default)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{key} 必须为整数，当前值: {value}") from exc

    @staticmethod
    def _to_int(value, default=0):
        try:
            return int(value if value is not None else default)
        except (TypeError, ValueError):
            return int(default)
