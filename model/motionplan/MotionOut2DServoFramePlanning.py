import math
import os
from dataclasses import dataclass
from model.motionplan.MachineAxisMap import get_axis_position_limits, get_axis_speed_limit
from model.motionplan.MotionToTarget import MotionToTarget
from model.motionplan.motionutil.AxisLimits import build_axis, clamp_to_limit_yx
from model.motionplan.motionutil.FrameSearchHelper import FrameSearchHelper
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


class MotionOut2DServoFramePlanning:
    """伺服二维按帧点云范围报文规划。"""

    def __init__(self, motion_to_target=None, read_data_cfg=None):
        config_dir = os.path.join(os.getcwd(), "model", "tomls")
        self.read_data_cfg = read_data_cfg if read_data_cfg is not None else TomlLoader.load(os.path.join(config_dir, "ReadDataConfig.toml"))
        self.motion_to_target = motion_to_target if motion_to_target is not None else MotionToTarget()

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
        del plc_data
        sn = self._to_int(machine_cfg.get("sn", 0), 0)
        try:
            if self._startup_config_error:
                raise ValueError(self._startup_config_error)
            config = self._resolve_config(machine_cfg, runtime_cfg)
            result = self._scan_window(machine_cfg, frame_queue_manager, config)
            return {
                "y": self._build_y_payload(config.sn, result),
                "x": self._build_x_axis(machine_cfg, config, result.x_min),
            }, False
        except Exception as exc:
            logger.error(f"SN[{sn}] out_2d_servo 配置或规划错误，发送零数据命令: {exc}")
            return self._build_fail_closed_commands(machine_cfg, runtime_cfg, sn), False

    def build_zero_commands(self, machine_cfg, runtime_cfg, plc_data):
        """非自动路径发送Y零载荷和X回零命令，只按X反馈判断到位。"""
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

    def _scan_window(self, machine_cfg, frame_queue_manager, config):
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

        return Servo2DWindowResult(y_min=y_min, y_max=y_max, x_min=x_min)

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
    def _build_x_axis(machine_cfg, config, x_min):
        x_target = 0 if x_min is None else int(x_min) - config.x_position - config.x_front_offset
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
