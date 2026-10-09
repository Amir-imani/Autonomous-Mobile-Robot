"""Bidirectional ROS 2 binary serial bridge for the RP2040 motor controller.

Wire protocol shared with the Pico firmware:

Command, Pi 5 -> Pico::

    0xAA 0xBB | RobotCommand (16 bytes) | CRC16 low | CRC16 high

Telemetry, Pico -> Pi 5::

    0xAA 0xBB | RobotState (76 bytes) | CRC16 low | CRC16 high

Both payloads are little-endian.  The CRC uses the reflected 0xA001
polynomial and the firmware's 0xFFFF initial value.  The CRC covers only the
payload, exactly matching the Pico implementation.
"""

from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
import math
import struct
import threading
import time
from typing import Deque, Dict, List, Optional, Sequence, Tuple

from geometry_msgs.msg import Twist
import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
from serial import SerialException, SerialTimeoutException
from std_msgs.msg import Bool, Float32MultiArray, Int32, UInt8, UInt32


FRAME_HEADER = b'\xAA\xBB'
CRC_SIZE = 2

COMMAND_PROTOCOL_VERSION = 1
COMMAND_FLAG_DRIVE_ENABLE = 0x01
COMMAND_ALLOWED_FLAGS = COMMAND_FLAG_DRIVE_ENABLE

DIAGNOSTIC_COMMAND_RECEIVED = 1 << 0
DIAGNOSTIC_COMMAND_FRESH = 1 << 1
DIAGNOSTIC_DRIVE_ENABLED = 1 << 2
DIAGNOSTIC_SAFE_STATE_ACTIVE = 1 << 3
DIAGNOSTIC_LEFT_ENCODER_VALID = 1 << 4
DIAGNOSTIC_RIGHT_ENCODER_VALID = 1 << 5
DIAGNOSTIC_ENCODER_DATA_STALE = 1 << 6
DIAGNOSTIC_I2C_FAULT_LATCHED = 1 << 7
DIAGNOSTIC_SYSTEM_POWER_VALID = 1 << 8
DIAGNOSTIC_MOTORS_POWER_VALID = 1 << 9
DIAGNOSTIC_LIDAR_VALID = 1 << 10
DIAGNOSTIC_BENCH_MODE_ACTIVE = 1 << 11

# C++ RobotCommand:
#   uint32_t sequence;
#   float    target_left_rpm;
#   float    target_right_rpm;
#   uint8_t  flags;
#   uint8_t  protocol_version;
#   uint16_t reserved_0;
ROBOT_COMMAND_STRUCT = struct.Struct('<IffBBH')
ROBOT_COMMAND_PAYLOAD_SIZE = 16

# C++ RobotState:
#   uint32_t timestamp_ms;
#   float left_rpm, right_rpm, left_total_revs, right_total_revs;
#   float roll, pitch, yaw, heading_mag;
#   float lidar_distance_mm, lidar_angle_deg;
#   float system_voltage_v, system_current_a;
#   float motors_voltage_v, motors_current_a;
#   int32_t left_encoder_ticks, right_encoder_ticks;
#   uint32_t diagnostic_flags;
#   uint8_t lidar_is_valid, system_status_code;
#   uint16_t reserved_0;
ROBOT_STATE_STRUCT = struct.Struct('<I14fiiIBBH')
ROBOT_STATE_PAYLOAD_SIZE = 76

assert ROBOT_COMMAND_STRUCT.size == ROBOT_COMMAND_PAYLOAD_SIZE
assert ROBOT_STATE_STRUCT.size == ROBOT_STATE_PAYLOAD_SIZE


def crc16_ibm(data: bytes) -> int:
    """Return the CRC used by the firmware (poly 0xA001, init 0xFFFF)."""
    crc = 0xFFFF

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc & 0xFFFF


@dataclass(frozen=True)
class RobotState:
    """Typed representation of the exact 76-byte Pico telemetry payload."""

    timestamp_ms: int
    left_rpm: float
    right_rpm: float
    left_total_revs: float
    right_total_revs: float
    roll: float
    pitch: float
    yaw: float
    heading_mag: float
    lidar_distance_mm: float
    lidar_angle_deg: float
    system_voltage_v: float
    system_current_a: float
    motors_voltage_v: float
    motors_current_a: float
    left_encoder_ticks: int
    right_encoder_ticks: int
    diagnostic_flags: int
    lidar_is_valid: int
    system_status_code: int

    @classmethod
    def from_payload(cls, payload: bytes) -> 'RobotState':
        """Decode and validate one exact-size RobotState payload."""
        if len(payload) != ROBOT_STATE_PAYLOAD_SIZE:
            raise ValueError(
                'RobotState payload must be %d bytes, got %d'
                % (ROBOT_STATE_PAYLOAD_SIZE, len(payload))
            )

        unpacked = ROBOT_STATE_STRUCT.unpack(payload)
        reserved_0 = int(unpacked[20])

        if reserved_0 != 0:
            raise ValueError('RobotState reserved_0 is not zero')

        floating_values = unpacked[1:15]
        if not all(math.isfinite(value) for value in floating_values):
            raise ValueError('RobotState contains a non-finite float')

        lidar_is_valid = int(unpacked[18])
        if lidar_is_valid not in (0, 1):
            raise ValueError('RobotState lidar_is_valid must be 0 or 1')

        return cls(
            timestamp_ms=int(unpacked[0]),
            left_rpm=float(unpacked[1]),
            right_rpm=float(unpacked[2]),
            left_total_revs=float(unpacked[3]),
            right_total_revs=float(unpacked[4]),
            roll=float(unpacked[5]),
            pitch=float(unpacked[6]),
            yaw=float(unpacked[7]),
            heading_mag=float(unpacked[8]),
            lidar_distance_mm=float(unpacked[9]),
            lidar_angle_deg=float(unpacked[10]),
            system_voltage_v=float(unpacked[11]),
            system_current_a=float(unpacked[12]),
            motors_voltage_v=float(unpacked[13]),
            motors_current_a=float(unpacked[14]),
            left_encoder_ticks=int(unpacked[15]),
            right_encoder_ticks=int(unpacked[16]),
            diagnostic_flags=int(unpacked[17]),
            lidar_is_valid=lidar_is_valid,
            system_status_code=int(unpacked[19]),
        )

    def as_float_array(self) -> List[float]:
        """Return the stable /pico/robot_state array layout.

        Indices 0 and 1 intentionally remain the wheel RPM values expected by
        differential_odometry_node.py. Index 12 intentionally remains LiDAR
        validity for lidar_scan_node.py. Exact integer timestamp, status, and
        power pairs are also published on dedicated topics.
        """
        return [
            self.left_rpm,              # 0
            self.right_rpm,             # 1
            self.left_total_revs,       # 2
            self.right_total_revs,      # 3
            self.roll,                  # 4
            self.pitch,                 # 5
            self.yaw,                   # 6
            self.heading_mag,           # 7
            self.lidar_distance_mm,     # 8
            self.lidar_angle_deg,       # 9
            self.system_voltage_v,      # 10; preserves legacy battery voltage
            self.motors_current_a,      # 11; preserves legacy motor current
            float(self.lidar_is_valid), # 12
            float(self.system_status_code),  # 13
            float(self.timestamp_ms),   # 14; exact value is on timestamp topic
            self.system_current_a,      # 15; appended in expanded layout
            self.motors_voltage_v,      # 16; appended in expanded layout
            float(self.left_encoder_ticks),   # 17; exact value has its own topic
            float(self.right_encoder_ticks),  # 18; exact value has its own topic
            float(self.diagnostic_flags),     # 19; exact value has its own topic
        ]

    def as_system_power_array(self) -> List[float]:
        """Return system voltage and current as [V, A]."""
        return [self.system_voltage_v, self.system_current_a]

    def as_motors_power_array(self) -> List[float]:
        """Return motors voltage and current as [V, A]."""
        return [self.motors_voltage_v, self.motors_current_a]


class TelemetryRxState(Enum):
    """States for the incremental, non-blocking telemetry parser."""

    WAIT_HEADER_1 = auto()
    WAIT_HEADER_2 = auto()
    READ_PAYLOAD = auto()
    READ_CRC_LOW = auto()
    READ_CRC_HIGH = auto()


class TelemetryFrameParser:
    """Incrementally parse fixed-size, CRC-protected RobotState frames."""

    PARTIAL_FRAME_TIMEOUT_SECONDS = 0.100

    def __init__(self) -> None:
        self._state = TelemetryRxState.WAIT_HEADER_1
        self._payload = bytearray()
        self._crc_low = 0
        self._last_byte_time: Optional[float] = None

    def reset(self) -> None:
        """Discard a partial frame, for example after reconnection."""
        self._state = TelemetryRxState.WAIT_HEADER_1
        self._payload.clear()
        self._crc_low = 0
        self._last_byte_time = None

    def feed(self, data: bytes) -> Tuple[List[RobotState], int, int]:
        """Consume available bytes without blocking.

        Returns decoded states, CRC failure count, and payload validation
        failure count.
        """
        decoded_states: List[RobotState] = []
        crc_failures = 0
        validation_failures = 0

        if not data:
            return decoded_states, crc_failures, validation_failures

        now = time.monotonic()
        if (
            self._state is not TelemetryRxState.WAIT_HEADER_1
            and self._last_byte_time is not None
            and now - self._last_byte_time > self.PARTIAL_FRAME_TIMEOUT_SECONDS
        ):
            self.reset()

        for byte in data:
            self._last_byte_time = time.monotonic()

            if self._state is TelemetryRxState.WAIT_HEADER_1:
                if byte == FRAME_HEADER[0]:
                    self._state = TelemetryRxState.WAIT_HEADER_2

            elif self._state is TelemetryRxState.WAIT_HEADER_2:
                if byte == FRAME_HEADER[1]:
                    self._payload.clear()
                    self._state = TelemetryRxState.READ_PAYLOAD
                elif byte == FRAME_HEADER[0]:
                    # Preserve an overlapping first header byte.
                    self._state = TelemetryRxState.WAIT_HEADER_2
                else:
                    self._state = TelemetryRxState.WAIT_HEADER_1

            elif self._state is TelemetryRxState.READ_PAYLOAD:
                self._payload.append(byte)
                if len(self._payload) == ROBOT_STATE_PAYLOAD_SIZE:
                    self._state = TelemetryRxState.READ_CRC_LOW

            elif self._state is TelemetryRxState.READ_CRC_LOW:
                self._crc_low = byte
                self._state = TelemetryRxState.READ_CRC_HIGH

            elif self._state is TelemetryRxState.READ_CRC_HIGH:
                received_crc = self._crc_low | (byte << 8)
                calculated_crc = crc16_ibm(bytes(self._payload))

                if received_crc == calculated_crc:
                    try:
                        decoded_states.append(
                            RobotState.from_payload(bytes(self._payload))
                        )
                    except (ValueError, struct.error):
                        validation_failures += 1
                    self.reset()
                else:
                    crc_failures += 1

                    # Recover if the expected CRC bytes are actually the next
                    # complete header after a dropped byte.
                    if self._crc_low == FRAME_HEADER[0] and byte == FRAME_HEADER[1]:
                        self._payload.clear()
                        self._crc_low = 0
                        self._state = TelemetryRxState.READ_PAYLOAD
                    elif byte == FRAME_HEADER[0]:
                        self._payload.clear()
                        self._crc_low = 0
                        self._state = TelemetryRxState.WAIT_HEADER_2
                    else:
                        self.reset()

        return decoded_states, crc_failures, validation_failures


class PicoSerialBridge(Node):
    """Exchange /cmd_vel commands and RobotState telemetry with the Pico."""

    COMMAND_TOPIC = '/cmd_vel'
    STATE_TOPIC = '/pico/robot_state'
    TIMESTAMP_TOPIC = '/pico/timestamp_ms'
    STATUS_TOPIC = '/pico/system_status_code'
    SYSTEM_POWER_TOPIC = '/pico/system_power'
    MOTORS_POWER_TOPIC = '/pico/motors_power'
    JOINT_STATE_TOPIC = '/joint_states'
    ENCODER_LEFT_TOPIC = '/diagnostics/encoder_left'
    ENCODER_RIGHT_TOPIC = '/diagnostics/encoder_right'
    DIAGNOSTIC_FLAGS_TOPIC = '/diagnostics/system_flags'
    SAFE_STATE_TOPIC = '/diagnostics/safe_state_active'
    COMMAND_FRESH_TOPIC = '/diagnostics/command_fresh'
    I2C_FAULT_TOPIC = '/diagnostics/i2c_fault_latched'

    JOINT_NAMES = (
        'left_wheel_joint',
        'right_wheel_joint',
        'laser_servo_joint',
        'camera_tilt_joint',
    )
    REVOLUTIONS_TO_RADIANS = 2.0 * math.pi
    RPM_TO_RADIANS_PER_SECOND = 2.0 * math.pi / 60.0
    LIDAR_FORWARD_ANGLE_DEG = 90.0
    LIDAR_MIN_ANGLE_DEG = 0.0
    LIDAR_MAX_ANGLE_DEG = 180.0
    CAMERA_TILT_MIN_RAD = math.radians(-20.0)
    CAMERA_TILT_MAX_RAD = math.radians(70.0)
    MAX_LIDAR_VELOCITY_INTERVAL_MS = 500

    SERIAL_READ_SIZE = 4096
    MAX_SERIAL_READS_PER_CYCLE = 8
    TELEMETRY_QUEUE_DEPTH = 512
    PUBLISH_PERIOD_SECONDS = 0.002
    WARNING_THROTTLE_SECONDS = 5.0

    def __init__(self) -> None:
        super().__init__('serial_bridge_node')

        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('fallback_port', '/dev/serial0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('cmd_vel_timeout', 2.0)
        self.declare_parameter('wheel_radius', 0.03335)
        self.declare_parameter('wheel_base', 0.28)
        self.declare_parameter('max_wheel_rpm', 100.0)
        self.declare_parameter('camera_tilt_position_rad', 0.0)

        primary_port = str(self.get_parameter('port').value)
        fallback_port = str(self.get_parameter('fallback_port').value)
        self._ports = self._unique_ports((primary_port, fallback_port))
        self._baud_rate = int(self.get_parameter('baud_rate').value)
        self._reconnect_interval = max(
            0.1,
            float(self.get_parameter('reconnect_interval').value),
        )
        self._command_rate_hz = float(
            self.get_parameter('command_rate_hz').value
        )
        self._cmd_vel_timeout = float(
            self.get_parameter('cmd_vel_timeout').value
        )
        self._wheel_radius = float(
            self.get_parameter('wheel_radius').value
        )
        self._wheel_base = float(
            self.get_parameter('wheel_base').value
        )
        self._max_wheel_rpm = float(
            self.get_parameter('max_wheel_rpm').value
        )
        self._camera_tilt_position_rad = float(
            self.get_parameter('camera_tilt_position_rad').value
        )

        self._validate_parameters()
        self._command_period = 1.0 / self._command_rate_hz

        self._state_publisher = self.create_publisher(
            Float32MultiArray,
            self.STATE_TOPIC,
            10,
        )
        self._timestamp_publisher = self.create_publisher(
            UInt32,
            self.TIMESTAMP_TOPIC,
            10,
        )
        self._status_publisher = self.create_publisher(
            UInt8,
            self.STATUS_TOPIC,
            10,
        )
        self._system_power_publisher = self.create_publisher(
            Float32MultiArray,
            self.SYSTEM_POWER_TOPIC,
            10,
        )
        self._motors_power_publisher = self.create_publisher(
            Float32MultiArray,
            self.MOTORS_POWER_TOPIC,
            10,
        )
        self._joint_state_publisher = self.create_publisher(
            JointState,
            self.JOINT_STATE_TOPIC,
            10,
        )
        self._encoder_left_publisher = self.create_publisher(
            Int32,
            self.ENCODER_LEFT_TOPIC,
            10,
        )
        self._encoder_right_publisher = self.create_publisher(
            Int32,
            self.ENCODER_RIGHT_TOPIC,
            10,
        )
        self._diagnostic_flags_publisher = self.create_publisher(
            UInt32,
            self.DIAGNOSTIC_FLAGS_TOPIC,
            10,
        )
        self._safe_state_publisher = self.create_publisher(
            Bool,
            self.SAFE_STATE_TOPIC,
            10,
        )
        self._command_fresh_publisher = self.create_publisher(
            Bool,
            self.COMMAND_FRESH_TOPIC,
            10,
        )
        self._i2c_fault_publisher = self.create_publisher(
            Bool,
            self.I2C_FAULT_TOPIC,
            10,
        )
        self._command_subscription = self.create_subscription(
            Twist,
            self.COMMAND_TOPIC,
            self._cmd_vel_callback,
            10,
        )
        self._publish_timer = self.create_timer(
            self.PUBLISH_PERIOD_SECONDS,
            self._publish_pending_states,
        )

        self._parser = TelemetryFrameParser()

        self._command_lock = threading.Lock()
        self._target_left_rpm = 0.0
        self._target_right_rpm = 0.0
        self._last_valid_cmd_time: Optional[float] = None
        self._next_command_sequence = 0

        self._telemetry_lock = threading.Lock()
        self._pending_states: Deque[RobotState] = deque(
            maxlen=self.TELEMETRY_QUEUE_DEPTH
        )

        self._serial_lock = threading.Lock()
        self._serial: Optional[serial.Serial] = None
        self._connected_port: Optional[str] = None

        self._warning_lock = threading.Lock()
        self._last_warning_times: Dict[str, float] = {}

        self._previous_lidar_timestamp_ms: Optional[int] = None
        self._previous_lidar_position_rad: Optional[float] = None

        self._stop_event = threading.Event()
        self._worker_thread = threading.Thread(
            target=self._serial_worker,
            name='pico_serial_io',
            daemon=True,
        )
        self._worker_thread.start()

        self.get_logger().info(
            'Pico bridge started: ports=%s, baud=%d, command=%s at %.1f Hz, '
            'telemetry=%s, joint_states=%s'
            % (
                ', '.join(self._ports),
                self._baud_rate,
                self.COMMAND_TOPIC,
                self._command_rate_hz,
                self.STATE_TOPIC,
                self.JOINT_STATE_TOPIC,
            )
        )
        self.get_logger().info(
            'camera_tilt_joint uses the configured %.4f rad fallback because '
            'camera tilt feedback is not present in RobotState'
            % self._camera_tilt_position_rad
        )
        self.get_logger().info(
            'Foxglove diagnostics enabled: encoders=%s,%s, flags=%s'
            % (
                self.ENCODER_LEFT_TOPIC,
                self.ENCODER_RIGHT_TOPIC,
                self.DIAGNOSTIC_FLAGS_TOPIC,
            )
        )

    @staticmethod
    def _unique_ports(ports: Sequence[str]) -> Tuple[str, ...]:
        return tuple(dict.fromkeys(port for port in ports if port))

    def _validate_parameters(self) -> None:
        if not self._ports:
            raise ValueError('At least one non-empty serial port is required')
        if self._baud_rate <= 0:
            raise ValueError('baud_rate must be greater than zero')

        positive_values = {
            'command_rate_hz': self._command_rate_hz,
            'cmd_vel_timeout': self._cmd_vel_timeout,
            'wheel_radius': self._wheel_radius,
            'wheel_base': self._wheel_base,
            'max_wheel_rpm': self._max_wheel_rpm,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError('%s must be finite and greater than zero' % name)

        if not math.isfinite(self._camera_tilt_position_rad):
            raise ValueError('camera_tilt_position_rad must be finite')
        if not (
            self.CAMERA_TILT_MIN_RAD
            <= self._camera_tilt_position_rad
            <= self.CAMERA_TILT_MAX_RAD
        ):
            raise ValueError(
                'camera_tilt_position_rad must remain within the URDF joint limits'
            )

        # The Pico declares commands stale after 100 ms.  Require at least two
        # transmitted heartbeats inside that interval.
        if self._command_rate_hz < 20.0:
            raise ValueError(
                'command_rate_hz must be at least 20 Hz for the Pico watchdog'
            )

    def _cmd_vel_callback(self, message: Twist) -> None:
        """Convert a valid differential-drive Twist into bounded wheel RPM."""
        try:
            linear_velocity = float(message.linear.x)
            angular_velocity = float(message.angular.z)

            if not (
                math.isfinite(linear_velocity)
                and math.isfinite(angular_velocity)
            ):
                raise ValueError('cmd_vel contains a non-finite value')

            half_track = 0.5 * self._wheel_base
            left_linear_velocity = (
                linear_velocity - angular_velocity * half_track
            )
            right_linear_velocity = (
                linear_velocity + angular_velocity * half_track
            )

            metres_per_second_to_rpm = (
                60.0 / (2.0 * math.pi * self._wheel_radius)
            )
            left_rpm = left_linear_velocity * metres_per_second_to_rpm
            right_rpm = right_linear_velocity * metres_per_second_to_rpm

            peak_rpm = max(abs(left_rpm), abs(right_rpm))
            if peak_rpm > self._max_wheel_rpm:
                scale = self._max_wheel_rpm / peak_rpm
                left_rpm *= scale
                right_rpm *= scale

            with self._command_lock:
                self._target_left_rpm = left_rpm
                self._target_right_rpm = right_rpm
                self._last_valid_cmd_time = time.monotonic()

        except (TypeError, ValueError, OverflowError) as exc:
            with self._command_lock:
                self._target_left_rpm = 0.0
                self._target_right_rpm = 0.0
                self._last_valid_cmd_time = None
            self._warn_throttled(
                'invalid_cmd_vel',
                'Rejected invalid /cmd_vel message: %s' % exc,
            )

    def _snapshot_command(self) -> Tuple[float, float, bool]:
        """Return the last valid command until its explicit safety timeout."""
        with self._command_lock:
            left_rpm = self._target_left_rpm
            right_rpm = self._target_right_rpm
            last_valid_time = self._last_valid_cmd_time

        command_is_fresh = (
            last_valid_time is not None
            and time.monotonic() - last_valid_time <= self._cmd_vel_timeout
        )

        if not command_is_fresh:
            return 0.0, 0.0, False

        return left_rpm, right_rpm, True

    def _build_command_frame(self) -> bytes:
        left_rpm, right_rpm, drive_enabled = self._snapshot_command()
        flags = COMMAND_FLAG_DRIVE_ENABLE if drive_enabled else 0

        if flags & ~COMMAND_ALLOWED_FLAGS:
            raise RuntimeError('Internal command flags are invalid')

        sequence = self._next_command_sequence
        self._next_command_sequence = (sequence + 1) & 0xFFFFFFFF

        payload = ROBOT_COMMAND_STRUCT.pack(
            sequence,
            left_rpm,
            right_rpm,
            flags,
            COMMAND_PROTOCOL_VERSION,
            0,
        )
        checksum = crc16_ibm(payload)

        return FRAME_HEADER + payload + struct.pack('<H', checksum)

    def _open_serial(self) -> bool:
        for port_name in self._ports:
            if self._stop_event.is_set():
                return False

            candidate: Optional[serial.Serial] = None
            try:
                candidate = serial.Serial(
                    port=port_name,
                    baudrate=self._baud_rate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.0,
                    write_timeout=0.050,
                )
                candidate.reset_input_buffer()
                candidate.reset_output_buffer()

                if self._stop_event.is_set():
                    candidate.close()
                    return False

                with self._serial_lock:
                    self._serial = candidate
                    self._connected_port = port_name

                self._parser.reset()
                self.get_logger().info('Connected to Pico on %s' % port_name)
                return True

            except (SerialException, OSError, ValueError) as exc:
                if candidate is not None:
                    try:
                        candidate.close()
                    except (SerialException, OSError):
                        pass
                self._warn_throttled(
                    'open_' + port_name,
                    'Could not open serial port %s: %s' % (port_name, exc),
                )

        return False

    def _get_serial(self) -> Optional[serial.Serial]:
        with self._serial_lock:
            return self._serial

    def _close_serial(self) -> None:
        with self._serial_lock:
            serial_port = self._serial
            self._serial = None
            self._connected_port = None

        if serial_port is not None:
            try:
                serial_port.close()
            except (SerialException, OSError):
                pass

    def _write_command(self, serial_port: serial.Serial) -> None:
        """Pack and write one heartbeat from the dedicated serial worker."""
        frame = self._build_command_frame()
        written = serial_port.write(frame)
        if written != len(frame):
            raise SerialTimeoutException(
                'Short command write: expected %d bytes, wrote %d'
                % (len(frame), written)
            )

    def _read_telemetry(self, serial_port: serial.Serial) -> None:
        states: List[RobotState] = []
        crc_failures = 0
        validation_failures = 0

        # The port is configured with timeout=0.0, so every read is non-blocking.
        # Drain several bounded chunks per worker cycle to prevent an older UART
        # backlog from adding avoidable latency while preserving command deadlines.
        for _ in range(self.MAX_SERIAL_READS_PER_CYCLE):
            available = serial_port.in_waiting
            if available <= 0:
                break

            data = serial_port.read(min(available, self.SERIAL_READ_SIZE))
            if not data:
                break

            (
                decoded_states,
                batch_crc_failures,
                batch_validation_failures,
            ) = self._parser.feed(data)
            states.extend(decoded_states)
            crc_failures += batch_crc_failures
            validation_failures += batch_validation_failures

        if crc_failures:
            self._warn_throttled(
                'telemetry_crc',
                'Discarded telemetry frame(s) with invalid CRC; latest batch=%d'
                % crc_failures,
            )
        if validation_failures:
            self._warn_throttled(
                'telemetry_payload',
                'Discarded invalid RobotState payload(s); latest batch=%d'
                % validation_failures,
            )

        if states:
            dropped_states = 0
            with self._telemetry_lock:
                queue_capacity = self._pending_states.maxlen
                assert queue_capacity is not None
                dropped_states = max(
                    0,
                    len(self._pending_states) + len(states) - queue_capacity,
                )
                for state in states:
                    self._pending_states.append(state)
            if dropped_states:
                self._warn_throttled(
                    'telemetry_queue_overflow',
                    'Telemetry queue overflow discarded %d oldest state(s)'
                    % dropped_states,
                )

    def _serial_worker(self) -> None:
        next_command_time = time.monotonic()

        while not self._stop_event.is_set():
            serial_port = self._get_serial()

            if serial_port is None:
                if not self._open_serial():
                    self._stop_event.wait(self._reconnect_interval)
                    continue
                serial_port = self._get_serial()
                next_command_time = time.monotonic()

            if serial_port is None:
                continue

            try:
                now = time.monotonic()
                if now >= next_command_time:
                    self._write_command(serial_port)

                    elapsed_periods = max(
                        1,
                        int((now - next_command_time) / self._command_period) + 1,
                    )
                    next_command_time += elapsed_periods * self._command_period

                self._read_telemetry(serial_port)
                self._stop_event.wait(0.001)

            except (
                SerialException,
                SerialTimeoutException,
                OSError,
            ) as exc:
                self._warn_throttled(
                    'serial_disconnect',
                    'Serial connection lost: %s; reconnecting' % exc,
                )
                self._close_serial()
                self._parser.reset()
                self._clear_pending_telemetry()
                self._stop_event.wait(self._reconnect_interval)
                next_command_time = time.monotonic()

            except Exception as exc:
                # Keep the safety heartbeat logic recoverable even if an
                # unexpected Python or driver error occurs.
                self.get_logger().error(
                    'Unexpected serial worker error: %s; reconnecting' % exc
                )
                self._close_serial()
                self._parser.reset()
                self._clear_pending_telemetry()
                self._stop_event.wait(self._reconnect_interval)
                next_command_time = time.monotonic()

    def _clear_pending_telemetry(self) -> None:
        with self._telemetry_lock:
            self._pending_states.clear()

    @classmethod
    def _wheel_position_from_revolutions(cls, revolutions: float) -> float:
        """Return a numerically stable continuous-joint phase in radians."""
        fractional_revolutions = math.remainder(revolutions, 1.0)
        return fractional_revolutions * cls.REVOLUTIONS_TO_RADIANS

    def _lidar_position_from_degrees(self, angle_deg: float) -> float:
        """Convert the right-to-left servo convention to URDF yaw radians."""
        bounded_angle_deg = min(
            max(angle_deg, self.LIDAR_MIN_ANGLE_DEG),
            self.LIDAR_MAX_ANGLE_DEG,
        )
        if bounded_angle_deg != angle_deg:
            self._warn_throttled(
                'lidar_joint_limit',
                'LiDAR angle %.3f deg exceeded the URDF limits and was clamped'
                % angle_deg,
            )

        return math.radians(
            bounded_angle_deg - self.LIDAR_FORWARD_ANGLE_DEG
        )

    def _estimate_lidar_velocity(
        self,
        timestamp_ms: int,
        position_rad: float,
    ) -> float:
        """Estimate servo angular velocity from consecutive Pico samples."""
        velocity_rad_s = 0.0

        if (
            self._previous_lidar_timestamp_ms is not None
            and self._previous_lidar_position_rad is not None
        ):
            elapsed_ms = (
                timestamp_ms - self._previous_lidar_timestamp_ms
            ) & 0xFFFFFFFF
            if 0 < elapsed_ms <= self.MAX_LIDAR_VELOCITY_INTERVAL_MS:
                velocity_rad_s = (
                    position_rad - self._previous_lidar_position_rad
                ) / (elapsed_ms / 1000.0)

        self._previous_lidar_timestamp_ms = timestamp_ms
        self._previous_lidar_position_rad = position_rad
        return velocity_rad_s

    def _build_joint_state_message(self, state: RobotState) -> JointState:
        """Build one complete JointState sample from validated Pico telemetry."""
        lidar_position_rad = self._lidar_position_from_degrees(
            state.lidar_angle_deg
        )
        lidar_velocity_rad_s = self._estimate_lidar_velocity(
            state.timestamp_ms,
            lidar_position_rad,
        )

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.JOINT_NAMES)
        message.position = [
            self._wheel_position_from_revolutions(state.left_total_revs),
            self._wheel_position_from_revolutions(state.right_total_revs),
            lidar_position_rad,
            self._camera_tilt_position_rad,
        ]
        message.velocity = [
            state.left_rpm * self.RPM_TO_RADIANS_PER_SECOND,
            state.right_rpm * self.RPM_TO_RADIANS_PER_SECOND,
            lidar_velocity_rad_s,
            0.0,
        ]
        return message

    def _publish_pending_states(self) -> None:
        states: List[RobotState] = []
        with self._telemetry_lock:
            while self._pending_states:
                states.append(self._pending_states.popleft())

        for state in states:
            try:
                state_message = Float32MultiArray()
                state_message.data = state.as_float_array()
                self._state_publisher.publish(state_message)

                timestamp_message = UInt32()
                timestamp_message.data = state.timestamp_ms
                self._timestamp_publisher.publish(timestamp_message)

                status_message = UInt8()
                status_message.data = state.system_status_code
                self._status_publisher.publish(status_message)

                encoder_left_message = Int32()
                encoder_left_message.data = state.left_encoder_ticks
                self._encoder_left_publisher.publish(encoder_left_message)

                encoder_right_message = Int32()
                encoder_right_message.data = state.right_encoder_ticks
                self._encoder_right_publisher.publish(encoder_right_message)

                diagnostic_flags_message = UInt32()
                diagnostic_flags_message.data = state.diagnostic_flags
                self._diagnostic_flags_publisher.publish(
                    diagnostic_flags_message
                )

                safe_state_message = Bool()
                safe_state_message.data = bool(
                    state.diagnostic_flags &
                    DIAGNOSTIC_SAFE_STATE_ACTIVE
                )
                self._safe_state_publisher.publish(safe_state_message)

                command_fresh_message = Bool()
                command_fresh_message.data = bool(
                    state.diagnostic_flags &
                    DIAGNOSTIC_COMMAND_FRESH
                )
                self._command_fresh_publisher.publish(
                    command_fresh_message
                )

                i2c_fault_message = Bool()
                i2c_fault_message.data = bool(
                    state.diagnostic_flags &
                    DIAGNOSTIC_I2C_FAULT_LATCHED
                )
                self._i2c_fault_publisher.publish(i2c_fault_message)

                system_power_message = Float32MultiArray()
                system_power_message.data = state.as_system_power_array()
                self._system_power_publisher.publish(system_power_message)

                motors_power_message = Float32MultiArray()
                motors_power_message.data = state.as_motors_power_array()
                self._motors_power_publisher.publish(motors_power_message)

                joint_state_message = self._build_joint_state_message(state)
                self._joint_state_publisher.publish(joint_state_message)

            except Exception as exc:
                self._warn_throttled(
                    'publish_state',
                    'Failed to publish decoded RobotState outputs: %s' % exc,
                    error=True,
                )

    def _warn_throttled(
        self,
        key: str,
        message: str,
        error: bool = False,
    ) -> None:
        now = time.monotonic()
        with self._warning_lock:
            previous_time = self._last_warning_times.get(key, -math.inf)
            if now - previous_time < self.WARNING_THROTTLE_SECONDS:
                return