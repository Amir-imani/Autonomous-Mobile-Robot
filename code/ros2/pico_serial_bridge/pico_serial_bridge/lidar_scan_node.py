"""Aggregate Pico servo-LiDAR telemetry into sensor_msgs/LaserScan.

The serial bridge publishes LiDAR distance, servo angle and validity inside
/pico/robot_state. Servo 90 deg is robot-forward. The accumulator detects
sweep reversals and publishes an angle-increasing scan in the fixed laser_link
frame.
"""

from dataclasses import dataclass
from enum import IntEnum
import math
import threading
import time
from typing import Dict, List, Optional, Set, Tuple

import rclpy
import rclpy.logging
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class SweepDirection(IntEnum):
    RIGHT_TO_LEFT = 1
    LEFT_TO_RIGHT = -1


@dataclass(frozen=True)
class ScanConfiguration:
    frame_id: str
    servo_min_angle_deg: float
    servo_max_angle_deg: float
    forward_angle_deg: float
    requested_increment_deg: float
    distance_scale_m_per_unit: float
    range_min_m: float
    range_max_m: float
    direction_epsilon_deg: float
    reversal_threshold_deg: float
    boundary_tolerance_deg: float
    sweep_timeout_seconds: float
    minimum_samples_per_scan: int

    def __post_init__(self) -> None:
        if not self.frame_id.strip():
            raise ValueError('frame_id must not be empty')
        vals=(self.servo_min_angle_deg,self.servo_max_angle_deg,self.forward_angle_deg,
              self.requested_increment_deg,self.distance_scale_m_per_unit,self.range_min_m,
              self.range_max_m,self.direction_epsilon_deg,self.reversal_threshold_deg,
              self.boundary_tolerance_deg,self.sweep_timeout_seconds)
        if not all(math.isfinite(v) for v in vals):
            raise ValueError('scan parameters must be finite')
        if self.servo_max_angle_deg <= self.servo_min_angle_deg:
            raise ValueError('servo_max_angle_deg must exceed servo_min_angle_deg')
        if not self.servo_min_angle_deg <= self.forward_angle_deg <= self.servo_max_angle_deg:
            raise ValueError('forward_angle_deg must be inside sweep')
        if self.requested_increment_deg <= 0.0 or self.distance_scale_m_per_unit <= 0.0:
            raise ValueError('angle increment and distance scale must be positive')
        if self.range_min_m < 0.0 or self.range_max_m <= self.range_min_m:
            raise ValueError('invalid range limits')
        if self.direction_epsilon_deg <= 0.0 or self.reversal_threshold_deg < self.direction_epsilon_deg:
            raise ValueError('invalid reversal thresholds')
        if self.sweep_timeout_seconds <= 0.0 or self.minimum_samples_per_scan < 2:
            raise ValueError('invalid sweep timeout/minimum sample count')
        if self.bin_count > 10000:
            raise ValueError('too many scan bins')

    @property
    def sweep_span_deg(self) -> float:
        return self.servo_max_angle_deg-self.servo_min_angle_deg

    @property
    def bin_count(self) -> int:
        intervals=max(1,int(round(self.sweep_span_deg/self.requested_increment_deg)))
        return intervals+1

    @property
    def effective_increment_deg(self) -> float:
        return self.sweep_span_deg/float(self.bin_count-1)

    @property
    def angle_min_rad(self) -> float:
        return math.radians(self.servo_min_angle_deg-self.forward_angle_deg)

    @property
    def angle_max_rad(self) -> float:
        return math.radians(self.servo_max_angle_deg-self.forward_angle_deg)

    @property
    def angle_increment_rad(self) -> float:
        return math.radians(self.effective_increment_deg)

    def normalize_angle(self, angle_deg: float) -> Optional[float]:
        lo=self.servo_min_angle_deg-self.boundary_tolerance_deg
        hi=self.servo_max_angle_deg+self.boundary_tolerance_deg
        if angle_deg<lo or angle_deg>hi:
            return None
        return min(self.servo_max_angle_deg,max(self.servo_min_angle_deg,angle_deg))

    def bin_index(self, angle_deg: float) -> int:
        idx=int(round((angle_deg-self.servo_min_angle_deg)/self.effective_increment_deg))
        return min(self.bin_count-1,max(0,idx))


@dataclass(frozen=True)
class LidarSample:
    angle_deg: float
    distance_m: float
    is_valid: bool
    stamp_nanoseconds: int


@dataclass(frozen=True)
class CompletedSweep:
    ranges: Tuple[float,...]
    start_nanoseconds: int
    end_nanoseconds: int
    direction: Optional[SweepDirection]
    observed_bin_count: int
    valid_bin_count: int
    completion_reason: str


class LidarSweepAccumulator:
    def __init__(self, config: ScanConfiguration) -> None:
        self.c=config
        self.reset()

    @property
    def has_active_sweep(self) -> bool:
        return self._ranges is not None and self._last_sample is not None

    @property
    def last_sample_nanoseconds(self) -> Optional[int]:
        return None if self._last_sample is None else self._last_sample.stamp_nanoseconds

    def reset(self) -> None:
        self._ranges: Optional[List[float]]=None
        self._observed_bins: Set[int]=set()
        self._valid_bins: Set[int]=set()
        self._start_nanoseconds=0
        self._last_sample: Optional[LidarSample]=None
        self._direction: Optional[SweepDirection]=None
        self._extreme: Optional[LidarSample]=None

    def _direction_from_delta(self, delta: float) -> Optional[SweepDirection]:
        if delta > self.c.direction_epsilon_deg: return SweepDirection.RIGHT_TO_LEFT
        if delta < -self.c.direction_epsilon_deg: return SweepDirection.LEFT_TO_RIGHT
        return None

    def _start(self, sample: LidarSample) -> None:
        self._ranges=[math.inf]*self.c.bin_count
        self._observed_bins=set(); self._valid_bins=set()
        self._start_nanoseconds=sample.stamp_nanoseconds
        self._last_sample=sample; self._direction=None; self._extreme=sample
        self._store(sample)

    def _store(self, sample: LidarSample) -> None:
        if self._ranges is None: return
        idx=self.c.bin_index(sample.angle_deg)
        self._observed_bins.add(idx)
        if sample.is_valid and self.c.range_min_m <= sample.distance_m <= self.c.range_max_m:
            current=self._ranges[idx]
            self._ranges[idx]=sample.distance_m if not math.isfinite(current) else min(current,sample.distance_m)
            self._valid_bins.add(idx)

    def _complete(self, reason: str, end_sample: Optional[LidarSample]=None) -> Optional[CompletedSweep]:
        if self._ranges is None or self._last_sample is None: return None
        if len(self._observed_bins) < self.c.minimum_samples_per_scan:
            return None
        end=end_sample or self._last_sample
        return CompletedSweep(tuple(self._ranges),self._start_nanoseconds,end.stamp_nanoseconds,
                              self._direction,len(self._observed_bins),len(self._valid_bins),reason)

    def consume(self, sample: LidarSample) -> Optional[CompletedSweep]:
        if not self.has_active_sweep:
            self._start(sample); return None
        prev=self._last_sample
        assert prev is not None
        if sample.stamp_nanoseconds < prev.stamp_nanoseconds:
            self.reset(); self._start(sample); return None
        d=sample.angle_deg-prev.angle_deg
        observed=self._direction_from_delta(d)
        if self._direction is None and observed is not None:
            self._direction=observed
            self._extreme=sample
        elif self._direction is not None and observed is not None and observed != self._direction:
            turning=self._extreme or prev
            if abs(sample.angle_deg-turning.angle_deg) < self.c.reversal_threshold_deg:
                self._last_sample=sample
                return None
            completed=self._complete('direction_reversal',turning)
            self.reset(); self._start(turning)
            self._direction=observed
            self._store(sample); self._last_sample=sample; self._extreme=sample
            return completed
        self._store(sample); self._last_sample=sample
        if self._direction==SweepDirection.RIGHT_TO_LEFT:
            if self._extreme is None or sample.angle_deg>self._extreme.angle_deg: self._extreme=sample
        elif self._direction==SweepDirection.LEFT_TO_RIGHT:
            if self._extreme is None or sample.angle_deg<self._extreme.angle_deg: self._extreme=sample
        return None

    def expire(self, now_ns: int) -> Optional[CompletedSweep]:
        if not self.has_active_sweep or self.last_sample_nanoseconds is None: return None
        if (now_ns-self.last_sample_nanoseconds)*1e-9 < self.c.sweep_timeout_seconds: return None
        completed=self._complete('timeout')
        self.reset()
        return completed


class LidarScanNode(Node):
    DISTANCE_INDEX=8
    ANGLE_INDEX=9
    VALIDITY_INDEX=12
    REQUIRED_STATE_LENGTH=13
    WARNING_THROTTLE_SECONDS=5.0

    def __init__(self) -> None:
        super().__init__('lidar_scan_node')
        self._declare_parameters()
        self._configuration=self._load_configuration()
        self._accumulator=LidarSweepAccumulator(self._configuration)
        self._accumulator_lock=threading.Lock()
        self._warning_lock=threading.Lock()
        self._last_warning_times: Dict[str,float]={}
        self._input_topic=str(self.get_parameter('input_topic').value)
        self._scan_topic=str(self.get_parameter('scan_topic').value)
        input_qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=100,
                             reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        self._scan_publisher=self.create_publisher(LaserScan,self._scan_topic,qos_profile_sensor_data)
        self._state_subscription=self.create_subscription(Float32MultiArray,self._input_topic,self._state_callback,input_qos)
        period=min(0.25,max(0.05,self._configuration.sweep_timeout_seconds*0.25))
        self._timeout_timer=self.create_timer(period,self._timeout_callback)
        self.get_logger().info('LiDAR translator started: %s -> %s, frame=%s, bins=%d' %
                               (self._input_topic,self._scan_topic,self._configuration.frame_id,self._configuration.bin_count))

    def _declare_parameters(self) -> None:
        ro=ParameterDescriptor(read_only=True,description='Configured at node startup; restart to change.')
        defaults={'input_topic':'/pico/robot_state','scan_topic':'/scan','frame_id':'laser_link',
                  'servo_min_angle_deg':0.0,'servo_max_angle_deg':180.0,'forward_angle_deg':90.0,
                  'angle_increment_deg':1.0,'lidar_distance_scale':0.01,'range_min':0.02,'range_max':4.0,
                  'direction_epsilon_deg':0.25,'reversal_threshold_deg':1.0,'boundary_tolerance_deg':2.0,
                  'sweep_timeout':1.0,'minimum_samples_per_scan':3}
        for name,value in defaults.items(): self.declare_parameter(name,value,ro)

    def _load_configuration(self) -> ScanConfiguration:
        g=lambda n:self.get_parameter(n).value
        return ScanConfiguration(str(g('frame_id')),float(g('servo_min_angle_deg')),float(g('servo_max_angle_deg')),
            float(g('forward_angle_deg')),float(g('angle_increment_deg')),float(g('lidar_distance_scale')),
            float(g('range_min')),float(g('range_max')),float(g('direction_epsilon_deg')),
            float(g('reversal_threshold_deg')),float(g('boundary_tolerance_deg')),float(g('sweep_timeout')),
            int(g('minimum_samples_per_scan')))

    def _state_callback(self, message: Float32MultiArray) -> None:
        try:
            if len(message.data)<self.REQUIRED_STATE_LENGTH:
                self._warn('short_state','Ignoring Pico state: expected at least %d values, got %d' % (self.REQUIRED_STATE_LENGTH,len(message.data))); return
            distance_raw=float(message.data[self.DISTANCE_INDEX])
            raw_angle=float(message.data[self.ANGLE_INDEX])
            validity=float(message.data[self.VALIDITY_INDEX])
            if not math.isfinite(raw_angle):
                self._warn('invalid_angle','Ignoring non-finite LiDAR servo angle'); return
            angle=self._configuration.normalize_angle(raw_angle)
            if angle is None:
                self._warn('angle_out_of_bounds','Ignoring LiDAR angle %.3f deg outside configured sweep' % raw_angle); return
            finite_distance=math.isfinite(distance_raw)
            valid=finite_distance and math.isfinite(validity) and validity>=0.5
            distance_m=distance_raw*self._configuration.distance_scale_m_per_unit if finite_distance else math.inf
            now=self.get_clock().now()
            sample=LidarSample(angle,distance_m,valid,now.nanoseconds)
            with self._accumulator_lock:
                completed=self._accumulator.consume(sample)
            if completed is not None: self._publish_sweep(completed)
        except Exception as exc:
            self._warn('state_callback_error','LiDAR aggregation failed; sample discarded: %s' % exc,True)

    def _timeout_callback(self) -> None:
        try:
            now=self.get_clock().now()
            with self._accumulator_lock: completed=self._accumulator.expire(now.nanoseconds)
            if completed is not None: self._publish_sweep(completed)
        except Exception as exc:
            self._warn('timeout_error','LiDAR sweep timeout handling failed: %s' % exc,True)

    def _publish_sweep(self, sweep: CompletedSweep) -> None:
        scan=LaserScan()
        scan.header.stamp=Time(nanoseconds=sweep.start_nanoseconds,clock_type=self.get_clock().clock_type).to_msg()
        scan.header.frame_id=self._configuration.frame_id
        scan.angle_min=self._configuration.angle_min_rad
        scan.angle_max=self._configuration.angle_max_rad
        scan.angle_increment=self._configuration.angle_increment_rad
        scan.time_increment=0.0
        scan.scan_time=max(0.0,(sweep.end_nanoseconds-sweep.start_nanoseconds)*1e-9)
        scan.range_min=self._configuration.range_min_m
        scan.range_max=self._configuration.range_max_m
        scan.ranges=list(sweep.ranges)
        scan.intensities=[]
        self._scan_publisher.publish(scan)

    def _warn(self,key: str,message: str,error: bool=False) -> None:
        now=time.monotonic()
        with self._warning_lock:
            prev=self._last_warning_times.get(key,-math.inf)
            if now-prev<self.WARNING_THROTTLE_SECONDS: return
            self._last_warning_times[key]=now
        (self.get_logger().error if error else self.get_logger().warning)(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[LidarScanNode]=None
    try:
        node=LidarScanNode(); rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger('lidar_scan_node').fatal('Fatal LiDAR scan node error: %s' % exc)
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__':
    main()
