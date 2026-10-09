"""Fuse planar wheel odometry with Pico IMU orientation for odometry and TF."""

import math
import threading
import time
from typing import Dict, Optional, Sequence, Tuple

import rclpy
import rclpy.logging
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster


class DifferentialDriveKinematics:
    """Integrate differential-drive wheel speeds into a planar pose."""

    RPM_TO_RAD_PER_SECOND = 2.0 * math.pi / 60.0
    STRAIGHT_LINE_EPSILON = 1.0e-9

    def __init__(self, wheel_radius: float, wheel_base: float) -> None:
        self._wheel_radius = wheel_radius
        self._wheel_base = wheel_base
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def set_geometry(self, wheel_radius: float, wheel_base: float) -> None:
        self._wheel_radius = wheel_radius
        self._wheel_base = wheel_base

    @property
    def geometry(self) -> Tuple[float, float]:
        return self._wheel_radius, self._wheel_base

    def update(self,left_rpm: float,right_rpm: float,dt: float) -> Tuple[float, float]:
        left_velocity = left_rpm * self.RPM_TO_RAD_PER_SECOND * self._wheel_radius
        right_velocity = right_rpm * self.RPM_TO_RAD_PER_SECOND * self._wheel_radius
        linear_velocity = 0.5 * (left_velocity + right_velocity)
        angular_velocity = (right_velocity - left_velocity) / self._wheel_base
        if dt <= 0.0:
            return linear_velocity, angular_velocity
        previous_theta = self.theta
        new_theta = previous_theta + angular_velocity * dt
        if abs(angular_velocity) > self.STRAIGHT_LINE_EPSILON:
            turning_radius = linear_velocity / angular_velocity
            self.x += turning_radius * (math.sin(new_theta) - math.sin(previous_theta))
            self.y -= turning_radius * (math.cos(new_theta) - math.cos(previous_theta))
        else:
            distance = linear_velocity * dt
            self.x += distance * math.cos(previous_theta)
            self.y += distance * math.sin(previous_theta)
        self.theta = math.atan2(math.sin(new_theta), math.cos(new_theta))
        return linear_velocity, angular_velocity


class DifferentialOdometryNode(Node):
    ODOM_FRAME='odom'
    BASE_FOOTPRINT_FRAME='base_footprint'
    BASE_LINK_FRAME='base_link'
    STATE_TOPIC='/pico/robot_state'
    ODOM_TOPIC='/odom'
    IMU_ROLL_INDEX=4
    IMU_PITCH_INDEX=5
    IMU_YAW_INDEX=6
    REQUIRED_IMU_STATE_LENGTH=IMU_YAW_INDEX+1
    MAX_ABS_IMU_ROLL_DEG=180.0
    MAX_ABS_IMU_PITCH_DEG=90.0
    MAX_INTEGRATION_DT=0.5
    WARNING_THROTTLE_SECONDS=5.0

    def __init__(self) -> None:
        super().__init__('differential_odometry_node')
        positive_double_range=[FloatingPointRange(from_value=1.0e-6,to_value=1000.0,step=0.0)]
        self.declare_parameter('wheel_radius',0.035,ParameterDescriptor(description='Driven-wheel radius in metres.',floating_point_range=positive_double_range))
        self.declare_parameter('wheel_base',0.15,ParameterDescriptor(description='Lateral distance between wheel contact centres in metres.',floating_point_range=positive_double_range))
        wheel_radius=float(self.get_parameter('wheel_radius').value)
        wheel_base=float(self.get_parameter('wheel_base').value)
        self._state_lock=threading.RLock()
        self._kinematics=DifferentialDriveKinematics(wheel_radius,wheel_base)
        self._last_update_ns: Optional[int]=None
        self._last_warning_times: Dict[str,float]={}
        self._imu_yaw_offset_rad: Optional[float]=None
        self._imu_orientation_active=False
        qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        self._odom_publisher=self.create_publisher(Odometry,self.ODOM_TOPIC,qos)
        self._tf_broadcaster=TransformBroadcaster(self)
        self._state_subscription=self.create_subscription(Float32MultiArray,self.STATE_TOPIC,self._state_callback,qos)
        self.add_on_set_parameters_callback(self._parameter_callback)
        self.get_logger().info('Differential odometry started: wheel_radius=%.4f m, wheel_base=%.4f m, TF=%s -> %s -> %s' % (wheel_radius,wheel_base,self.ODOM_FRAME,self.BASE_FOOTPRINT_FRAME,self.BASE_LINK_FRAME))

    def _parameter_callback(self,parameters: Sequence[Parameter]) -> SetParametersResult:
        try:
            with self._state_lock:
                wheel_radius,wheel_base=self._kinematics.geometry
                geometry_changed=False
                for parameter in parameters:
                    if parameter.name not in ('wheel_radius','wheel_base'):
                        continue
                    geometry_changed=True
                    if isinstance(parameter.value,bool) or not isinstance(parameter.value,(float,int)) or not math.isfinite(float(parameter.value)) or float(parameter.value)<=0.0:
                        return SetParametersResult(successful=False,reason='%s must be a finite value greater than zero.' % parameter.name)
                    if parameter.name=='wheel_radius': wheel_radius=float(parameter.value)
                    else: wheel_base=float(parameter.value)
                if geometry_changed:
                    self._kinematics.set_geometry(wheel_radius,wheel_base)
                    self._last_update_ns=None
            if geometry_changed:
                self.get_logger().info('Updated drive geometry: wheel_radius=%.4f m, wheel_base=%.4f m' % (wheel_radius,wheel_base))
            return SetParametersResult(successful=True)
        except Exception as exc:
            self.get_logger().error('Failed to update drive geometry parameters: %s' % exc)
            return SetParametersResult(successful=False,reason=str(exc))

    def _state_callback(self,message: Float32MultiArray) -> None:
        try:
            if len(message.data)<2:
                self._warn_throttled('short_state','Ignoring /pico/robot_state: expected at least 2 values, got %d.' % len(message.data)); return
            left_rpm=float(message.data[0]); right_rpm=float(message.data[1])
            if not (math.isfinite(left_rpm) and math.isfinite(right_rpm)):
                self._warn_throttled('invalid_rpm','Ignoring non-finite wheel RPM values.'); return
            imu_euler=self._extract_imu_euler(message.data)
            now=self.get_clock().now(); now_ns=now.nanoseconds
            with self._state_lock:
                dt=self._calculate_dt(now_ns)
                linear_velocity,angular_velocity=self._kinematics.update(left_rpm,right_rpm,dt)
                x=self._kinematics.x; y=self._kinematics.y; encoder_yaw=self._kinematics.theta
                wheel_radius,_=self._kinematics.geometry
                roll,pitch,fused_yaw=self._resolve_orientation(encoder_yaw,imu_euler)
                self._publish_odometry_and_tf(now.to_msg(),x,y,wheel_radius,roll,pitch,fused_yaw,linear_velocity,angular_velocity)
        except Exception as exc:
            self._warn_throttled('state_callback_error','Odometry update failed; sample discarded: %s' % exc,error=True)

    def _extract_imu_euler(self,state_values: Sequence[float]) -> Optional[Tuple[float,float,float]]:
        if len(state_values)<self.REQUIRED_IMU_STATE_LENGTH:
            self._warn_throttled('missing_imu_orientation','IMU orientation is unavailable in /pico/robot_state; using encoder yaw fallback.'); return None
        try:
            roll_deg=float(state_values[self.IMU_ROLL_INDEX]); pitch_deg=float(state_values[self.IMU_PITCH_INDEX]); yaw_deg=float(state_values[self.IMU_YAW_INDEX])
        except (IndexError,TypeError,ValueError,OverflowError) as exc:
            self._warn_throttled('invalid_imu_orientation','IMU orientation could not be decoded; using encoder yaw: %s' % exc); return None
        if not all(math.isfinite(v) for v in (roll_deg,pitch_deg,yaw_deg)):
            self._warn_throttled('non_finite_imu_orientation','IMU orientation contains a non-finite value; using encoder yaw fallback.'); return None
        if abs(roll_deg)>self.MAX_ABS_IMU_ROLL_DEG or abs(pitch_deg)>self.MAX_ABS_IMU_PITCH_DEG:
            self._warn_throttled('imu_orientation_out_of_range','IMU roll or pitch exceeds the physical Euler-angle range; using encoder yaw fallback.'); return None
        return (self._normalize_angle(math.radians(roll_deg)),math.radians(pitch_deg),self._normalize_angle(math.radians(yaw_deg)))

    def _resolve_orientation(self,encoder_yaw: float,imu_euler: Optional[Tuple[float,float,float]]) -> Tuple[float,float,float]:
        if imu_euler is None:
            if self._imu_orientation_active:
                self._warn_throttled('imu_orientation_lost','IMU orientation was lost; TF is using encoder yaw fallback.')
            self._imu_orientation_active=False; self._imu_yaw_offset_rad=None
            return 0.0,0.0,encoder_yaw
        roll,pitch,raw_imu_yaw=imu_euler
        if not self._imu_orientation_active or self._imu_yaw_offset_rad is None:
            self._imu_yaw_offset_rad=self._normalize_angle(encoder_yaw-raw_imu_yaw)
            self._imu_orientation_active=True
            self.get_logger().info('Live IMU orientation acquired; full 3D base_link rotation is active.')
        fused_yaw=self._normalize_angle(raw_imu_yaw+self._imu_yaw_offset_rad)
        return roll,pitch,fused_yaw

    @staticmethod
    def _normalize_angle(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad),math.cos(angle_rad))

    @staticmethod
    def _quaternion_from_euler(roll: float,pitch: float,yaw: float) -> Tuple[float,float,float,float]:
        half_roll=0.5*roll; half_pitch=0.5*pitch; half_yaw=0.5*yaw
        cr=math.cos(half_roll); sr=math.sin(half_roll); cp=math.cos(half_pitch); sp=math.sin(half_pitch); cy=math.cos(half_yaw); sy=math.sin(half_yaw)
        qx=sr*cp*cy-cr*sp*sy; qy=cr*sp*cy+sr*cp*sy; qz=cr*cp*sy-sr*sp*cy; qw=cr*cp*cy+sr*sp*sy
        norm=math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
        if norm<=1.0e-12: raise ValueError('Cannot normalize a zero-length orientation quaternion')
        return qx/norm,qy/norm,qz/norm,qw/norm

    def _calculate_dt(self,now_ns: int) -> float:
        previous_ns=self._last_update_ns; self._last_update_ns=now_ns
        if previous_ns is None: return 0.0
        dt=(now_ns-previous_ns)*1.0e-9
        if dt<=0.0:
            self._warn_throttled('non_monotonic_clock','ROS clock did not advance; odometry integration skipped.'); return 0.0
        if dt>self.MAX_INTEGRATION_DT:
            self._warn_throttled('large_dt','Robot-state gap was %.3f s; pose integration skipped to prevent a jump.' % dt); return 0.0
        return dt

    def _publish_odometry_and_tf(self,stamp,x: float,y: float,base_link_height: float,roll: float,pitch: float,yaw: float,linear_velocity: float,angular_velocity: float) -> None:
        full_q=self._quaternion_from_euler(roll,pitch,yaw)
        planar_q=self._quaternion_from_euler(0.0,0.0,yaw)
        tilt_q=self._quaternion_from_euler(roll,pitch,0.0)
        odometry=Odometry()
        odometry.header.stamp=stamp; odometry.header.frame_id=self.ODOM_FRAME; odometry.child_frame_id=self.BASE_LINK_FRAME
        odometry.pose.pose.position.x=x; odometry.pose.pose.position.y=y; odometry.pose.pose.position.z=base_link_height
        odometry.pose.pose.orientation.x=full_q[0]; odometry.pose.pose.orientation.y=full_q[1]; odometry.pose.pose.orientation.z=full_q[2]; odometry.pose.pose.orientation.w=full_q[3]
        odometry.twist.twist.linear.x=linear_velocity; odometry.twist.twist.angular.z=angular_velocity
        self._odom_publisher.publish(odometry)
        footprint=TransformStamped(); footprint.header.stamp=stamp; footprint.header.frame_id=self.ODOM_FRAME; footprint.child_frame_id=self.BASE_FOOTPRINT_FRAME
        footprint.transform.translation.x=x; footprint.transform.translation.y=y; footprint.transform.translation.z=0.0
        footprint.transform.rotation.x=planar_q[0]; footprint.transform.rotation.y=planar_q[1]; footprint.transform.rotation.z=planar_q[2]; footprint.transform.rotation.w=planar_q[3]
        base=TransformStamped(); base.header.stamp=stamp; base.header.frame_id=self.BASE_FOOTPRINT_FRAME; base.child_frame_id=self.BASE_LINK_FRAME
        base.transform.translation.z=base_link_height
        base.transform.rotation.x=tilt_q[0]; base.transform.rotation.y=tilt_q[1]; base.transform.rotation.z=tilt_q[2]; base.transform.rotation.w=tilt_q[3]
        self._tf_broadcaster.sendTransform(footprint); self._tf_broadcaster.sendTransform(base)

    def _warn_throttled(self,key: str,message: str,error: bool=False) -> None:
        now=time.monotonic(); last=self._last_warning_times.get(key,-math.inf)
        if now-last<self.WARNING_THROTTLE_SECONDS: return
        self._last_warning_times[key]=now
        if error: self.get_logger().error(message)
        else: self.get_logger().warning(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[DifferentialOdometryNode]=None
    try:
        node=DifferentialOdometryNode(); rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger('differential_odometry_node').fatal('Fatal odometry node error: %s' % exc)
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__':
    main()
