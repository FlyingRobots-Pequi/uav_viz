#!/usr/bin/env python3
"""
UAV Visualizer Node

Sistema de visualização e monitoramento em tempo real para operações de drones.
Integra dados do PX4, tópicos customizados e cria visualizações abrangentes no RViz.

Author: UAV Team
License: MIT
"""

import rclpy
import numpy as np
import math
from typing import Optional, Dict, Any
from dataclasses import dataclass
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# ROS2 message types
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import PoseStamped, Point, Vector3, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# PX4 messages
from px4_msgs.msg import (
    VehicleLocalPosition, VehicleAttitude, VehicleStatus,
    TrajectorySetpoint, BatteryStatus, FailsafeFlags
)

# Custom UAV interfaces
from uav_interfaces.msg import UavStatus, MissionState, MissionCommand


@dataclass
class VisualizationConfig:
    """Configuration for visualization parameters"""
    vehicle_trail_size: int = 1000
    setpoint_trail_size: int = 500
    marker_scale: float = 0.5
    velocity_arrow_scale: float = 0.3
    battery_warning_threshold: float = 0.3
    battery_critical_threshold: float = 0.15
    update_rate: float = 20.0  # Hz


class UAVVisualizer(Node):
    """
    Comprehensive UAV visualization and monitoring system.
    
    Features:
    - Real-time vehicle position and orientation visualization
    - Flight path tracking with different colors for different states
    - Vehicle status indicators (armed, mode, battery, etc.)
    - Mission progress visualization
    - Setpoint and trajectory visualization
    - Emergency and failsafe indicators
    """

    def __init__(self):
        super().__init__('uav_visualizer')
        
        # Configuration
        self.config = VisualizationConfig()
        
        # Declare parameters
        self._declare_parameters()
        
        # Initialize state variables
        self._init_state_variables()
        
        # Setup QoS profiles
        self._setup_qos_profiles()
        
        # Create subscribers
        self._create_subscribers()
        
        # Create publishers
        self._create_publishers()
        
        # Setup transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer for main visualization loop
        timer_period = 1.0 / self.config.update_rate
        self.timer = self.create_timer(timer_period, self.visualization_callback)
        
        self.get_logger().info('UAV Visualizer initialized - Real-time monitoring active')
    
    def _declare_parameters(self):
        """Declare ROS2 parameters with default values"""
        self.declare_parameter('vehicle_trail_size', self.config.vehicle_trail_size)
        self.declare_parameter('setpoint_trail_size', self.config.setpoint_trail_size)
        self.declare_parameter('marker_scale', self.config.marker_scale)
        self.declare_parameter('update_rate', self.config.update_rate)
        self.declare_parameter('base_frame', 'map')
        self.declare_parameter('vehicle_frame', 'base_link')
        self.declare_parameter('uav_namespace', '')
        
        # Get parameter values
        self.config.vehicle_trail_size = self.get_parameter('vehicle_trail_size').value
        self.config.setpoint_trail_size = self.get_parameter('setpoint_trail_size').value
        self.config.marker_scale = self.get_parameter('marker_scale').value
        self.config.update_rate = self.get_parameter('update_rate').value
        self.base_frame = self.get_parameter('base_frame').value
        self.vehicle_frame = self.get_parameter('vehicle_frame').value
        self.uav_namespace = self.get_parameter('uav_namespace').value
        
        if self.uav_namespace:
            self.get_logger().info(f'Using UAV namespace: {self.uav_namespace}')
        else:
            self.get_logger().info('Using default UAV namespace (no prefix)')

    def _build_uav_topic(self, topic):
        """Build complete topic name with namespace prefix."""
        if self.uav_namespace:
            return f"{self.uav_namespace}{topic}"
        return topic
    
    def _init_state_variables(self):
        """Initialize all state tracking variables"""
        # Vehicle state
        self.vehicle_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        self.vehicle_velocity = np.array([0.0, 0.0, 0.0])
        self.vehicle_heading = 0.0
        
        # UAV status
        self.uav_status: Optional[UavStatus] = None
        self.last_uav_status_time = self.get_clock().now()
        
        # Mission state
        self.mission_state: Optional[MissionState] = None
        self.current_setpoint = np.array([0.0, 0.0, 0.0])
        
        # Battery state
        self.battery_voltage = 0.0
        self.battery_remaining = 1.0
        self.battery_warning_level = 0
        
        # Paths for trail visualization
        self.vehicle_path = Path()
        self.setpoint_path = Path()
        self.vehicle_path.header.frame_id = self.base_frame
        self.setpoint_path.header.frame_id = self.base_frame
        
        # Visualization state
        self.last_position_update = self.get_clock().now()
        self.visualization_enabled = True
    
    def _setup_qos_profiles(self):
        """Setup QoS profiles for different types of communication"""
        # Best effort for high-frequency sensor data
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Reliable for status and command data
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Transient local for visualization markers
        self.marker_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
    
    def _create_subscribers(self):
        """Create all ROS2 subscribers"""
        # PX4 vehicle data
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            self._build_uav_topic('/fmu/out/vehicle_local_position'),
            self.vehicle_local_position_callback,
            self.sensor_qos
        )
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            self._build_uav_topic('/fmu/out/vehicle_attitude'),
            self.vehicle_attitude_callback,
            self.sensor_qos
        )
        
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            self._build_uav_topic('/fmu/out/battery_status'),
            self.battery_status_callback,
            self.sensor_qos
        )
        
        # Custom UAV interfaces
        self.uav_status_sub = self.create_subscription(
            UavStatus,
            self._build_uav_topic('/uav_status'),
            self.uav_status_callback,
            self.reliable_qos
        )
        
        self.mission_state_sub = self.create_subscription(
            MissionState,
            self._build_uav_topic('/mission_state'),
            self.mission_state_callback,
            self.reliable_qos
        )
        
        self.mission_cmd_sub = self.create_subscription(
            MissionCommand,
            self._build_uav_topic('/mission_cmd'),
            self.mission_command_callback,
            self.reliable_qos
        )
        
        # Trajectory setpoints
        self.setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            self._build_uav_topic('/fmu/in/trajectory_setpoint'),
            self.trajectory_setpoint_callback,
            self.sensor_qos
        )
    
    def _create_publishers(self):
        """Create all ROS2 publishers for visualization"""
        # Vehicle visualization
        self.vehicle_pose_pub = self.create_publisher(
            PoseStamped, '/uav_viz/vehicle_pose', 10
        )
        
        self.vehicle_path_pub = self.create_publisher(
            Path, '/uav_viz/vehicle_path', 10
        )
        
        self.setpoint_path_pub = self.create_publisher(
            Path, '/uav_viz/setpoint_path', 10
        )
        
        # Status and indicators
        self.status_markers_pub = self.create_publisher(
            MarkerArray, '/uav_viz/status_markers', self.marker_qos
        )
        
        self.velocity_marker_pub = self.create_publisher(
            Marker, '/uav_viz/velocity_vector', 10
        )
        
        self.mission_markers_pub = self.create_publisher(
            MarkerArray, '/uav_viz/mission_markers', self.marker_qos
        )
        
        # Emergency indicators
        self.alert_markers_pub = self.create_publisher(
            MarkerArray, '/uav_viz/alert_markers', self.marker_qos
        )
    
    # ============================================================================
    # CALLBACK FUNCTIONS
    # ============================================================================
    
    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        """Process vehicle local position updates"""
        # Convert NED to ENU for ROS visualization
        self.vehicle_position[0] = msg.y  # North -> East
        self.vehicle_position[1] = msg.x  # East -> North  
        self.vehicle_position[2] = -msg.z  # Down -> Up
        
        self.vehicle_velocity[0] = msg.vy
        self.vehicle_velocity[1] = msg.vx
        self.vehicle_velocity[2] = -msg.vz
        
        self.last_position_update = self.get_clock().now()
        
        # Add to vehicle path
        self._add_to_vehicle_path()
    
    def vehicle_attitude_callback(self, msg: VehicleAttitude):
        """Process vehicle attitude updates"""
        # Convert NED quaternion to ENU for ROS
        # PX4 sends [w, x, y, z] in NED frame
        q_ned = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])
        
        # Transform NED to ENU: 
        # ENU = R_ned_to_enu * NED where R rotates 90deg about Z then 180deg about X
        q_enu = self._ned_to_enu_quaternion(q_ned)
        q_enu = q_enu / np.linalg.norm(q_enu)  # Normalize
        
        self.vehicle_attitude = q_enu
        
        # Extract heading for additional use
        self.vehicle_heading = self._quaternion_to_yaw(q_enu)
    
    def battery_status_callback(self, msg: BatteryStatus):
        """Process battery status updates"""
        self.battery_voltage = msg.voltage_v
        self.battery_remaining = msg.remaining
        self.battery_warning_level = msg.warning
    
    def uav_status_callback(self, msg: UavStatus):
        """Process UAV status updates from flight manager"""
        self.uav_status = msg
        self.last_uav_status_time = self.get_clock().now()
    
    def mission_state_callback(self, msg: MissionState):
        """Process mission state updates"""
        self.mission_state = msg
    
    def mission_command_callback(self, msg: MissionCommand):
        """Process mission command updates"""
        # Extract target position from command
        target_pose = msg.target_pose
        self.current_setpoint[0] = target_pose.pose.position.y  # NED to ENU
        self.current_setpoint[1] = target_pose.pose.position.x
        self.current_setpoint[2] = -target_pose.pose.position.z
        
        # Add to setpoint path
        self._add_to_setpoint_path()
    
    def trajectory_setpoint_callback(self, msg: TrajectorySetpoint):
        """Process trajectory setpoint updates from PX4"""
        # Convert NED setpoint to ENU
        if not (math.isnan(msg.position[0]) or math.isnan(msg.position[1]) or math.isnan(msg.position[2])):
            self.current_setpoint[0] = msg.position[1]  # NED to ENU
            self.current_setpoint[1] = msg.position[0]
            self.current_setpoint[2] = -msg.position[2]
            
            self._add_to_setpoint_path()
    
    # ============================================================================
    # VISUALIZATION FUNCTIONS
    # ============================================================================
    
    def visualization_callback(self):
        """Main visualization update loop"""
        if not self.visualization_enabled:
            return
        
        current_time = self.get_clock().now()
        
        # Publish vehicle pose and transform
        self._publish_vehicle_pose()
        self._publish_vehicle_transform()
        
        # Publish paths
        self._publish_paths()
        
        # Publish status markers
        self._publish_status_markers()
        
        # Publish velocity visualization
        self._publish_velocity_marker()
        
        # Publish mission markers if mission is active
        if self.mission_state:
            self._publish_mission_markers()
        
        # Publish alert markers for warnings/emergencies
        self._publish_alert_markers()
    
    def _publish_vehicle_pose(self):
        """Publish current vehicle pose"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.base_frame
        
        # Position (convert numpy types to Python float)
        pose_msg.pose.position.x = float(self.vehicle_position[0])
        pose_msg.pose.position.y = float(self.vehicle_position[1]) 
        pose_msg.pose.position.z = float(self.vehicle_position[2])
        
        # Orientation (convert numpy types to Python float)
        pose_msg.pose.orientation.w = float(self.vehicle_attitude[0])
        pose_msg.pose.orientation.x = float(self.vehicle_attitude[1])
        pose_msg.pose.orientation.y = float(self.vehicle_attitude[2])
        pose_msg.pose.orientation.z = float(self.vehicle_attitude[3])
        
        self.vehicle_pose_pub.publish(pose_msg)
    
    def _publish_vehicle_transform(self):
        """Publish vehicle transform for TF tree"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.vehicle_frame
        
        # Position (convert numpy types to Python float)
        t.transform.translation.x = float(self.vehicle_position[0])
        t.transform.translation.y = float(self.vehicle_position[1])
        t.transform.translation.z = float(self.vehicle_position[2])
        
        # Orientation (convert numpy types to Python float)
        t.transform.rotation.w = float(self.vehicle_attitude[0])
        t.transform.rotation.x = float(self.vehicle_attitude[1])
        t.transform.rotation.y = float(self.vehicle_attitude[2])
        t.transform.rotation.z = float(self.vehicle_attitude[3])
        
        self.tf_broadcaster.sendTransform(t)
    
    def _publish_paths(self):
        """Publish vehicle and setpoint paths"""
        # Update headers with current time
        current_time = self.get_clock().now().to_msg()
        self.vehicle_path.header.stamp = current_time
        self.setpoint_path.header.stamp = current_time
        
        # Publish paths
        self.vehicle_path_pub.publish(self.vehicle_path)
        self.setpoint_path_pub.publish(self.setpoint_path)
    
    def _publish_status_markers(self):
        """Publish visual status indicators"""
        markers = MarkerArray()
        current_time = self.get_clock().now().to_msg()
        
        # Vehicle status text marker
        if self.uav_status:
            text_marker = self._create_text_marker(
                id=1,
                position=self.vehicle_position + np.array([0, 0, 1.5]),
                text=self._get_status_text(),
                color=self._get_status_color(),
                scale=0.3
            )
            text_marker.header.stamp = current_time
            markers.markers.append(text_marker)
        
        # Battery level indicator
        battery_marker = self._create_battery_marker(
            id=2,
            position=self.vehicle_position + np.array([0, 0, 1.0])
        )
        battery_marker.header.stamp = current_time
        markers.markers.append(battery_marker)
        
        # Home position marker
        if self.uav_status and (self.uav_status.home_x != 0 or self.uav_status.home_y != 0):
            home_pos = np.array([self.uav_status.home_y, self.uav_status.home_x, -self.uav_status.home_z])
            home_marker = self._create_home_marker(id=3, position=home_pos)
            home_marker.header.stamp = current_time
            markers.markers.append(home_marker)
        
        self.status_markers_pub.publish(markers)
    
    def _publish_velocity_marker(self):
        """Publish velocity vector visualization"""
        if np.linalg.norm(self.vehicle_velocity) < 0.1:
            return  # Don't show velocity arrow for very low velocities
        
        marker = self._create_arrow_marker(
            id=1,
            start_pos=self.vehicle_position,
            end_pos=self.vehicle_position + self.vehicle_velocity * self.config.velocity_arrow_scale,
            color=ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8),  # Yellow
            scale=0.1
        )
        marker.header.stamp = self.get_clock().now().to_msg()
        
        self.velocity_marker_pub.publish(marker)
    
    def _publish_mission_markers(self):
        """Publish mission-related visualization markers"""
        if not self.mission_state:
            return
        
        markers = MarkerArray()
        current_time = self.get_clock().now().to_msg()
        
        # Current mission status
        mission_text = f"Mission: {self.mission_state.status}\nInfo: {self.mission_state.info}"
        mission_marker = self._create_text_marker(
            id=10,
            position=self.vehicle_position + np.array([2.0, 0, 0.5]),
            text=mission_text,
            color=ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),  # Cyan
            scale=0.25
        )
        mission_marker.header.stamp = current_time
        markers.markers.append(mission_marker)
        
        self.mission_markers_pub.publish(markers)
    
    def _publish_alert_markers(self):
        """Publish emergency and warning markers"""
        markers = MarkerArray()
        current_time = self.get_clock().now().to_msg()
        
        alert_position = self.vehicle_position + np.array([0, 0, 2.0])
        
        # Emergency/failsafe alerts
        if self.uav_status and self.uav_status.failsafe:
            emergency_marker = self._create_sphere_marker(
                id=20,
                position=alert_position,
                color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),  # Bright red
                scale=0.5
            )
            emergency_marker.header.stamp = current_time
            markers.markers.append(emergency_marker)
            
            # Emergency text
            emergency_text = self._create_text_marker(
                id=21,
                position=alert_position + np.array([0, 0, 0.5]),
                text="EMERGENCY!\nFAILSAFE ACTIVE",
                color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
                scale=0.4
            )
            emergency_text.header.stamp = current_time
            markers.markers.append(emergency_text)
        
        # Battery warnings
        if self.battery_remaining < self.config.battery_critical_threshold:
            battery_alert = self._create_text_marker(
                id=22,
                position=alert_position + np.array([1.0, 0, 0]),
                text="CRITICAL BATTERY!",
                color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
                scale=0.3
            )
            battery_alert.header.stamp = current_time
            markers.markers.append(battery_alert)
        elif self.battery_remaining < self.config.battery_warning_threshold:
            battery_warn = self._create_text_marker(
                id=23,
                position=alert_position + np.array([1.0, 0, 0]),
                text="LOW BATTERY",
                color=ColorRGBA(r=1.0, g=0.65, b=0.0, a=1.0),  # Orange
                scale=0.25
            )
            battery_warn.header.stamp = current_time
            markers.markers.append(battery_warn)
        
        self.alert_markers_pub.publish(markers)
    
    # ============================================================================
    # HELPER FUNCTIONS
    # ============================================================================
    
    def _add_to_vehicle_path(self):
        """Add current position to vehicle path trail"""
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position (convert numpy types to Python float)
        pose.pose.position.x = float(self.vehicle_position[0])
        pose.pose.position.y = float(self.vehicle_position[1])
        pose.pose.position.z = float(self.vehicle_position[2])
        
        # Orientation (convert numpy types to Python float)
        pose.pose.orientation.w = float(self.vehicle_attitude[0])
        pose.pose.orientation.x = float(self.vehicle_attitude[1])
        pose.pose.orientation.y = float(self.vehicle_attitude[2])
        pose.pose.orientation.z = float(self.vehicle_attitude[3])
        
        self.vehicle_path.poses.append(pose)
        
        # Limit trail size
        if len(self.vehicle_path.poses) > self.config.vehicle_trail_size:
            self.vehicle_path.poses.pop(0)
    
    def _add_to_setpoint_path(self):
        """Add current setpoint to setpoint path trail"""
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position (convert numpy types to Python float)
        pose.pose.position.x = float(self.current_setpoint[0])
        pose.pose.position.y = float(self.current_setpoint[1])
        pose.pose.position.z = float(self.current_setpoint[2])
        
        # Use vehicle orientation for setpoint visualization
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        
        self.setpoint_path.poses.append(pose)
        
        # Limit trail size
        if len(self.setpoint_path.poses) > self.config.setpoint_trail_size:
            self.setpoint_path.poses.pop(0)
    
    def _ned_to_enu_quaternion(self, q_ned: np.ndarray) -> np.ndarray:
        """Convert quaternion from NED to ENU frame"""
        # Rotation from NED to ENU: 90° around Z, then 180° around X
        # This can be represented as a single quaternion multiplication
        w, x, y, z = q_ned
        # ENU quaternion: [w, y, x, -z] (approximate conversion)
        return np.array([w, y, x, -z])
    
    def _quaternion_to_yaw(self, q: np.ndarray) -> float:
        """Extract yaw angle from quaternion"""
        w, x, y, z = q
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    
    def _get_status_text(self) -> str:
        """Generate status text for display"""
        if not self.uav_status:
            return "No UAV Status"
        
        status_lines = [
            f"Mode: {self.uav_status.nav_state_text}",
            f"Armed: {'YES' if self.uav_status.armed else 'NO'}",
            f"Flying: {'YES' if self.uav_status.is_flying else 'NO'}",
            f"Battery: {self.battery_remaining*100:.0f}%"
        ]
        
        if self.uav_status.offboard_enabled:
            status_lines.append("OFFBOARD")
        
        return "\n".join(status_lines)
    
    def _get_status_color(self) -> ColorRGBA:
        """Get color for status display based on vehicle state"""
        if not self.uav_status:
            return ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)  # Gray
        
        if self.uav_status.failsafe:
            return ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
        elif not self.uav_status.armed:
            return ColorRGBA(r=1.0, g=0.65, b=0.0, a=1.0)  # Orange
        elif self.uav_status.is_flying:
            return ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
        else:
            return ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Blue
    
    # ============================================================================
    # MARKER CREATION FUNCTIONS
    # ============================================================================
    
    def _create_text_marker(self, id: int, position: np.ndarray, text: str, 
                           color: ColorRGBA, scale: float) -> Marker:
        """Create a text marker"""
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.ns = "uav_viz_text"
        marker.id = id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position (convert numpy types to Python float)
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        
        marker.scale.z = scale
        marker.color = color
        marker.text = text
        
        return marker
    
    def _create_sphere_marker(self, id: int, position: np.ndarray, 
                             color: ColorRGBA, scale: float) -> Marker:
        """Create a sphere marker"""
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.ns = "uav_viz_spheres"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position (convert numpy types to Python float)
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color = color
        
        return marker
    
    def _create_arrow_marker(self, id: int, start_pos: np.ndarray, end_pos: np.ndarray,
                            color: ColorRGBA, scale: float) -> Marker:
        """Create an arrow marker"""
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.ns = "uav_viz_arrows"
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Arrow from start to end (convert numpy types to Python float)
        start_point = Point()
        start_point.x = float(start_pos[0])
        start_point.y = float(start_pos[1])
        start_point.z = float(start_pos[2])
        
        end_point = Point()
        end_point.x = float(end_pos[0])
        end_point.y = float(end_pos[1])
        end_point.z = float(end_pos[2])
        
        marker.points = [start_point, end_point]
        
        marker.scale.x = scale  # Shaft diameter
        marker.scale.y = scale * 2  # Head diameter
        marker.scale.z = 0.0  # Head length (auto)
        marker.color = color
        
        return marker
    
    def _create_battery_marker(self, id: int, position: np.ndarray) -> Marker:
        """Create battery level visualization marker"""
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.ns = "uav_viz_battery"
        marker.id = id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position (convert numpy types to Python float)
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        
        # Battery percentage text
        battery_text = f"🔋 {self.battery_remaining*100:.0f}%"
        if self.battery_voltage > 0:
            battery_text += f" ({self.battery_voltage:.1f}V)"
        
        marker.text = battery_text
        marker.scale.z = 0.2
        
        # Color based on battery level
        if self.battery_remaining < self.config.battery_critical_threshold:
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
        elif self.battery_remaining < self.config.battery_warning_threshold:
            marker.color = ColorRGBA(r=1.0, g=0.65, b=0.0, a=1.0)  # Orange
        else:
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
        
        return marker
    
    def _create_home_marker(self, id: int, position: np.ndarray) -> Marker:
        """Create home position marker"""
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.ns = "uav_viz_home"
        marker.id = id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position (convert numpy types to Python float)
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2]) + 0.1  # Slightly above ground
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.2
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.6)  # Semi-transparent blue
        
        return marker


def main(args=None):
    """Main entry point for UAV visualizer node"""
    rclpy.init(args=args)
    
    try:
        uav_visualizer = UAVVisualizer()
        rclpy.spin(uav_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        if 'uav_visualizer' in locals():
            uav_visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 