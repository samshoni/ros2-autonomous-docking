#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan, BatteryState
import math
import time
from enum import Enum

class DockingState(Enum):
    IDLE = 0
    NAVIGATING_TO_DOCK = 1
    APPROACHING_DOCK = 2
    FINE_ALIGNMENT = 3
    DOCKING = 4
    DOCKED = 5

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.low_battery_sub = self.create_subscription(Bool, '/low_battery_signal', self.low_battery_callback, 10)
        self.battery_sub = self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        
        # State variables
        self.current_state = DockingState.IDLE
        self.robot_pose = None
        self.laser_data = None
        self.low_battery = False
        self.battery_percentage = 100.0
        
        # Docking parameters
        self.dock_position = [2.0, 2.0]  # Known dock position in world coordinates
        self.approach_distance = 1.0  # Distance to start fine approach
        self.dock_distance = 0.3  # Final docking distance
        self.alignment_tolerance = 0.1  # Angular tolerance for alignment
        
        # Control parameters
        self.max_linear_speed = 0.3
        self.max_angular_speed = 0.5
        
        # Timer for state machine
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Docking Controller Node Started')
        self.get_logger().info(f'Dock position: {self.dock_position}')

    def odom_callback(self, msg):
        """Store current robot pose"""
        self.robot_pose = msg.pose.pose

    def laser_callback(self, msg):
        """Store laser scan data"""
        self.laser_data = msg

    def low_battery_callback(self, msg):
        """Handle low battery signal"""
        if msg.data and not self.low_battery:
            self.get_logger().info('Low battery detected! Initiating automatic docking...')
            self.current_state = DockingState.NAVIGATING_TO_DOCK
        self.low_battery = msg.data

    def battery_callback(self, msg):
        """Monitor battery percentage"""
        self.battery_percentage = msg.percentage * 100

    def get_distance_to_dock(self):
        """Calculate distance to dock"""
        if not self.robot_pose:
            return float('inf')
        
        dx = self.dock_position[0] - self.robot_pose.position.x
        dy = self.dock_position[1] - self.robot_pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    def get_angle_to_dock(self):
        """Calculate angle to dock relative to robot"""
        if not self.robot_pose:
            return 0.0
        
        dx = self.dock_position[0] - self.robot_pose.position.x
        dy = self.dock_position[1] - self.robot_pose.position.y
        target_angle = math.atan2(dy, dx)
        
        # Convert quaternion to yaw
        q = self.robot_pose.orientation
        robot_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        
        angle_diff = target_angle - robot_yaw
        
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        return angle_diff

    def control_loop(self):
        """Main control loop - state machine"""
        if not self.robot_pose:
            return
        
        cmd = Twist()
        
        if self.current_state == DockingState.IDLE:
            # Do nothing, wait for low battery signal
            pass
            
        elif self.current_state == DockingState.NAVIGATING_TO_DOCK:
            distance = self.get_distance_to_dock()
            angle = self.get_angle_to_dock()
            
            if distance < self.approach_distance:
                self.current_state = DockingState.APPROACHING_DOCK
                self.get_logger().info('Entering approach phase...')
            else:
                # Navigate towards dock
                if abs(angle) > 0.2:  # Need to turn first
                    cmd.angular.z = self.max_angular_speed * (1.0 if angle > 0 else -1.0)
                else:
                    cmd.linear.x = min(self.max_linear_speed, distance * 0.5)
                    cmd.angular.z = angle * 2.0
            
        elif self.current_state == DockingState.APPROACHING_DOCK:
            distance = self.get_distance_to_dock()
            angle = self.get_angle_to_dock()
            
            if distance < self.dock_distance + 0.2:
                self.current_state = DockingState.FINE_ALIGNMENT
                self.get_logger().info('Starting fine alignment...')
            else:
                # Slow approach
                cmd.linear.x = min(0.1, distance * 0.3)
                cmd.angular.z = angle * 1.5
        
        elif self.current_state == DockingState.FINE_ALIGNMENT:
            distance = self.get_distance_to_dock()
            angle = self.get_angle_to_dock()
            
            if abs(angle) < self.alignment_tolerance and distance < self.dock_distance + 0.1:
                self.current_state = DockingState.DOCKING
                self.get_logger().info('Starting final docking maneuver...')
            else:
                # Fine alignment
                cmd.linear.x = 0.05 if distance > self.dock_distance else 0.0
                cmd.angular.z = angle * 0.8
        
        elif self.current_state == DockingState.DOCKING:
            distance = self.get_distance_to_dock()
            
            if distance < self.dock_distance:
                self.current_state = DockingState.DOCKED
                self.get_logger().info('DOCKED SUCCESSFULLY! Starting charging simulation...')
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                # Final approach
                cmd.linear.x = 0.03
                cmd.angular.z = 0.0
        
        elif self.current_state == DockingState.DOCKED:
            # Stay docked, simulate charging
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
            # Check if we should undock (battery full or manual override)
            if self.battery_percentage > 90.0:  # Simulate charging complete
                self.get_logger().info('Charging complete! Ready for undocking.')
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
