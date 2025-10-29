#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
import random

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        
        # Publishers
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.low_battery_pub = self.create_publisher(Bool, '/low_battery_signal', 10)
        
        # Timer for battery updates
        self.timer = self.create_timer(1.0, self.publish_battery_status)
        
        # Battery parameters
        self.battery_percentage = 100.0  # Start with full battery
        self.discharge_rate = 0.5  # Discharge rate per second
        self.low_battery_threshold = 20.0  # Threshold for low battery
        
        self.get_logger().info('Battery Monitor Node Started')
        self.get_logger().info(f'Low battery threshold: {self.low_battery_threshold}%')

    def publish_battery_status(self):
        # Simulate battery discharge
        self.battery_percentage -= self.discharge_rate
        
        # Add some randomness to make it more realistic
        noise = random.uniform(-0.1, 0.1)
        current_percentage = max(0.0, self.battery_percentage + noise)
        
        # Create battery state message
        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.voltage = 12.0 * (current_percentage / 100.0)  # Simulate voltage drop
        battery_msg.current = -2.0 if current_percentage > 0 else 0.0  # Discharge current
        battery_msg.charge = current_percentage
        battery_msg.capacity = 100.0
        battery_msg.design_capacity = 100.0
        battery_msg.percentage = current_percentage / 100.0
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        battery_msg.present = True
        
        # Publish battery state
        self.battery_pub.publish(battery_msg)
        
        # Check for low battery
        low_battery = Bool()
        if current_percentage <= self.low_battery_threshold:
            low_battery.data = True
            if hasattr(self, '_last_warning') and self.get_clock().now().nanoseconds - self._last_warning > 5e9:  # Every 5 seconds
                self.get_logger().warn(f'LOW BATTERY WARNING: {current_percentage:.1f}% - Docking required!')
                self._last_warning = self.get_clock().now().nanoseconds
            elif not hasattr(self, '_last_warning'):
                self.get_logger().warn(f'LOW BATTERY WARNING: {current_percentage:.1f}% - Docking required!')
                self._last_warning = self.get_clock().now().nanoseconds
        else:
            low_battery.data = False
        
        self.low_battery_pub.publish(low_battery)
        
        # Reset battery if it reaches 0 (for demonstration)
        if current_percentage <= 0:
            self.get_logger().info('Battery depleted. Resetting to 100% for demo.')
            self.battery_percentage = 100.0

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

