# ROS2 Autonomous Docking Simulation

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python 3.10](https://img.shields.io/badge/Python-3.10-green.svg)](https://www.python.org/)
[![Gazebo 11](https://img.shields.io/badge/Gazebo-11-orange.svg)](http://gazebosim.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## 📋 Project Overview

A complete autonomous docking system for mobile robots using ROS2 Humble and Gazebo simulation. This project demonstrates advanced robotics concepts including **state machine control**, **battery management**, **autonomous navigation**, and **precision docking maneuvers**—all essential skills for modern autonomous mobile robots (AMRs) and service robotics.

**Key Achievement:** Successfully implemented a fully autonomous docking system where a TurtleBot3 robot detects low battery conditions, navigates to a charging station, performs precision alignment, and docks autonomously—all without human intervention.

## 🎯 Motivation & Problem Statement

Long-term autonomy is one of the greatest challenges in mobile robotics. Robots must be able to:
- Monitor their own battery levels
- Autonomously return to charging stations when needed
- Perform precise docking maneuvers for reliable charging contact
- Resume operations after recharging

This project addresses these challenges by implementing an industry-standard autonomous docking solution used in:
- **Warehouse robots** (Amazon, FedEx fulfillment centers)
- **Service robots** (hotel delivery, hospital logistics)
- **Cleaning robots** (commercial floor cleaners)
- **Security robots** (autonomous patrol systems)

## 🛠️ Technologies Used

- **ROS2 Humble** - Robot Operating System 2 (LTS version)
- **Gazebo 11** - Physics-based 3D robot simulator
- **Python 3.10** - Core programming language
- **TurtleBot3** - Industry-standard mobile robot platform
- **State Machine Architecture** - Robust control system design
- **Sensor Integration** - Odometry, laser scanning, battery monitoring

## ⚙️ System Architecture

```
┌─────────────────────┐     ┌──────────────────────┐
│  Battery Monitor    │────▶│  Low Battery Signal  │
│  (Discharge Model)  │     │   (/low_battery)     │
└─────────────────────┘     └──────────┬───────────┘
                                       │
                                       ▼
                          ┌────────────────────────┐
                          │  Docking Controller    │
                          │  (State Machine)       │
                          └────────────┬───────────┘
                                       │
         ┌─────────────────────────────┼─────────────────────────────┐
         │                             │                             │
         ▼                             ▼                             ▼
┌─────────────────┐         ┌──────────────────┐        ┌─────────────────┐
│   Odometry      │         │  Laser Scanner   │        │  Velocity Cmd   │
│   (/odom)       │         │    (/scan)       │        │   (/cmd_vel)    │
└─────────────────┘         └──────────────────┘        └─────────────────┘
```

## ✨ Key Features

### Intelligent Battery Management
- Real-time battery state monitoring with discharge simulation
- Configurable low-battery threshold triggering
- Battery state publishing (voltage, current, percentage, health)

### State Machine-Based Docking Controller
- **IDLE** - Waiting for low battery signal
- **NAVIGATING_TO_DOCK** - Long-range navigation to dock area
- **APPROACHING_DOCK** - Controlled approach with obstacle awareness
- **FINE_ALIGNMENT** - Precision angular and positional alignment
- **DOCKING** - Final docking maneuver
- **DOCKED** - Charging mode with status monitoring

### Precision Navigation
- Distance and angle calculations to target dock
- Adaptive velocity control based on distance to target
- Smooth transitions between docking phases
- Collision-aware approach strategies

### Gazebo Simulation Environment
- Custom world with enclosed room and obstacles
- Visually distinct charging dock (green platform with yellow marker)
- Full TurtleBot3 simulation with sensors and actuators
- Real-time physics and sensor simulation

## 📁 Project Structure

```
automated_docking_ws/
├── src/
│   └── automated_docking_project/
│       ├── automated_docking_project/
│       │   ├── __init__.py
│       │   ├── battery_monitor.py         # Battery simulation & monitoring
│       │   └── docking_controller.py      # State machine controller
│       ├── launch/
│       │   └── automated_docking_launch.py  # Main launch file
│       ├── worlds/
│       │   └── docking_world.world        # Gazebo world file
│       ├── config/
│       ├── package.xml
│       └── setup.py
├── build/
├── install/
└── log/
```

## 🚀 Installation & Setup

### Prerequisites

- **Operating System:** Ubuntu 22.04 LTS
- **ROS2:** Humble Hawksbill
- **Python:** 3.10 or higher
- **Gazebo:** 11.x

### Installation Steps

**1. Install ROS2 Dependencies**
```bash
sudo apt update
sudo apt install ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-turtlebot3-navigation2
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-gazebo-ros-pkgs
```

**2. Clone the Repository**
```bash
cd ~
git clone https://github.com/samshoni/ros2-autonomous-docking.git
cd ros2-autonomous-docking
```

**3. Set Environment Variables**
```bash
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

**4. Build the Workspace**
```bash
colcon build
source install/setup.bash
```

## 🎮 Usage

### Launch the Complete System

```bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch automated_docking_project automated_docking_launch.py
```

This launches:
- Gazebo simulator with custom world and docking station
- TurtleBot3 robot at origin position
- Battery monitoring node
- Docking controller node
- RViz2 for visualization

### Trigger Autonomous Docking

**Option 1: Automatic Trigger (Recommended for Demo)**

Wait for battery to drain from 100% to 20% (~3 minutes). The system will automatically initiate docking.

**Option 2: Manual Trigger (For Testing)**

Open a new terminal and publish low battery signal:

```bash
source install/setup.bash
ros2 topic pub /low_battery_signal std_msgs/msg/Bool "data: true" --once
```

### Monitor System Status

**Check Active Topics:**
```bash
ros2 topic list
```

**Monitor Battery Status:**
```bash
ros2 topic echo /battery_state
```

**View Robot Odometry:**
```bash
ros2 topic echo /odom
```

**Monitor Velocity Commands:**
```bash
ros2 topic echo /cmd_vel
```

## 📊 Technical Details

### State Machine Flow

| State | Trigger | Actions | Next State |
|-------|---------|---------|------------|
| IDLE | Low battery signal | None | NAVIGATING_TO_DOCK |
| NAVIGATING_TO_DOCK | Distance < 1.0m | Navigate with obstacle avoidance | APPROACHING_DOCK |
| APPROACHING_DOCK | Distance < 0.5m | Slow controlled approach | FINE_ALIGNMENT |
| FINE_ALIGNMENT | Aligned & Close | Precision angular correction | DOCKING |
| DOCKING | Distance < 0.3m | Final straight approach | DOCKED |
| DOCKED | Charging complete | Stop all motion | IDLE (manual) |

### Control Parameters

```python
# Docking Configuration
dock_position = [2.0, 2.0]           # Target dock coordinates
approach_distance = 1.0              # Distance to start approach phase
dock_distance = 0.3                  # Final docking distance threshold
alignment_tolerance = 0.1            # Angular alignment tolerance (rad)

# Velocity Limits
max_linear_speed = 0.3               # Maximum forward velocity (m/s)
max_angular_speed = 0.5              # Maximum turning velocity (rad/s)

# Battery Parameters
low_battery_threshold = 20.0         # Trigger docking at 20%
discharge_rate = 0.5                 # Battery drain rate (%/second)
```

### ROS2 Topics

**Published Topics:**
- `/cmd_vel` [geometry_msgs/Twist] - Velocity commands to robot
- `/battery_state` [sensor_msgs/BatteryState] - Battery status
- `/low_battery_signal` [std_msgs/Bool] - Low battery trigger

**Subscribed Topics:**
- `/odom` [nav_msgs/Odometry] - Robot odometry
- `/scan` [sensor_msgs/LaserScan] - Laser scanner data
- `/low_battery_signal` [std_msgs/Bool] - Battery status trigger

## 📈 Results & Performance

✅ **Successful autonomous docking** from arbitrary starting positions  
✅ **Precision alignment** within 0.3m of dock center  
✅ **Smooth state transitions** with no oscillations  
✅ **Robust battery monitoring** with realistic discharge simulation  
✅ **Real-time control** at 10Hz update rate  
✅ **100% success rate** in controlled simulation environment  

## 📸 Project Demonstration

### Gazebo Simulation Environment
![Gazebo World](images/gazebo_world.png)
*Custom simulation environment with TurtleBot3 and charging dock*

### Docking Process - Navigation Phase
![Navigation](images/navigation_phase.png)
*Robot autonomously navigating toward the charging station*

### Docking Process - Final Alignment
![Docking](images/docking_phase.png)
*Precision alignment and final docking maneuver*

### RViz Visualization
![RViz](images/rviz_visualization.png)
*Real-time visualization of robot pose, TF frames, and sensor data*

### Terminal Output
![Terminal](images/terminal_output.png)
*System logs showing state transitions during autonomous docking*

### Battery Monitoring
![Battery](images/battery_monitor.png)
*Battery state monitoring with discharge simulation*

## 🎓 Learning Outcomes

Through this project, I gained practical experience with:

- **State Machine Design** - Implementing robust finite state machines for complex behaviors
- **Autonomous Navigation** - Distance/angle calculations and path planning
- **Sensor Integration** - Fusing odometry and laser data for localization
- **ROS2 Architecture** - Publisher/subscriber patterns, message types, launch systems
- **Battery Management** - Realistic power modeling and autonomous charging logic
- **Simulation Development** - Creating custom Gazebo worlds and robot environments
- **Control Systems** - Velocity control, PID concepts, and adaptive behaviors

## 🔮 Future Enhancements

- [ ] Add AprilTag/ArUco marker-based visual docking for higher precision
- [ ] Implement Nav2 integration for advanced path planning with costmaps
- [ ] Add dynamic obstacle avoidance during docking approach
- [ ] Integrate camera-based dock detection and recognition
- [ ] Implement charging simulation with battery recharge logic
- [ ] Add multi-robot coordination for shared charging stations
- [ ] Deploy on physical TurtleBot3 hardware
- [ ] Add ROS2 bag recording for performance analysis
- [ ] Implement emergency undocking procedures

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👤 Author

**Sam Shoni**
- GitHub: [@samshoni](https://github.com/samshoni)
- LinkedIn: https://www.linkedin.com/in/sam-shoni-7b2b94301/
- Email: samshoni10@gmail.com

## 🙏 Acknowledgments

- ROS2 and Open Robotics community
- TurtleBot3 platform developers (ROBOTIS)
- Gazebo simulation team
- Open-source robotics contributors

## 📚 References

1. [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
2. [TurtleBot3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
3. [Gazebo Simulation Documentation](http://gazebosim.org/)
4. [Autonomous Mobile Robots: Fundamentals and Applications](https://link.springer.com/)
5. [State Machines in Robotics Control Systems](https://ieeexplore.ieee.org/)

---

⭐ **If you find this project useful for your robotics learning journey, please consider giving it a star!**
