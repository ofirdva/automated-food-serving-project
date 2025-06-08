# 🤖 ABB IRB 1200 Robotic Arm – Final Project (Simulation to Real Robot Control)

This project focuses on developing and deploying an automated robotic food-serving system using the **ABB IRB 1200** robotic arm. The work progressed from simulation and motion planning to real-time control of the physical robot using multiple communication protocols.

---

## 🔧 Phase 1: RViz Simulation with ROS 2

The first stage of the project involved controlling the ABB IRB 1200 in a simulated environment using **RViz** and **ROS 2**. Key features:

- Implemented joystick-based motion control in RViz.
- Used inverse kinematics to validate robot behavior.
- Built a ROS 2 launch system to visualize robot state and plan movements interactively.

---

## 🧩 Phase 2: Real Robot Control via Communication Protocols

After simulation, the next step was to control the actual robot. This required investigating multiple protocols to find the most suitable method for reliable operation.

### ✅ Robot Web Services (RWS) via ROS 2
- Used [PickNik’s `abb_ros2` package](https://github.com/PickNikRobotics/abb_ros2) to send RWS commands from Python.
- Successfully controlled motors, operation mode, and robot state monitoring.
- Limited to basic command execution — not real-time motion.

### ❌ Externally Guided Motion (EGM)
- Attempted to implement real-time joint control using UDP-based EGM.
- Tried both through ROS 2 and directly via standalone Python.
- Encountered persistent execution errors (e.g., **40223**). EGM was ultimately not viable for this setup.

### ✅ TCP/IP Communication (Final Approach)
- Implemented direct control using TCP/IP sockets.
- Sent both joint and Cartesian commands from Python, enabling full motion control.
- This solution proved most stable and effective for predefined and repeated tasks.

> 🔗 **TCP/IP Control Code adapted from:**  
> [`LeRobot/pyABB`](https://github.com/LeRobot/pyABB) – A Python interface for ABB robots over sockets.

---

## 🧠 What I Learned

- Hands-on experience with ABB industrial robot systems.
- Deepened understanding of ROS 2 and robotic communication protocols.
- Troubleshooting and debugging real-time control interfaces (EGM, RWS, TCP).
- Integration of software and hardware for an automated, real-world task.

---

## 📹 Coming Soon

🎥 A video demonstration of the working robotic system in action will be uploaded in the next few days.

---

