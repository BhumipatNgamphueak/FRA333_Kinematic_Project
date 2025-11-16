# 🚀 Quick Reference - Hexapod Cascade Control

## ✅ แก้ไขแล้ว: Topic Namespace

ตอนนี้ทุก topic มี namespace ที่ชัดเจน:

```bash
# ✅ ถูกต้อง (มี namespace):
/hexapod/leg_1/end_effector_target      # Input: Target position
/hexapod/leg_1/end_effector_position    # Output: Current position (FK)
/hexapod/leg_1/joint_position_target    # IK result
/hexapod/leg_1/joint_velocity_target    # Position PID output
/hexapod/leg_1/joint_velocity_feedforward  # IVK feedforward
/effort_controller_leg_1/commands       # Final effort commands
```

---

## 🎮 3 วิธีการใช้งาน

### **Option 1: Full System (Gazebo + Control)** ⭐ แนะนำ

```bash
# ทุกอย่างในคำสั่งเดียว
./launch_full_system.sh

# รอ Gazebo โหลด (~15 วินาที)
# จะเปิด Gazebo + RViz + Control System

# ทดสอบ (terminal ใหม่):
./send_target.sh
```

### **Option 2: แยก Launch (ควบคุมได้มากกว่า)**

```bash
# Terminal 1: Gazebo
ros2 launch hexapod_simulation simulation-full.launch.py

# รอ Gazebo โหลดเสร็จ (~15 วินาที)

# Terminal 2: Control System
./test_leg1.sh

# Terminal 3: Send Targets
./send_target.sh
```

### **Option 3: Control Only (ไม่มี Gazebo)**

```bash
# สำหรับทดสอบโค้ด (ไม่มีหุ่นยนต์จำลอง)
./test_leg1.sh

# ส่ง target
./send_target.sh

# ระบบจะทำงาน แต่หุ่นยนต์จะไม่เคลื่อนที่
```

---

## 📊 Monitor Topics

### **ดูผลลัพธ์:**

```bash
# ตำแหน่งปัจจุบัน (จาก FK)
ros2 topic echo /hexapod/leg_1/end_effector_position

# Joint angles target (จาก IK)
ros2 topic echo /hexapod/leg_1/joint_position_target

# Effort commands (ไปที่ Gazebo)
ros2 topic echo /effort_controller_leg_1/commands

# ดูทั้งหมดพร้อมกัน
ros2 topic echo /hexapod/leg_1/end_effector_position --once && \
ros2 topic echo /hexapod/leg_1/joint_position_target --once && \
ros2 topic echo /effort_controller_leg_1/commands --once
```

### **Plot Real-time:**

```bash
# Plot end effector position (X, Y, Z)
rqt_plot /hexapod/leg_1/end_effector_position/point/x:y:z

# Plot joint angles
rqt_plot /hexapod/leg_1/joint_position_target/data[0]:data[1]:data[2]
```

---

## 🎯 Send Targets

### **ใช้ Helper Script:**

```bash
# Default position
./send_target.sh

# Custom position (X, Y, Z in meters)
./send_target.sh 0.20 -0.15 -0.04
```

### **Manual:**

```bash
ros2 topic pub --once /hexapod/leg_1/end_effector_target \
  geometry_msgs/msg/PointStamped \
  "{point: {x: 0.15, y: -0.1, z: -0.05}}"
```

---

## 🔍 Debug Commands

### **ตรวจสอบ Nodes:**

```bash
# ดู nodes ที่ทำงาน
ros2 node list

# ควรเห็น:
# /forward_kinematics
# /inverse_kinematics
# /inverse_velocity_kinematics
# /joint_state_splitter
# /position_pid_controller
# /velocity_pid_controller
```

### **ตรวจสอบ Topics:**

```bash
# ดู topics ทั้งหมด
ros2 topic list | grep leg_1

# เช็ค frequency
ros2 topic hz /hexapod/leg_1/end_effector_position

# เช็ค bandwidth
ros2 topic bw /effort_controller_leg_1/commands
```

### **ดู Node Graph:**

```bash
# แสดง connections ระหว่าง nodes
rqt_graph
```

---

## ⚙️ การแก้ปัญหา

### **Problem: Gazebo ไม่เปิด**

```bash
# ตรวจสอบว่ามี Gazebo
gz sim --version

# ถ้าไม่มี ติดตั้ง:
sudo apt install ros-humble-ros-gz
```

### **Problem: Topic ไม่มีข้อมูล**

```bash
# 1. ตรวจสอบ nodes ทำงาน
ros2 node list

# 2. ตรวจสอบ topic connections
ros2 topic info /hexapod/leg_1/end_effector_target

# 3. Restart nodes
pkill -f "ros2 run hexapod"
./test_leg1.sh
```

### **Problem: Robot ไม่เคลื่อนที่**

```bash
# 1. ตรวจสอบว่ามี joint_states จาก Gazebo
ros2 topic echo /joint_states --once

# 2. ตรวจสอบ effort commands
ros2 topic echo /effort_controller_leg_1/commands --once

# 3. ตรวจสอบว่า controller load แล้ว
ros2 control list_controllers
```

---

## 📈 Expected Performance

```
Component              Performance
─────────────────────────────────────
IK Speed              1.97ms
IK Accuracy           1.03mm
Jacobian Error        0.006mm
Control Rate          100 Hz
Total Latency         ~2.7ms
Max Frequency         507 Hz
```

---

## 🎓 Topic Flow ใหม่ (มี Namespace)

```
Target:
  /hexapod/leg_1/end_effector_target
    ↓
[Inverse Kinematics]
    ↓
  /hexapod/leg_1/joint_position_target
    ↓
[Position PID]
    ↓
  /hexapod/leg_1/joint_velocity_target
    ↓                                    ↓
[Velocity PID] ← /hexapod/leg_1/joint_velocity_feedforward
    ↓
  /effort_controller_leg_1/commands
    ↓
[Gazebo Simulation]
    ↓
  /joint_states → [Splitter] → /hexapod/leg_1/joint_states
                                    ↓
                              (Feedback to controllers)
```

---

## ✨ Files Overview

```
./launch_full_system.sh    # Launch everything (Gazebo + Control)
./test_leg1.sh             # Launch control only (leg 1)
./send_target.sh           # Send target position
QUICK_REFERENCE.md         # This file
TESTING_GUIDE.md           # Detailed testing guide
CASCADE_CONTROL_SUMMARY.md # Full system documentation
```

---

**Updated:** 2025-11-16 | **Namespace Fix Applied** ✅
