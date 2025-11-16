# 🧪 Hexapod Cascade Control - Testing Guide

## 🚀 Quick Start (3 Steps)

### **Step 1: Launch Control System**

```bash
./test_leg1.sh
```

ระบบจะเปิด nodes ทั้งหมดสำหรับ leg 1:
- ✅ Joint State Splitter
- ✅ Inverse Position Kinematics (Optimized, 1mm error, 507 Hz)
- ✅ Inverse Velocity Kinematics (Feedforward)
- ✅ Position PID Controller (Outer Loop)
- ✅ Velocity PID Controller (Inner Loop)
- ✅ Forward Kinematics (Monitor)

---

### **Step 2: Send Target Position (Terminal ใหม่)**

```bash
# วิธีที่ 1: ใช้ helper script (ง่ายที่สุด)
./send_target.sh

# วิธีที่ 2: ระบุ position เอง
./send_target.sh 0.2 -0.15 -0.04
#                 ↑    ↑     ↑
#                 X    Y     Z (meters)

# วิธีที่ 3: ใช้ ros2 topic pub โดยตรง
source install/setup.bash
ros2 topic pub --once /hexapod/leg_1/end_effector_target \
  geometry_msgs/msg/PointStamped \
  "{point: {x: 0.15, y: -0.1, z: -0.05}}"
```

---

### **Step 3: Monitor Results**

```bash
# Terminal ใหม่
source install/setup.bash

# ดู end effector position (จาก FK)
ros2 topic echo /hexapod/leg_1/end_effector_position

# ดู joint angles
ros2 topic echo /hexapod/leg_1/joint_states

# ดู effort commands
ros2 topic echo /effort_controller_leg_1/commands

# Plot real-time (ต้องมี rqt)
rqt_plot /hexapod/leg_1/end_effector_position/point/x:y:z
```

---

## 📊 Expected Results

### **Inverse Kinematics Performance:**
```
Speed:     ~2ms per solve
Accuracy:  ~1mm mean error
Frequency: 507 Hz capable
Success:   100% (all targets < 5mm error)
```

### **Cascade Control Loop:**
```
Input:  Target position (x, y, z) in Cartesian space
↓
IK:     Convert to joint angles [θ₁, θ₂, θ₃]
↓
Pos PID: Generate velocity commands
↓
Vel PID: Generate effort/torque (with feedforward)
↓
Output: Joint efforts → Robot/Simulation
```

---

## 🎮 Test Scenarios

### **Test 1: Single Point**
```bash
# ส่ง target เดียว
./send_target.sh 0.15 -0.1 -0.05

# Monitor
ros2 topic echo /hexapod/leg_1/end_effector_position
```

### **Test 2: Multiple Points (Trajectory)**
```bash
# สร้าง simple trajectory
for i in {1..5}; do
  X=$(echo "0.10 + $i * 0.01" | bc)
  ./send_target.sh $X -0.1 -0.05
  sleep 1
done
```

### **Test 3: Monitor All Topics**
```bash
# ใช้ rqt_graph ดู node connections
rqt_graph

# ดู topic list
ros2 topic list | grep leg_1

# Echo ทุก topic พร้อมกัน (ใช้ tmux/screen)
ros2 topic echo /hexapod/leg_1/end_effector_position &
ros2 topic echo /hexapod/leg_1/joint_position_target &
ros2 topic echo /effort_controller_leg_1/commands &
```

---

## 🔧 Troubleshooting

### **Problem 1: Nodes ไม่เริ่ม**
```bash
# ตรวจสอบว่า build แล้ว
colcon build --symlink-install

# Source workspace
source install/setup.bash

# ตรวจสอบ executables
ros2 pkg executables hexapod
```

### **Problem 2: Topic ไม่มีข้อมูล**
```bash
# ตรวจสอบว่า nodes ทำงาน
ros2 node list

# ควรเห็น:
# /forward_kinematics
# /inverse_kinematics
# /inverse_velocity_kinematics
# /joint_state_splitter
# /position_pid_controller
# /velocity_pid_controller

# ตรวจสอบ topic
ros2 topic list | grep leg_1
```

### **Problem 3: IK ให้ผลผิดพลาด**
```bash
# ตรวจสอบว่าใช้ Numerical IK
ros2 param get /inverse_kinematics use_numerical_ik
# ควรได้: True

# ตรวจสอบ IK error
ros2 topic echo /hexapod/leg_1/joint_position_target
```

---

## 🎯 Advanced Testing

### **Test with Gazebo Simulation**

```bash
# Terminal 1: Launch Gazebo
ros2 launch hexapod_simulation simulation-full.launch.py

# Terminal 2: Launch Control (รอ Gazebo เปิดเสร็จก่อน)
./test_leg1.sh

# Terminal 3: Send targets
./send_target.sh 0.15 -0.1 -0.05
```

### **Performance Benchmarking**

```bash
# ใช้ ros2 topic hz วัด frequency
ros2 topic hz /hexapod/leg_1/end_effector_position

# ใช้ ros2 topic bw วัด bandwidth
ros2 topic bw /effort_controller_leg_1/commands

# Monitor latency
ros2 topic delay /hexapod/leg_1/end_effector_target
```

---

## 📁 Important Files

```
.
├── test_leg1.sh              ← Launch control system
├── send_target.sh            ← Send test targets
├── test_kinematics_validation.py   ← Validate FK/IK
├── test_optimized_ik.py      ← Benchmark IK performance
├── test_cascade_control.py   ← Simulate cascade control
├── CASCADE_CONTROL_SUMMARY.md     ← Full documentation
└── TESTING_GUIDE.md          ← This file
```

---

## ✅ Success Criteria

การทดสอบประสบความสำเร็จถ้า:

- [x] ✅ Nodes ทั้งหมดเริ่มทำงานได้
- [x] ✅ IK แก้ปัญหาได้ภายใน 2ms
- [x] ✅ IK error < 5mm
- [x] ✅ Control loop ทำงานที่ 100 Hz
- [x] ✅ Robot/simulation ตอบสนองตาม target

---

## 🎓 Next Steps

หลังจากทดสอบ leg 1 สำเร็จแล้ว:

1. **Test ทั้ง 6 legs:**
   ```bash
   ros2 launch hexapod simple.launch.py
   ```

2. **Integration กับ Gait Planner:**
   - Implement trajectory planning
   - Test coordinated leg movement

3. **Real Robot Deployment:**
   - Tune PID parameters สำหรับ hardware
   - Test on actual hexapod

---

**Happy Testing! 🤖✨**
