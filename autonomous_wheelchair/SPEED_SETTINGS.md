# Speed Settings Guide for Wheelchair

## 🎮 **Speed Parameters Explained**

### **Forward Speed (Linear X)**
- **What it does:** Controls forward/backward movement
- **Recommended for SLAM:** 0.3 - 0.5 m/s
- **Maximum:** 1.0 m/s (wheelchair max speed)

### **Vertical Speed (Linear Z)**
- **What it does:** Controls up/down movement (for flying robots)
- **For wheelchair:** Set to 0.0 (not used - wheelchair is ground-based)
- **Ignore this parameter** - wheelchair doesn't move vertically

### **Yaw Speed (Angular Z)**
- **What it does:** Controls rotation/turning (left/right)
- **Recommended for SLAM:** 0.3 - 0.5 rad/s
- **Maximum:** 0.5 - 1.0 rad/s

## 📊 **Recommended Settings for SLAM Mapping**

### **Optimal Settings:**
```
Forward (m/s):  0.3 - 0.5
Vertical (m/s):  0.0 (not used)
Yaw (rad/s):    0.3 - 0.5
```

### **Conservative Settings (Best Quality):**
```
Forward (m/s):  0.2 - 0.3
Vertical (m/s):  0.0
Yaw (rad/s):    0.2 - 0.3
```

### **Faster Settings (Less Accurate):**
```
Forward (m/s):  0.5 - 0.8
Vertical (m/s):  0.0
Yaw (rad/s):    0.5 - 0.8
```

## 🎯 **How to Set Speeds**

### **In Gazebo Teleop Panel:**
- **Forward (m/s):** Set to `0.3` or `0.5`
- **Vertical (m/s):** Set to `0.0` (or ignore)
- **Yaw (rad/s):** Set to `0.3` or `0.5`

### **In Keyboard Teleop:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Adjust speeds:**
- Press `,` (comma) = Decrease linear speed (forward)
- Press `.` (period) = Increase linear speed (forward)
- Press `u` = Decrease angular speed (yaw/turn)
- Press `o` = Increase angular speed (yaw/turn)

**Current speeds are shown at the top of the terminal**

## 📐 **Speed Conversions**

### **Yaw Speed Reference:**
- `0.1 rad/s` = ~5.7 degrees/second (very slow turn)
- `0.3 rad/s` = ~17 degrees/second (slow turn)
- `0.5 rad/s` = ~29 degrees/second (moderate turn)
- `1.0 rad/s` = ~57 degrees/second (fast turn)

### **Forward Speed Reference:**
- `0.1 m/s` = 0.36 km/h (very slow walk)
- `0.3 m/s` = 1.08 km/h (slow walk)
- `0.5 m/s` = 1.8 km/h (normal walk)
- `1.0 m/s` = 3.6 km/h (fast walk)

## 🚗 **Driving Patterns**

### **Pattern 1: Grid Mapping**
```
Forward: 0.3 m/s
Yaw: 0.3 rad/s
Pattern: Drive forward → Turn 90° → Drive forward → Repeat
```

### **Pattern 2: Spiral Mapping**
```
Forward: 0.4 m/s
Yaw: 0.2 rad/s (continuous gentle turn)
Pattern: Drive in expanding spiral
```

### **Pattern 3: Room-by-Room**
```
Forward: 0.3 m/s
Yaw: 0.4 rad/s
Pattern: Explore each room completely before moving to next
```

## ⚠️ **Common Mistakes**

❌ **Too Fast:**
- Forward > 1.0 m/s → Poor scan matching
- Yaw > 1.0 rad/s → Map distortion

❌ **Too Slow:**
- Forward < 0.1 m/s → Takes forever, may cause drift
- Yaw < 0.1 rad/s → Very slow turns

❌ **Using Vertical Speed:**
- Vertical > 0.0 → Not applicable for wheelchair
- Ignore vertical speed parameter

## ✅ **Best Practice**

**Start with:**
- Forward: 0.3 m/s
- Yaw: 0.3 rad/s

**Adjust based on:**
- Map quality in RViz
- Robot stability
- Scan matching success

**If map looks good:** Can increase slightly
**If map distorts:** Decrease speeds

## 🔧 **Quick Commands**

```bash
# Check current cmd_vel values
ros2 topic echo /cmd_vel --once

# Publish specific speed for testing
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" \
  --once

# Continuous forward at 0.3 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  -r 10
```

---

**Remember:**
- **Forward** = Movement speed (0.3-0.5 m/s recommended)
- **Vertical** = Ignore (0.0 for ground robot)
- **Yaw** = Turn speed (0.3-0.5 rad/s recommended)

