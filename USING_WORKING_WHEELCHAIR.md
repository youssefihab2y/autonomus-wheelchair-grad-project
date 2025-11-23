# ✅ Using ros2-wheelchair-main in Your Project

## 📜 License Information

**ros2-wheelchair-main** uses **Apache License 2.0** - a permissive open-source license.

### ✅ What You CAN Do:
- ✅ Use it in your project (commercial or non-commercial)
- ✅ Modify it to fit your needs
- ✅ Distribute it as part of your project
- ✅ Use it for research/education

### 📋 What You MUST Do:
1. **Include the LICENSE file** from ros2-wheelchair-main
2. **Include copyright notices** (already in the code)
3. **State any modifications** you made (optional but good practice)

---

## 🎯 Option 1: Use It Directly (Recommended)

### Just use the working package as-is:

```bash
# The ros2-wheelchair-main is already in your workspace
cd /home/nadafouad/Downloads/Autonomous-Wheelchair-master-branch/ros2-wheelchair-main

# Build it
colcon build --symlink-install
source install/setup.bash

# Launch it
./LAUNCH_WHEELCHAIR.sh
```

**This is the simplest approach - it already works!**

---

## 🎯 Option 2: Integrate Into Your Project

### Copy the working model into your autonomous_wheelchair:

1. **Copy the world/model files:**
```bash
# Copy the working world file
cp ros2-wheelchair-main/gazebo/world.xml \
   autonomous_wheelchair/src/wheelchair_gazebo/worlds/wheelchair_simple.sdf

# Or use the simplified URDF we already created
# (wheelchair_simple.urdf.xacro already matches the working model)
```

2. **Use the simplified URDF we created:**
   - Already done! `wheelchair_simple.urdf.xacro` matches the working model exactly

3. **Copy the launch configuration:**
```bash
# The launch file is already updated to use wheelchair_simple.urdf.xacro
# Just build and run!
```

---

## 🎯 Option 3: Use Both Packages Together

### Keep both and use whichever you need:

```bash
# Build both workspaces
cd ros2-wheelchair-main
colcon build --symlink-install
source install/setup.bash

cd ../autonomous_wheelchair
colcon build --symlink-install
source install/setup.bash

# Use either one:
# - ros2-wheelchair-main: Simple, working, voice control
# - autonomous_wheelchair: Your custom version with sensors
```

---

## 📝 License Compliance

### To properly use ros2-wheelchair-main in your project:

1. **Create a LICENSE file** in your project root:
```bash
# Copy the Apache 2.0 license
cp ros2-wheelchair-main/LICENSE autonomous_wheelchair/LICENSE_ros2_wheelchair
```

2. **Add attribution** in your README:
```markdown
## Acknowledgments

This project uses code from [ros2-wheelchair](https://github.com/Marthenn/ros2-wheelchair-interfaces)
by Marthen, licensed under Apache License 2.0.
```

3. **Keep copyright notices** in any files you copy/modify

---

## 🚀 Recommended Approach

### Since we already simplified your wheelchair to match:

**Just use your simplified version!** It has:
- ✅ Same structure as working model
- ✅ Same control logic
- ✅ Same plugin configuration
- ✅ Already integrated into your project

**To use it:**
```bash
cd autonomous_wheelchair
colcon build --symlink-install
source install/setup.bash
ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py
```

---

## 📊 Comparison: Which Should You Use?

| Aspect | ros2-wheelchair-main | Your Simplified Version |
|--------|----------------------|------------------------|
| **License** | Apache 2.0 ✅ | Your choice |
| **Features** | Voice control, alerts | Your custom features |
| **Complexity** | Simple | Simple (matches it) |
| **Integration** | Separate package | Integrated in your project |
| **Customization** | Limited | Full control |

**Recommendation:** Use your simplified version - it's already integrated and matches the working model exactly!

---

## ✅ Summary

**YES, you can use ros2-wheelchair-main!**

- ✅ **Legally allowed** - Apache 2.0 license
- ✅ **Already integrated** - We simplified your wheelchair to match it
- ✅ **Best approach** - Use your simplified version (same logic, your project)

**Your simplified wheelchair (`wheelchair_simple.urdf.xacro`) already has the exact same logic as the working one!**

Just build and run:
```bash
cd autonomous_wheelchair
colcon build --symlink-install
source install/setup.bash
ros2 launch wheelchair_gazebo warehouse_with_robot.launch.py
```

🎉 **You're all set!**



