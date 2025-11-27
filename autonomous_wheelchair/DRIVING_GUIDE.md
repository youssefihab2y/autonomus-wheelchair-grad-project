# Driving Guide for SLAM Mapping

## 🚗 **How to Drive for Best SLAM Results**

### **Speed Recommendations**

**✅ OPTIMAL SPEED:**
- **Linear speed: 0.3 - 0.5 m/s** (slow to moderate)
- **Angular speed: 0.3 - 0.5 rad/s** (slow turns)

**❌ TOO FAST:**
- Linear > 1.0 m/s (causes poor scan matching)
- Angular > 1.0 rad/s (causes map distortion)

**❌ TOO SLOW:**
- Linear < 0.1 m/s (takes too long, may cause drift)

### **Driving Technique**

#### **1. Start Slow**
- Begin with very slow movements (0.2 m/s)
- Let SLAM establish initial map
- Gradually increase speed once map is building

#### **2. Smooth Movements**
- **Avoid sudden stops/starts**
- **Avoid sharp turns** - use gentle curves
- **Accelerate/decelerate gradually**

#### **3. Coverage Strategy**
- **Drive in a pattern:**
  1. Start from center
  2. Drive forward in straight lines
  3. Turn gradually (90° turns)
  4. Continue in grid-like pattern
  5. Return to starting area to close loops

#### **4. Loop Closure**
- **Important:** Return to previously visited areas
- This helps SLAM correct accumulated errors
- Map may "jump" when loop closes - this is normal!

### **Keyboard Controls (teleop_twist_keyboard)**

```
Movement Controls:
  i/j/k/l : Move forward/left/backward/right
  u/o     : Increase/decrease linear speed
  ,/.     : Increase/decrease angular speed
  Space   : Emergency stop
  q       : Quit
```

### **Recommended Speed Settings**

**For Mapping:**
```bash
# Start with these speeds:
Linear speed:  0.3 m/s  (press ',' to decrease, '.' to increase)
Angular speed: 0.3 rad/s (press 'u' to decrease, 'o' to increase)
```

**For Navigation (after map is saved):**
```bash
# Can use faster speeds:
Linear speed:  0.5 - 1.0 m/s
Angular speed: 0.5 - 1.0 rad/s
```

### **Step-by-Step Mapping Process**

#### **Step 1: Launch Everything**
```bash
ros2 launch wheelchair_gazebo warehouse_with_slam_rviz.launch.py
```

#### **Step 2: Wait for Initialization**
- Wait ~20 seconds for everything to load
- Check RViz shows robot and map topic

#### **Step 3: Start Driving**
```bash
# In new terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### **Step 4: Drive Pattern**
1. **Forward slowly** (press 'i', speed ~0.3 m/s)
2. **Turn gradually** (press 'j' or 'l', speed ~0.3 rad/s)
3. **Cover all areas** - drive in a systematic pattern
4. **Close loops** - return to starting point

#### **Step 5: Monitor in RViz**
- Watch map build in real-time
- Gray = unknown
- White = free space
- Black = obstacles
- Map should look clean and accurate

#### **Step 6: Save Map**
Once entire warehouse is mapped:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/wheelchair_warehouse_map
```

### **Common Mistakes to Avoid**

❌ **Driving too fast**
- Causes poor scan matching
- Map becomes distorted
- Solution: Slow down to 0.3-0.5 m/s

❌ **Sharp turns**
- Causes map distortion
- Solution: Use gentle curves, turn gradually

❌ **Not closing loops**
- Accumulates error over time
- Solution: Return to starting area periodically

❌ **Stopping suddenly**
- Can cause scan matching issues
- Solution: Decelerate gradually

### **Signs of Good Mapping**

✅ **Map looks accurate:**
- Walls are straight
- Obstacles are clear
- No ghost obstacles

✅ **Robot localization is stable:**
- Robot position in RViz matches Gazebo
- No sudden jumps (except during loop closure)

✅ **Map updates smoothly:**
- New areas appear as you explore
- Map doesn't flicker or jump constantly

### **Troubleshooting**

**Problem: Map looks distorted**
- **Solution:** Drive slower (0.2-0.3 m/s)

**Problem: Map not updating**
- **Solution:** Check `/scan` topic: `ros2 topic hz /scan`
- **Solution:** Verify robot is moving: `ros2 topic echo /odom`

**Problem: Map jumps around**
- **Solution:** This is normal during loop closure
- **Solution:** Continue driving, map will stabilize

**Problem: Can't turn the wheelchair**
- **Solution:** Check front wheel friction settings
- **Solution:** Try slower angular speeds (0.2 rad/s)

### **Quick Reference**

| Action | Speed | Duration |
|--------|-------|----------|
| Forward | 0.3-0.5 m/s | Continuous |
| Turn | 0.3-0.5 rad/s | Until desired angle |
| Stop | Gradual deceleration | 1-2 seconds |
| Loop closure | 0.3 m/s | Return to start |

### **Pro Tips**

1. **Start in open area** - easier for SLAM to initialize
2. **Drive in patterns** - grid-like or spiral patterns work well
3. **Take your time** - better map quality is worth slower speed
4. **Monitor RViz** - watch for issues as you drive
5. **Save frequently** - save map at different stages as backup

---

**Remember: Slow and steady wins the race! 🐢🏆**

