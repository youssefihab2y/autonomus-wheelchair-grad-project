# New Center of Mass Calculation

## Updated Mass Distribution (relative to chassis at z=0)

| Component | Mass (kg) | Z Position (m) | Mass × Z |
|-----------|-----------|-----------------|----------|
| chassis | 15.0 | 0.0 | 0.0 |
| base_uplayer_link | 15.0 | 0.15 | 2.25 |
| wheel1 | 0.5 | 0.0 | 0.0 |
| wheel2 | 0.5 | 0.0 | 0.0 |
| imu_sensor_link | 0.001 | 0.17 | 0.00017 |
| camera_stand_link | 0.5 | 0.3925 | 0.19625 |
| camera_link | 0.5 | 0.63 | 0.315 |
| base_laser | 0.05 | 0.8475 | 0.042375 |
| seat_link | 0.5 | 0.34 | 0.17 |
| back_link | 0.5 | 0.50 | 0.25 |
| **TOTAL** | **33.001** | | **3.223795** |

## New Center of Mass

**CoM_z = 3.223795 / 33.001 = 0.0977 m**

## Stability Analysis

- **Wheel separation:** 0.52 m
- **Wheel contact point:** z = 0.0 m (wheels at ground level)
- **CoM height above ground:** 0.0977 m
- **Stability ratio:** CoM height / (wheel_separation/2) = 0.0977 / 0.26 = 0.376

**✅ MUCH BETTER!**
- CoM is now at 0.0977m (very low, close to wheels)
- Stability ratio < 0.5 (good for 2-wheel robot)
- CoM is centered (x=0, y=0)

## Improvements Made

1. ✅ Increased base mass: 10kg → 15kg
2. ✅ Lowered base position: 0.21m → 0.15m
3. ✅ Reduced upper component masses: 1.0kg → 0.5kg (camera, seat, back)
4. ✅ Reduced lidar mass: 0.1kg → 0.05kg
5. ✅ Adjusted all joint positions to match new base height

**Result: CoM lowered from 0.1806m to 0.0977m (46% reduction!)**
