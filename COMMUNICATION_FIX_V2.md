# Aggressive Communication Error Fixes

## Problem Analysis
The errors are still occurring even at 15 Hz because:
- **每個控制循環 (Every control cycle) takes ~125ms** through SyncTable relay
- At 15 Hz: cycles happen every 67ms → async operations never complete in time
- Error pattern shows failures **every single cycle** - systematic bottleneck

## Applied Changes (Revision 2)

### 1. Reduce Update Rate Further → 8 Hz
**Before**: `update_rate: 15` (67ms cycles)  
**After**: `update_rate: 8` (125ms cycles)

**Why**: Gives async operations enough time to complete before next cycle triggered.

### 2. Remove Effort State Reading from effort_r_controller
**Before**:
```yaml
state_interfaces:
  - position
  - velocity
  - effort    # ← Requires extra device communication
```

**After**:
```yaml
state_interfaces:
  - position
  - velocity  # ← Only critical readings
```

**Why**: Reduces SyncTable traffic by 33%. Effort reading adds unnecessary latency through relay.

### 3. Increase Error Timeout to 2500ms
**Before**: `error_timeout_ms: 1500`  
**After**: `error_timeout_ms: 2500`

**Why**: At 8 Hz (125ms cycles), 2500ms = ~20 cycles of buffer for worst-case async delays.

## Expected Results

| Metric | Before (15Hz) | After (8Hz) | Improvement |
|--------|---------------|------------|-------------|
| Cycle time | 67ms | 125ms | 87% more time |
| Data volume | 60 state vars/cycle | 40 state vars/cycle | 33% less traffic |
| Timeout buffer | 22 cycles | 20 cycles | Still safe |
| Trigger failures | Every cycle | Rare/none | ✅ Should fix |

## Rebuild & Test

```bash
cd ~/omy_remote

# Rebuild to pick up XACRO changes
colcon build --packages-select open_manipulator_description open_manipulator_bringup

# Source workspace
source install/setup.bash

# Launch with verbose logging
ros2 launch open_manipulator_bringup omy_f3m_end_hx5_right.launch.py 2>&1 | tee test_output.log

# In another terminal, monitor for errors
tail -f test_output.log | grep -E "COMM_ERROR|Trigger read/write"
```

## Success Criteria
- ❌ Should NOT see: `Trigger read/write called while...` warnings
- ❌ Should NOT see: `COMM_ERROR.*There is no status packet` repeated errors
- ✅ Should see: Controllers loading and activating successfully
- ✅ Should see: Smooth operation for >2 minutes without errors

## If Still Not Working

```bash
# Try even more conservative settings:
# 1. Edit config file directly
nano open_manipulator_bringup/config/omy_end_hx5_right_l100/hardware_controller_manager.yaml

# 2. Change to:
#    update_rate: 5  # Back to original but with fixes above

# 3. Rebuild and test again
colcon build --packages-select open_manipulator_description open_manipulator_bringup
source install/setup.bash
ros2 launch open_manipulator_bringup omy_f3m_end_hx5_right.launch.py
```

## Diagnostic Commands

```bash
# Count errors in real-time
ros2 launch open_manipulator_bringup omy_f3m_end_hx5_right.launch.py 2>&1 | tee /tmp/test.log &
sleep 10
grep -c "COMM_ERROR" /tmp/test.log
grep -c "Trigger read/write" /tmp/test.log

# Should be 0 or very few after fixes
```

## Summary
The root issue was **data volume + timing mismatch** through SyncTable relay. By reducing the update rate to 8 Hz (giving 125ms per cycle) and cutting unnecessary state reads, the system should have time to complete async operations before new ones are triggered.
