# Dynamic Sensor Enable/Disable Feature

This document describes the new dynamic sensor enable/disable feature for robot_localization.

## Overview

The enable/disable feature allows you to:
1. Configure sensors to be enabled or disabled at startup
2. Dynamically enable or disable sensors at runtime using ROS2 parameters
3. Monitor which sensors are currently enabled/disabled via the diagnostics topic

When a sensor is disabled, its measurements are not integrated into the filter state, but the sensor subscription remains active.

## Configuration

### Adding Enable/Disable Parameters

For each sensor input (odom, pose, twist, imu), you can add an `_enabled` parameter:

```yaml
ekf_filter_node:
    ros__parameters:
        # Odometry sensor
        odom0: /robot/odom
        odom0_config: [true, true, false, ...]
        odom0_enabled: true  # Enable at startup (default: true)
        
        # Pose sensor
        pose0: /robot/pose
        pose0_config: [true, true, false, ...]
        pose0_enabled: false  # Disable at startup
        
        # Twist sensor
        twist0: /robot/twist
        twist0_config: [false, false, false, true, ...]
        twist0_enabled: true  # Enable at startup
        
        # IMU sensor
        imu0: /robot/imu
        imu0_config: [false, false, false, true, ...]
        imu0_enabled: true  # Enable at startup
```

**Note:** If the `_enabled` parameter is not specified, the sensor is enabled by default.

## Runtime Usage

### Disable a Sensor at Runtime

```bash
ros2 param set /ekf_filter_node odom0_enabled false
```

### Enable a Sensor at Runtime

```bash
ros2 param set /ekf_filter_node imu0_enabled true
```

### Check Current Parameter Values

```bash
ros2 param get /ekf_filter_node odom0_enabled
```

### List All Parameters

```bash
ros2 param list /ekf_filter_node
```

## Monitoring Sensor Status

The sensor enable/disable status is published in the diagnostics topic:

```bash
ros2 topic echo /diagnostics
```

Look for the "Sensor Status" field in the diagnostic message, which will show:
- **Enabled:** List of currently enabled sensors
- **Disabled:** List of currently disabled sensors

Example output:
```
values:
  - key: "Sensor Status"
    value: "Enabled: odom0, imu0 | Disabled: pose0, twist0"
```

## Use Cases

1. **Sensor Fault Recovery:** Quickly disable a malfunctioning sensor without restarting the node
2. **Testing:** Enable/disable specific sensors to test their impact on filter performance
3. **Conditional Sensing:** Disable certain sensors in specific operating conditions (e.g., disable GPS indoors)
4. **Resource Management:** Temporarily disable sensors to reduce computational load
5. **Sensor Calibration:** Disable a sensor while it's being recalibrated

## Implementation Details

- Sensor subscriptions remain active even when disabled
- Disabled sensors do not contribute to the filter state estimation
- The enable/disable state can be changed at any time during operation
- Changes take effect immediately on the next sensor callback
- The feature is backward compatible - existing configurations without `_enabled` parameters will work as before

## Example Workflow

```bash
# 1. Start the EKF node with a configuration file
ros2 run robot_localization ekf_node --ros-args --params-file config.yaml

# 2. Check initial sensor status
ros2 topic echo /diagnostics

# 3. Disable a sensor during runtime
ros2 param set /ekf_filter_node odom0_enabled false

# 4. Verify the change
ros2 param get /ekf_filter_node odom0_enabled
ros2 topic echo /diagnostics

# 5. Re-enable the sensor
ros2 param set /ekf_filter_node odom0_enabled true
```

## Backward Compatibility

This feature is fully backward compatible. Existing configuration files without `_enabled` parameters will continue to work as before, with all sensors enabled by default.
