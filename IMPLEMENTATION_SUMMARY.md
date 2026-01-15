# Implementation Summary: Dynamic Sensor Enable/Disable Feature

## Problem Statement (Turkish)
Girişteki her bir sensör için, enable/disable parametresi eklemek istiyorum ayrıca, bu parametreler dinamik olsun yani runtime esnasında ros2 parameter set komutu ile değiştirebiliyim. Eğer bir girdi disable ise bu girdiyi ekfye sokmasın enable durumunda ise eklesin. Ayrıca hangi sensörlerin enable/disable olduğunu görmem gerekiyor bunun içinde bir diagnostik topiği yaz yada varsa bu topiği revize et.

## Problem Statement (English Translation)
For each sensor input, I want to add enable/disable parameters, and these parameters should be dynamic so that I can change them at runtime using the ros2 parameter set command. If an input is disabled, it should not be added to the EKF; if enabled, it should be added. Also, I need to see which sensors are enabled/disabled, so write a diagnostic topic for this or revise it if it exists.

## Solution Overview

The implementation adds a dynamic enable/disable feature for all sensor inputs in the robot_localization package. Each sensor (odom, pose, twist, imu) can now be:
1. Configured as enabled/disabled at startup
2. Dynamically enabled/disabled at runtime using ROS2 parameter commands
3. Monitored through the diagnostics topic

## Implementation Details

### 1. Data Structure
- Added `sensor_enabled_` map (`std::map<std::string, bool>`) to track the enable/disable state of each sensor
- Sensor names are stored with `_enabled` suffix (e.g., "odom0_enabled", "imu0_enabled")

### 2. Parameter Declaration
In `loadParams()`, for each sensor type (odom, pose, twist, imu), the code now declares an `_enabled` parameter:
```cpp
bool sensor_enabled = this->declare_parameter(
    sensor_name + std::string("_enabled"), true);  // Default: true
sensor_enabled_[sensor_name + "_enabled"] = sensor_enabled;
```

### 3. Runtime Parameter Changes
Implemented `parametersCallback()` method that:
- Listens for parameter changes
- Updates the `sensor_enabled_` map when `_enabled` parameters are changed
- Provides feedback through ROS logging
- Returns success/failure status

### 4. Sensor Callback Modifications
All sensor callbacks now check if the sensor is enabled before processing:
- `odometryCallback()` - checks "odom<N>_enabled"
- `poseCallback()` - checks "pose<N>_enabled"
- `twistCallback()` - checks "twist<N>_enabled"
- `imuCallback()` - checks "imu<N>_enabled"
- `accelerationCallback()` - checks the parent sensor's enable state

Example check:
```cpp
std::string enable_param_name = sensor_name + "_enabled";
if (sensor_enabled_.find(enable_param_name) != sensor_enabled_.end() &&
    !sensor_enabled_[enable_param_name]) {
  RF_DEBUG("Sensor " << sensor_name << " is disabled, ignoring measurement");
  return;
}
```

### 5. Diagnostics Enhancement
Modified `aggregateDiagnostics()` to include sensor status information:
- Lists all enabled sensors
- Lists all disabled sensors
- Displays in the format: "Enabled: odom0, imu0 | Disabled: pose0, twist0"
- Integrated into existing diagnostics workflow

## Files Modified

### Core Implementation
1. **include/robot_localization/ros_filter.hpp**
   - Added `sensor_enabled_` member variable
   - Added `parametersCallback()` method declaration
   - Added `parameters_callback_handle_` for callback registration

2. **src/ros_filter.cpp**
   - Implemented `parametersCallback()` method
   - Modified `loadParams()` to declare `_enabled` parameters
   - Modified `initialize()` to register parameter callback
   - Updated all sensor callbacks with enable checks
   - Enhanced `aggregateDiagnostics()` to show sensor status

### Documentation
3. **doc/SENSOR_ENABLE_DISABLE.md**
   - Complete usage guide
   - Configuration examples
   - Runtime usage commands
   - Use cases and implementation details

4. **params/ekf_with_enable_disable_example.yaml**
   - Example configuration file
   - Shows how to use `_enabled` parameters for all sensor types
   - Includes inline comments explaining usage

### Testing
5. **test/test_sensor_enable_disable.cpp**
   - Unit tests for parameter declaration
   - Tests for runtime parameter changes
   - Backward compatibility tests

## Usage Examples

### Configuration (YAML)
```yaml
ekf_filter_node:
    ros__parameters:
        odom0: /robot/odom
        odom0_config: [...]
        odom0_enabled: true  # Enable at startup
        
        imu0: /robot/imu
        imu0_config: [...]
        imu0_enabled: false  # Disable at startup
```

### Runtime Commands
```bash
# Disable a sensor
ros2 param set /ekf_filter_node odom0_enabled false

# Enable a sensor
ros2 param set /ekf_filter_node imu0_enabled true

# Check current status
ros2 param get /ekf_filter_node odom0_enabled

# View diagnostics
ros2 topic echo /diagnostics
```

## Key Features

### 1. Backward Compatibility
- Existing configurations work without modification
- Sensors without `_enabled` parameter default to enabled
- No breaking changes to existing API

### 2. Runtime Flexibility
- Parameters can be changed while the node is running
- Changes take effect immediately on next sensor callback
- No need to restart the node

### 3. Diagnostic Monitoring
- Real-time visibility of sensor states
- Integrated into existing diagnostics infrastructure
- Easy to monitor with standard ROS2 tools

### 4. Safety Features
- Subscriptions remain active even when disabled
- State can be quickly restored by re-enabling
- No data loss or reconnection overhead

## Benefits

1. **Debugging**: Quickly isolate sensor issues by disabling suspect sensors
2. **Testing**: A/B test different sensor configurations without restarts
3. **Fault Recovery**: Automatically disable malfunctioning sensors
4. **Conditional Operation**: Disable GPS indoors, enable outdoors
5. **Resource Management**: Reduce computational load by disabling unnecessary sensors
6. **Calibration**: Disable sensor during recalibration procedures

## Testing Recommendations

When testing in a ROS2 environment:

1. **Basic Functionality Test**:
   ```bash
   ros2 run robot_localization ekf_node --ros-args --params-file test_config.yaml
   ros2 param set /ekf_filter_node odom0_enabled false
   ros2 topic echo /diagnostics
   ```

2. **Integration Test**:
   - Publish sensor data to multiple topics
   - Toggle enable/disable parameters
   - Verify filter output changes appropriately

3. **Performance Test**:
   - Measure CPU usage with all sensors enabled vs disabled
   - Verify no memory leaks during enable/disable cycles

4. **Stress Test**:
   - Rapidly toggle sensor states
   - Ensure filter remains stable

## Future Enhancements (Optional)

1. Service interface for batch enable/disable operations
2. Automatic sensor health monitoring with auto-disable
3. Save/restore sensor configuration profiles
4. Integration with parameter file generation tools

## Conclusion

This implementation successfully addresses all requirements from the problem statement:
- ✅ Enable/disable parameters for each sensor
- ✅ Dynamic runtime configuration via ROS2 parameters
- ✅ Disabled sensors excluded from EKF
- ✅ Diagnostic topic showing sensor status
- ✅ Backward compatible implementation
- ✅ Documentation and examples provided
