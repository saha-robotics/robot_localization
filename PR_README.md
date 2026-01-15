# Pull Request: Dynamic Sensor Enable/Disable Feature

## Summary

This PR implements a dynamic enable/disable feature for all sensor inputs in robot_localization, allowing sensors to be configured and controlled at runtime without restarting the node.

## Problem Statement (Original - Turkish)

Girişteki her bir sensör için, enable/disable parametresi eklemek istiyorum ayrıca, bu parametreler dinamik olsun yani runtime esnasında ros2 parameter set komutu ile değiştirebiliyim. Eğer bir girdi disable ise bu girdiyi ekfye sokmasın enable durumunda ise eklesin. Ayrıca hangi sensörlerin enable/disable olduğunu görmem gerekiyor bunun içinde bir diagnostik topiği yaz yada varsa bu topiği revize et.

**Translation:**
For each sensor input, I want to add enable/disable parameters, and these parameters should be dynamic so that I can change them at runtime using the ros2 parameter set command. If an input is disabled, it should not be added to the EKF; if enabled, it should be added. Also, I need to see which sensors are enabled/disabled, so write a diagnostic topic for this or revise it if it exists.

## Key Features

✅ **Dynamic Configuration**: Each sensor (odom, pose, twist, imu) can be enabled/disabled at startup  
✅ **Runtime Control**: Change sensor state at runtime using `ros2 param set` without restarting  
✅ **EKF Integration**: Disabled sensors are excluded from state estimation  
✅ **Diagnostics**: Real-time visibility of sensor states via diagnostics topic  
✅ **Backward Compatible**: Existing configurations work without modification  

## Implementation Details

### Files Modified

1. **include/robot_localization/ros_filter.hpp**
   - Added `sensor_enabled_` map for tracking sensor states
   - Added `parametersCallback` method declaration
   - Added `parameters_callback_handle_` member variable

2. **src/ros_filter.cpp**
   - Added `ENABLED_SUFFIX_LENGTH` constant
   - Implemented `parametersCallback` for runtime changes
   - Modified `loadParams` to declare `_enabled` parameters
   - Updated all sensor callbacks with enable checks:
     - `odometryCallback`
     - `poseCallback`
     - `twistCallback`
     - `imuCallback`
     - `accelerationCallback`
   - Enhanced `aggregateDiagnostics` with sensor status display

### Files Created

3. **doc/SENSOR_ENABLE_DISABLE.md**
   - Complete usage documentation
   - Configuration examples
   - Use cases and best practices

4. **params/ekf_with_enable_disable_example.yaml**
   - Example configuration file
   - Shows usage for all sensor types

5. **test/test_sensor_enable_disable.cpp**
   - Unit tests for parameter declaration
   - Runtime parameter change tests
   - Backward compatibility tests

6. **IMPLEMENTATION_SUMMARY.md**
   - Detailed implementation notes
   - Architecture decisions
   - Future enhancement suggestions

## Usage Examples

### Configuration File (YAML)

```yaml
ekf_filter_node:
    ros__parameters:
        # Odometry sensor - enabled at startup
        odom0: /robot/odom
        odom0_config: [true, true, false, ...]
        odom0_enabled: true  # NEW PARAMETER
        
        # IMU sensor - disabled at startup
        imu0: /robot/imu
        imu0_config: [false, false, false, true, ...]
        imu0_enabled: false  # NEW PARAMETER
```

### Runtime Commands

```bash
# Disable a sensor
ros2 param set /ekf_filter_node odom0_enabled false

# Enable a sensor
ros2 param set /ekf_filter_node imu0_enabled true

# Check current value
ros2 param get /ekf_filter_node odom0_enabled

# View all parameters
ros2 param list /ekf_filter_node

# Monitor sensor status
ros2 topic echo /diagnostics
```

### Diagnostic Output

```
values:
  - key: "Sensor Status"
    value: "Enabled: odom0, imu0 | Disabled: pose0, twist0"
```

## Code Quality Improvements

✅ Replaced magic numbers with named constants (`ENABLED_SUFFIX_LENGTH`)  
✅ Optimized map lookups (single lookup instead of double)  
✅ Added comprehensive inline documentation  
✅ Included unit tests for new functionality  

## Testing

### Manual Testing Checklist

When testing in a ROS2 environment:

- [ ] Start EKF node with example configuration
- [ ] Verify sensors configured as enabled are processing data
- [ ] Verify sensors configured as disabled are not processing data
- [ ] Disable an enabled sensor at runtime
- [ ] Verify disabled sensor stops contributing to EKF
- [ ] Re-enable the sensor at runtime
- [ ] Verify re-enabled sensor resumes contributing to EKF
- [ ] Check diagnostics topic shows correct sensor states
- [ ] Test with existing configuration files (backward compatibility)
- [ ] Verify no memory leaks during enable/disable cycles

### Unit Tests

Run the included unit tests:
```bash
colcon test --packages-select robot_localization
colcon test-result --verbose
```

## Backward Compatibility

✅ **Fully backward compatible**  
- Existing configuration files work without modification
- Sensors without `_enabled` parameter default to enabled
- No breaking changes to existing API
- No changes to message formats or topics

## Use Cases

1. **Sensor Fault Recovery**: Quickly disable malfunctioning sensors
2. **A/B Testing**: Compare filter performance with different sensor combinations
3. **Conditional Sensing**: Disable GPS indoors, enable outdoors
4. **Resource Management**: Reduce CPU load by disabling unused sensors
5. **Sensor Calibration**: Disable sensor during recalibration
6. **Debugging**: Isolate sensor issues by selectively disabling inputs

## Performance Impact

- **Minimal overhead**: Single map lookup per sensor callback
- **No impact when disabled**: Callbacks return early
- **No memory overhead**: Small map structure with boolean values
- **No latency increase**: Constant-time enable state check

## Documentation

- ✅ Usage guide: `doc/SENSOR_ENABLE_DISABLE.md`
- ✅ Example configuration: `params/ekf_with_enable_disable_example.yaml`
- ✅ Implementation details: `IMPLEMENTATION_SUMMARY.md`
- ✅ Inline code documentation
- ✅ Unit test examples

## Future Enhancements (Optional)

The following enhancements could be added in future PRs:

1. Service interface for batch operations
2. Automatic sensor health monitoring with auto-disable
3. Configuration profile save/restore functionality
4. Sensor status publisher (dedicated topic)

## Commits

1. `415403b` - Add enable/disable parameters and runtime reconfiguration support
2. `de611af` - Add documentation and example configuration
3. `65f26f6` - Add acceleration callback check and unit tests
4. `bc096d1` - Address code review feedback (constants and optimizations)

## Testing Status

⚠️ **Note**: Full integration testing requires a ROS2 build environment which is not available in the current development environment. The code has been:
- ✅ Reviewed for syntax correctness
- ✅ Checked for logical correctness
- ✅ Verified for backward compatibility
- ⏳ Pending: Build verification
- ⏳ Pending: Integration testing
- ⏳ Pending: Existing test suite execution

## Review Checklist

- [x] Code implements all requirements from problem statement
- [x] Backward compatible with existing configurations
- [x] No breaking changes
- [x] Documentation provided
- [x] Examples provided
- [x] Unit tests included
- [x] Code review feedback addressed
- [x] Performance optimizations applied
- [x] Inline documentation added
- [ ] Build verification (requires ROS2 environment)
- [ ] Integration tests passed (requires ROS2 environment)
- [ ] Existing test suite passed (requires ROS2 environment)

## Deployment Notes

After merging:
1. Update package version number
2. Update CHANGELOG.rst
3. Announce new feature in release notes
4. Update user documentation/wiki
5. Consider blog post for major feature

## Support

For questions or issues with this feature:
- See documentation: `doc/SENSOR_ENABLE_DISABLE.md`
- See examples: `params/ekf_with_enable_disable_example.yaml`
- Open an issue on GitHub with the `enhancement` label
