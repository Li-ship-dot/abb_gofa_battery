# Launch Integration Tests

This directory contains smoke tests for abb_bringup launch files.

## Tests

- `test_abb_cscan_bringup_launch.py` - Smoke test for C-scan bringup launch
- `test_abb_alignment_bringup_launch.py` - Smoke test for alignment bringup launch
- `test_abb_hand_eye_calibration_launch.py` - Smoke test for hand-eye calibration launch

## Running Tests

```bash
# Run all tests for abb_bringup
colcon test --packages-select abb_bringup --event-handlers console_direct+

# View test results
colcon test-result --verbose --packages-select abb_bringup

# Run a specific test
python3 src/abb_omnicore_ros2/abb_bringup/test/test_abb_cscan_bringup_launch.py
```

## Purpose

These tests verify that launch files:
1. Parse without Python syntax errors
2. Can be loaded by the ROS2 launch system
3. Reference valid packages and actions

They do NOT verify runtime behavior (that requires actual hardware/simulation).
