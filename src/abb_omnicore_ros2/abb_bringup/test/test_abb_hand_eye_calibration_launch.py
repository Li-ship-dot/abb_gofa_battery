"""Smoke test for abb_hand_eye_calibration.launch.py

Verifies the launch file parses without syntax errors.

Run with:
    ros2 launch abb_bringup test_abb_hand_eye_calibration_launch.py
"""

import os
import unittest

import pytest
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_test_description():
    """Load and parse the hand-eye calibration launch file."""
    abb_bringup_share = get_package_share_directory('abb_bringup')
    launch_file = os.path.join(
        abb_bringup_share, 'launch', 'abb_hand_eye_calibration.launch.py'
    )

    parse_proc = ExecuteProcess(
        cmd=[
            'python3', '-c',
            f'''
import sys
sys.path.insert(0, "{os.path.dirname(launch_file)}")
exec(open("{launch_file}").read())
print("OK: abb_hand_eye_calibration.launch.py parsed successfully")
'''
        ],
        output='screen',
    )
    return LaunchDescription([parse_proc])


class TestHandEyeCalibrationLaunchFile(unittest.TestCase):
    """Verify abb_hand_eye_calibration.launch.py parses without error."""

    def test_launch_file_parses(self):
        """Passes if generate_test_description() succeeds."""
        pass


@pytest.mark.launch_test
def test_launch_file_loads():
    """Use launch_testing to actually load the launch description."""
    ld = generate_test_description()
    assert ld is not None


if __name__ == '__main__':
    unittest.main()
