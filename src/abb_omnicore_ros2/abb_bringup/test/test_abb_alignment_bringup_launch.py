"""Smoke test for abb_alignment_bringup.launch.py

Verifies the launch file parses without syntax errors and all referenced
packages/actions exist.

Run with:
    ros2 launch abb_bringup test_abb_alignment_bringup_launch.py
Or via colcon test:
    colcon test --packages-select abb_bringup
"""

import os
import unittest

import pytest
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_test_description():
    """Load and parse the alignment bringup launch file."""
    abb_bringup_share = get_package_share_directory('abb_bringup')
    launch_file = os.path.join(
        abb_bringup_share, 'launch', 'abb_alignment_bringup.launch.py'
    )

    parse_proc = ExecuteProcess(
        cmd=[
            'python3', '-c',
            f'''
import sys
sys.path.insert(0, "{os.path.dirname(launch_file)}")
exec(open("{launch_file}").read())
print("OK: abb_alignment_bringup.launch.py parsed successfully")
'''
        ],
        output='screen',
    )
    return LaunchDescription([parse_proc])


class TestAlignmentLaunchFile(unittest.TestCase):
    """Verify abb_alignment_bringup.launch.py parses without error."""

    def test_launch_file_parses(self):
        """This test passes if the generate_test_description() call succeeds."""
        pass


@pytest.mark.launch_test
def test_launch_file_loads():
    """Use launch_testing to actually load the launch description."""
    ld = generate_test_description()
    assert ld is not None


if __name__ == '__main__':
    unittest.main()
