"""Mock Chain E2E Integration Test for abb_cscan_bringup.

Tests the complete mock hardware chain: sim=true + use_mock_hand_eye=true + use_mock_ni=true.
Verifies all critical nodes start and key topics are published.

Run with:
    colcon test --packages-select abb_bringup --event-handlers console_direct+
    colcon test-result --verbose

Or standalone:
    python3 test_mock_chain_integration.py
"""

import os
import subprocess
import sys
import time
import unittest

import pytest


# Critical topics that MUST be published in mock mode
CRITICAL_TOPICS = [
    '/joint_states',
    '/battery_poses',
    '/battery_bboxes',
    '/target_joint_states',
    '/abb_controller/joint_trajectory',
]


class TestMockChainIntegration(unittest.TestCase):
    """E2E test for full mock hardware chain."""

    @classmethod
    def setUpClass(cls):
        """Launch the mock chain once before all tests."""
        # Kill any existing ros2 processes from previous tests
        subprocess.run(['pkill', '-9', '-f', 'abb_bringup|abb_vision'], stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-9', '-f', 'ros2 daemon'], stderr=subprocess.DEVNULL)
        subprocess.run(['sleep', '2'], stderr=subprocess.DEVNULL)

        # Start fresh ros2 daemon
        subprocess.run(['ros2', 'daemon', 'stop'], stderr=subprocess.DEVNULL)
        subprocess.run(['ros2', 'daemon', 'start'], stderr=subprocess.DEVNULL)
        time.sleep(2)

        # Source environment
        cls.env = os.environ.copy()
        ws_setup = os.path.join(os.path.dirname(__file__), '../../../install/setup.bash')
        if os.path.exists(ws_setup):
            cls.env['BASH_ENV'] = ws_setup

        # Launch the bringup in background
        launch_cmd = [
            'bash', '-c',
            'source /opt/ros/humble/setup.bash 2>/dev/null; '
            'source install/setup.bash 2>/dev/null; '
            'ros2 launch abb_bringup abb_cscan_bringup.launch.py '
            'sim:=true use_mock_hand_eye:=true use_mock_ni:=true'
        ]

        cls.launch_proc = subprocess.Popen(
            launch_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=cls.env,
        )

        # Wait for nodes to initialize (25 seconds)
        print(f"\n[MockChain] Launch PID: {cls.launch_proc.pid}, waiting 25s for initialization...")
        time.sleep(25)
        print("[MockChain] Initialization wait complete, running checks...")

    @classmethod
    def tearDownClass(cls):
        """Kill the launch process."""
        if hasattr(cls, 'launch_proc') and cls.launch_proc:
            print(f"\n[MockChain] Killing launch process {cls.launch_proc.pid}")
            cls.launch_proc.terminate()
            try:
                cls.launch_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                cls.launch_proc.kill()

    def _run_ros2(self, args, timeout=15):
        """Run a ros2 CLI command and return stdout."""
        cmd = ['bash', '-c', f'source /opt/ros/humble/setup.bash 2>/dev/null; source install/setup.bash 2>/dev/null; ros2 {" ".join(args)}']
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            env=self.env,
        )
        return result.stdout + result.stderr

    def test_topics_exist(self):
        """Verify all critical topics are published."""
        topics_output = self._run_ros2(['topic', 'list'])
        print(f"\n[Topics]\n{topics_output[:500]}")

        for topic in CRITICAL_TOPICS:
            self.assertIn(topic, topics_output,
                f"Critical topic '{topic}' not found. Available topics:\n{topics_output[:1000]}")

    def test_battery_poses_has_publisher(self):
        """Verify /battery_poses has an active publisher."""
        info_output = self._run_ros2(['topic', 'info', '/battery_poses'])
        print(f"\n[/battery_poses info]\n{info_output}")

        # Should have Publisher count >= 1
        self.assertIn('Publisher count: 1', info_output,
            f"/battery_poses has no publisher. Info:\n{info_output}")

    def test_target_joint_states_has_publisher(self):
        """Verify /target_joint_states has an active publisher."""
        info_output = self._run_ros2(['topic', 'info', '/target_joint_states'])
        print(f"\n[/target_joint_states info]\n{info_output}")

        self.assertIn('Publisher count: 1', info_output,
            f"/target_joint_states has no publisher. Info:\n{info_output}")

    def test_no_node_crashes(self):
        """Verify no nodes crashed during startup."""
        # Check if any abb_vision nodes crashed
        ps_output = subprocess.run(
            ['bash', '-c', 'ps aux | grep -E "battery|cscan|mock_data|quintic|joint_to_cart" | grep -v grep'],
            capture_output=True, text=True,
        )
        print(f"\n[Node processes]\n{ps_output.stdout}")

        # Verify at least some nodes are running
        self.assertGreater(len(ps_output.stdout.strip().split('\n')), 2,
            "Too few nodes running - possible crash during startup")

    def test_cscan_udp_bridge_mock_mode(self):
        """Verify cscan_udp_bridge is running in mock NI mode."""
        ni_status = self._run_ros2(['topic', 'echo', '/cscan_ni_status', '--once'])
        print(f"\n[/cscan_ni_status]\n{ni_status}")
        # In mock mode, NI status should be True (connected)
        self.assertIn('data: true', ni_status.lower(),
            f"cscan_udp_bridge not in mock mode. NI status:\n{ni_status}")


if __name__ == '__main__':
    # Allow running standalone for local debugging
    unittest.main()
