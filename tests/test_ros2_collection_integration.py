"""Opt-in real ROS regression; never connects to hardware or the default domain."""
import os
import pytest

@pytest.mark.skipif(os.environ.get('GAMEPAD_VLA_TEST_ROS2')!='1', reason='set GAMEPAD_VLA_TEST_ROS2=1 and an isolated ROS_DOMAIN_ID to run real rclpy')
def test_real_ros_lossless_latest_and_latency(tmp_path):
    pytest.importorskip('rclpy')
    from gamepad_vla.workstation.ros2_check import check_ros2
    result=check_ros2(tmp_path/'real-ros',duration=2.)
    assert result['recorded_total']==result['published_total']==800
    assert result['publish_to_apply_p95_ms']<=25
    assert result['pass_gate']
