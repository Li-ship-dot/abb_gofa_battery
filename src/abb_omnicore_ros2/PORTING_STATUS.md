# abb_omnicore_ros2 / abb_rws_client porting status

## Verified working on real controller
- /rws/system_states
- /rws/joint_states
- /rws/get_robot_controller_description
- /rws/get_speed_ratio
- /rws/get_file_contents
- /rws/set_file_contents

## Implemented, compile OK, runtime pending
- /rws/set_speed_ratio
- /rws/pp_to_main
- /rws/set_motors_off
- /rws/set_motors_on
- /rws/start_rapid
- /rws/stop_rapid

## Next offline restore batch
- /rws/get_rapid_symbol
- /rws/set_rapid_symbol
- /rws/run_rapid_routine
- /rws/run_sg_routine
- /rws/set_rapid_routine
- /rws/start_egm_joint
- /rws/start_egm_pose
- /rws/start_egm_stream
- /rws/stop_egm
- /rws/stop_egm_stream

## Still stub / unresolved
- IO services
- typed RAPID bool/dnum/num/string get/set
- EGM settings
- SG command
