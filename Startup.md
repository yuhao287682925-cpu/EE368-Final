yuhao@ubuntu:~/catkin_workspace$ roslaunch kortex_driver kortex_driver.launch arm:="gen3_lite" ip_address:="192.168.1.10"
... logging to /home/yuhao/.ros/log/82589fe6-542f-11f1-bb14-63fc6edc440d/roslaunch-ubuntu-4312.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

xacro: in-order processing became default in ROS Melodic. You can drop the option.
started roslaunch server http://ubuntu:45213/

SUMMARY
=======

PARAMETERS

* /my_gen3_lite/hztest_test_base_feedback/hz: 40
* /my_gen3_lite/hztest_test_base_feedback/hzerror: 5.0
* /my_gen3_lite/hztest_test_base_feedback/test_duration: 30.0
* /my_gen3_lite/hztest_test_base_feedback/topic: base_feedback
* /my_gen3_lite/hztest_test_base_feedback/wait_time: 10.0
* /my_gen3_lite/hztest_test_driver_joint_state/hz: 40
* /my_gen3_lite/hztest_test_driver_joint_state/hzerror: 5.0
* /my_gen3_lite/hztest_test_driver_joint_state/test_duration: 30.0
* /my_gen3_lite/hztest_test_driver_joint_state/topic: base_feedback/joi...
* /my_gen3_lite/hztest_test_driver_joint_state/wait_time: 10.0
* /my_gen3_lite/hztest_test_joint_state_publisher_joint_states/hz: 40
* /my_gen3_lite/hztest_test_joint_state_publisher_joint_states/hzerror: 5.0
* /my_gen3_lite/hztest_test_joint_state_publisher_joint_states/test_duration: 30.0
* /my_gen3_lite/hztest_test_joint_state_publisher_joint_states/topic: joint_states
* /my_gen3_lite/hztest_test_joint_state_publisher_joint_states/wait_time: 10.0
* /my_gen3_lite/joint_state_publisher/rate: 40
* /my_gen3_lite/joint_state_publisher/source_list: ['base_feedback/j...
* /my_gen3_lite/kortex_driver_tests/api_connection_inactivity_timeout_ms: 20000
* /my_gen3_lite/kortex_driver_tests/api_rpc_timeout_ms: 2000
* /my_gen3_lite/kortex_driver_tests/api_session_inactivity_timeout_ms: 35000
* /my_gen3_lite/kortex_driver_tests/arm: gen3_lite
* /my_gen3_lite/kortex_driver_tests/cyclic_data_publish_rate: 40
* /my_gen3_lite/kortex_driver_tests/default_goal_time_tolerance: 0.5
* /my_gen3_lite/kortex_driver_tests/default_goal_tolerance: 0.005
* /my_gen3_lite/kortex_driver_tests/dof: 6
* /my_gen3_lite/kortex_driver_tests/gripper: gen3_lite_2f
* /my_gen3_lite/kortex_driver_tests/gripper_joint_limits_max: [-0.09]
* /my_gen3_lite/kortex_driver_tests/gripper_joint_limits_min: [0.96]
* /my_gen3_lite/kortex_driver_tests/gripper_joint_names: ['right_finger_bo...
* /my_gen3_lite/kortex_driver_tests/ip_address: 192.168.1.10
* /my_gen3_lite/kortex_driver_tests/joint_names: ['joint_1', 'join...
* /my_gen3_lite/kortex_driver_tests/maximum_accelerations: [1.0, 0.5, 0.4, 1...
* /my_gen3_lite/kortex_driver_tests/maximum_velocities: [0.5, 0.5, 0.5, 0...
* /my_gen3_lite/move_group/allow_trajectory_execution: True
* /my_gen3_lite/move_group/arm/default_planner_config: RRTConnect
* /my_gen3_lite/move_group/arm/longest_valid_segment_fraction: 0.005
* /my_gen3_lite/move_group/arm/planner_configs: ['SBL', 'EST', 'L...
* /my_gen3_lite/move_group/arm/projection_evaluator: joints(joint_1,jo...
* /my_gen3_lite/move_group/capabilities:
* /my_gen3_lite/move_group/controller_list: [{'name': 'gen3_l...
* /my_gen3_lite/move_group/default_workspace_bounds: 10
* /my_gen3_lite/move_group/disable_capabilities:
* /my_gen3_lite/move_group/gripper/default_planner_config: RRTConnect
* /my_gen3_lite/move_group/gripper/planner_configs: ['SBL', 'EST', 'L...
* /my_gen3_lite/move_group/jiggle_fraction: 0.05
* /my_gen3_lite/move_group/joint_state_controller/publish_rate: 50
* /my_gen3_lite/move_group/joint_state_controller/type: joint_state_contr...
* /my_gen3_lite/move_group/max_safe_path_cost: 1
* /my_gen3_lite/move_group/max_sampling_attempts: 100
* /my_gen3_lite/move_group/moveit_controller_manager: moveit_simple_con...
* /my_gen3_lite/move_group/moveit_manage_controllers: True
* /my_gen3_lite/move_group/planner_configs/BFMT/balanced: 0
* /my_gen3_lite/move_group/planner_configs/BFMT/cache_cc: 1
* /my_gen3_lite/move_group/planner_configs/BFMT/extended_fmt: 1
* /my_gen3_lite/move_group/planner_configs/BFMT/heuristics: 1
* /my_gen3_lite/move_group/planner_configs/BFMT/nearest_k: 1
* /my_gen3_lite/move_group/planner_configs/BFMT/num_samples: 1000
* /my_gen3_lite/move_group/planner_configs/BFMT/optimality: 1
* /my_gen3_lite/move_group/planner_configs/BFMT/radius_multiplier: 1.0
* /my_gen3_lite/move_group/planner_configs/BFMT/type: geometric::BFMT
* /my_gen3_lite/move_group/planner_configs/BKPIECE/border_fraction: 0.9
* /my_gen3_lite/move_group/planner_configs/BKPIECE/failed_expansion_score_factor: 0.5
* /my_gen3_lite/move_group/planner_configs/BKPIECE/min_valid_path_fraction: 0.5
* /my_gen3_lite/move_group/planner_configs/BKPIECE/range: 0.0
* /my_gen3_lite/move_group/planner_configs/BKPIECE/type: geometric::BKPIECE
* /my_gen3_lite/move_group/planner_configs/BiEST/range: 0.0
* /my_gen3_lite/move_group/planner_configs/BiEST/type: geometric::BiEST
* /my_gen3_lite/move_group/planner_configs/BiTRRT/cost_threshold: 1e300
* /my_gen3_lite/move_group/planner_configs/BiTRRT/frountier_node_ratio: 0.1
* /my_gen3_lite/move_group/planner_configs/BiTRRT/frountier_threshold: 0.0
* /my_gen3_lite/move_group/planner_configs/BiTRRT/init_temperature: 100
* /my_gen3_lite/move_group/planner_configs/BiTRRT/range: 0.0
* /my_gen3_lite/move_group/planner_configs/BiTRRT/temp_change_factor: 0.1
* /my_gen3_lite/move_group/planner_configs/BiTRRT/type: geometric::BiTRRT
* /my_gen3_lite/move_group/planner_configs/EST/goal_bias: 0.05
* /my_gen3_lite/move_group/planner_configs/EST/range: 0.0
* /my_gen3_lite/move_group/planner_configs/EST/type: geometric::EST
* /my_gen3_lite/move_group/planner_configs/FMT/cache_cc: 1
* /my_gen3_lite/move_group/planner_configs/FMT/extended_fmt: 1
* /my_gen3_lite/move_group/planner_configs/FMT/heuristics: 0
* /my_gen3_lite/move_group/planner_configs/FMT/nearest_k: 1
* /my_gen3_lite/move_group/planner_configs/FMT/num_samples: 1000
* /my_gen3_lite/move_group/planner_configs/FMT/radius_multiplier: 1.1
* /my_gen3_lite/move_group/planner_configs/FMT/type: geometric::FMT
* /my_gen3_lite/move_group/planner_configs/KPIECE/border_fraction: 0.9
* /my_gen3_lite/move_group/planner_configs/KPIECE/failed_expansion_score_factor: 0.5
* /my_gen3_lite/move_group/planner_configs/KPIECE/goal_bias: 0.05
* /my_gen3_lite/move_group/planner_configs/KPIECE/min_valid_path_fraction: 0.5
* /my_gen3_lite/move_group/planner_configs/KPIECE/range: 0.0
* /my_gen3_lite/move_group/planner_configs/KPIECE/type: geometric::KPIECE
* /my_gen3_lite/move_group/planner_configs/LBKPIECE/border_fraction: 0.9
* /my_gen3_lite/move_group/planner_configs/LBKPIECE/min_valid_path_fraction: 0.5
* /my_gen3_lite/move_group/planner_configs/LBKPIECE/range: 0.0
* /my_gen3_lite/move_group/planner_configs/LBKPIECE/type: geometric::LBKPIECE
* /my_gen3_lite/move_group/planner_configs/LBTRRT/epsilon: 0.4
* /my_gen3_lite/move_group/planner_configs/LBTRRT/goal_bias: 0.05
* /my_gen3_lite/move_group/planner_configs/LBTRRT/range: 0.0
* /my_gen3_lite/move_group/planner_configs/LBTRRT/type: geometric::LBTRRT
* /my_gen3_lite/move_group/planner_configs/LazyPRM/range: 0.0
* /my_gen3_lite/move_group/planner_configs/LazyPRM/type: geometric::LazyPRM
* /my_gen3_lite/move_group/planner_configs/LazyPRMstar/type: geometric::LazyPR...
* /my_gen3_lite/move_group/planner_configs/PDST/type: geometric::PDST
* /my_gen3_lite/move_group/planner_configs/PRM/max_nearest_neighbors: 10
* /my_gen3_lite/move_group/planner_configs/PRM/type: geometric::PRM
* /my_gen3_lite/move_group/planner_configs/PRMstar/type: geometric::PRMstar
* /my_gen3_lite/move_group/planner_configs/ProjEST/goal_bias: 0.05
* /my_gen3_lite/move_group/planner_configs/ProjEST/range: 0.0
* /my_gen3_lite/move_group/planner_configs/ProjEST/type: geometric::ProjEST
* /my_gen3_lite/move_group/planner_configs/RRT/goal_bias: 0.05
* /my_gen3_lite/move_group/planner_configs/RRT/range: 0.0
* /my_gen3_lite/move_group/planner_configs/RRT/type: geometric::RRT
* /my_gen3_lite/move_group/planner_configs/RRTConnect/range: 0.0
* /my_gen3_lite/move_group/planner_configs/RRTConnect/type: geometric::RRTCon...
* /my_gen3_lite/move_group/planner_configs/RRTstar/delay_collision_checking: 1
* /my_gen3_lite/move_group/planner_configs/RRTstar/goal_bias: 0.05
* /my_gen3_lite/move_group/planner_configs/RRTstar/range: 0.0
* /my_gen3_lite/move_group/planner_configs/RRTstar/type: geometric::RRTstar
* /my_gen3_lite/move_group/planner_configs/SBL/range: 0.0
* /my_gen3_lite/move_group/planner_configs/SBL/type: geometric::SBL
* /my_gen3_lite/move_group/planner_configs/SPARS/dense_delta_fraction: 0.001
* /my_gen3_lite/move_group/planner_configs/SPARS/max_failures: 1000
* /my_gen3_lite/move_group/planner_configs/SPARS/sparse_delta_fraction: 0.25
* /my_gen3_lite/move_group/planner_configs/SPARS/stretch_factor: 3.0
* /my_gen3_lite/move_group/planner_configs/SPARS/type: geometric::SPARS
* /my_gen3_lite/move_group/planner_configs/SPARStwo/dense_delta_fraction: 0.001
* /my_gen3_lite/move_group/planner_configs/SPARStwo/max_failures: 5000
* /my_gen3_lite/move_group/planner_configs/SPARStwo/sparse_delta_fraction: 0.25
* /my_gen3_lite/move_group/planner_configs/SPARStwo/stretch_factor: 3.0
* /my_gen3_lite/move_group/planner_configs/SPARStwo/type: geometric::SPARStwo
* /my_gen3_lite/move_group/planner_configs/STRIDE/degree: 16
* /my_gen3_lite/move_group/planner_configs/STRIDE/estimated_dimension: 0.0
* /my_gen3_lite/move_group/planner_configs/STRIDE/goal_bias: 0.05
* /my_gen3_lite/move_group/planner_configs/STRIDE/max_degree: 18
* /my_gen3_lite/move_group/planner_configs/STRIDE/max_pts_per_leaf: 6
* /my_gen3_lite/move_group/planner_configs/STRIDE/min_degree: 12
* /my_gen3_lite/move_group/planner_configs/STRIDE/min_valid_path_fraction: 0.2
* /my_gen3_lite/move_group/planner_configs/STRIDE/range: 0.0
* /my_gen3_lite/move_group/planner_configs/STRIDE/type: geometric::STRIDE
* /my_gen3_lite/move_group/planner_configs/STRIDE/use_projected_distance: 0
* /my_gen3_lite/move_group/planner_configs/TRRT/frountierNodeRatio: 0.1
* /my_gen3_lite/move_group/planner_configs/TRRT/frountier_threshold: 0.0
* /my_gen3_lite/move_group/planner_configs/TRRT/goal_bias: 0.05
* /my_gen3_lite/move_group/planner_configs/TRRT/init_temperature: 10e-6
* /my_gen3_lite/move_group/planner_configs/TRRT/k_constant: 0.0
* /my_gen3_lite/move_group/planner_configs/TRRT/max_states_failed: 10
* /my_gen3_lite/move_group/planner_configs/TRRT/min_temperature: 10e-10
* /my_gen3_lite/move_group/planner_configs/TRRT/range: 0.0
* /my_gen3_lite/move_group/planner_configs/TRRT/temp_change_factor: 2.0
* /my_gen3_lite/move_group/planner_configs/TRRT/type: geometric::TRRT
* /my_gen3_lite/move_group/planning_plugin: ompl_interface/OM...
* /my_gen3_lite/move_group/planning_scene_monitor/publish_geometry_updates: True
* /my_gen3_lite/move_group/planning_scene_monitor/publish_planning_scene: True
* /my_gen3_lite/move_group/planning_scene_monitor/publish_state_updates: True
* /my_gen3_lite/move_group/planning_scene_monitor/publish_transforms_updates: True
* /my_gen3_lite/move_group/request_adapters: default_planner_r...
* /my_gen3_lite/move_group/start_state_max_bounds_error: 0.1
* /my_gen3_lite/move_group/start_state_max_dt: 0.5
* /my_gen3_lite/move_group/trajectory_execution/allowed_execution_duration_scaling: 1.2
* /my_gen3_lite/move_group/trajectory_execution/allowed_goal_duration_margin: 2.0
* /my_gen3_lite/move_group/trajectory_execution/allowed_start_tolerance: 0.01
* /my_gen3_lite/my_gen3_lite_driver/api_connection_inactivity_timeout_ms: 20000
* /my_gen3_lite/my_gen3_lite_driver/api_rpc_timeout_ms: 2000
* /my_gen3_lite/my_gen3_lite_driver/api_session_inactivity_timeout_ms: 35000
* /my_gen3_lite/my_gen3_lite_driver/arm: gen3_lite
* /my_gen3_lite/my_gen3_lite_driver/cyclic_data_publish_rate: 40
* /my_gen3_lite/my_gen3_lite_driver/default_goal_time_tolerance: 0.5
* /my_gen3_lite/my_gen3_lite_driver/default_goal_tolerance: 0.005
* /my_gen3_lite/my_gen3_lite_driver/dof: 6
* /my_gen3_lite/my_gen3_lite_driver/gripper: gen3_lite_2f
* /my_gen3_lite/my_gen3_lite_driver/gripper_joint_limits_max: [-0.09]
* /my_gen3_lite/my_gen3_lite_driver/gripper_joint_limits_min: [0.96]
* /my_gen3_lite/my_gen3_lite_driver/gripper_joint_names: ['right_finger_bo...
* /my_gen3_lite/my_gen3_lite_driver/ip_address: 192.168.1.10
* /my_gen3_lite/my_gen3_lite_driver/joint_names: ['joint_1', 'join...
* /my_gen3_lite/my_gen3_lite_driver/maximum_accelerations: [1.0, 0.5, 0.4, 1...
* /my_gen3_lite/my_gen3_lite_driver/maximum_velocities: [0.5, 0.5, 0.5, 0...
* /my_gen3_lite/my_gen3_lite_driver/password: admin
* /my_gen3_lite/my_gen3_lite_driver/prefix:
* /my_gen3_lite/my_gen3_lite_driver/robot_name: my_gen3_lite
* /my_gen3_lite/my_gen3_lite_driver/sim: False
* /my_gen3_lite/my_gen3_lite_driver/use_hard_limits: False
* /my_gen3_lite/my_gen3_lite_driver/username: admin
* /my_gen3_lite/publish_test_kortex_driver/topics: [{'name': '/my_ge...
* /my_gen3_lite/robot_description: <?xml version="1....
* /my_gen3_lite/robot_description_kinematics/arm/kinematics_solver: kdl_kinematics_pl...
* /my_gen3_lite/robot_description_kinematics/arm/kinematics_solver_search_resolution: 0.005
* /my_gen3_lite/robot_description_kinematics/arm/kinematics_solver_timeout: 0.005
* /my_gen3_lite/robot_description_planning/default_acceleration_scaling_factor: 1
* /my_gen3_lite/robot_description_planning/default_velocity_scaling_factor: 1
* /my_gen3_lite/robot_description_planning/joint_limits/joint_1/has_acceleration_limits: True
* /my_gen3_lite/robot_description_planning/joint_limits/joint_1/has_velocity_limits: True
* /my_gen3_lite/robot_description_planning/joint_limits/joint_1/max_acceleration: 0.86
* /my_gen3_lite/robot_description_planning/joint_limits/joint_1/max_velocity: 0.48
* /my_gen3_lite/robot_description_planning/joint_limits/joint_2/has_acceleration_limits: True
* /my_gen3_lite/robot_description_planning/joint_limits/joint_2/has_velocity_limits: True
* /my_gen3_lite/robot_description_planning/joint_limits/joint_2/max_acceleration: 0.43
* /my_gen3_lite/robot_description_planning/joint_limits/joint_2/max_velocity: 0.48
* /my_gen3_lite/robot_description_planning/joint_limits/joint_3/has_acceleration_limits: True
* /my_gen3_lite/robot_description_planning/joint_limits/joint_3/has_velocity_limits: True
* /my_gen3_lite/robot_description_planning/joint_limits/joint_3/max_acceleration: 0.34
* /my_gen3_lite/robot_description_planning/joint_limits/joint_3/max_velocity: 0.48
* /my_gen3_lite/robot_description_planning/joint_limits/joint_4/has_acceleration_limits: True
* /my_gen3_lite/robot_description_planning/joint_limits/joint_4/has_velocity_limits: True
* /my_gen3_lite/robot_description_planning/joint_limits/joint_4/max_acceleration: 0.86
* /my_gen3_lite/robot_description_planning/joint_limits/joint_4/max_velocity: 0.76
* /my_gen3_lite/robot_description_planning/joint_limits/joint_5/has_acceleration_limits: True
* /my_gen3_lite/robot_description_planning/joint_limits/joint_5/has_velocity_limits: True
* /my_gen3_lite/robot_description_planning/joint_limits/joint_5/max_acceleration: 8.6
* /my_gen3_lite/robot_description_planning/joint_limits/joint_5/max_velocity: 0.76
* /my_gen3_lite/robot_description_planning/joint_limits/joint_6/has_acceleration_limits: True
* /my_gen3_lite/robot_description_planning/joint_limits/joint_6/has_velocity_limits: True
* /my_gen3_lite/robot_description_planning/joint_limits/joint_6/max_acceleration: 8.6
* /my_gen3_lite/robot_description_planning/joint_limits/joint_6/max_velocity: 1.52
* /my_gen3_lite/robot_description_planning/joint_limits/right_finger_bottom_joint/has_acceleration_limits: False
* /my_gen3_lite/robot_description_planning/joint_limits/right_finger_bottom_joint/has_velocity_limits: True
* /my_gen3_lite/robot_description_planning/joint_limits/right_finger_bottom_joint/max_acceleration: 0
* /my_gen3_lite/robot_description_planning/joint_limits/right_finger_bottom_joint/max_velocity: 1000
* /my_gen3_lite/robot_description_semantic: <?xml version="1....
* /rosdistro: noetic
* /rosversion: 1.17.4

NODES
  /my_gen3_lite/
    joint_state_publisher (joint_state_publisher/joint_state_publisher)
    move_group (moveit_ros_move_group/move_group)
    my_gen3_lite_driver (kortex_driver/kortex_arm_driver)
    robot_state_publisher (robot_state_publisher/robot_state_publisher)
    rviz (rviz/rviz)

auto-starting new master
process[master]: started with pid [4324]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 82589fe6-542f-11f1-bb14-63fc6edc440d
process[rosout-1]: started with pid [4334]
started core service [/rosout]
process[my_gen3_lite/my_gen3_lite_driver-2]: started with pid [4341]
process[my_gen3_lite/move_group-3]: started with pid [4342]
process[my_gen3_lite/joint_state_publisher-4]: started with pid [4343]
process[my_gen3_lite/robot_state_publisher-5]: started with pid [4344]
[WARN] [1779269846.240070623]: The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
process[my_gen3_lite/rviz-6]: started with pid [4350]
[WARN] [1779269846.264172215]: Falling back to using the move_group node's namespace (deprecated Melodic behavior).
[INFO] [1779269846.271430130]: Loading robot model 'gen3_lite_gen3_lite_2f'...
[INFO] [1779269846.271509710]: No root/virtual joint specified in SRDF. Assuming fixed joint
[INFO] [1779269846.801842537]: Session created successfully for TCP services
[INFO] [1779269846.810825318]: Session created successfully for UDP services
[INFO] [1779269846.820424916]: -------------------------------------------------
[INFO] [1779269846.820459828]: Scanning all devices in robot...
[INFO] [1779269846.845290583]: Base device was found on device identifier 0
[INFO] [1779269846.845548466]: Actuator device of type SMALL_ACTUATOR was found on device identifier 7
[INFO] [1779269846.845558607]: Actuator device of type SMALL_ACTUATOR was found on device identifier 5
[INFO] [1779269846.845562589]: Actuator device of type MEDIUM_ACTUATOR was found on device identifier 1
[INFO] [1779269846.845565978]: Actuator device of type BIG_ACTUATOR was found on device identifier 2
[INFO] [1779269846.845571278]: Actuator device of type MEDIUM_ACTUATOR was found on device identifier 3
[INFO] [1779269846.845574370]: Actuator device of type SMALL_ACTUATOR was found on device identifier 4
[INFO] [1779269846.845596839]: -------------------------------------------------
[INFO] [1779269846.898446684]: State changed from INITIALIZING to IDLE

[WARN] [1779269846.911573310]: Could not identify parent group for end-effector 'end_effector'
[INFO] [1779269846.924424238]: State changed from INITIALIZING to IDLE

[INFO] [1779269846.928074521]: -------------------------------------------------
[INFO] [1779269846.928114590]: Initializing Kortex Driver's services...
[WARN] [1779269846.933664327]: The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
[INFO] [1779269847.428595748]: Publishing maintained planning scene on 'monitored_planning_scene'
[INFO] [1779269847.430426475]: Listening to 'joint_states' for joint states
[INFO] [1779269847.432437537]: Listening to '/my_gen3_lite/attached_collision_object' for attached collision objects
[INFO] [1779269847.432491782]: Starting planning scene monitor
[INFO] [1779269847.433380368]: Listening to '/my_gen3_lite/planning_scene'
[INFO] [1779269847.433425233]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1779269847.434290364]: Listening to '/my_gen3_lite/collision_object'
[INFO] [1779269847.435233350]: Listening to '/my_gen3_lite/planning_scene_world' for planning scene world geometry
[WARN] [1779269847.435524451]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[INFO] [1779269847.435769676]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1779269847.761992635]: Loading planning pipeline ''
[INFO] [1779269847.816439181]: Using planning interface 'OMPL'
[INFO] [1779269847.820755704]: Param 'default_workspace_bounds' was set to 10
[INFO] [1779269847.821097065]: Param 'start_state_max_bounds_error' was set to 0.1
[INFO] [1779269847.821344188]: Param 'start_state_max_dt' was set to 0.5
[INFO] [1779269847.821693268]: Param 'start_state_max_dt' was set to 0.5
[INFO] [1779269847.821989815]: Param 'jiggle_fraction' was set to 0.05
[INFO] [1779269847.822301848]: Param 'max_sampling_attempts' was set to 100
[INFO] [1779269847.822363691]: Using planning request adapter 'Add Time Parameterization'
[INFO] [1779269847.822403513]: Using planning request adapter 'Fix Workspace Bounds'
[INFO] [1779269847.822441173]: Using planning request adapter 'Fix Start State Bounds'
[INFO] [1779269847.822471724]: Using planning request adapter 'Fix Start State In Collision'
[INFO] [1779269847.822497848]: Using planning request adapter 'Fix Start State Path Constraints'
[INFO] [1779269848.071939760]: Added FollowJointTrajectory controller for gen3_lite_joint_trajectory_controller
[INFO] [1779269848.370104009]: Added GripperCommand controller for gen3_lite_2f_gripper_controller
[INFO] [1779269848.370379155]: Returned 2 controllers in list
[INFO] [1779269848.383135721]: Trajectory execution is managing controllers
[INFO] [1779269848.383265250]: MoveGroup debug mode is OFF
Loading 'move_group/ApplyPlanningSceneService'...
Loading 'move_group/ClearOctomapService'...
Loading 'move_group/MoveGroupCartesianPathService'...
Loading 'move_group/MoveGroupExecuteTrajectoryAction'...
Loading 'move_group/MoveGroupGetPlanningSceneService'...
Loading 'move_group/MoveGroupKinematicsService'...
Loading 'move_group/MoveGroupMoveAction'...
Loading 'move_group/MoveGroupPickPlaceAction'...
Loading 'move_group/MoveGroupPlanService'...
Loading 'move_group/MoveGroupQueryPlannersService'...
Loading 'move_group/MoveGroupStateValidationService'...
[INFO] [1779269848.436981107]:

---

* MoveGroup using:
* - ApplyPlanningSceneService
* - ClearOctomapService
* - CartesianPathService
* - ExecuteTrajectoryAction
* - GetPlanningSceneService
* - KinematicsService
* - MoveAction
* - PickPlaceAction
* - MotionPlanService
* - QueryPlannersService
* - StateValidationService

---

[INFO] [1779269848.437444806]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[INFO] [1779269848.437675583]: MoveGroup context initialization complete

You can start planning now!

[INFO] [1779269848.994354260]: Kortex Driver's services initialized correctly.
[INFO] [1779269848.994638741]: -------------------------------------------------
[INFO] [1779269849.024555279]: The Kortex driver has been initialized correctly!
