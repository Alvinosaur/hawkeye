# hawkeye
Autonomous Drone Target Following 

## Tutorials and resources uused:
http://gazebosim.org/tutorials?tut=animated_box
https://github.com/icsl-Jeon/px4_code/blob/master/launch/run_px4_for_sitl.launch


## Running the Drone Simulator
1. Start up roscore in a separate tab by running:
```
roscore
```
2. Start up Px4 Drone Simulator using ROS Gazebo with custom world:
```
catkin build
source your_workspace/setup.bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/alvin/drone_ws/src/PX4-Autopilot/build/px4_sitl_default/build_g$
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/alvin/drone_ws/src/PX4-Autopilot/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/alvin/drone_ws/src/PX4-Autopilot/build/px4_sitl_default/build_gazebo

roslaunch you_workspace run_px4_sitl.launch
```

3. Setup SITL connection with simulator
```
cd /path/to/PX4-AutoPilot
no_sim=1 make px4_sitl_default gazebo
```



### Old Version of Launching (for reference)
2. Start up Px4 Drone Simulator using ROS Gazebo:
```
cd ~/src/Firmware  # assuming you have downloaded the most recent Px4 Firmware. Do not include in workspace!
make px4_sitl_default gazebo
```
A gazebo window should now pop up with the default Iris 3DR+ Drone loaded.

#### NOTE: Please make sure you've built and sourced your workspace! Need to use catkin build, not catkin_make.

To source:
```
source auton_drone/devel/setup.bash
```

3. Run mavros to set up ROS communication with drone simulator, almost like real setup:
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
```
Information should scroll down and you should see information such as the following:
```
[ INFO] [1548025762.529638959]: MAVROS started. MY ID 1.240, TARGET ID 1.1
[ INFO] [1548025801.985629583]: udp0: Remote address: 127.0.0.1:14580
[ INFO] [1548025801.986042035]: IMU: High resolution IMU detected!
[ INFO] [1548025802.249840146]: FCU: [logger] file: ./log/2019-01-20/23_10_01.ulg
[ INFO] [1548025802.859082897]: IMU: Attitude quaternion IMU detected!
[ INFO] [1548025802.947095577]: CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot
[ INFO] [1548025802.948727744]: IMU: High resolution IMU detected!
[ INFO] [1548025802.958893900]: IMU: Attitude quaternion IMU detected!
[ INFO] [1548025803.952132087]: VER: 1.1: Capabilities         0x000000000000e4ef
[ INFO] [1548025803.952201659]: VER: 1.1: Flight software:     01090040 (0000000004AEF5AB)
[ INFO] [1548025803.952241386]: VER: 1.1: Middleware software: 01090040 (0000000004AEF5AB)
[ INFO] [1548025803.952273969]: VER: 1.1: OS software:         040f00ff (0000000000000000)
[ INFO] [1548025803.952310971]: VER: 1.1: Board hardware:      00000001
[ INFO] [1548025803.952340975]: VER: 1.1: VID/PID:             0000:0000
[ INFO] [1548025803.952363484]: VER: 1.1: UID:                 4954414c44494e4f
```
This shows that mavros was able to establish a connection with the drone and receive information such as IMU data and firmware version.

4. Now that the simulator and ros connections are set up, we can try to send commands to the drone!
```
rosrun hawkeye offboard
```
You should see the following output over time:
```
[ INFO] [1548025836.440769542]: Waiting for FCU Connection
[ INFO] [1548025836.490617899]: Waiting for FCU Connection
[ INFO] [1548025836.540612568]: Waiting for FCU Connection
[ INFO] [1548025836.590615629]: Waiting for FCU Connection
[ INFO] [1548025836.640600265]: Waiting for FCU Connection
[ INFO] [1548025836.690592362]: Waiting for FCU Connection
[ INFO] [1548025836.740608801]: Sending initial points before starting offboard
[ INFO] [1548025846.792831818]: Offboard enabled
[ INFO] [1548025851.849003067]: Vehicle armed
```
Now your simulated drone should rise up and hover!
# Troubleshooting
## Airsim ROS:
### Error when running ```catkin_make -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8``` in Airsim/ros directory:
```
...
fatal error: Eigen/Dense: No such file or directory
 #include "Eigen/Dense"
...
```
Solution:
```
cd /usr/local/include
sudo ln -sf eigen3/Eigen Eigen
sudo ln -sf eigen3/unsupported unsupported
```

### Troubleshooting:
```
CMake Error at /path/to/AirSim/cmake/rpclib_wrapper/CMakeLists.txt:13 (add_subdirectory):
  add_subdirectory given source
  "/path/to/AirSim/ros/src/airsim_ros_pkgs/../../..//external/rpclib/rpclib-2.2.1"
  which is not an existing directory.
```

Solution:
```
mkdir -p /path/to/AirSim/ros/src/airsim_ros_pkgs/../../..//external/rpclib/rpclib-2.2.1
cd /path/to/drone_ws/src/AirSim/external/rpclib/
git clone --depth 1 https://github.com/rpclib/rpclib.git
mv rpclib/* rpclib-2.2.1
rm -rf rpclib
```

```
INFO [simulator] Waiting for simulator to accept connection on TCP port 4560
```
This happens when we try connecting SITL with the Gazebo simulator. This is caused by the simulator being improperly set up. Specifically, when launchign the simulator, you need to make sure to define all those gazebo and LD_LIBRARY paths with export. You can just add those to your bashrc.


```
[Err] [Plugin.hh:178] Failed to load plugin libgazebo_motor_model.so: libgazebo_motor_model.so: cannot open shared object file: No such file or directory
... other libgazebo models specific to the Iris drone cannot load
```

## Successful launch of custom world:

```
(venv) alvin@alvin:~/drone_ws$ roslaunch hawkeye run_px4_sitl.launch 
... logging to /home/alvin/.ros/log/abe4901e-886e-11eb-a515-40ec993a4bde/roslaunch-alvin-4059.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://alvin:37269/

SUMMARY
========

CLEAR PARAMETERS
 * /mavros/

PARAMETERS
 * /gazebo/enable_ros_network: True
 * /mavros/cmd/use_comp_id_system_control: False
 * /mavros/conn/heartbeat_rate: 1.0
 * /mavros/conn/system_time_rate: 1.0
 * /mavros/conn/timeout: 10.0
 * /mavros/conn/timesync_rate: 10.0
 * /mavros/distance_sensor/hrlv_ez4_pub/field_of_view: 0.0
 * /mavros/distance_sensor/hrlv_ez4_pub/frame_id: hrlv_ez4_sonar
 * /mavros/distance_sensor/hrlv_ez4_pub/id: 0
 * /mavros/distance_sensor/hrlv_ez4_pub/orientation: PITCH_270
 * /mavros/distance_sensor/hrlv_ez4_pub/send_tf: True
 * /mavros/distance_sensor/hrlv_ez4_pub/sensor_position/x: 0.0
 * /mavros/distance_sensor/hrlv_ez4_pub/sensor_position/y: 0.0
 * /mavros/distance_sensor/hrlv_ez4_pub/sensor_position/z: -0.1
 * /mavros/distance_sensor/laser_1_sub/id: 3
 * /mavros/distance_sensor/laser_1_sub/orientation: PITCH_270
 * /mavros/distance_sensor/laser_1_sub/subscriber: True
 * /mavros/distance_sensor/lidarlite_pub/field_of_view: 0.0
 * /mavros/distance_sensor/lidarlite_pub/frame_id: lidarlite_laser
 * /mavros/distance_sensor/lidarlite_pub/id: 1
 * /mavros/distance_sensor/lidarlite_pub/orientation: PITCH_270
 * /mavros/distance_sensor/lidarlite_pub/send_tf: True
 * /mavros/distance_sensor/lidarlite_pub/sensor_position/x: 0.0
 * /mavros/distance_sensor/lidarlite_pub/sensor_position/y: 0.0
 * /mavros/distance_sensor/lidarlite_pub/sensor_position/z: -0.1
 * /mavros/distance_sensor/sonar_1_sub/id: 2
 * /mavros/distance_sensor/sonar_1_sub/orientation: PITCH_270
 * /mavros/distance_sensor/sonar_1_sub/subscriber: True
 * /mavros/fake_gps/eph: 2.0
 * /mavros/fake_gps/epv: 2.0
 * /mavros/fake_gps/fix_type: 3
 * /mavros/fake_gps/geo_origin/alt: 408.0
 * /mavros/fake_gps/geo_origin/lat: 47.3667
 * /mavros/fake_gps/geo_origin/lon: 8.55
 * /mavros/fake_gps/gps_rate: 5.0
 * /mavros/fake_gps/mocap_transform: True
 * /mavros/fake_gps/satellites_visible: 5
 * /mavros/fake_gps/tf/child_frame_id: fix
 * /mavros/fake_gps/tf/frame_id: map
 * /mavros/fake_gps/tf/listen: False
 * /mavros/fake_gps/tf/rate_limit: 10.0
 * /mavros/fake_gps/tf/send: False
 * /mavros/fake_gps/use_mocap: True
 * /mavros/fake_gps/use_vision: False
 * /mavros/fcu_protocol: v2.0
 * /mavros/fcu_url: udp://:14540@192....
 * /mavros/gcs_url: 
 * /mavros/global_position/child_frame_id: base_link
 * /mavros/global_position/frame_id: map
 * /mavros/global_position/gps_uere: 1.0
 * /mavros/global_position/rot_covariance: 99999.0
 * /mavros/global_position/tf/child_frame_id: base_link
 * /mavros/global_position/tf/frame_id: map
 * /mavros/global_position/tf/global_frame_id: earth
 * /mavros/global_position/tf/send: False
 * /mavros/global_position/use_relative_alt: True
 * /mavros/image/frame_id: px4flow
 * /mavros/imu/angular_velocity_stdev: 0.0003490659 // 0...
 * /mavros/imu/frame_id: base_link
 * /mavros/imu/linear_acceleration_stdev: 0.0003
 * /mavros/imu/magnetic_stdev: 0.0
 * /mavros/imu/orientation_stdev: 1.0
 * /mavros/landing_target/camera/fov_x: 2.0071286398
 * /mavros/landing_target/camera/fov_y: 2.0071286398
 * /mavros/landing_target/image/height: 480
 * /mavros/landing_target/image/width: 640
 * /mavros/landing_target/land_target_type: VISION_FIDUCIAL
 * /mavros/landing_target/listen_lt: False
 * /mavros/landing_target/mav_frame: LOCAL_NED
 * /mavros/landing_target/target_size/x: 0.3
 * /mavros/landing_target/target_size/y: 0.3
 * /mavros/landing_target/tf/child_frame_id: camera_center
 * /mavros/landing_target/tf/frame_id: landing_target
 * /mavros/landing_target/tf/listen: False
 * /mavros/landing_target/tf/rate_limit: 10.0
 * /mavros/landing_target/tf/send: True
 * /mavros/local_position/frame_id: map
 * /mavros/local_position/tf/child_frame_id: base_link
 * /mavros/local_position/tf/frame_id: map
 * /mavros/local_position/tf/send: False
 * /mavros/local_position/tf/send_fcu: False
 * /mavros/mission/pull_after_gcs: True
 * /mavros/mission/use_mission_item_int: True
 * /mavros/mocap/use_pose: True
 * /mavros/mocap/use_tf: False
 * /mavros/odometry/fcu/odom_child_id_des: base_link
 * /mavros/odometry/fcu/odom_parent_id_des: map
 * /mavros/plugin_blacklist: ['safety_area', '...
 * /mavros/plugin_whitelist: []
 * /mavros/px4flow/frame_id: px4flow
 * /mavros/px4flow/ranger_fov: 0.118682
 * /mavros/px4flow/ranger_max_range: 5.0
 * /mavros/px4flow/ranger_min_range: 0.3
 * /mavros/safety_area/p1/x: 1.0
 * /mavros/safety_area/p1/y: 1.0
 * /mavros/safety_area/p1/z: 1.0
 * /mavros/safety_area/p2/x: -1.0
 * /mavros/safety_area/p2/y: -1.0
 * /mavros/safety_area/p2/z: -1.0
 * /mavros/setpoint_accel/send_force: False
 * /mavros/setpoint_attitude/reverse_thrust: False
 * /mavros/setpoint_attitude/tf/child_frame_id: target_attitude
 * /mavros/setpoint_attitude/tf/frame_id: map
 * /mavros/setpoint_attitude/tf/listen: False
 * /mavros/setpoint_attitude/tf/rate_limit: 50.0
 * /mavros/setpoint_attitude/use_quaternion: False
 * /mavros/setpoint_position/mav_frame: LOCAL_NED
 * /mavros/setpoint_position/tf/child_frame_id: target_position
 * /mavros/setpoint_position/tf/frame_id: map
 * /mavros/setpoint_position/tf/listen: False
 * /mavros/setpoint_position/tf/rate_limit: 50.0
 * /mavros/setpoint_raw/thrust_scaling: 1.0
 * /mavros/setpoint_velocity/mav_frame: LOCAL_NED
 * /mavros/startup_px4_usb_quirk: True
 * /mavros/sys/disable_diag: False
 * /mavros/sys/min_voltage: 10.0
 * /mavros/target_component_id: 1
 * /mavros/target_system_id: 1
 * /mavros/tdr_radio/low_rssi: 40
 * /mavros/time/time_ref_source: fcu
 * /mavros/time/timesync_avg_alpha: 0.6
 * /mavros/time/timesync_mode: MAVLINK
 * /mavros/vibration/frame_id: base_link
 * /mavros/vision_pose/tf/child_frame_id: vision_estimate
 * /mavros/vision_pose/tf/frame_id: odom
 * /mavros/vision_pose/tf/listen: False
 * /mavros/vision_pose/tf/rate_limit: 10.0
 * /mavros/vision_speed/listen_twist: True
 * /mavros/vision_speed/twist_cov: True
 * /mavros/wheel_odometry/child_frame_id: base_link
 * /mavros/wheel_odometry/count: 2
 * /mavros/wheel_odometry/frame_id: odom
 * /mavros/wheel_odometry/send_raw: True
 * /mavros/wheel_odometry/send_twist: False
 * /mavros/wheel_odometry/tf/child_frame_id: base_link
 * /mavros/wheel_odometry/tf/frame_id: odom
 * /mavros/wheel_odometry/tf/send: False
 * /mavros/wheel_odometry/use_rpm: False
 * /mavros/wheel_odometry/vel_error: 0.1
 * /mavros/wheel_odometry/wheel0/radius: 0.05
 * /mavros/wheel_odometry/wheel0/x: 0.0
 * /mavros/wheel_odometry/wheel0/y: -0.15
 * /mavros/wheel_odometry/wheel1/radius: 0.05
 * /mavros/wheel_odometry/wheel1/x: 0.0
 * /mavros/wheel_odometry/wheel1/y: 0.15
 * /rosdistro: melodic
 * /rosversion: 1.14.10
 * /use_sim_time: True

NODES
  /
    gazebo (gazebo_ros/gzserver)
    gazebo_gui (gazebo_ros/gzclient)
    mavros (mavros/mavros_node)

auto-starting new master
process[master]: started with pid [4069]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to abe4901e-886e-11eb-a515-40ec993a4bde
process[rosout-1]: started with pid [4080]
started core service [/rosout]
process[gazebo-2]: started with pid [4088]
Gazebo multi-robot simulator, version 11.3.0
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org

process[gazebo_gui-3]: started with pid [4093]
[ INFO] [1616129449.554702571]: Finished loading Gazebo ROS API Plugin.
[ INFO] [1616129449.555821567]: waitForService: Service [/gazebo/set_physics_properties] has not been advertised, waiting...
[Msg] Waiting for master.
[Msg] Connected to gazebo master @ http://127.0.0.1:11345
[Msg] Publicized address: 192.168.1.27
Gazebo multi-robot simulator, version 11.3.0
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org

[Wrn] [GuiIface.cc:200] g/gui-plugin is really loading a SystemPlugin. To load a GUI plugin please use --gui-client-plugin 
process[mavros-4]: started with pid [4136]
[ INFO] [1616129449.942964341]: FCU URL: udp://:14540@192.168.1.36:14557
[ INFO] [1616129449.945457148]: udp0: Bind address: 0.0.0.0:14540
[ INFO] [1616129449.945528629]: udp0: Remote address: 192.168.1.36:14557
[ INFO] [1616129449.945744656]: GCS bridge disabled
[ INFO] [1616129449.955483790]: Plugin 3dr_radio loaded
[ INFO] [1616129449.956844884]: Plugin 3dr_radio initialized
[ INFO] [1616129449.956905986]: Plugin actuator_control loaded
[ INFO] [1616129449.958991241]: Plugin actuator_control initialized
[ INFO] [1616129449.962267137]: Plugin adsb loaded
[ INFO] [1616129449.964475268]: Plugin adsb initialized
[ INFO] [1616129449.964580216]: Plugin altitude loaded
[ INFO] [1616129449.965402903]: Plugin altitude initialized
[ INFO] [1616129449.965479266]: Plugin cam_imu_sync loaded
[ INFO] [1616129449.966171882]: Plugin cam_imu_sync initialized
[ INFO] [1616129449.966327091]: Plugin command loaded
[ INFO] [1616129449.972089692]: Plugin command initialized
[ INFO] [1616129449.972211169]: Plugin companion_process_status loaded
[ INFO] [1616129449.974190974]: Plugin companion_process_status initialized
[ INFO] [1616129449.974292110]: Plugin debug_value loaded
[ INFO] [1616129449.977919701]: Plugin debug_value initialized
[ INFO] [1616129449.977946019]: Plugin distance_sensor blacklisted
[ INFO] [1616129449.978054695]: Plugin esc_status loaded
[ INFO] [1616129449.978968116]: Plugin esc_status initialized
[ INFO] [1616129449.979075598]: Plugin fake_gps loaded
[ INFO] [1616129449.990196308]: Plugin fake_gps initialized
[ INFO] [1616129449.990384172]: Plugin ftp loaded
[ INFO] [1616129449.996946395]: Plugin ftp initialized
[ INFO] [1616129449.997066000]: Plugin global_position loaded
[ INFO] [1616129450.007998104]: Plugin global_position initialized
[ INFO] [1616129450.008108926]: Plugin gps_rtk loaded
[ INFO] [1616129450.009847619]: Plugin gps_rtk initialized
[ INFO] [1616129450.009955337]: Plugin gps_status loaded
[ INFO] [1616129450.011567125]: Plugin gps_status initialized
[ INFO] [1616129450.011694639]: Plugin hil loaded
[ INFO] [1616129450.019359777]: Plugin hil initialized
[ INFO] [1616129450.019484552]: Plugin home_position loaded
[ INFO] [1616129450.021749544]: Plugin home_position initialized
[ INFO] [1616129450.021908992]: Plugin imu loaded
[ INFO] [1616129450.026521502]: Plugin imu initialized
[ INFO] [1616129450.026644137]: Plugin landing_target loaded
[ INFO] [1616129450.036112953]: Plugin landing_target initialized
[ INFO] [1616129450.036287483]: Plugin local_position loaded
[ INFO] [1616129450.038537562]: Finished loading Gazebo ROS API Plugin.
[ INFO] [1616129450.039977271]: waitForService: Service [/gazebo_gui/set_physics_properties] has not been advertised, waiting...
[Msg] Waiting for master.
[ INFO] [1616129450.041633616]: Plugin local_position initialized
[ INFO] [1616129450.041771221]: Plugin log_transfer loaded
[ INFO] [1616129450.043663054]: Plugin log_transfer initialized
[ INFO] [1616129450.043789912]: Plugin manual_control loaded
[ INFO] [1616129450.045322290]: Plugin manual_control initialized
[ INFO] [1616129450.045441794]: Plugin mocap_pose_estimate loaded
[Msg] Connected to gazebo master @ http://127.0.0.1:11345
[Msg] Publicized address: 192.168.1.27
[ INFO] [1616129450.047374287]: Plugin mocap_pose_estimate initialized
[ INFO] [1616129450.047490892]: Plugin mount_control loaded
[ INFO] [1616129450.049605117]: Plugin mount_control initialized
[ INFO] [1616129450.049729926]: Plugin obstacle_distance loaded
[ INFO] [1616129450.051616147]: Plugin obstacle_distance initialized
[ INFO] [1616129450.051734072]: Plugin odom loaded
[ INFO] [1616129450.054308866]: Plugin odom initialized
[ INFO] [1616129450.054458632]: Plugin onboard_computer_status loaded
[ INFO] [1616129450.055815342]: Plugin onboard_computer_status initialized
[ INFO] [1616129450.055992227]: Plugin param loaded
[ INFO] [1616129450.058145110]: Plugin param initialized
[ INFO] [1616129450.058256777]: Plugin play_tune loaded
[ INFO] [1616129450.060233113]: Plugin play_tune initialized
[ INFO] [1616129450.060433799]: Plugin px4flow loaded
[ INFO] [1616129450.064375498]: Plugin px4flow initialized
[ INFO] [1616129450.064405406]: Plugin rangefinder blacklisted
[ INFO] [1616129450.064550110]: Plugin rc_io loaded
[ INFO] [1616129450.067300703]: Plugin rc_io initialized
[ INFO] [1616129450.067335742]: Plugin safety_area blacklisted
[ INFO] [1616129450.067465873]: Plugin setpoint_accel loaded
[ INFO] [1616129450.069499720]: Plugin setpoint_accel initialized
[ INFO] [1616129450.069761237]: Plugin setpoint_attitude loaded
[ INFO] [1616129450.076684701]: Plugin setpoint_attitude initialized
[ INFO] [1616129450.076835220]: Plugin setpoint_position loaded
[ INFO] [1616129450.085695822]: Plugin setpoint_position initialized
[ INFO] [1616129450.085853747]: Plugin setpoint_raw loaded
[ INFO] [1616129450.090878701]: Plugin setpoint_raw initialized
[ INFO] [1616129450.091030857]: Plugin setpoint_trajectory loaded
[ INFO] [1616129450.093585580]: Plugin setpoint_trajectory initialized
[ INFO] [1616129450.093702854]: Plugin setpoint_velocity loaded
[ INFO] [1616129450.096904040]: Plugin setpoint_velocity initialized
[ INFO] [1616129450.097164781]: Plugin sys_status loaded
[ INFO] [1616129450.103995445]: Plugin sys_status initialized
[ INFO] [1616129450.104168826]: Plugin sys_time loaded
[ INFO] [1616129450.107142193]: TM: Timesync mode: MAVLINK
[ INFO] [1616129450.107768753]: Plugin sys_time initialized
[ INFO] [1616129450.107881210]: Plugin trajectory loaded
[ INFO] [1616129450.110823451]: Plugin trajectory initialized
[ INFO] [1616129450.110971653]: Plugin vfr_hud loaded
[ INFO] [1616129450.111433107]: Plugin vfr_hud initialized
[ INFO] [1616129450.111459200]: Plugin vibration blacklisted
[ INFO] [1616129450.111564760]: Plugin vision_pose_estimate loaded
[ INFO] [1616129450.116306293]: Plugin vision_pose_estimate initialized
[ INFO] [1616129450.116417359]: Plugin vision_speed_estimate loaded
[ INFO] [1616129450.119047446]: Plugin vision_speed_estimate initialized
[ INFO] [1616129450.119193134]: Plugin waypoint loaded
[ INFO] [1616129450.122948564]: Plugin waypoint initialized
[ INFO] [1616129450.122988319]: Plugin wheel_odometry blacklisted
[ INFO] [1616129450.123105817]: Plugin wind_estimation loaded
[ INFO] [1616129450.123546043]: Plugin wind_estimation initialized
[ INFO] [1616129450.123579731]: Autostarting mavlink via USB on PX4
[ INFO] [1616129450.123671620]: Built-in SIMD instructions: SSE, SSE2
[ INFO] [1616129450.123688505]: Built-in MAVLink package version: 2021.3.3
[ INFO] [1616129450.123707715]: Known MAVLink dialects: common ardupilotmega ASLUAV all autoquad icarous matrixpilot paparazzi standard uAvionix ualberta
[ INFO] [1616129450.123734131]: MAVROS started. MY ID 1.240, TARGET ID 1.1
[Msg] Loading world file [/home/alvin/drone_ws/src/hawkeye/worlds/target_tracking.world]
[Err] [Joint.cc:297] EXCEPTION: Couldn't Find Child Link[gps0::link]

[Err] [Model.cc:272] LoadJoint Failed
[ INFO] [1616129450.230926000]: waitForService: Service [/gazebo/set_physics_properties] is now available.
[ INFO] [1616129450.247721631]: Physics dynamic reconfigure ready.
[Wrn] [Event.cc:61] Warning: Deleting a connection right after creation. Make sure to save the ConnectionPtr from a Connect call
[ INFO] [1616129450.453394186]: Camera Plugin: The 'robotNamespace' param was empty
[ INFO] [1616129450.455439775]: Camera Plugin (ns = iris_fpv_cam)  <tf_prefix_>, set to "iris_fpv_cam"
[Wrn] [gazebo_gps_plugin.cpp:76] [gazebo_gps_plugin]: iris_fpv_cam::gps0 using gps topic "gps0"
[Wrn] [gazebo_gps_plugin.cpp:201] [gazebo_gps_plugin] Using default update rate of 5hz 
[Msg] Connecting to PX4 SITL using TCP
[Msg] Lockstep is enabled
[Msg] Speed factor set to: 1
[Msg] Using MAVLink protocol v2.0
[ INFO] [1616129511.405519804, 60.740000000]: udp0: Remote address: 127.0.0.1:14580
[ INFO] [1616129511.405814945, 60.740000000]: IMU: High resolution IMU detected!
[ INFO] [1616129512.374206419, 61.708000000]: CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot
[ INFO] [1616129512.385967704, 61.720000000]: IMU: High resolution IMU detected!
[ INFO] [1616129513.384720484, 62.716000000]: WP: Using MISSION_ITEM_INT
[ INFO] [1616129513.384878838, 62.716000000]: VER: 1.1: Capabilities         0x000000000000e4ef
[ INFO] [1616129513.384977124, 62.716000000]: VER: 1.1: Flight software:     010b00c0 (28681405ae000000)
[ INFO] [1616129513.385059465, 62.716000000]: VER: 1.1: Middleware software: 010b00c0 (28681405ae000000)
[ INFO] [1616129513.385150528, 62.716000000]: VER: 1.1: OS software:         050400ff (9e077c2f806acd5b)
[ INFO] [1616129513.385222284, 62.716000000]: VER: 1.1: Board hardware:      00000001
[ INFO] [1616129513.385297351, 62.716000000]: VER: 1.1: VID/PID:             0000:0000
[ INFO] [1616129513.385371812, 62.716000000]: VER: 1.1: UID:                 4954414c44494e4f
[ WARN] [1616129513.385657969, 62.716000000]: CMD: Unexpected command 520, result 0
[ INFO] [1616129515.503254427, 64.836000000]: IMU: Attitude quaternion IMU detected!
[ INFO] [1616129527.387266489, 76.708000000]: WP: mission received

```

Successful connection on TCP 4560:

```
(venv) alvin@alvin:~/drone_ws/src/PX4-Autopilot$ no_sim=1 make px4_sitl_default gazebo
[0/4] Performing build step for 'sitl_gazebo'
ninja: no work to do.
[3/4] cd /home/alvin/drone_ws/src/PX4-Autopilot/build/px4_sitl_default/tmp && /hom...ws/src/PX4-Autopilot /home/alvin/drone_ws/src/PX4-Autopilot/build/px4_sitl_default
SITL ARGS
sitl_bin: /home/alvin/drone_ws/src/PX4-Autopilot/build/px4_sitl_default/bin/px4
debugger: none
program: gazebo
model: none
world: none
src_path: /home/alvin/drone_ws/src/PX4-Autopilot
build_path: /home/alvin/drone_ws/src/PX4-Autopilot/build/px4_sitl_default
empty model, setting iris as default
SITL COMMAND: "/home/alvin/drone_ws/src/PX4-Autopilot/build/px4_sitl_default/bin/px4" "/home/alvin/drone_ws/src/PX4-Autopilot/build/px4_sitl_default"/etc -s etc/init.d-posix/rcS -t "/home/alvin/drone_ws/src/PX4-Autopilot"/test_data
INFO  [px4] Creating symlink /home/alvin/drone_ws/src/PX4-Autopilot/build/px4_sitl_default/etc -> /home/alvin/drone_ws/src/PX4-Autopilot/build/px4_sitl_default/tmp/rootfs/etc

______  __   __    ___ 
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.

INFO  [px4] Calling startup script: /bin/sh etc/init.d-posix/rcS 0
Info: found model autostart file as SYS_AUTOSTART=10016
INFO  [param] selected parameter default file eeprom/parameters_10016
[param] Loaded: eeprom/parameters_10016
INFO  [dataman] Unknown restart, data manager file './dataman' size is 11798680 bytes
INFO  [simulator] Waiting for simulator to accept connection on TCP port 4560
INFO  [simulator] Simulator connected on TCP port 4560.
INFO  [commander] LED: open /dev/led0 failed (22)
INFO  [init] Mixer: etc/mixers/quad_w.main.mix on /dev/pwm_output0
INFO  [ekf2] starting instance 0, IMU:0 (1310988), MAG:0 (197388)
INFO  [ekf2] starting instance 1, IMU:1 (1310996), MAG:0 (197388)
INFO  [ekf2] starting instance 2, IMU:2 (1311004), MAG:0 (197388)
INFO  [ekf2] starting instance 3, IMU:0 (1310988), MAG:1 (197644)
INFO  [ekf2] starting instance 4, IMU:1 (1310996), MAG:1 (197644)
INFO  [ekf2] starting instance 5, IMU:2 (1311004), MAG:1 (197644)
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14580 remote port 14540
INFO  [mavlink] mode: Onboard, data rate: 4000 B/s on udp port 14280 remote port 14030
INFO  [mavlink] mode: Gimbal, data rate: 400000 B/s on udp port 13030 remote port 13280
INFO  [logger] logger started (mode=all)
INFO  [logger] Start file log (type: full)
INFO  [logger] [logger] ./log/2021-03-19/04_51_51.ulg
INFO  [logger] Opened full log file: ./log/2021-03-19/04_51_51.ulg
INFO  [px4] Startup script returned successfully
pxh> INFO  [mavlink] partner IP: 127.0.0.1
INFO  [mavlink] using network interface wlo1, IP: 192.168.1.27
INFO  [mavlink] with netmask: 255.255.255.0
INFO  [mavlink] and broadcast IP: 192.168.1.255
INFO  [ecl/EKF] reset position to last known position
INFO  [ecl/EKF] reset velocity to zero
INFO  INFO  [ecl/EKF] reset position to last known position
INFO  [ecl/EKF] [ecl/EKF] INFO  reset position to last known position
INFO  [ecl/EKF] reset velocity to zero
reset velocity to zero
[ecl/EKF] reset position to last known position
INFO  [ecl/EKF] reset velocity to zero
INFO  [ecl/EKF] reset position to last known position
INFO  [ecl/EKF] reset velocity to zero
INFO  [ecl/EKF] reset position to last known position
INFO  [ecl/EKF] reset velocity to zero
INFO  [ecl/EKF] GPS checks passed
INFO  [ecl/EKF] GPS checks passed
INFO  [ecl/EKF] GPS checks passed
INFO  [ecl/EKF] GPS checks passed
INFO  [ecl/EKF] GPS checks passed
INFO  [ecl/EKF] GPS checks passed
INFO  [mavlink] using network interface wlo1, IP: 192.168.1.27
INFO  [mavlink] with netmask: 255.255.255.0
INFO  [mavlink] and broadcast IP: 192.168.1.255
INFO  [ecl/EKF] 4148000: EKF aligned, (baro hgt, IMU buf: 12, OBS buf: 9)
INFO  [ecl/EKF] 4156000: EKF aligned, (baro hgt, IMU buf: 12, OBS buf: 9)INFO  [ecl/EKF] 4156000: EKF aligned, (baro hgt, IMU buf: 12, OBS buf: 9)

INFO  [ecl/EKF] 4156000: EKF aligned, (baro hgt, IMU buf: 12, OBS buf: 9)
INFO  [ecl/EKF] 4156000: EKF aligned, (baro hgt, IMU buf: 12, OBS buf: 9)
INFO  [ecl/EKF] 4156000: EKF aligned, (baro hgt, IMU buf: 12, OBS buf: 9)
INFO  [ecl/EKF] reset position to GPS
INFO  [ecl/EKF] reset velocity to GPS
INFO  [ecl/EKF] starting GPS fusion
INFO  [ecl/EKF] reset position to GPS
INFO  [ecl/EKF] reset velocity to GPS
INFO  [ecl/EKF] starting GPS fusion
INFO  [ecl/EKF] reset position to GPS
INFO  [ecl/EKF] reset velocity to GPS
INFO  [ecl/EKF] starting GPS fusion
INFO  [ecl/EKF] reset position to GPS
INFO  [ecl/EKF] reset velocity to GPS
INFO  [ecl/EKF] starting GPS fusion
INFO  [ecl/EKF] reset position to GPS
INFO  INFO  [ecl/EKF] reset position to GPS
INFO  [ecl/EKF] reset velocity to GPS
[ecl/EKF] reset velocity to GPS
INFO  [ecl/EKF] starting GPS fusion
INFO  [ecl/EKF] starting GPS fusion
INFO  [tone_alarm] home set
INFO  [tone_alarm] notify negative
```




