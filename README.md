# ORB_SLAM3-RGBD-Inertial
RGBD-inertial mode and its ROS interface was added to ORB_SLAM3.

ROS interfaces Mono_inertial and Stereo_inertial were provided.

Modified the loading vocabulary in a faster binary way.

增加了RGBD-IMU的运行模式和ROS接口，增加了单目IMU和双目IMU的ROS接口，替换了词典为二进制格式，加载速度更快。

Command

RGBD_inertial: rosrun ORB_SLAM3 RGBD_inertial /YOUR_PATH/ORBvoc.bin /YOUR_PATH/Myd435i.yaml

Stereo_inertial: rosrun ORB_SLAM3 Stereo_inertial /YOUR_PATH/ORBvoc.bin /YOUR_PATH/EuRoC.yaml

Mono_inertial: rosrun ORB_SLAM3 Mono_inertial /YOUR_PATH/ORBvoc.bin /YOUR_PATH/EuRoC.yaml


RGBD-inertial Mode Tested with d435i rosbag: 

https://star-center.shanghaitech.edu.cn/seafile/d/0ea45d1878914077ade5/


