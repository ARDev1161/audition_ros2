# M4 ROS 2 pose covariance message admission

`audition_fusion/ros_pose_adapter.py` converts validated ROS 2
`geometry_msgs/PoseWithCovarianceStamped` or `nav_msgs/Odometry`-
shaped messages into stamped planar microphone-origin uncertainty.
The implementation is a pure-Python adapter and uses the existing
`project_microphone_origin` calculation; **it is not a live ROS subscriber**.

The adapter enforces exact declared world-frame matching, finite normalized
quaternion, small roll/pitch, nonzero measured covariance, timestamp bounds,
and symmetry/positive semidefiniteness of the x/y/yaw submatrix. Covariance
uses the ROS 6x6 pose covariance rows and columns 0,1,5 (x,y,yaw);
`Odometry.header.frame_id`, **not** `child_frame_id`, names the pose frame.

ROS Odometry messages may express twist covariance in a different coordinate
system; that matrix is intentionally unused. The selected 3x3 covariance
cannot substitute for unknown microphone extrinsic uncertainty or correlated
SLAM drift. A normal TF2 lookup does not provide pose covariance. Consumers
must ensure pose estimate and acoustic sample share a synchronized clock and
valid TF/calibration epoch before use. Samples are not interpolated by this
adapter.

Default admission: 500ms maximum pose age and 50ms future tolerance, both
measured against the caller-provided ROS clock. A real ROS node still needs
explicit synchronization, bounded buffering, producer resets and diagnostics;
passing this adapter does **not** authorize metric localization publishing.

Unit tests:
`python3 -m unittest discover -s audition_fusion/test -p 'test_ros_pose_adapter.py' -v`
