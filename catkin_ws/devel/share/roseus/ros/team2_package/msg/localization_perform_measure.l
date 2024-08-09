;; Auto-generated. Do not edit!


(when (boundp 'team2_package::localization_perform_measure)
  (if (not (find-package "TEAM2_PACKAGE"))
    (make-package "TEAM2_PACKAGE"))
  (shadow 'localization_perform_measure (find-package "TEAM2_PACKAGE")))
(unless (find-package "TEAM2_PACKAGE::LOCALIZATION_PERFORM_MEASURE")
  (make-package "TEAM2_PACKAGE::LOCALIZATION_PERFORM_MEASURE"))

(in-package "ROS")
;;//! \htmlinclude localization_perform_measure.msg.html
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass team2_package::localization_perform_measure
  :super ros::object
  :slots (_Odometry _vehicle_state _NavSatFix ))

(defmethod team2_package::localization_perform_measure
  (:init
   (&key
    ((:Odometry __Odometry) (instance nav_msgs::Odometry :init))
    ((:vehicle_state __vehicle_state) (instance team2_package::vehicle_state :init))
    ((:NavSatFix __NavSatFix) (instance sensor_msgs::NavSatFix :init))
    )
   (send-super :init)
   (setq _Odometry __Odometry)
   (setq _vehicle_state __vehicle_state)
   (setq _NavSatFix __NavSatFix)
   self)
  (:Odometry
   (&rest __Odometry)
   (if (keywordp (car __Odometry))
       (send* _Odometry __Odometry)
     (progn
       (if __Odometry (setq _Odometry (car __Odometry)))
       _Odometry)))
  (:vehicle_state
   (&rest __vehicle_state)
   (if (keywordp (car __vehicle_state))
       (send* _vehicle_state __vehicle_state)
     (progn
       (if __vehicle_state (setq _vehicle_state (car __vehicle_state)))
       _vehicle_state)))
  (:NavSatFix
   (&rest __NavSatFix)
   (if (keywordp (car __NavSatFix))
       (send* _NavSatFix __NavSatFix)
     (progn
       (if __NavSatFix (setq _NavSatFix (car __NavSatFix)))
       _NavSatFix)))
  (:serialization-length
   ()
   (+
    ;; nav_msgs/Odometry _Odometry
    (send _Odometry :serialization-length)
    ;; team2_package/vehicle_state _vehicle_state
    (send _vehicle_state :serialization-length)
    ;; sensor_msgs/NavSatFix _NavSatFix
    (send _NavSatFix :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/Odometry _Odometry
       (send _Odometry :serialize s)
     ;; team2_package/vehicle_state _vehicle_state
       (send _vehicle_state :serialize s)
     ;; sensor_msgs/NavSatFix _NavSatFix
       (send _NavSatFix :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/Odometry _Odometry
     (send _Odometry :deserialize buf ptr-) (incf ptr- (send _Odometry :serialization-length))
   ;; team2_package/vehicle_state _vehicle_state
     (send _vehicle_state :deserialize buf ptr-) (incf ptr- (send _vehicle_state :serialization-length))
   ;; sensor_msgs/NavSatFix _NavSatFix
     (send _NavSatFix :deserialize buf ptr-) (incf ptr- (send _NavSatFix :serialization-length))
   ;;
   self)
  )

(setf (get team2_package::localization_perform_measure :md5sum-) "63820517e6241f49dc999aa8e05f4c4a")
(setf (get team2_package::localization_perform_measure :datatype-) "team2_package/localization_perform_measure")
(setf (get team2_package::localization_perform_measure :definition-)
      "nav_msgs/Odometry Odometry
team2_package/vehicle_state vehicle_state
sensor_msgs/NavSatFix NavSatFix
================================================================================
MSG: nav_msgs/Odometry
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: team2_package/vehicle_state
float32 x
float32 y
float32 yaw
================================================================================
MSG: sensor_msgs/NavSatFix
# Navigation Satellite fix for any Global Navigation Satellite System
#
# Specified using the WGS 84 reference ellipsoid

# header.stamp specifies the ROS time for this measurement (the
#        corresponding satellite time may be reported using the
#        sensor_msgs/TimeReference message).
#
# header.frame_id is the frame of reference reported by the satellite
#        receiver, usually the location of the antenna.  This is a
#        Euclidean frame relative to the vehicle, not a reference
#        ellipsoid.
Header header

# satellite fix status information
NavSatStatus status

# Latitude [degrees]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
float64 altitude

# Position covariance [m^2] defined relative to a tangential plane
# through the reported position. The components are East, North, and
# Up (ENU), in row-major order.
#
# Beware: this coordinate system exhibits singularities at the poles.

float64[9] position_covariance

# If the covariance of the fix is known, fill it in completely. If the
# GPS receiver provides the variance of each measurement, put them
# along the diagonal. If only Dilution of Precision is available,
# estimate an approximate covariance from that.

uint8 COVARIANCE_TYPE_UNKNOWN = 0
uint8 COVARIANCE_TYPE_APPROXIMATED = 1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
uint8 COVARIANCE_TYPE_KNOWN = 3

uint8 position_covariance_type

================================================================================
MSG: sensor_msgs/NavSatStatus
# Navigation Satellite fix status for any Global Navigation Satellite System

# Whether to output an augmented fix is determined by both the fix
# type and the last time differential corrections were received.  A
# fix is valid when status >= STATUS_FIX.

int8 STATUS_NO_FIX =  -1        # unable to fix position
int8 STATUS_FIX =      0        # unaugmented fix
int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation

int8 status

# Bits defining which Global Navigation Satellite System signals were
# used by the receiver.

uint16 SERVICE_GPS =     1
uint16 SERVICE_GLONASS = 2
uint16 SERVICE_COMPASS = 4      # includes BeiDou.
uint16 SERVICE_GALILEO = 8

uint16 service

")



(provide :team2_package/localization_perform_measure "63820517e6241f49dc999aa8e05f4c4a")


