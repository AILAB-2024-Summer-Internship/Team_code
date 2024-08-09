;; Auto-generated. Do not edit!


(when (boundp 'team2_package::tracked_object_array)
  (if (not (find-package "TEAM2_PACKAGE"))
    (make-package "TEAM2_PACKAGE"))
  (shadow 'tracked_object_array (find-package "TEAM2_PACKAGE")))
(unless (find-package "TEAM2_PACKAGE::TRACKED_OBJECT_ARRAY")
  (make-package "TEAM2_PACKAGE::TRACKED_OBJECT_ARRAY"))

(in-package "ROS")
;;//! \htmlinclude tracked_object_array.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass team2_package::tracked_object_array
  :super ros::object
  :slots (_header _objects ))

(defmethod team2_package::tracked_object_array
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:objects __objects) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _objects __objects)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:objects
   (&rest __objects)
   (if (keywordp (car __objects))
       (send* _objects __objects)
     (progn
       (if __objects (setq _objects (car __objects)))
       _objects)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; team2_package/tracked_object[] _objects
    (apply #'+ (send-all _objects :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; team2_package/tracked_object[] _objects
     (write-long (length _objects) s)
     (dolist (elem _objects)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; team2_package/tracked_object[] _objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _objects (let (r) (dotimes (i n) (push (instance team2_package::tracked_object :init) r)) r))
     (dolist (elem- _objects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get team2_package::tracked_object_array :md5sum-) "c50235ee5e169f41fa0e37ba85beeba3")
(setf (get team2_package::tracked_object_array :datatype-) "team2_package/tracked_object_array")
(setf (get team2_package::tracked_object_array :definition-)
      "std_msgs/Header header
tracked_object[] objects
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
MSG: team2_package/tracked_object
float64 center_x
float64 center_y
float64 v_x
float64 v_y
float64 size_x
float64 size_y
")



(provide :team2_package/tracked_object_array "c50235ee5e169f41fa0e37ba85beeba3")


