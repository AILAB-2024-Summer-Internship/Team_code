; Auto-generated. Do not edit!


(cl:in-package team2_package-msg)


;//! \htmlinclude tracked_object_array.msg.html

(cl:defclass <tracked_object_array> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (objects
    :reader objects
    :initarg :objects
    :type (cl:vector team2_package-msg:tracked_object)
   :initform (cl:make-array 0 :element-type 'team2_package-msg:tracked_object :initial-element (cl:make-instance 'team2_package-msg:tracked_object))))
)

(cl:defclass tracked_object_array (<tracked_object_array>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <tracked_object_array>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'tracked_object_array)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name team2_package-msg:<tracked_object_array> is deprecated: use team2_package-msg:tracked_object_array instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <tracked_object_array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:header-val is deprecated.  Use team2_package-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'objects-val :lambda-list '(m))
(cl:defmethod objects-val ((m <tracked_object_array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:objects-val is deprecated.  Use team2_package-msg:objects instead.")
  (objects m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <tracked_object_array>) ostream)
  "Serializes a message object of type '<tracked_object_array>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'objects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'objects))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <tracked_object_array>) istream)
  "Deserializes a message object of type '<tracked_object_array>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'objects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'objects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'team2_package-msg:tracked_object))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<tracked_object_array>)))
  "Returns string type for a message object of type '<tracked_object_array>"
  "team2_package/tracked_object_array")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tracked_object_array)))
  "Returns string type for a message object of type 'tracked_object_array"
  "team2_package/tracked_object_array")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<tracked_object_array>)))
  "Returns md5sum for a message object of type '<tracked_object_array>"
  "c50235ee5e169f41fa0e37ba85beeba3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'tracked_object_array)))
  "Returns md5sum for a message object of type 'tracked_object_array"
  "c50235ee5e169f41fa0e37ba85beeba3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<tracked_object_array>)))
  "Returns full string definition for message of type '<tracked_object_array>"
  (cl:format cl:nil "std_msgs/Header header~%tracked_object[] objects~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: team2_package/tracked_object~%float64 center_x~%float64 center_y~%float64 v_x~%float64 v_y~%float64 size_x~%float64 size_y~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'tracked_object_array)))
  "Returns full string definition for message of type 'tracked_object_array"
  (cl:format cl:nil "std_msgs/Header header~%tracked_object[] objects~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: team2_package/tracked_object~%float64 center_x~%float64 center_y~%float64 v_x~%float64 v_y~%float64 size_x~%float64 size_y~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <tracked_object_array>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'objects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <tracked_object_array>))
  "Converts a ROS message object to a list"
  (cl:list 'tracked_object_array
    (cl:cons ':header (header msg))
    (cl:cons ':objects (objects msg))
))
