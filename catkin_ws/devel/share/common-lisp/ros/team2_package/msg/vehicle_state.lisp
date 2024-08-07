; Auto-generated. Do not edit!


(cl:in-package team2_package-msg)


;//! \htmlinclude vehicle_state.msg.html

(cl:defclass <vehicle_state> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass vehicle_state (<vehicle_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vehicle_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vehicle_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name team2_package-msg:<vehicle_state> is deprecated: use team2_package-msg:vehicle_state instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <vehicle_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:x-val is deprecated.  Use team2_package-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <vehicle_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:y-val is deprecated.  Use team2_package-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <vehicle_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:yaw-val is deprecated.  Use team2_package-msg:yaw instead.")
  (yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vehicle_state>) ostream)
  "Serializes a message object of type '<vehicle_state>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vehicle_state>) istream)
  "Deserializes a message object of type '<vehicle_state>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vehicle_state>)))
  "Returns string type for a message object of type '<vehicle_state>"
  "team2_package/vehicle_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vehicle_state)))
  "Returns string type for a message object of type 'vehicle_state"
  "team2_package/vehicle_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vehicle_state>)))
  "Returns md5sum for a message object of type '<vehicle_state>"
  "47802147045815b06859cca542c21d31")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vehicle_state)))
  "Returns md5sum for a message object of type 'vehicle_state"
  "47802147045815b06859cca542c21d31")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vehicle_state>)))
  "Returns full string definition for message of type '<vehicle_state>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 yaw~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vehicle_state)))
  "Returns full string definition for message of type 'vehicle_state"
  (cl:format cl:nil "float32 x~%float32 y~%float32 yaw~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vehicle_state>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vehicle_state>))
  "Converts a ROS message object to a list"
  (cl:list 'vehicle_state
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':yaw (yaw msg))
))
