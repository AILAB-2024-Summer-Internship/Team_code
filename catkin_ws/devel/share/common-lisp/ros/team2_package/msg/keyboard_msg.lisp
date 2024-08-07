; Auto-generated. Do not edit!


(cl:in-package team2_package-msg)


;//! \htmlinclude keyboard_msg.msg.html

(cl:defclass <keyboard_msg> (roslisp-msg-protocol:ros-message)
  ((fb_speed
    :reader fb_speed
    :initarg :fb_speed
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (gear
    :reader gear
    :initarg :gear
    :type cl:integer
    :initform 0)
   (brake
    :reader brake
    :initarg :brake
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass keyboard_msg (<keyboard_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <keyboard_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'keyboard_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name team2_package-msg:<keyboard_msg> is deprecated: use team2_package-msg:keyboard_msg instead.")))

(cl:ensure-generic-function 'fb_speed-val :lambda-list '(m))
(cl:defmethod fb_speed-val ((m <keyboard_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:fb_speed-val is deprecated.  Use team2_package-msg:fb_speed instead.")
  (fb_speed m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <keyboard_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:yaw-val is deprecated.  Use team2_package-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'gear-val :lambda-list '(m))
(cl:defmethod gear-val ((m <keyboard_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:gear-val is deprecated.  Use team2_package-msg:gear instead.")
  (gear m))

(cl:ensure-generic-function 'brake-val :lambda-list '(m))
(cl:defmethod brake-val ((m <keyboard_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:brake-val is deprecated.  Use team2_package-msg:brake instead.")
  (brake m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <keyboard_msg>) ostream)
  "Serializes a message object of type '<keyboard_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fb_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'gear)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'brake) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <keyboard_msg>) istream)
  "Deserializes a message object of type '<keyboard_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fb_speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gear) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'brake) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<keyboard_msg>)))
  "Returns string type for a message object of type '<keyboard_msg>"
  "team2_package/keyboard_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'keyboard_msg)))
  "Returns string type for a message object of type 'keyboard_msg"
  "team2_package/keyboard_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<keyboard_msg>)))
  "Returns md5sum for a message object of type '<keyboard_msg>"
  "bd4b2bffdbfc91ff88466d3ab2a24fe4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'keyboard_msg)))
  "Returns md5sum for a message object of type 'keyboard_msg"
  "bd4b2bffdbfc91ff88466d3ab2a24fe4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<keyboard_msg>)))
  "Returns full string definition for message of type '<keyboard_msg>"
  (cl:format cl:nil "float32 fb_speed~%float32 yaw~%int32 gear~%bool brake~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'keyboard_msg)))
  "Returns full string definition for message of type 'keyboard_msg"
  (cl:format cl:nil "float32 fb_speed~%float32 yaw~%int32 gear~%bool brake~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <keyboard_msg>))
  (cl:+ 0
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <keyboard_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'keyboard_msg
    (cl:cons ':fb_speed (fb_speed msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':gear (gear msg))
    (cl:cons ':brake (brake msg))
))
