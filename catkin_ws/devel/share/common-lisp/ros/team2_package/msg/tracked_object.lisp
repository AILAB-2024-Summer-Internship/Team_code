; Auto-generated. Do not edit!


(cl:in-package team2_package-msg)


;//! \htmlinclude tracked_object.msg.html

(cl:defclass <tracked_object> (roslisp-msg-protocol:ros-message)
  ((center_x
    :reader center_x
    :initarg :center_x
    :type cl:float
    :initform 0.0)
   (center_y
    :reader center_y
    :initarg :center_y
    :type cl:float
    :initform 0.0)
   (v_x
    :reader v_x
    :initarg :v_x
    :type cl:float
    :initform 0.0)
   (v_y
    :reader v_y
    :initarg :v_y
    :type cl:float
    :initform 0.0)
   (size_x
    :reader size_x
    :initarg :size_x
    :type cl:float
    :initform 0.0)
   (size_y
    :reader size_y
    :initarg :size_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass tracked_object (<tracked_object>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <tracked_object>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'tracked_object)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name team2_package-msg:<tracked_object> is deprecated: use team2_package-msg:tracked_object instead.")))

(cl:ensure-generic-function 'center_x-val :lambda-list '(m))
(cl:defmethod center_x-val ((m <tracked_object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:center_x-val is deprecated.  Use team2_package-msg:center_x instead.")
  (center_x m))

(cl:ensure-generic-function 'center_y-val :lambda-list '(m))
(cl:defmethod center_y-val ((m <tracked_object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:center_y-val is deprecated.  Use team2_package-msg:center_y instead.")
  (center_y m))

(cl:ensure-generic-function 'v_x-val :lambda-list '(m))
(cl:defmethod v_x-val ((m <tracked_object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:v_x-val is deprecated.  Use team2_package-msg:v_x instead.")
  (v_x m))

(cl:ensure-generic-function 'v_y-val :lambda-list '(m))
(cl:defmethod v_y-val ((m <tracked_object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:v_y-val is deprecated.  Use team2_package-msg:v_y instead.")
  (v_y m))

(cl:ensure-generic-function 'size_x-val :lambda-list '(m))
(cl:defmethod size_x-val ((m <tracked_object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:size_x-val is deprecated.  Use team2_package-msg:size_x instead.")
  (size_x m))

(cl:ensure-generic-function 'size_y-val :lambda-list '(m))
(cl:defmethod size_y-val ((m <tracked_object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:size_y-val is deprecated.  Use team2_package-msg:size_y instead.")
  (size_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <tracked_object>) ostream)
  "Serializes a message object of type '<tracked_object>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'center_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'center_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'v_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'v_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'size_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'size_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <tracked_object>) istream)
  "Deserializes a message object of type '<tracked_object>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'center_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'center_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'size_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'size_y) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<tracked_object>)))
  "Returns string type for a message object of type '<tracked_object>"
  "team2_package/tracked_object")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tracked_object)))
  "Returns string type for a message object of type 'tracked_object"
  "team2_package/tracked_object")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<tracked_object>)))
  "Returns md5sum for a message object of type '<tracked_object>"
  "ee6d8746eebf377eed126cb7859c31c3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'tracked_object)))
  "Returns md5sum for a message object of type 'tracked_object"
  "ee6d8746eebf377eed126cb7859c31c3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<tracked_object>)))
  "Returns full string definition for message of type '<tracked_object>"
  (cl:format cl:nil "float64 center_x~%float64 center_y~%float64 v_x~%float64 v_y~%float64 size_x~%float64 size_y~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'tracked_object)))
  "Returns full string definition for message of type 'tracked_object"
  (cl:format cl:nil "float64 center_x~%float64 center_y~%float64 v_x~%float64 v_y~%float64 size_x~%float64 size_y~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <tracked_object>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <tracked_object>))
  "Converts a ROS message object to a list"
  (cl:list 'tracked_object
    (cl:cons ':center_x (center_x msg))
    (cl:cons ':center_y (center_y msg))
    (cl:cons ':v_x (v_x msg))
    (cl:cons ':v_y (v_y msg))
    (cl:cons ':size_x (size_x msg))
    (cl:cons ':size_y (size_y msg))
))
