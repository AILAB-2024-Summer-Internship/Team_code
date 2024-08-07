; Auto-generated. Do not edit!


(cl:in-package team2_package-msg)


;//! \htmlinclude longitudinal_controller.msg.html

(cl:defclass <longitudinal_controller> (roslisp-msg-protocol:ros-message)
  ((throttle
    :reader throttle
    :initarg :throttle
    :type cl:float
    :initform 0.0)
   (brake
    :reader brake
    :initarg :brake
    :type cl:float
    :initform 0.0))
)

(cl:defclass longitudinal_controller (<longitudinal_controller>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <longitudinal_controller>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'longitudinal_controller)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name team2_package-msg:<longitudinal_controller> is deprecated: use team2_package-msg:longitudinal_controller instead.")))

(cl:ensure-generic-function 'throttle-val :lambda-list '(m))
(cl:defmethod throttle-val ((m <longitudinal_controller>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:throttle-val is deprecated.  Use team2_package-msg:throttle instead.")
  (throttle m))

(cl:ensure-generic-function 'brake-val :lambda-list '(m))
(cl:defmethod brake-val ((m <longitudinal_controller>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team2_package-msg:brake-val is deprecated.  Use team2_package-msg:brake instead.")
  (brake m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <longitudinal_controller>) ostream)
  "Serializes a message object of type '<longitudinal_controller>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'throttle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'brake))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <longitudinal_controller>) istream)
  "Deserializes a message object of type '<longitudinal_controller>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'throttle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'brake) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<longitudinal_controller>)))
  "Returns string type for a message object of type '<longitudinal_controller>"
  "team2_package/longitudinal_controller")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'longitudinal_controller)))
  "Returns string type for a message object of type 'longitudinal_controller"
  "team2_package/longitudinal_controller")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<longitudinal_controller>)))
  "Returns md5sum for a message object of type '<longitudinal_controller>"
  "1220857b9c02b48ec2fc3654fbd7e1a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'longitudinal_controller)))
  "Returns md5sum for a message object of type 'longitudinal_controller"
  "1220857b9c02b48ec2fc3654fbd7e1a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<longitudinal_controller>)))
  "Returns full string definition for message of type '<longitudinal_controller>"
  (cl:format cl:nil "float32 throttle~%float32 brake~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'longitudinal_controller)))
  "Returns full string definition for message of type 'longitudinal_controller"
  (cl:format cl:nil "float32 throttle~%float32 brake~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <longitudinal_controller>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <longitudinal_controller>))
  "Converts a ROS message object to a list"
  (cl:list 'longitudinal_controller
    (cl:cons ':throttle (throttle msg))
    (cl:cons ':brake (brake msg))
))
