;; Auto-generated. Do not edit!


(when (boundp 'team2_package::globalwaypoints)
  (if (not (find-package "TEAM2_PACKAGE"))
    (make-package "TEAM2_PACKAGE"))
  (shadow 'globalwaypoints (find-package "TEAM2_PACKAGE")))
(unless (find-package "TEAM2_PACKAGE::GLOBALWAYPOINTS")
  (make-package "TEAM2_PACKAGE::GLOBALWAYPOINTS"))

(in-package "ROS")
;;//! \htmlinclude globalwaypoints.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*VOID*" (find-package "TEAM2_PACKAGE::GLOBALWAYPOINTS"))
(shadow '*VOID* (find-package "TEAM2_PACKAGE::GLOBALWAYPOINTS"))
(defconstant team2_package::globalwaypoints::*VOID* -1)
(intern "*LEFT*" (find-package "TEAM2_PACKAGE::GLOBALWAYPOINTS"))
(shadow '*LEFT* (find-package "TEAM2_PACKAGE::GLOBALWAYPOINTS"))
(defconstant team2_package::globalwaypoints::*LEFT* 1)
(intern "*RIGHT*" (find-package "TEAM2_PACKAGE::GLOBALWAYPOINTS"))
(shadow '*RIGHT* (find-package "TEAM2_PACKAGE::GLOBALWAYPOINTS"))
(defconstant team2_package::globalwaypoints::*RIGHT* 2)
(intern "*STRAIGHT*" (find-package "TEAM2_PACKAGE::GLOBALWAYPOINTS"))
(shadow '*STRAIGHT* (find-package "TEAM2_PACKAGE::GLOBALWAYPOINTS"))
(defconstant team2_package::globalwaypoints::*STRAIGHT* 3)
(intern "*LANEFOLLOW*" (find-package "TEAM2_PACKAGE::GLOBALWAYPOINTS"))
(shadow '*LANEFOLLOW* (find-package "TEAM2_PACKAGE::GLOBALWAYPOINTS"))
(defconstant team2_package::globalwaypoints::*LANEFOLLOW* 4)
(intern "*CHANGELANELEFT*" (find-package "TEAM2_PACKAGE::GLOBALWAYPOINTS"))
(shadow '*CHANGELANELEFT* (find-package "TEAM2_PACKAGE::GLOBALWAYPOINTS"))
(defconstant team2_package::globalwaypoints::*CHANGELANELEFT* 5)
(intern "*CHANGELANERIGHT*" (find-package "TEAM2_PACKAGE::GLOBALWAYPOINTS"))
(shadow '*CHANGELANERIGHT* (find-package "TEAM2_PACKAGE::GLOBALWAYPOINTS"))
(defconstant team2_package::globalwaypoints::*CHANGELANERIGHT* 6)

(defun team2_package::globalwaypoints-to-symbol (const)
  (cond
        ((= const -1) 'team2_package::globalwaypoints::*VOID*)
        ((= const 1) 'team2_package::globalwaypoints::*LEFT*)
        ((= const 2) 'team2_package::globalwaypoints::*RIGHT*)
        ((= const 3) 'team2_package::globalwaypoints::*STRAIGHT*)
        ((= const 4) 'team2_package::globalwaypoints::*LANEFOLLOW*)
        ((= const 5) 'team2_package::globalwaypoints::*CHANGELANELEFT*)
        ((= const 6) 'team2_package::globalwaypoints::*CHANGELANERIGHT*)
        (t nil)))

(defclass team2_package::globalwaypoints
  :super ros::object
  :slots (_header _road_options _x _y ))

(defmethod team2_package::globalwaypoints
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:road_options __road_options) (make-array 0 :initial-element 0 :element-type :integer))
    ((:x __x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y __y) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _road_options __road_options)
   (setq _x __x)
   (setq _y __y)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:road_options
   (&optional __road_options)
   (if __road_options (setq _road_options __road_options)) _road_options)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int8[] _road_options
    (* 1    (length _road_options)) 4
    ;; float64[] _x
    (* 8    (length _x)) 4
    ;; float64[] _y
    (* 8    (length _y)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int8[] _road_options
     (write-long (length _road_options) s)
     (dotimes (i (length _road_options))
       (write-byte (elt _road_options i) s)
       )
     ;; float64[] _x
     (write-long (length _x) s)
     (dotimes (i (length _x))
       (sys::poke (elt _x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _y
     (write-long (length _y) s)
     (dotimes (i (length _y))
       (sys::poke (elt _y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int8[] _road_options
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _road_options (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _road_options i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _road_options i) 127) (setf (elt _road_options i) (- (elt _road_options i) 256)))
     ))
   ;; float64[] _x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get team2_package::globalwaypoints :md5sum-) "f169d64c8dec16596456b30665e5fbde")
(setf (get team2_package::globalwaypoints :datatype-) "team2_package/globalwaypoints")
(setf (get team2_package::globalwaypoints :definition-)
      "std_msgs/Header header

int8[] road_options
float64[] x
float64[] y

int8 VOID=-1
int8 LEFT=1
int8 RIGHT=2
int8 STRAIGHT=3
int8 LANEFOLLOW=4
int8 CHANGELANELEFT=5
int8 CHANGELANERIGHT=6
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

")



(provide :team2_package/globalwaypoints "f169d64c8dec16596456b30665e5fbde")


