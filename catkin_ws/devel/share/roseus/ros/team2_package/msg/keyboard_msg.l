;; Auto-generated. Do not edit!


(when (boundp 'team2_package::keyboard_msg)
  (if (not (find-package "TEAM2_PACKAGE"))
    (make-package "TEAM2_PACKAGE"))
  (shadow 'keyboard_msg (find-package "TEAM2_PACKAGE")))
(unless (find-package "TEAM2_PACKAGE::KEYBOARD_MSG")
  (make-package "TEAM2_PACKAGE::KEYBOARD_MSG"))

(in-package "ROS")
;;//! \htmlinclude keyboard_msg.msg.html


(defclass team2_package::keyboard_msg
  :super ros::object
  :slots (_fb_speed _yaw _gear _brake ))

(defmethod team2_package::keyboard_msg
  (:init
   (&key
    ((:fb_speed __fb_speed) 0.0)
    ((:yaw __yaw) 0.0)
    ((:gear __gear) 0)
    ((:brake __brake) nil)
    )
   (send-super :init)
   (setq _fb_speed (float __fb_speed))
   (setq _yaw (float __yaw))
   (setq _gear (round __gear))
   (setq _brake __brake)
   self)
  (:fb_speed
   (&optional __fb_speed)
   (if __fb_speed (setq _fb_speed __fb_speed)) _fb_speed)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:gear
   (&optional __gear)
   (if __gear (setq _gear __gear)) _gear)
  (:brake
   (&optional (__brake :null))
   (if (not (eq __brake :null)) (setq _brake __brake)) _brake)
  (:serialization-length
   ()
   (+
    ;; float32 _fb_speed
    4
    ;; float32 _yaw
    4
    ;; int32 _gear
    4
    ;; bool _brake
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _fb_speed
       (sys::poke _fb_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _gear
       (write-long _gear s)
     ;; bool _brake
       (if _brake (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _fb_speed
     (setq _fb_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _gear
     (setq _gear (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _brake
     (setq _brake (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get team2_package::keyboard_msg :md5sum-) "bd4b2bffdbfc91ff88466d3ab2a24fe4")
(setf (get team2_package::keyboard_msg :datatype-) "team2_package/keyboard_msg")
(setf (get team2_package::keyboard_msg :definition-)
      "float32 fb_speed
float32 yaw
int32 gear
bool brake
")



(provide :team2_package/keyboard_msg "bd4b2bffdbfc91ff88466d3ab2a24fe4")


