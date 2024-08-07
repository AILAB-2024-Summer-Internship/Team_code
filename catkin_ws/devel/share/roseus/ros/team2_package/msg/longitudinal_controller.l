;; Auto-generated. Do not edit!


(when (boundp 'team2_package::longitudinal_controller)
  (if (not (find-package "TEAM2_PACKAGE"))
    (make-package "TEAM2_PACKAGE"))
  (shadow 'longitudinal_controller (find-package "TEAM2_PACKAGE")))
(unless (find-package "TEAM2_PACKAGE::LONGITUDINAL_CONTROLLER")
  (make-package "TEAM2_PACKAGE::LONGITUDINAL_CONTROLLER"))

(in-package "ROS")
;;//! \htmlinclude longitudinal_controller.msg.html


(defclass team2_package::longitudinal_controller
  :super ros::object
  :slots (_throttle _brake ))

(defmethod team2_package::longitudinal_controller
  (:init
   (&key
    ((:throttle __throttle) 0.0)
    ((:brake __brake) 0.0)
    )
   (send-super :init)
   (setq _throttle (float __throttle))
   (setq _brake (float __brake))
   self)
  (:throttle
   (&optional __throttle)
   (if __throttle (setq _throttle __throttle)) _throttle)
  (:brake
   (&optional __brake)
   (if __brake (setq _brake __brake)) _brake)
  (:serialization-length
   ()
   (+
    ;; float32 _throttle
    4
    ;; float32 _brake
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _throttle
       (sys::poke _throttle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _brake
       (sys::poke _brake (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _throttle
     (setq _throttle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _brake
     (setq _brake (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get team2_package::longitudinal_controller :md5sum-) "1220857b9c02b48ec2fc3654fbd7e1a3")
(setf (get team2_package::longitudinal_controller :datatype-) "team2_package/longitudinal_controller")
(setf (get team2_package::longitudinal_controller :definition-)
      "float32 throttle
float32 brake
")



(provide :team2_package/longitudinal_controller "1220857b9c02b48ec2fc3654fbd7e1a3")


