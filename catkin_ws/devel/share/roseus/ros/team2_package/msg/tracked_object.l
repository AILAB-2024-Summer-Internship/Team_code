;; Auto-generated. Do not edit!


(when (boundp 'team2_package::tracked_object)
  (if (not (find-package "TEAM2_PACKAGE"))
    (make-package "TEAM2_PACKAGE"))
  (shadow 'tracked_object (find-package "TEAM2_PACKAGE")))
(unless (find-package "TEAM2_PACKAGE::TRACKED_OBJECT")
  (make-package "TEAM2_PACKAGE::TRACKED_OBJECT"))

(in-package "ROS")
;;//! \htmlinclude tracked_object.msg.html


(defclass team2_package::tracked_object
  :super ros::object
  :slots (_center_x _center_y _v_x _v_y _size_x _size_y ))

(defmethod team2_package::tracked_object
  (:init
   (&key
    ((:center_x __center_x) 0.0)
    ((:center_y __center_y) 0.0)
    ((:v_x __v_x) 0.0)
    ((:v_y __v_y) 0.0)
    ((:size_x __size_x) 0.0)
    ((:size_y __size_y) 0.0)
    )
   (send-super :init)
   (setq _center_x (float __center_x))
   (setq _center_y (float __center_y))
   (setq _v_x (float __v_x))
   (setq _v_y (float __v_y))
   (setq _size_x (float __size_x))
   (setq _size_y (float __size_y))
   self)
  (:center_x
   (&optional __center_x)
   (if __center_x (setq _center_x __center_x)) _center_x)
  (:center_y
   (&optional __center_y)
   (if __center_y (setq _center_y __center_y)) _center_y)
  (:v_x
   (&optional __v_x)
   (if __v_x (setq _v_x __v_x)) _v_x)
  (:v_y
   (&optional __v_y)
   (if __v_y (setq _v_y __v_y)) _v_y)
  (:size_x
   (&optional __size_x)
   (if __size_x (setq _size_x __size_x)) _size_x)
  (:size_y
   (&optional __size_y)
   (if __size_y (setq _size_y __size_y)) _size_y)
  (:serialization-length
   ()
   (+
    ;; float64 _center_x
    8
    ;; float64 _center_y
    8
    ;; float64 _v_x
    8
    ;; float64 _v_y
    8
    ;; float64 _size_x
    8
    ;; float64 _size_y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _center_x
       (sys::poke _center_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _center_y
       (sys::poke _center_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v_x
       (sys::poke _v_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v_y
       (sys::poke _v_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _size_x
       (sys::poke _size_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _size_y
       (sys::poke _size_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _center_x
     (setq _center_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _center_y
     (setq _center_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v_x
     (setq _v_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v_y
     (setq _v_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _size_x
     (setq _size_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _size_y
     (setq _size_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get team2_package::tracked_object :md5sum-) "ee6d8746eebf377eed126cb7859c31c3")
(setf (get team2_package::tracked_object :datatype-) "team2_package/tracked_object")
(setf (get team2_package::tracked_object :definition-)
      "float64 center_x
float64 center_y
float64 v_x
float64 v_y
float64 size_x
float64 size_y
")



(provide :team2_package/tracked_object "ee6d8746eebf377eed126cb7859c31c3")


