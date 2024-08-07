
(cl:in-package :asdf)

(defsystem "team2_package-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "globalwaypoints" :depends-on ("_package_globalwaypoints"))
    (:file "_package_globalwaypoints" :depends-on ("_package"))
    (:file "keyboard_msg" :depends-on ("_package_keyboard_msg"))
    (:file "_package_keyboard_msg" :depends-on ("_package"))
    (:file "longitudinal_controller" :depends-on ("_package_longitudinal_controller"))
    (:file "_package_longitudinal_controller" :depends-on ("_package"))
    (:file "vehicle_state" :depends-on ("_package_vehicle_state"))
    (:file "_package_vehicle_state" :depends-on ("_package"))
  ))