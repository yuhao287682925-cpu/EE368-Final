
(cl:in-package :asdf)

(defsystem "block_drawing_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :block_drawing_msgs-msg
               :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "ExecuteDrawing" :depends-on ("_package_ExecuteDrawing"))
    (:file "_package_ExecuteDrawing" :depends-on ("_package"))
    (:file "GenerateTrajectory" :depends-on ("_package_GenerateTrajectory"))
    (:file "_package_GenerateTrajectory" :depends-on ("_package"))
    (:file "SetBlockPose" :depends-on ("_package_SetBlockPose"))
    (:file "_package_SetBlockPose" :depends-on ("_package"))
  ))