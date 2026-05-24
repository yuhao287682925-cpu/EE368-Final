
(cl:in-package :asdf)

(defsystem "block_drawing_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ContactState" :depends-on ("_package_ContactState"))
    (:file "_package_ContactState" :depends-on ("_package"))
    (:file "DrawingExecutionAction" :depends-on ("_package_DrawingExecutionAction"))
    (:file "_package_DrawingExecutionAction" :depends-on ("_package"))
    (:file "DrawingExecutionActionFeedback" :depends-on ("_package_DrawingExecutionActionFeedback"))
    (:file "_package_DrawingExecutionActionFeedback" :depends-on ("_package"))
    (:file "DrawingExecutionActionGoal" :depends-on ("_package_DrawingExecutionActionGoal"))
    (:file "_package_DrawingExecutionActionGoal" :depends-on ("_package"))
    (:file "DrawingExecutionActionResult" :depends-on ("_package_DrawingExecutionActionResult"))
    (:file "_package_DrawingExecutionActionResult" :depends-on ("_package"))
    (:file "DrawingExecutionFeedback" :depends-on ("_package_DrawingExecutionFeedback"))
    (:file "_package_DrawingExecutionFeedback" :depends-on ("_package"))
    (:file "DrawingExecutionGoal" :depends-on ("_package_DrawingExecutionGoal"))
    (:file "_package_DrawingExecutionGoal" :depends-on ("_package"))
    (:file "DrawingExecutionResult" :depends-on ("_package_DrawingExecutionResult"))
    (:file "_package_DrawingExecutionResult" :depends-on ("_package"))
    (:file "SurfaceTrajectory" :depends-on ("_package_SurfaceTrajectory"))
    (:file "_package_SurfaceTrajectory" :depends-on ("_package"))
  ))