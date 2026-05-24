;; Auto-generated. Do not edit!


(when (boundp 'block_drawing_msgs::SurfaceTrajectory)
  (if (not (find-package "BLOCK_DRAWING_MSGS"))
    (make-package "BLOCK_DRAWING_MSGS"))
  (shadow 'SurfaceTrajectory (find-package "BLOCK_DRAWING_MSGS")))
(unless (find-package "BLOCK_DRAWING_MSGS::SURFACETRAJECTORY")
  (make-package "BLOCK_DRAWING_MSGS::SURFACETRAJECTORY"))

(in-package "ROS")
;;//! \htmlinclude SurfaceTrajectory.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass block_drawing_msgs::SurfaceTrajectory
  :super ros::object
  :slots (_face_id _waypoints _arc_lengths ))

(defmethod block_drawing_msgs::SurfaceTrajectory
  (:init
   (&key
    ((:face_id __face_id) 0)
    ((:waypoints __waypoints) ())
    ((:arc_lengths __arc_lengths) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _face_id (round __face_id))
   (setq _waypoints __waypoints)
   (setq _arc_lengths __arc_lengths)
   self)
  (:face_id
   (&optional __face_id)
   (if __face_id (setq _face_id __face_id)) _face_id)
  (:waypoints
   (&rest __waypoints)
   (if (keywordp (car __waypoints))
       (send* _waypoints __waypoints)
     (progn
       (if __waypoints (setq _waypoints (car __waypoints)))
       _waypoints)))
  (:arc_lengths
   (&optional __arc_lengths)
   (if __arc_lengths (setq _arc_lengths __arc_lengths)) _arc_lengths)
  (:serialization-length
   ()
   (+
    ;; int32 _face_id
    4
    ;; geometry_msgs/Pose[] _waypoints
    (apply #'+ (send-all _waypoints :serialization-length)) 4
    ;; float64[] _arc_lengths
    (* 8    (length _arc_lengths)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _face_id
       (write-long _face_id s)
     ;; geometry_msgs/Pose[] _waypoints
     (write-long (length _waypoints) s)
     (dolist (elem _waypoints)
       (send elem :serialize s)
       )
     ;; float64[] _arc_lengths
     (write-long (length _arc_lengths) s)
     (dotimes (i (length _arc_lengths))
       (sys::poke (elt _arc_lengths i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _face_id
     (setq _face_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Pose[] _waypoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoints (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _waypoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[] _arc_lengths
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _arc_lengths (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _arc_lengths i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get block_drawing_msgs::SurfaceTrajectory :md5sum-) "5b572666885f29b8eb6f14e36c9ed7c7")
(setf (get block_drawing_msgs::SurfaceTrajectory :datatype-) "block_drawing_msgs/SurfaceTrajectory")
(setf (get block_drawing_msgs::SurfaceTrajectory :definition-)
      "# SurfaceTrajectory.msg
# 单面轨迹: 包含该面所有 waypoints

int32   face_id                    # 0=顶面, 1=前面, 2=右面, 3=后面, 4=左面
geometry_msgs/Pose[] waypoints     # 末端期望位姿 (含法兰补偿)
float64[] arc_lengths              # 累计弧长, 用于速度规划 [m]

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :block_drawing_msgs/SurfaceTrajectory "5b572666885f29b8eb6f14e36c9ed7c7")


