;; Auto-generated. Do not edit!


(when (boundp 'block_drawing_msgs::SetBlockPose)
  (if (not (find-package "BLOCK_DRAWING_MSGS"))
    (make-package "BLOCK_DRAWING_MSGS"))
  (shadow 'SetBlockPose (find-package "BLOCK_DRAWING_MSGS")))
(unless (find-package "BLOCK_DRAWING_MSGS::SETBLOCKPOSE")
  (make-package "BLOCK_DRAWING_MSGS::SETBLOCKPOSE"))
(unless (find-package "BLOCK_DRAWING_MSGS::SETBLOCKPOSEREQUEST")
  (make-package "BLOCK_DRAWING_MSGS::SETBLOCKPOSEREQUEST"))
(unless (find-package "BLOCK_DRAWING_MSGS::SETBLOCKPOSERESPONSE")
  (make-package "BLOCK_DRAWING_MSGS::SETBLOCKPOSERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass block_drawing_msgs::SetBlockPoseRequest
  :super ros::object
  :slots (_block_pose _L _W _H _face_offset_u _face_offset_v _center_face _center_u_mm _center_v_mm ))

(defmethod block_drawing_msgs::SetBlockPoseRequest
  (:init
   (&key
    ((:block_pose __block_pose) (instance geometry_msgs::Pose :init))
    ((:L __L) 0.0)
    ((:W __W) 0.0)
    ((:H __H) 0.0)
    ((:face_offset_u __face_offset_u) (make-array 5 :initial-element 0.0 :element-type :float))
    ((:face_offset_v __face_offset_v) (make-array 5 :initial-element 0.0 :element-type :float))
    ((:center_face __center_face) 0)
    ((:center_u_mm __center_u_mm) 0.0)
    ((:center_v_mm __center_v_mm) 0.0)
    )
   (send-super :init)
   (setq _block_pose __block_pose)
   (setq _L (float __L))
   (setq _W (float __W))
   (setq _H (float __H))
   (setq _face_offset_u __face_offset_u)
   (setq _face_offset_v __face_offset_v)
   (setq _center_face (round __center_face))
   (setq _center_u_mm (float __center_u_mm))
   (setq _center_v_mm (float __center_v_mm))
   self)
  (:block_pose
   (&rest __block_pose)
   (if (keywordp (car __block_pose))
       (send* _block_pose __block_pose)
     (progn
       (if __block_pose (setq _block_pose (car __block_pose)))
       _block_pose)))
  (:L
   (&optional __L)
   (if __L (setq _L __L)) _L)
  (:W
   (&optional __W)
   (if __W (setq _W __W)) _W)
  (:H
   (&optional __H)
   (if __H (setq _H __H)) _H)
  (:face_offset_u
   (&optional __face_offset_u)
   (if __face_offset_u (setq _face_offset_u __face_offset_u)) _face_offset_u)
  (:face_offset_v
   (&optional __face_offset_v)
   (if __face_offset_v (setq _face_offset_v __face_offset_v)) _face_offset_v)
  (:center_face
   (&optional __center_face)
   (if __center_face (setq _center_face __center_face)) _center_face)
  (:center_u_mm
   (&optional __center_u_mm)
   (if __center_u_mm (setq _center_u_mm __center_u_mm)) _center_u_mm)
  (:center_v_mm
   (&optional __center_v_mm)
   (if __center_v_mm (setq _center_v_mm __center_v_mm)) _center_v_mm)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _block_pose
    (send _block_pose :serialization-length)
    ;; float64 _L
    8
    ;; float64 _W
    8
    ;; float64 _H
    8
    ;; float64[5] _face_offset_u
    (* 8    5)
    ;; float64[5] _face_offset_v
    (* 8    5)
    ;; int32 _center_face
    4
    ;; float64 _center_u_mm
    8
    ;; float64 _center_v_mm
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _block_pose
       (send _block_pose :serialize s)
     ;; float64 _L
       (sys::poke _L (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _W
       (sys::poke _W (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _H
       (sys::poke _H (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[5] _face_offset_u
     (dotimes (i 5)
       (sys::poke (elt _face_offset_u i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[5] _face_offset_v
     (dotimes (i 5)
       (sys::poke (elt _face_offset_v i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32 _center_face
       (write-long _center_face s)
     ;; float64 _center_u_mm
       (sys::poke _center_u_mm (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _center_v_mm
       (sys::poke _center_v_mm (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _block_pose
     (send _block_pose :deserialize buf ptr-) (incf ptr- (send _block_pose :serialization-length))
   ;; float64 _L
     (setq _L (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _W
     (setq _W (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _H
     (setq _H (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[5] _face_offset_u
   (dotimes (i (length _face_offset_u))
     (setf (elt _face_offset_u i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[5] _face_offset_v
   (dotimes (i (length _face_offset_v))
     (setf (elt _face_offset_v i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int32 _center_face
     (setq _center_face (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _center_u_mm
     (setq _center_u_mm (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _center_v_mm
     (setq _center_v_mm (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass block_drawing_msgs::SetBlockPoseResponse
  :super ros::object
  :slots (_success _message ))

(defmethod block_drawing_msgs::SetBlockPoseResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass block_drawing_msgs::SetBlockPose
  :super ros::object
  :slots ())

(setf (get block_drawing_msgs::SetBlockPose :md5sum-) "39ae2210f4fabcad1e458e4361dd4a03")
(setf (get block_drawing_msgs::SetBlockPose :datatype-) "block_drawing_msgs/SetBlockPose")
(setf (get block_drawing_msgs::SetBlockPose :request) block_drawing_msgs::SetBlockPoseRequest)
(setf (get block_drawing_msgs::SetBlockPose :response) block_drawing_msgs::SetBlockPoseResponse)

(defmethod block_drawing_msgs::SetBlockPoseRequest
  (:response () (instance block_drawing_msgs::SetBlockPoseResponse :init)))

(setf (get block_drawing_msgs::SetBlockPoseRequest :md5sum-) "39ae2210f4fabcad1e458e4361dd4a03")
(setf (get block_drawing_msgs::SetBlockPoseRequest :datatype-) "block_drawing_msgs/SetBlockPoseRequest")
(setf (get block_drawing_msgs::SetBlockPoseRequest :definition-)
      "# SetBlockPose.srv
# 设置物块在机械臂基坐标系下的位姿、尺寸、以及各面的图案投影偏移

geometry_msgs/Pose   block_pose    # T_block_base (物块底面中心→基座)
float64 L                          # 物块长 [m]
float64 W                          # 物块宽 [m]
float64 H                          # 物块高 [m]

# 各面图案投影偏移 [mm], 相对于面中心
# face_offset_u[i]: 沿面内 u 轴偏移, face_offset_v[i]: 沿面内 v 轴偏移
# i=0顶面, 1前面, 2右面, 3后面, 4左面
# 默认全0 = 图案中心对齐面中心
float64[5] face_offset_u          # 每面 u 向偏移 [mm] (独立面模式)
float64[5] face_offset_v          # 每面 v 向偏移 [mm] (独立面模式)

# 跨面连续模式: 图案中心在块表面上的锚点
# center_face = -1 表示使用独立面模式 (每个面独立画)
# center_face = 0~4 时使用连续模式, 图案从此面此点出发, 超界自动跨邻面
int32    center_face              # -1=独立面模式, 0~4=连续模式起始面
float64  center_u_mm              # 图案中心 u 坐标 [mm]
float64  center_v_mm              # 图案中心 v 坐标 [mm]

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
---
bool success
string message

")

(setf (get block_drawing_msgs::SetBlockPoseResponse :md5sum-) "39ae2210f4fabcad1e458e4361dd4a03")
(setf (get block_drawing_msgs::SetBlockPoseResponse :datatype-) "block_drawing_msgs/SetBlockPoseResponse")
(setf (get block_drawing_msgs::SetBlockPoseResponse :definition-)
      "# SetBlockPose.srv
# 设置物块在机械臂基坐标系下的位姿、尺寸、以及各面的图案投影偏移

geometry_msgs/Pose   block_pose    # T_block_base (物块底面中心→基座)
float64 L                          # 物块长 [m]
float64 W                          # 物块宽 [m]
float64 H                          # 物块高 [m]

# 各面图案投影偏移 [mm], 相对于面中心
# face_offset_u[i]: 沿面内 u 轴偏移, face_offset_v[i]: 沿面内 v 轴偏移
# i=0顶面, 1前面, 2右面, 3后面, 4左面
# 默认全0 = 图案中心对齐面中心
float64[5] face_offset_u          # 每面 u 向偏移 [mm] (独立面模式)
float64[5] face_offset_v          # 每面 v 向偏移 [mm] (独立面模式)

# 跨面连续模式: 图案中心在块表面上的锚点
# center_face = -1 表示使用独立面模式 (每个面独立画)
# center_face = 0~4 时使用连续模式, 图案从此面此点出发, 超界自动跨邻面
int32    center_face              # -1=独立面模式, 0~4=连续模式起始面
float64  center_u_mm              # 图案中心 u 坐标 [mm]
float64  center_v_mm              # 图案中心 v 坐标 [mm]

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
---
bool success
string message

")



(provide :block_drawing_msgs/SetBlockPose "39ae2210f4fabcad1e458e4361dd4a03")


