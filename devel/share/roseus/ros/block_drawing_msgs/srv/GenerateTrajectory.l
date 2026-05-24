;; Auto-generated. Do not edit!


(when (boundp 'block_drawing_msgs::GenerateTrajectory)
  (if (not (find-package "BLOCK_DRAWING_MSGS"))
    (make-package "BLOCK_DRAWING_MSGS"))
  (shadow 'GenerateTrajectory (find-package "BLOCK_DRAWING_MSGS")))
(unless (find-package "BLOCK_DRAWING_MSGS::GENERATETRAJECTORY")
  (make-package "BLOCK_DRAWING_MSGS::GENERATETRAJECTORY"))
(unless (find-package "BLOCK_DRAWING_MSGS::GENERATETRAJECTORYREQUEST")
  (make-package "BLOCK_DRAWING_MSGS::GENERATETRAJECTORYREQUEST"))
(unless (find-package "BLOCK_DRAWING_MSGS::GENERATETRAJECTORYRESPONSE")
  (make-package "BLOCK_DRAWING_MSGS::GENERATETRAJECTORYRESPONSE"))

(in-package "ROS")





(defclass block_drawing_msgs::GenerateTrajectoryRequest
  :super ros::object
  :slots (_svg_file _target_width_mm _target_height_mm _faces _test_pattern ))

(defmethod block_drawing_msgs::GenerateTrajectoryRequest
  (:init
   (&key
    ((:svg_file __svg_file) "")
    ((:target_width_mm __target_width_mm) 0.0)
    ((:target_height_mm __target_height_mm) 0.0)
    ((:faces __faces) (make-array 0 :initial-element 0 :element-type :integer))
    ((:test_pattern __test_pattern) "")
    )
   (send-super :init)
   (setq _svg_file (string __svg_file))
   (setq _target_width_mm (float __target_width_mm))
   (setq _target_height_mm (float __target_height_mm))
   (setq _faces __faces)
   (setq _test_pattern (string __test_pattern))
   self)
  (:svg_file
   (&optional __svg_file)
   (if __svg_file (setq _svg_file __svg_file)) _svg_file)
  (:target_width_mm
   (&optional __target_width_mm)
   (if __target_width_mm (setq _target_width_mm __target_width_mm)) _target_width_mm)
  (:target_height_mm
   (&optional __target_height_mm)
   (if __target_height_mm (setq _target_height_mm __target_height_mm)) _target_height_mm)
  (:faces
   (&optional __faces)
   (if __faces (setq _faces __faces)) _faces)
  (:test_pattern
   (&optional __test_pattern)
   (if __test_pattern (setq _test_pattern __test_pattern)) _test_pattern)
  (:serialization-length
   ()
   (+
    ;; string _svg_file
    4 (length _svg_file)
    ;; float64 _target_width_mm
    8
    ;; float64 _target_height_mm
    8
    ;; int32[] _faces
    (* 4    (length _faces)) 4
    ;; string _test_pattern
    4 (length _test_pattern)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _svg_file
       (write-long (length _svg_file) s) (princ _svg_file s)
     ;; float64 _target_width_mm
       (sys::poke _target_width_mm (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _target_height_mm
       (sys::poke _target_height_mm (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32[] _faces
     (write-long (length _faces) s)
     (dotimes (i (length _faces))
       (write-long (elt _faces i) s)
       )
     ;; string _test_pattern
       (write-long (length _test_pattern) s) (princ _test_pattern s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _svg_file
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _svg_file (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _target_width_mm
     (setq _target_width_mm (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _target_height_mm
     (setq _target_height_mm (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32[] _faces
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _faces (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _faces i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; string _test_pattern
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _test_pattern (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass block_drawing_msgs::GenerateTrajectoryResponse
  :super ros::object
  :slots (_trajectories _success _message ))

(defmethod block_drawing_msgs::GenerateTrajectoryResponse
  (:init
   (&key
    ((:trajectories __trajectories) ())
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _trajectories __trajectories)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:trajectories
   (&rest __trajectories)
   (if (keywordp (car __trajectories))
       (send* _trajectories __trajectories)
     (progn
       (if __trajectories (setq _trajectories (car __trajectories)))
       _trajectories)))
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; block_drawing_msgs/SurfaceTrajectory[] _trajectories
    (apply #'+ (send-all _trajectories :serialization-length)) 4
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; block_drawing_msgs/SurfaceTrajectory[] _trajectories
     (write-long (length _trajectories) s)
     (dolist (elem _trajectories)
       (send elem :serialize s)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; block_drawing_msgs/SurfaceTrajectory[] _trajectories
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _trajectories (let (r) (dotimes (i n) (push (instance block_drawing_msgs::SurfaceTrajectory :init) r)) r))
     (dolist (elem- _trajectories)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass block_drawing_msgs::GenerateTrajectory
  :super ros::object
  :slots ())

(setf (get block_drawing_msgs::GenerateTrajectory :md5sum-) "8fbb008b13b5fc9766400a28fbedf619")
(setf (get block_drawing_msgs::GenerateTrajectory :datatype-) "block_drawing_msgs/GenerateTrajectory")
(setf (get block_drawing_msgs::GenerateTrajectory :request) block_drawing_msgs::GenerateTrajectoryRequest)
(setf (get block_drawing_msgs::GenerateTrajectory :response) block_drawing_msgs::GenerateTrajectoryResponse)

(defmethod block_drawing_msgs::GenerateTrajectoryRequest
  (:response () (instance block_drawing_msgs::GenerateTrajectoryResponse :init)))

(setf (get block_drawing_msgs::GenerateTrajectoryRequest :md5sum-) "8fbb008b13b5fc9766400a28fbedf619")
(setf (get block_drawing_msgs::GenerateTrajectoryRequest :datatype-) "block_drawing_msgs/GenerateTrajectoryRequest")
(setf (get block_drawing_msgs::GenerateTrajectoryRequest :definition-)
      "# GenerateTrajectory.srv
# 模块A服务: SVG/图案 → 轨迹序列

string   svg_file                          # SVG文件路径 (空=使用内置测试图案)
float64  target_width_mm                   # 目标图案宽度 [mm]
float64  target_height_mm                  # 目标图案高度 [mm]
int32[]  faces                             # 要画的面的列表 (0~4)
string   test_pattern                      # 内置测试图案类型: \"square\" / \"circle\" / \"star\" (svg_file为空时生效)
---
block_drawing_msgs/SurfaceTrajectory[] trajectories  # 每个面对应一条轨迹
bool     success
string   message


================================================================================
MSG: block_drawing_msgs/SurfaceTrajectory
# SurfaceTrajectory.msg
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

(setf (get block_drawing_msgs::GenerateTrajectoryResponse :md5sum-) "8fbb008b13b5fc9766400a28fbedf619")
(setf (get block_drawing_msgs::GenerateTrajectoryResponse :datatype-) "block_drawing_msgs/GenerateTrajectoryResponse")
(setf (get block_drawing_msgs::GenerateTrajectoryResponse :definition-)
      "# GenerateTrajectory.srv
# 模块A服务: SVG/图案 → 轨迹序列

string   svg_file                          # SVG文件路径 (空=使用内置测试图案)
float64  target_width_mm                   # 目标图案宽度 [mm]
float64  target_height_mm                  # 目标图案高度 [mm]
int32[]  faces                             # 要画的面的列表 (0~4)
string   test_pattern                      # 内置测试图案类型: \"square\" / \"circle\" / \"star\" (svg_file为空时生效)
---
block_drawing_msgs/SurfaceTrajectory[] trajectories  # 每个面对应一条轨迹
bool     success
string   message


================================================================================
MSG: block_drawing_msgs/SurfaceTrajectory
# SurfaceTrajectory.msg
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



(provide :block_drawing_msgs/GenerateTrajectory "8fbb008b13b5fc9766400a28fbedf619")


