; Auto-generated. Do not edit!


(cl:in-package block_drawing_msgs-srv)


;//! \htmlinclude SetBlockPose-request.msg.html

(cl:defclass <SetBlockPose-request> (roslisp-msg-protocol:ros-message)
  ((block_pose
    :reader block_pose
    :initarg :block_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (L
    :reader L
    :initarg :L
    :type cl:float
    :initform 0.0)
   (W
    :reader W
    :initarg :W
    :type cl:float
    :initform 0.0)
   (H
    :reader H
    :initarg :H
    :type cl:float
    :initform 0.0)
   (face_offset_u
    :reader face_offset_u
    :initarg :face_offset_u
    :type (cl:vector cl:float)
   :initform (cl:make-array 5 :element-type 'cl:float :initial-element 0.0))
   (face_offset_v
    :reader face_offset_v
    :initarg :face_offset_v
    :type (cl:vector cl:float)
   :initform (cl:make-array 5 :element-type 'cl:float :initial-element 0.0))
   (center_face
    :reader center_face
    :initarg :center_face
    :type cl:integer
    :initform 0)
   (center_u_mm
    :reader center_u_mm
    :initarg :center_u_mm
    :type cl:float
    :initform 0.0)
   (center_v_mm
    :reader center_v_mm
    :initarg :center_v_mm
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetBlockPose-request (<SetBlockPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetBlockPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetBlockPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name block_drawing_msgs-srv:<SetBlockPose-request> is deprecated: use block_drawing_msgs-srv:SetBlockPose-request instead.")))

(cl:ensure-generic-function 'block_pose-val :lambda-list '(m))
(cl:defmethod block_pose-val ((m <SetBlockPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:block_pose-val is deprecated.  Use block_drawing_msgs-srv:block_pose instead.")
  (block_pose m))

(cl:ensure-generic-function 'L-val :lambda-list '(m))
(cl:defmethod L-val ((m <SetBlockPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:L-val is deprecated.  Use block_drawing_msgs-srv:L instead.")
  (L m))

(cl:ensure-generic-function 'W-val :lambda-list '(m))
(cl:defmethod W-val ((m <SetBlockPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:W-val is deprecated.  Use block_drawing_msgs-srv:W instead.")
  (W m))

(cl:ensure-generic-function 'H-val :lambda-list '(m))
(cl:defmethod H-val ((m <SetBlockPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:H-val is deprecated.  Use block_drawing_msgs-srv:H instead.")
  (H m))

(cl:ensure-generic-function 'face_offset_u-val :lambda-list '(m))
(cl:defmethod face_offset_u-val ((m <SetBlockPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:face_offset_u-val is deprecated.  Use block_drawing_msgs-srv:face_offset_u instead.")
  (face_offset_u m))

(cl:ensure-generic-function 'face_offset_v-val :lambda-list '(m))
(cl:defmethod face_offset_v-val ((m <SetBlockPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:face_offset_v-val is deprecated.  Use block_drawing_msgs-srv:face_offset_v instead.")
  (face_offset_v m))

(cl:ensure-generic-function 'center_face-val :lambda-list '(m))
(cl:defmethod center_face-val ((m <SetBlockPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:center_face-val is deprecated.  Use block_drawing_msgs-srv:center_face instead.")
  (center_face m))

(cl:ensure-generic-function 'center_u_mm-val :lambda-list '(m))
(cl:defmethod center_u_mm-val ((m <SetBlockPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:center_u_mm-val is deprecated.  Use block_drawing_msgs-srv:center_u_mm instead.")
  (center_u_mm m))

(cl:ensure-generic-function 'center_v_mm-val :lambda-list '(m))
(cl:defmethod center_v_mm-val ((m <SetBlockPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:center_v_mm-val is deprecated.  Use block_drawing_msgs-srv:center_v_mm instead.")
  (center_v_mm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetBlockPose-request>) ostream)
  "Serializes a message object of type '<SetBlockPose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'block_pose) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'L))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'W))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'H))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'face_offset_u))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'face_offset_v))
  (cl:let* ((signed (cl:slot-value msg 'center_face)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'center_u_mm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'center_v_mm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetBlockPose-request>) istream)
  "Deserializes a message object of type '<SetBlockPose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'block_pose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'L) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'W) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'H) (roslisp-utils:decode-double-float-bits bits)))
  (cl:setf (cl:slot-value msg 'face_offset_u) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'face_offset_u)))
    (cl:dotimes (i 5)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'face_offset_v) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'face_offset_v)))
    (cl:dotimes (i 5)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'center_face) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'center_u_mm) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'center_v_mm) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetBlockPose-request>)))
  "Returns string type for a service object of type '<SetBlockPose-request>"
  "block_drawing_msgs/SetBlockPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBlockPose-request)))
  "Returns string type for a service object of type 'SetBlockPose-request"
  "block_drawing_msgs/SetBlockPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetBlockPose-request>)))
  "Returns md5sum for a message object of type '<SetBlockPose-request>"
  "39ae2210f4fabcad1e458e4361dd4a03")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetBlockPose-request)))
  "Returns md5sum for a message object of type 'SetBlockPose-request"
  "39ae2210f4fabcad1e458e4361dd4a03")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetBlockPose-request>)))
  "Returns full string definition for message of type '<SetBlockPose-request>"
  (cl:format cl:nil "# SetBlockPose.srv~%# 设置物块在机械臂基坐标系下的位姿、尺寸、以及各面的图案投影偏移~%~%geometry_msgs/Pose   block_pose    # T_block_base (物块底面中心→基座)~%float64 L                          # 物块长 [m]~%float64 W                          # 物块宽 [m]~%float64 H                          # 物块高 [m]~%~%# 各面图案投影偏移 [mm], 相对于面中心~%# face_offset_u[i]: 沿面内 u 轴偏移, face_offset_v[i]: 沿面内 v 轴偏移~%# i=0顶面, 1前面, 2右面, 3后面, 4左面~%# 默认全0 = 图案中心对齐面中心~%float64[5] face_offset_u          # 每面 u 向偏移 [mm] (独立面模式)~%float64[5] face_offset_v          # 每面 v 向偏移 [mm] (独立面模式)~%~%# 跨面连续模式: 图案中心在块表面上的锚点~%# center_face = -1 表示使用独立面模式 (每个面独立画)~%# center_face = 0~~4 时使用连续模式, 图案从此面此点出发, 超界自动跨邻面~%int32    center_face              # -1=独立面模式, 0~~4=连续模式起始面~%float64  center_u_mm              # 图案中心 u 坐标 [mm]~%float64  center_v_mm              # 图案中心 v 坐标 [mm]~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetBlockPose-request)))
  "Returns full string definition for message of type 'SetBlockPose-request"
  (cl:format cl:nil "# SetBlockPose.srv~%# 设置物块在机械臂基坐标系下的位姿、尺寸、以及各面的图案投影偏移~%~%geometry_msgs/Pose   block_pose    # T_block_base (物块底面中心→基座)~%float64 L                          # 物块长 [m]~%float64 W                          # 物块宽 [m]~%float64 H                          # 物块高 [m]~%~%# 各面图案投影偏移 [mm], 相对于面中心~%# face_offset_u[i]: 沿面内 u 轴偏移, face_offset_v[i]: 沿面内 v 轴偏移~%# i=0顶面, 1前面, 2右面, 3后面, 4左面~%# 默认全0 = 图案中心对齐面中心~%float64[5] face_offset_u          # 每面 u 向偏移 [mm] (独立面模式)~%float64[5] face_offset_v          # 每面 v 向偏移 [mm] (独立面模式)~%~%# 跨面连续模式: 图案中心在块表面上的锚点~%# center_face = -1 表示使用独立面模式 (每个面独立画)~%# center_face = 0~~4 时使用连续模式, 图案从此面此点出发, 超界自动跨邻面~%int32    center_face              # -1=独立面模式, 0~~4=连续模式起始面~%float64  center_u_mm              # 图案中心 u 坐标 [mm]~%float64  center_v_mm              # 图案中心 v 坐标 [mm]~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetBlockPose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'block_pose))
     8
     8
     8
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'face_offset_u) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'face_offset_v) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetBlockPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetBlockPose-request
    (cl:cons ':block_pose (block_pose msg))
    (cl:cons ':L (L msg))
    (cl:cons ':W (W msg))
    (cl:cons ':H (H msg))
    (cl:cons ':face_offset_u (face_offset_u msg))
    (cl:cons ':face_offset_v (face_offset_v msg))
    (cl:cons ':center_face (center_face msg))
    (cl:cons ':center_u_mm (center_u_mm msg))
    (cl:cons ':center_v_mm (center_v_mm msg))
))
;//! \htmlinclude SetBlockPose-response.msg.html

(cl:defclass <SetBlockPose-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass SetBlockPose-response (<SetBlockPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetBlockPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetBlockPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name block_drawing_msgs-srv:<SetBlockPose-response> is deprecated: use block_drawing_msgs-srv:SetBlockPose-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetBlockPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:success-val is deprecated.  Use block_drawing_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetBlockPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:message-val is deprecated.  Use block_drawing_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetBlockPose-response>) ostream)
  "Serializes a message object of type '<SetBlockPose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetBlockPose-response>) istream)
  "Deserializes a message object of type '<SetBlockPose-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetBlockPose-response>)))
  "Returns string type for a service object of type '<SetBlockPose-response>"
  "block_drawing_msgs/SetBlockPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBlockPose-response)))
  "Returns string type for a service object of type 'SetBlockPose-response"
  "block_drawing_msgs/SetBlockPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetBlockPose-response>)))
  "Returns md5sum for a message object of type '<SetBlockPose-response>"
  "39ae2210f4fabcad1e458e4361dd4a03")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetBlockPose-response)))
  "Returns md5sum for a message object of type 'SetBlockPose-response"
  "39ae2210f4fabcad1e458e4361dd4a03")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetBlockPose-response>)))
  "Returns full string definition for message of type '<SetBlockPose-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetBlockPose-response)))
  "Returns full string definition for message of type 'SetBlockPose-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetBlockPose-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetBlockPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetBlockPose-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetBlockPose)))
  'SetBlockPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetBlockPose)))
  'SetBlockPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBlockPose)))
  "Returns string type for a service object of type '<SetBlockPose>"
  "block_drawing_msgs/SetBlockPose")