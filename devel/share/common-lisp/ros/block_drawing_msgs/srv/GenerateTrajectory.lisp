; Auto-generated. Do not edit!


(cl:in-package block_drawing_msgs-srv)


;//! \htmlinclude GenerateTrajectory-request.msg.html

(cl:defclass <GenerateTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((svg_file
    :reader svg_file
    :initarg :svg_file
    :type cl:string
    :initform "")
   (target_width_mm
    :reader target_width_mm
    :initarg :target_width_mm
    :type cl:float
    :initform 0.0)
   (target_height_mm
    :reader target_height_mm
    :initarg :target_height_mm
    :type cl:float
    :initform 0.0)
   (faces
    :reader faces
    :initarg :faces
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (test_pattern
    :reader test_pattern
    :initarg :test_pattern
    :type cl:string
    :initform ""))
)

(cl:defclass GenerateTrajectory-request (<GenerateTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GenerateTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GenerateTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name block_drawing_msgs-srv:<GenerateTrajectory-request> is deprecated: use block_drawing_msgs-srv:GenerateTrajectory-request instead.")))

(cl:ensure-generic-function 'svg_file-val :lambda-list '(m))
(cl:defmethod svg_file-val ((m <GenerateTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:svg_file-val is deprecated.  Use block_drawing_msgs-srv:svg_file instead.")
  (svg_file m))

(cl:ensure-generic-function 'target_width_mm-val :lambda-list '(m))
(cl:defmethod target_width_mm-val ((m <GenerateTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:target_width_mm-val is deprecated.  Use block_drawing_msgs-srv:target_width_mm instead.")
  (target_width_mm m))

(cl:ensure-generic-function 'target_height_mm-val :lambda-list '(m))
(cl:defmethod target_height_mm-val ((m <GenerateTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:target_height_mm-val is deprecated.  Use block_drawing_msgs-srv:target_height_mm instead.")
  (target_height_mm m))

(cl:ensure-generic-function 'faces-val :lambda-list '(m))
(cl:defmethod faces-val ((m <GenerateTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:faces-val is deprecated.  Use block_drawing_msgs-srv:faces instead.")
  (faces m))

(cl:ensure-generic-function 'test_pattern-val :lambda-list '(m))
(cl:defmethod test_pattern-val ((m <GenerateTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:test_pattern-val is deprecated.  Use block_drawing_msgs-srv:test_pattern instead.")
  (test_pattern m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GenerateTrajectory-request>) ostream)
  "Serializes a message object of type '<GenerateTrajectory-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'svg_file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'svg_file))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'target_width_mm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'target_height_mm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'faces))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'faces))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'test_pattern))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'test_pattern))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GenerateTrajectory-request>) istream)
  "Deserializes a message object of type '<GenerateTrajectory-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'svg_file) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'svg_file) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_width_mm) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_height_mm) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'faces) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'faces)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'test_pattern) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'test_pattern) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GenerateTrajectory-request>)))
  "Returns string type for a service object of type '<GenerateTrajectory-request>"
  "block_drawing_msgs/GenerateTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GenerateTrajectory-request)))
  "Returns string type for a service object of type 'GenerateTrajectory-request"
  "block_drawing_msgs/GenerateTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GenerateTrajectory-request>)))
  "Returns md5sum for a message object of type '<GenerateTrajectory-request>"
  "8fbb008b13b5fc9766400a28fbedf619")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GenerateTrajectory-request)))
  "Returns md5sum for a message object of type 'GenerateTrajectory-request"
  "8fbb008b13b5fc9766400a28fbedf619")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GenerateTrajectory-request>)))
  "Returns full string definition for message of type '<GenerateTrajectory-request>"
  (cl:format cl:nil "# GenerateTrajectory.srv~%# 模块A服务: SVG/图案 → 轨迹序列~%~%string   svg_file                          # SVG文件路径 (空=使用内置测试图案)~%float64  target_width_mm                   # 目标图案宽度 [mm]~%float64  target_height_mm                  # 目标图案高度 [mm]~%int32[]  faces                             # 要画的面的列表 (0~~4)~%string   test_pattern                      # 内置测试图案类型: \"square\" / \"circle\" / \"star\" (svg_file为空时生效)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GenerateTrajectory-request)))
  "Returns full string definition for message of type 'GenerateTrajectory-request"
  (cl:format cl:nil "# GenerateTrajectory.srv~%# 模块A服务: SVG/图案 → 轨迹序列~%~%string   svg_file                          # SVG文件路径 (空=使用内置测试图案)~%float64  target_width_mm                   # 目标图案宽度 [mm]~%float64  target_height_mm                  # 目标图案高度 [mm]~%int32[]  faces                             # 要画的面的列表 (0~~4)~%string   test_pattern                      # 内置测试图案类型: \"square\" / \"circle\" / \"star\" (svg_file为空时生效)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GenerateTrajectory-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'svg_file))
     8
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'faces) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:length (cl:slot-value msg 'test_pattern))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GenerateTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GenerateTrajectory-request
    (cl:cons ':svg_file (svg_file msg))
    (cl:cons ':target_width_mm (target_width_mm msg))
    (cl:cons ':target_height_mm (target_height_mm msg))
    (cl:cons ':faces (faces msg))
    (cl:cons ':test_pattern (test_pattern msg))
))
;//! \htmlinclude GenerateTrajectory-response.msg.html

(cl:defclass <GenerateTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((trajectories
    :reader trajectories
    :initarg :trajectories
    :type (cl:vector block_drawing_msgs-msg:SurfaceTrajectory)
   :initform (cl:make-array 0 :element-type 'block_drawing_msgs-msg:SurfaceTrajectory :initial-element (cl:make-instance 'block_drawing_msgs-msg:SurfaceTrajectory)))
   (success
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

(cl:defclass GenerateTrajectory-response (<GenerateTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GenerateTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GenerateTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name block_drawing_msgs-srv:<GenerateTrajectory-response> is deprecated: use block_drawing_msgs-srv:GenerateTrajectory-response instead.")))

(cl:ensure-generic-function 'trajectories-val :lambda-list '(m))
(cl:defmethod trajectories-val ((m <GenerateTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:trajectories-val is deprecated.  Use block_drawing_msgs-srv:trajectories instead.")
  (trajectories m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GenerateTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:success-val is deprecated.  Use block_drawing_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <GenerateTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:message-val is deprecated.  Use block_drawing_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GenerateTrajectory-response>) ostream)
  "Serializes a message object of type '<GenerateTrajectory-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trajectories))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'trajectories))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GenerateTrajectory-response>) istream)
  "Deserializes a message object of type '<GenerateTrajectory-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trajectories) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trajectories)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'block_drawing_msgs-msg:SurfaceTrajectory))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GenerateTrajectory-response>)))
  "Returns string type for a service object of type '<GenerateTrajectory-response>"
  "block_drawing_msgs/GenerateTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GenerateTrajectory-response)))
  "Returns string type for a service object of type 'GenerateTrajectory-response"
  "block_drawing_msgs/GenerateTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GenerateTrajectory-response>)))
  "Returns md5sum for a message object of type '<GenerateTrajectory-response>"
  "8fbb008b13b5fc9766400a28fbedf619")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GenerateTrajectory-response)))
  "Returns md5sum for a message object of type 'GenerateTrajectory-response"
  "8fbb008b13b5fc9766400a28fbedf619")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GenerateTrajectory-response>)))
  "Returns full string definition for message of type '<GenerateTrajectory-response>"
  (cl:format cl:nil "block_drawing_msgs/SurfaceTrajectory[] trajectories  # 每个面对应一条轨迹~%bool     success~%string   message~%~%~%================================================================================~%MSG: block_drawing_msgs/SurfaceTrajectory~%# SurfaceTrajectory.msg~%# 单面轨迹: 包含该面所有 waypoints~%~%int32   face_id                    # 0=顶面, 1=前面, 2=右面, 3=后面, 4=左面~%geometry_msgs/Pose[] waypoints     # 末端期望位姿 (含法兰补偿)~%float64[] arc_lengths              # 累计弧长, 用于速度规划 [m]~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GenerateTrajectory-response)))
  "Returns full string definition for message of type 'GenerateTrajectory-response"
  (cl:format cl:nil "block_drawing_msgs/SurfaceTrajectory[] trajectories  # 每个面对应一条轨迹~%bool     success~%string   message~%~%~%================================================================================~%MSG: block_drawing_msgs/SurfaceTrajectory~%# SurfaceTrajectory.msg~%# 单面轨迹: 包含该面所有 waypoints~%~%int32   face_id                    # 0=顶面, 1=前面, 2=右面, 3=后面, 4=左面~%geometry_msgs/Pose[] waypoints     # 末端期望位姿 (含法兰补偿)~%float64[] arc_lengths              # 累计弧长, 用于速度规划 [m]~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GenerateTrajectory-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trajectories) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GenerateTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GenerateTrajectory-response
    (cl:cons ':trajectories (trajectories msg))
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GenerateTrajectory)))
  'GenerateTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GenerateTrajectory)))
  'GenerateTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GenerateTrajectory)))
  "Returns string type for a service object of type '<GenerateTrajectory>"
  "block_drawing_msgs/GenerateTrajectory")