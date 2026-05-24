; Auto-generated. Do not edit!


(cl:in-package block_drawing_msgs-msg)


;//! \htmlinclude SurfaceTrajectory.msg.html

(cl:defclass <SurfaceTrajectory> (roslisp-msg-protocol:ros-message)
  ((face_id
    :reader face_id
    :initarg :face_id
    :type cl:integer
    :initform 0)
   (waypoints
    :reader waypoints
    :initarg :waypoints
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose)))
   (arc_lengths
    :reader arc_lengths
    :initarg :arc_lengths
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SurfaceTrajectory (<SurfaceTrajectory>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SurfaceTrajectory>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SurfaceTrajectory)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name block_drawing_msgs-msg:<SurfaceTrajectory> is deprecated: use block_drawing_msgs-msg:SurfaceTrajectory instead.")))

(cl:ensure-generic-function 'face_id-val :lambda-list '(m))
(cl:defmethod face_id-val ((m <SurfaceTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:face_id-val is deprecated.  Use block_drawing_msgs-msg:face_id instead.")
  (face_id m))

(cl:ensure-generic-function 'waypoints-val :lambda-list '(m))
(cl:defmethod waypoints-val ((m <SurfaceTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:waypoints-val is deprecated.  Use block_drawing_msgs-msg:waypoints instead.")
  (waypoints m))

(cl:ensure-generic-function 'arc_lengths-val :lambda-list '(m))
(cl:defmethod arc_lengths-val ((m <SurfaceTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:arc_lengths-val is deprecated.  Use block_drawing_msgs-msg:arc_lengths instead.")
  (arc_lengths m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SurfaceTrajectory>) ostream)
  "Serializes a message object of type '<SurfaceTrajectory>"
  (cl:let* ((signed (cl:slot-value msg 'face_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'waypoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'waypoints))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'arc_lengths))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'arc_lengths))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SurfaceTrajectory>) istream)
  "Deserializes a message object of type '<SurfaceTrajectory>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'face_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'waypoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'waypoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'arc_lengths) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'arc_lengths)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SurfaceTrajectory>)))
  "Returns string type for a message object of type '<SurfaceTrajectory>"
  "block_drawing_msgs/SurfaceTrajectory")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SurfaceTrajectory)))
  "Returns string type for a message object of type 'SurfaceTrajectory"
  "block_drawing_msgs/SurfaceTrajectory")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SurfaceTrajectory>)))
  "Returns md5sum for a message object of type '<SurfaceTrajectory>"
  "5b572666885f29b8eb6f14e36c9ed7c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SurfaceTrajectory)))
  "Returns md5sum for a message object of type 'SurfaceTrajectory"
  "5b572666885f29b8eb6f14e36c9ed7c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SurfaceTrajectory>)))
  "Returns full string definition for message of type '<SurfaceTrajectory>"
  (cl:format cl:nil "# SurfaceTrajectory.msg~%# 单面轨迹: 包含该面所有 waypoints~%~%int32   face_id                    # 0=顶面, 1=前面, 2=右面, 3=后面, 4=左面~%geometry_msgs/Pose[] waypoints     # 末端期望位姿 (含法兰补偿)~%float64[] arc_lengths              # 累计弧长, 用于速度规划 [m]~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SurfaceTrajectory)))
  "Returns full string definition for message of type 'SurfaceTrajectory"
  (cl:format cl:nil "# SurfaceTrajectory.msg~%# 单面轨迹: 包含该面所有 waypoints~%~%int32   face_id                    # 0=顶面, 1=前面, 2=右面, 3=后面, 4=左面~%geometry_msgs/Pose[] waypoints     # 末端期望位姿 (含法兰补偿)~%float64[] arc_lengths              # 累计弧长, 用于速度规划 [m]~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SurfaceTrajectory>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'arc_lengths) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SurfaceTrajectory>))
  "Converts a ROS message object to a list"
  (cl:list 'SurfaceTrajectory
    (cl:cons ':face_id (face_id msg))
    (cl:cons ':waypoints (waypoints msg))
    (cl:cons ':arc_lengths (arc_lengths msg))
))
