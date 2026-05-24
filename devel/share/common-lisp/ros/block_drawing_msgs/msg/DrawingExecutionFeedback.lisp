; Auto-generated. Do not edit!


(cl:in-package block_drawing_msgs-msg)


;//! \htmlinclude DrawingExecutionFeedback.msg.html

(cl:defclass <DrawingExecutionFeedback> (roslisp-msg-protocol:ros-message)
  ((current_face
    :reader current_face
    :initarg :current_face
    :type cl:integer
    :initform 0)
   (current_waypoint
    :reader current_waypoint
    :initarg :current_waypoint
    :type cl:integer
    :initform 0)
   (progress_fraction
    :reader progress_fraction
    :initarg :progress_fraction
    :type cl:float
    :initform 0.0)
   (estimated_force
    :reader estimated_force
    :initarg :estimated_force
    :type cl:float
    :initform 0.0)
   (state
    :reader state
    :initarg :state
    :type cl:string
    :initform ""))
)

(cl:defclass DrawingExecutionFeedback (<DrawingExecutionFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DrawingExecutionFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DrawingExecutionFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name block_drawing_msgs-msg:<DrawingExecutionFeedback> is deprecated: use block_drawing_msgs-msg:DrawingExecutionFeedback instead.")))

(cl:ensure-generic-function 'current_face-val :lambda-list '(m))
(cl:defmethod current_face-val ((m <DrawingExecutionFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:current_face-val is deprecated.  Use block_drawing_msgs-msg:current_face instead.")
  (current_face m))

(cl:ensure-generic-function 'current_waypoint-val :lambda-list '(m))
(cl:defmethod current_waypoint-val ((m <DrawingExecutionFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:current_waypoint-val is deprecated.  Use block_drawing_msgs-msg:current_waypoint instead.")
  (current_waypoint m))

(cl:ensure-generic-function 'progress_fraction-val :lambda-list '(m))
(cl:defmethod progress_fraction-val ((m <DrawingExecutionFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:progress_fraction-val is deprecated.  Use block_drawing_msgs-msg:progress_fraction instead.")
  (progress_fraction m))

(cl:ensure-generic-function 'estimated_force-val :lambda-list '(m))
(cl:defmethod estimated_force-val ((m <DrawingExecutionFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:estimated_force-val is deprecated.  Use block_drawing_msgs-msg:estimated_force instead.")
  (estimated_force m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <DrawingExecutionFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:state-val is deprecated.  Use block_drawing_msgs-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DrawingExecutionFeedback>) ostream)
  "Serializes a message object of type '<DrawingExecutionFeedback>"
  (cl:let* ((signed (cl:slot-value msg 'current_face)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'current_waypoint)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'progress_fraction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'estimated_force))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'state))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DrawingExecutionFeedback>) istream)
  "Deserializes a message object of type '<DrawingExecutionFeedback>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_face) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_waypoint) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'progress_fraction) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'estimated_force) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DrawingExecutionFeedback>)))
  "Returns string type for a message object of type '<DrawingExecutionFeedback>"
  "block_drawing_msgs/DrawingExecutionFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DrawingExecutionFeedback)))
  "Returns string type for a message object of type 'DrawingExecutionFeedback"
  "block_drawing_msgs/DrawingExecutionFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DrawingExecutionFeedback>)))
  "Returns md5sum for a message object of type '<DrawingExecutionFeedback>"
  "17f64e75c596ccac5812e834eece6216")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DrawingExecutionFeedback)))
  "Returns md5sum for a message object of type 'DrawingExecutionFeedback"
  "17f64e75c596ccac5812e834eece6216")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DrawingExecutionFeedback>)))
  "Returns full string definition for message of type '<DrawingExecutionFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Feedback~%int32   current_face        # 当前正在画的面 (0~~4)~%int32   current_waypoint    # 当前 waypoint 索引~%float64 progress_fraction   # 完成比例 [0.0, 1.0]~%float64 estimated_force     # 当前估计接触力 [N]~%string  state               # 执行器状态字符串~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DrawingExecutionFeedback)))
  "Returns full string definition for message of type 'DrawingExecutionFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Feedback~%int32   current_face        # 当前正在画的面 (0~~4)~%int32   current_waypoint    # 当前 waypoint 索引~%float64 progress_fraction   # 完成比例 [0.0, 1.0]~%float64 estimated_force     # 当前估计接触力 [N]~%string  state               # 执行器状态字符串~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DrawingExecutionFeedback>))
  (cl:+ 0
     4
     4
     8
     8
     4 (cl:length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DrawingExecutionFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'DrawingExecutionFeedback
    (cl:cons ':current_face (current_face msg))
    (cl:cons ':current_waypoint (current_waypoint msg))
    (cl:cons ':progress_fraction (progress_fraction msg))
    (cl:cons ':estimated_force (estimated_force msg))
    (cl:cons ':state (state msg))
))
