; Auto-generated. Do not edit!


(cl:in-package block_drawing_msgs-msg)


;//! \htmlinclude DrawingExecutionResult.msg.html

(cl:defclass <DrawingExecutionResult> (roslisp-msg-protocol:ros-message)
  ((completed
    :reader completed
    :initarg :completed
    :type cl:boolean
    :initform cl:nil)
   (faces_drawn
    :reader faces_drawn
    :initarg :faces_drawn
    :type cl:integer
    :initform 0)
   (total_arc_length
    :reader total_arc_length
    :initarg :total_arc_length
    :type cl:float
    :initform 0.0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass DrawingExecutionResult (<DrawingExecutionResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DrawingExecutionResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DrawingExecutionResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name block_drawing_msgs-msg:<DrawingExecutionResult> is deprecated: use block_drawing_msgs-msg:DrawingExecutionResult instead.")))

(cl:ensure-generic-function 'completed-val :lambda-list '(m))
(cl:defmethod completed-val ((m <DrawingExecutionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:completed-val is deprecated.  Use block_drawing_msgs-msg:completed instead.")
  (completed m))

(cl:ensure-generic-function 'faces_drawn-val :lambda-list '(m))
(cl:defmethod faces_drawn-val ((m <DrawingExecutionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:faces_drawn-val is deprecated.  Use block_drawing_msgs-msg:faces_drawn instead.")
  (faces_drawn m))

(cl:ensure-generic-function 'total_arc_length-val :lambda-list '(m))
(cl:defmethod total_arc_length-val ((m <DrawingExecutionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:total_arc_length-val is deprecated.  Use block_drawing_msgs-msg:total_arc_length instead.")
  (total_arc_length m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <DrawingExecutionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:message-val is deprecated.  Use block_drawing_msgs-msg:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DrawingExecutionResult>) ostream)
  "Serializes a message object of type '<DrawingExecutionResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'completed) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'faces_drawn)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'total_arc_length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DrawingExecutionResult>) istream)
  "Deserializes a message object of type '<DrawingExecutionResult>"
    (cl:setf (cl:slot-value msg 'completed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'faces_drawn) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'total_arc_length) (roslisp-utils:decode-double-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DrawingExecutionResult>)))
  "Returns string type for a message object of type '<DrawingExecutionResult>"
  "block_drawing_msgs/DrawingExecutionResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DrawingExecutionResult)))
  "Returns string type for a message object of type 'DrawingExecutionResult"
  "block_drawing_msgs/DrawingExecutionResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DrawingExecutionResult>)))
  "Returns md5sum for a message object of type '<DrawingExecutionResult>"
  "97fbbf989dd52a803d908804451ece8a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DrawingExecutionResult)))
  "Returns md5sum for a message object of type 'DrawingExecutionResult"
  "97fbbf989dd52a803d908804451ece8a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DrawingExecutionResult>)))
  "Returns full string definition for message of type '<DrawingExecutionResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Result~%bool    completed           # 是否全部完成~%int32   faces_drawn         # 已完成的面数~%float64 total_arc_length    # 总弧长 [m]~%string  message             # 错误信息 (如有)~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DrawingExecutionResult)))
  "Returns full string definition for message of type 'DrawingExecutionResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Result~%bool    completed           # 是否全部完成~%int32   faces_drawn         # 已完成的面数~%float64 total_arc_length    # 总弧长 [m]~%string  message             # 错误信息 (如有)~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DrawingExecutionResult>))
  (cl:+ 0
     1
     4
     8
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DrawingExecutionResult>))
  "Converts a ROS message object to a list"
  (cl:list 'DrawingExecutionResult
    (cl:cons ':completed (completed msg))
    (cl:cons ':faces_drawn (faces_drawn msg))
    (cl:cons ':total_arc_length (total_arc_length msg))
    (cl:cons ':message (message msg))
))
