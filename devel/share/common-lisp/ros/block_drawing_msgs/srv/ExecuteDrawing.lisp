; Auto-generated. Do not edit!


(cl:in-package block_drawing_msgs-srv)


;//! \htmlinclude ExecuteDrawing-request.msg.html

(cl:defclass <ExecuteDrawing-request> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ExecuteDrawing-request (<ExecuteDrawing-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteDrawing-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteDrawing-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name block_drawing_msgs-srv:<ExecuteDrawing-request> is deprecated: use block_drawing_msgs-srv:ExecuteDrawing-request instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <ExecuteDrawing-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:start-val is deprecated.  Use block_drawing_msgs-srv:start instead.")
  (start m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteDrawing-request>) ostream)
  "Serializes a message object of type '<ExecuteDrawing-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'start) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteDrawing-request>) istream)
  "Deserializes a message object of type '<ExecuteDrawing-request>"
    (cl:setf (cl:slot-value msg 'start) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteDrawing-request>)))
  "Returns string type for a service object of type '<ExecuteDrawing-request>"
  "block_drawing_msgs/ExecuteDrawingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteDrawing-request)))
  "Returns string type for a service object of type 'ExecuteDrawing-request"
  "block_drawing_msgs/ExecuteDrawingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteDrawing-request>)))
  "Returns md5sum for a message object of type '<ExecuteDrawing-request>"
  "570b7d04f9d3b17893f17c4fdcf5ca06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteDrawing-request)))
  "Returns md5sum for a message object of type 'ExecuteDrawing-request"
  "570b7d04f9d3b17893f17c4fdcf5ca06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteDrawing-request>)))
  "Returns full string definition for message of type '<ExecuteDrawing-request>"
  (cl:format cl:nil "# ExecuteDrawing.srv~%# 模块B服务: 启动绘画执行 (使用 Action 模式时为简单触发)~%~%bool start    # true=启动, false=取消~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteDrawing-request)))
  "Returns full string definition for message of type 'ExecuteDrawing-request"
  (cl:format cl:nil "# ExecuteDrawing.srv~%# 模块B服务: 启动绘画执行 (使用 Action 模式时为简单触发)~%~%bool start    # true=启动, false=取消~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteDrawing-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteDrawing-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteDrawing-request
    (cl:cons ':start (start msg))
))
;//! \htmlinclude ExecuteDrawing-response.msg.html

(cl:defclass <ExecuteDrawing-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass ExecuteDrawing-response (<ExecuteDrawing-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteDrawing-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteDrawing-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name block_drawing_msgs-srv:<ExecuteDrawing-response> is deprecated: use block_drawing_msgs-srv:ExecuteDrawing-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ExecuteDrawing-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:success-val is deprecated.  Use block_drawing_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ExecuteDrawing-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-srv:message-val is deprecated.  Use block_drawing_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteDrawing-response>) ostream)
  "Serializes a message object of type '<ExecuteDrawing-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteDrawing-response>) istream)
  "Deserializes a message object of type '<ExecuteDrawing-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteDrawing-response>)))
  "Returns string type for a service object of type '<ExecuteDrawing-response>"
  "block_drawing_msgs/ExecuteDrawingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteDrawing-response)))
  "Returns string type for a service object of type 'ExecuteDrawing-response"
  "block_drawing_msgs/ExecuteDrawingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteDrawing-response>)))
  "Returns md5sum for a message object of type '<ExecuteDrawing-response>"
  "570b7d04f9d3b17893f17c4fdcf5ca06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteDrawing-response)))
  "Returns md5sum for a message object of type 'ExecuteDrawing-response"
  "570b7d04f9d3b17893f17c4fdcf5ca06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteDrawing-response>)))
  "Returns full string definition for message of type '<ExecuteDrawing-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteDrawing-response)))
  "Returns full string definition for message of type 'ExecuteDrawing-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteDrawing-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteDrawing-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteDrawing-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ExecuteDrawing)))
  'ExecuteDrawing-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ExecuteDrawing)))
  'ExecuteDrawing-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteDrawing)))
  "Returns string type for a service object of type '<ExecuteDrawing>"
  "block_drawing_msgs/ExecuteDrawing")